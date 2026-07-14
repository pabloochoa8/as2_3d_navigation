// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//      of its contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "jps3d_planner.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>

// JPS3D headers (vendored under thirdparty/jps3d/)
#include <jps_basis/data_type.h>
#include <jps_collision/map_util.h>
#include <jps_planner/jps_planner/jps_planner.h>

using VoxelMapUtil = JPS::MapUtil<3>;
using Clock = std::chrono::steady_clock;
static double ms_since(const Clock::time_point & t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

Jps3DPlanner::Jps3DPlanner(
  std::shared_ptr<as2_3d_map_interface::MapInterface> map,
  const Params & params)
: map_(std::move(map)), params_(params)
{
  res_ = map_->getResolution();

  // Same epsilon fix as Astar3D to avoid float→double precision errors.
  inflation_r_ = static_cast<int>(
    std::ceil(static_cast<double>(params_.inflation_radius) / res_ - 1e-6));

  map_util_ = std::make_shared<VoxelMapUtil>();
  planner_  = std::make_shared<JPSPlanner<3>>(false);   // false = no verbose

  buildMap();
}

Jps3DPlanner::~Jps3DPlanner() = default;

// ---------------------------------------------------------------------------
// Public
// ---------------------------------------------------------------------------

Jps3DPlanner::Result Jps3DPlanner::lastResult() const {return last_result_;}

std::vector<Eigen::Vector3d> Jps3DPlanner::plan(
  const Eigen::Vector3d & start,
  const Eigen::Vector3d & goal)
{
  const auto t_total0 = Clock::now();
  last_result_ = Result::NO_PATH;

  // Conditional map rebuild (rate == 0 → never rebuild after first build).
  ++call_count_;
  const auto t_rebuild0 = Clock::now();
  if (params_.map_update_rate > 0 && (call_count_ % params_.map_update_rate) == 0) {
    buildMap();
  }
  const double ms_rebuild_check = ms_since(t_rebuild0);

  const auto & b = map_->getBounds();

  // --- bounds check ---
  auto inBounds = [&](const Eigen::Vector3d & p) {
    return p.x() >= b.x_min && p.x() <= b.x_max &&
           p.y() >= b.y_min && p.y() <= b.y_max &&
           p.z() >= b.z_min && p.z() <= b.z_max;
  };
  if (!inBounds(start) || !inBounds(goal)) {
    last_result_ = Result::OUT_OF_BOUNDS;
    return {};
  }

  // JPS3D uses decimal_t = double, Vec3f = Eigen::Matrix<double,3,1>
  const Vec3f s(start.x(), start.y(), start.z());
  const Vec3f g(goal.x(),  goal.y(),  goal.z());

  // --- occupancy checks (after dilation) ---
  const auto t_occ0 = Clock::now();
  const Vec3i si = map_util_->floatToInt(s);
  const Vec3i gi = map_util_->floatToInt(g);

  if (map_util_->isOutside(si) || !map_util_->isFree(si)) {
    last_result_ = Result::START_OCCUPIED;
    return {};
  }
  if (map_util_->isOutside(gi) || !map_util_->isFree(gi)) {
    last_result_ = Result::GOAL_OCCUPIED;
    return {};
  }
  const double ms_occ = ms_since(t_occ0);

  // --- plan ---
  // eps=1 (admissible), use_jps=true (JPS mode; false would fall back to A*)
  const auto t_jps0 = Clock::now();
  const bool ok = planner_->plan(s, g, 1.0, true);
  const double ms_jps_plan = ms_since(t_jps0);

  const double ms_total = ms_since(t_total0);
  std::printf(
    "[jps3d_planner] plan() timing: total=%.2f ms | rebuild_check=%.2f ms | "
    "floatToInt+isFree=%.2f ms | planner_->plan()=%.2f ms | call_count=%d\n",
    ms_total, ms_rebuild_check, ms_occ, ms_jps_plan, call_count_);
  std::fflush(stdout);

  if (!ok || planner_->status() != 0) {
    last_result_ = Result::NO_PATH;
    return {};
  }

  // --- convert path ---
  const auto & raw = planner_->getRawPath();
  std::vector<Eigen::Vector3d> path;
  path.reserve(raw.size());
  for (const auto & wp : raw) {
    path.emplace_back(wp.x(), wp.y(), wp.z());
  }

  last_result_ = Result::SUCCESS;
  return path;
}

// ---------------------------------------------------------------------------
// Private: build JPS3D collision map from MapInterface
// ---------------------------------------------------------------------------

void Jps3DPlanner::buildMap()
{
  const auto t_build0 = Clock::now();
  std::printf(
    "[jps3d_planner] buildMap() EXECUTING | call_count=%d | map_update_rate=%d\n",
    call_count_, params_.map_update_rate);
  std::fflush(stdout);

  const auto & b = map_->getBounds();

  // Grid origin placed 0.5*res before the first valid cell so that
  // JPS::MapUtil::floatToInt(bounds_min) maps to cell index 0.
  // (floatToInt uses round(x - 0.5) which equals floor(x), so origin
  //  must satisfy: floor((bounds_min - origin) / res) = 0.)
  const Vec3f origin(
    b.x_min - 0.5 * res_,
    b.y_min - 0.5 * res_,
    b.z_min - 0.5 * res_);

  // Number of cells: one cell per res_ interval covering [bounds_min, bounds_max].
  const int nx = static_cast<int>(std::round((b.x_max - b.x_min) / res_)) + 1;
  const int ny = static_cast<int>(std::round((b.y_max - b.y_min) / res_)) + 1;
  const int nz = static_cast<int>(std::round((b.z_max - b.z_min) / res_)) + 1;
  const Vec3i dim(nx, ny, nz);

  // Build flat occupancy array (0=free, 100=occupied).
  // UNKNOWN is treated as free (optimistic planning for sparse maps).
  JPS::Tmap jps_map(nx * ny * nz, 0);

  for (int ix = 0; ix < nx; ++ix) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int iz = 0; iz < nz; ++iz) {
        // Cell centre: intToFloat would give (pn+0.5)*res+origin, but
        // map_util_ isn't configured yet, so compute directly.
        const double wx = (ix + 0.5) * res_ + origin.x();
        const double wy = (iy + 0.5) * res_ + origin.y();
        const double wz = (iz + 0.5) * res_ + origin.z();

        const auto state = map_->getVoxelState(wx, wy, wz);
        if (state == as2_3d_map_interface::VoxelState::OCCUPIED) {
          jps_map[ix + nx * iy + nx * ny * iz] = 100;
        }
        // FREE, UNKNOWN, OUT_OF_BOUNDS within grid → 0 (passable)
      }
    }
  }

  map_util_->setMap(origin, dim, jps_map, res_);

  // Spherical inflation: dilate occupied cells by inflation_r_ voxels.
  if (inflation_r_ > 0) {
    vec_Veci<3> sphere;
    const int r2 = inflation_r_ * inflation_r_;
    for (int dx = -inflation_r_; dx <= inflation_r_; ++dx) {
      for (int dy = -inflation_r_; dy <= inflation_r_; ++dy) {
        for (int dz = -inflation_r_; dz <= inflation_r_; ++dz) {
          if (dx * dx + dy * dy + dz * dz <= r2) {
            sphere.push_back(Vec3i(dx, dy, dz));
          }
        }
      }
    }
    map_util_->dilate(sphere);
  }

  planner_->setMapUtil(map_util_);
  planner_->updateMap();
  map_built_ = true;

  std::printf(
    "[jps3d_planner] buildMap() DONE in %.2f ms | grid=%dx%dx%d (%d cells)\n",
    ms_since(t_build0), nx, ny, nz, nx * ny * nz);
  std::fflush(stdout);
}
