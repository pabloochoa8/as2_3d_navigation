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

#include "astar3d.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>

static constexpr float kSqrt2 = 1.41421356f;
static constexpr float kSqrt3 = 1.73205081f;

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

Astar3D::Astar3D(
  std::shared_ptr<as2_3d_map_interface::MapInterface> map,
  const Params & params)
: map_(std::move(map)), params_(params)
{
  res_ = map_->getResolution();
  const auto b = map_->getBounds();
  x_min_ = b.x_min;
  y_min_ = b.y_min;
  z_min_ = b.z_min;

  nx_ = static_cast<int>(std::ceil((b.x_max - b.x_min) / res_)) + 1;
  ny_ = static_cast<int>(std::ceil((b.y_max - b.y_min) / res_)) + 1;
  nz_ = static_cast<int>(std::ceil((b.z_max - b.z_min) / res_)) + 1;

  // Subtract a small epsilon before ceil to counteract float→double precision
  // loss (e.g. 0.10f / 0.05 → 2.0000000298... → ceil = 3 instead of 2).
  inflation_r_ = static_cast<int>(
    std::ceil(static_cast<double>(params_.inflation_radius) / res_ - 1e-6));
}

// ---------------------------------------------------------------------------
// Public
// ---------------------------------------------------------------------------

Astar3D::Result Astar3D::lastResult() const {return last_result_;}

std::vector<Eigen::Vector3d> Astar3D::plan(
  const Eigen::Vector3d & start,
  const Eigen::Vector3d & goal)
{
  last_result_ = Result::NO_PATH;

  // --- bounds check ---
  if (!map_->isInBounds(start.x(), start.y(), start.z()) ||
    !map_->isInBounds(goal.x(), goal.y(), goal.z()))
  {
    last_result_ = Result::OUT_OF_BOUNDS;
    return {};
  }

  // --- voxel indices ---
  int sx, sy, sz, gx, gy, gz;
  worldToVoxel(start.x(), start.y(), start.z(), sx, sy, sz);
  worldToVoxel(goal.x(),  goal.y(),  goal.z(),  gx, gy, gz);

  // --- validity checks ---
  if (!isInflatedFree(sx, sy, sz)) {
    last_result_ = Result::START_OCCUPIED;
    return {};
  }
  if (!isInflatedFree(gx, gy, gz)) {
    last_result_ = Result::GOAL_OCCUPIED;
    return {};
  }

  // --- A* data structures ---
  // Priority queue: (f_value, voxel_key) — min-heap by f
  using PQ = std::pair<float, int64_t>;
  std::priority_queue<PQ, std::vector<PQ>, std::greater<PQ>> open;

  std::unordered_map<int64_t, float>   g_cost;
  std::unordered_map<int64_t, int64_t> came_from;
  std::unordered_set<int64_t>          closed;

  const int64_t start_key = key(sx, sy, sz);
  const int64_t goal_key  = key(gx, gy, gz);

  g_cost[start_key] = 0.0f;
  open.push({heuristic(sx, sy, sz, gx, gy, gz), start_key});

  // --- pre-compute neighbor offsets ---
  struct NbrOffset
  {
    int dx, dy, dz;
    float edge_cost;
  };
  std::vector<NbrOffset> nbrs;
  nbrs.reserve(26);
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {continue;}
        const int m = std::abs(dx) + std::abs(dy) + std::abs(dz);
        if (!params_.allow_diagonal && m > 1) {continue;}
        const float c = (m == 1) ? 1.0f : (m == 2) ? kSqrt2 : kSqrt3;
        nbrs.push_back({dx, dy, dz, c * static_cast<float>(res_)});
      }
    }
  }

  const int nynz = ny_ * nz_;
  int iterations = 0;

  // --- main loop ---
  while (!open.empty()) {
    if (++iterations > params_.max_iterations) {
      last_result_ = Result::MAX_ITER_REACHED;
      return {};
    }

    const int64_t cur_key = open.top().second;
    open.pop();

    // With a consistent heuristic the first expansion is always optimal.
    if (closed.count(cur_key)) {continue;}
    closed.insert(cur_key);

    // --- goal reached: reconstruct path ---
    if (cur_key == goal_key) {
      std::vector<Eigen::Vector3d> path;
      int64_t k = goal_key;
      while (true) {
        const int ix = static_cast<int>(k / nynz);
        const int iy = static_cast<int>((k / nz_) % ny_);
        const int iz = static_cast<int>(k % nz_);
        path.push_back(voxelToWorld(ix, iy, iz));
        if (k == start_key) {break;}
        k = came_from.at(k);
      }
      std::reverse(path.begin(), path.end());
      last_result_ = Result::SUCCESS;
      return path;
    }

    // --- expand neighbors ---
    const float g_cur = g_cost.at(cur_key);
    const int cx = static_cast<int>(cur_key / nynz);
    const int cy = static_cast<int>((cur_key / nz_) % ny_);
    const int cz = static_cast<int>(cur_key % nz_);

    for (const auto & nbr : nbrs) {
      const int ni_x = cx + nbr.dx;
      const int ni_y = cy + nbr.dy;
      const int ni_z = cz + nbr.dz;

      if (ni_x < 0 || ni_y < 0 || ni_z < 0 ||
        ni_x >= nx_ || ni_y >= ny_ || ni_z >= nz_)
      {
        continue;
      }

      const int64_t nbr_key = key(ni_x, ni_y, ni_z);
      if (closed.count(nbr_key)) {continue;}
      if (!isInflatedFree(ni_x, ni_y, ni_z)) {continue;}

      const float new_g = g_cur + nbr.edge_cost;
      auto it = g_cost.find(nbr_key);
      if (it == g_cost.end() || new_g < it->second) {
        g_cost[nbr_key] = new_g;
        came_from[nbr_key] = cur_key;
        open.push({new_g + heuristic(ni_x, ni_y, ni_z, gx, gy, gz), nbr_key});
      }
    }
  }

  last_result_ = Result::NO_PATH;
  return {};
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

bool Astar3D::isInflatedFree(int ix, int iy, int iz) const
{
  const int r  = inflation_r_;
  const int r2 = r * r;
  for (int dx = -r; dx <= r; ++dx) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dz = -r; dz <= r; ++dz) {
        if (dx * dx + dy * dy + dz * dz > r2) {continue;}
        const double wx = x_min_ + (ix + dx) * res_;
        const double wy = y_min_ + (iy + dy) * res_;
        const double wz = z_min_ + (iz + dz) * res_;
        // OCCUPIED and OUT_OF_BOUNDS block navigation.
        // UNKNOWN is treated as FREE (optimistic) so the planner can navigate
        // through areas not yet observed during the mapping scan.
        const auto s = map_->getVoxelState(wx, wy, wz);
        if (s == as2_3d_map_interface::VoxelState::OCCUPIED ||
          s == as2_3d_map_interface::VoxelState::OUT_OF_BOUNDS)
        {
          return false;
        }
      }
    }
  }
  return true;
}

float Astar3D::heuristic(int ix, int iy, int iz, int gx, int gy, int gz) const
{
  // Octile 3D distance: exact cost of the diagonal move in 3D.
  float a = static_cast<float>(std::abs(ix - gx));
  float b = static_cast<float>(std::abs(iy - gy));
  float c = static_cast<float>(std::abs(iz - gz));

  // Sort descending so a >= b >= c.
  if (a < b) {std::swap(a, b);}
  if (b < c) {std::swap(b, c);}
  if (a < b) {std::swap(a, b);}

  return params_.w_heuristic * static_cast<float>(res_) *
         (a + (kSqrt2 - 1.0f) * b + (kSqrt3 - kSqrt2) * c);
}

int64_t Astar3D::key(int ix, int iy, int iz) const
{
  return static_cast<int64_t>(ix) * ny_ * nz_ + iy * nz_ + iz;
}

void Astar3D::worldToVoxel(
  double x, double y, double z,
  int & ix, int & iy, int & iz) const
{
  ix = static_cast<int>((x - x_min_) / res_);
  iy = static_cast<int>((y - y_min_) / res_);
  iz = static_cast<int>((z - z_min_) / res_);
}

Eigen::Vector3d Astar3D::voxelToWorld(int ix, int iy, int iz) const
{
  return {x_min_ + ix * res_, y_min_ + iy * res_, z_min_ + iz * res_};
}
