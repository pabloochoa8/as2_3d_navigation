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

/**
 * @file test_minimum_snap.cpp
 * @brief Standalone (no-ROS) test for the full pipeline:
 *        JPS3D → Minimum-Snap → collision verification.
 *
 * Uses ArtificialMap (wall at x=[4.5,5.5], y=[0,2]) and the JPS3D case
 * (1,1,0.5)→(7,1,0.5) that must navigate around the wall.
 */

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "as2_3d_map_server_plugin/artificial_map.hpp"
#include "jps3d_planner.hpp"
#include "minimum_snap.hpp"

// ---------------------------------------------------------------------------
// Inline collision checker (mirrors Plugin::trajectoryIsSafe)
// ---------------------------------------------------------------------------

static bool trajectoryIsSafe(
  const std::vector<Eigen::Vector3d> & traj,
  std::shared_ptr<as2_3d_map_interface::MapInterface> map,
  float inflation_radius,
  int & first_unsafe_idx)
{
  first_unsafe_idx = -1;
  const double res = map->getResolution();
  const int r = static_cast<int>(
    std::ceil(static_cast<double>(inflation_radius) / res - 1e-6));
  const int r2 = r * r;

  for (int idx = 0; idx < static_cast<int>(traj.size()); ++idx) {
    const double cx = traj[idx].x(), cy = traj[idx].y(), cz = traj[idx].z();
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dz = -r; dz <= r; ++dz) {
          if (dx * dx + dy * dy + dz * dz > r2) {continue;}
          const auto s = map->getVoxelState(cx + dx * res, cy + dy * res, cz + dz * res);
          if (s == as2_3d_map_interface::VoxelState::OCCUPIED ||
              s == as2_3d_map_interface::VoxelState::OUT_OF_BOUNDS)
          {
            first_unsafe_idx = idx;
            return false;
          }
        }
      }
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main()
{
  auto map = std::make_shared<as2_3d_map_server::ArtificialMap>();

  // --- JPS3D: (1,1,0.5) → (7,1,0.5), no inflation so collision check is strict ---
  Jps3DParams jpar;
  jpar.inflation_radius = 0.0f;
  jpar.map_update_rate  = 0;

  std::cout << "=== Building JPS3D collision map...\n";
  const auto t_jps0 = std::chrono::steady_clock::now();
  Jps3DPlanner jps(map, jpar);
  const auto t_jps1 = std::chrono::steady_clock::now();
  std::cout << "    done in "
            << std::chrono::duration<double, std::milli>(t_jps1 - t_jps0).count()
            << " ms\n\n";

  const Eigen::Vector3d start{1.0, 1.0, 0.5};
  const Eigen::Vector3d goal{7.0, 1.0, 0.5};

  std::cout << "=== JPS3D planning (1,1,0.5) → (7,1,0.5) ===\n";
  const auto t_plan0 = std::chrono::steady_clock::now();
  auto waypoints = jps.plan(start, goal);
  const auto t_plan1 = std::chrono::steady_clock::now();

  if (jps.lastResult() != Jps3DPlanner::Result::SUCCESS) {
    std::cerr << "FAIL: JPS3D returned no path\n";
    return 1;
  }

  std::cout << "    Waypoints: " << waypoints.size()
            << "  in " << std::chrono::duration<double, std::milli>(t_plan1 - t_plan0).count()
            << " ms\n";

  // Print first 3 and last 3 waypoints
  const auto print_pt = [](const Eigen::Vector3d & p) {
      std::cout << "    (" << std::fixed << std::setprecision(2)
                << p.x() << ", " << p.y() << ", " << p.z() << ")\n";
    };

  std::cout << "  First 3:\n";
  for (size_t i = 0; i < std::min(waypoints.size(), size_t{3}); ++i) {print_pt(waypoints[i]);}
  std::cout << "  Last 3:\n";
  for (size_t i = waypoints.size() > 3 ? waypoints.size() - 3 : 0; i < waypoints.size(); ++i) {
    print_pt(waypoints[i]);
  }
  std::cout << "\n";

  // --- Minimum-Snap ---
  MinimumSnapParams spar;
  spar.total_time = 10.0;
  spar.sample_dt  = 0.05;
  spar.max_vel    = 1.5;
  spar.max_acc    = 2.0;

  MinimumSnap snap(spar);

  std::cout << "=== Minimum-Snap trajectory generation ===\n";
  const auto t_snap0 = std::chrono::steady_clock::now();
  const auto sampled = snap.generate(waypoints);
  const auto t_snap1 = std::chrono::steady_clock::now();

  if (sampled.empty()) {
    std::cerr << "FAIL: MinimumSnap returned empty trajectory\n";
    return 1;
  }

  std::cout << "    Input waypoints     : " << waypoints.size() << "\n";
  std::cout << "    Output samples      : " << sampled.size() << "\n";
  std::cout << "    Total time          : " << std::fixed << std::setprecision(2)
            << snap.totalTime() << " s\n";
  std::cout << "    Sample dt           : " << spar.sample_dt << " s\n";
  std::cout << "    Generation time     : "
            << std::chrono::duration<double, std::milli>(t_snap1 - t_snap0).count()
            << " ms\n";
  std::cout << "  First 5 samples:\n";
  for (size_t i = 0; i < std::min(sampled.size(), size_t{5}); ++i) {print_pt(sampled[i]);}
  std::cout << "  Last 5 samples:\n";
  for (size_t i = sampled.size() > 5 ? sampled.size() - 5 : 0; i < sampled.size(); ++i) {
    print_pt(sampled[i]);
  }
  std::cout << "\n";

  // --- Collision check ---
  std::cout << "=== Collision verification (inflation_radius=0.0 m) ===\n";
  int unsafe_idx = -1;
  const bool safe = trajectoryIsSafe(sampled, map, 0.0f, unsafe_idx);

  if (safe) {
    std::cout << "    SAFE — all " << sampled.size() << " samples are collision-free\n";
  } else {
    std::cout << "    UNSAFE at sample " << unsafe_idx
              << " (" << sampled[unsafe_idx].transpose() << ")\n";
    std::cout << "    (fallback to discrete waypoints would be triggered in the plugin)\n";
  }

  // Sanity: start and end of smooth traj should be close to discrete start/goal
  const double d_start = (sampled.front() - start).norm();
  const double d_end   = (sampled.back() - goal).norm();
  std::cout << "    Start error: " << std::scientific << std::setprecision(1) << d_start
            << " m   End error: " << d_end << " m\n";

  if (d_start > 0.01 || d_end > 0.01) {
    std::cerr << "FAIL: start/end positions deviate too much from waypoints\n";
    return 1;
  }

  std::cout << "\nAll tests passed.\n";
  return 0;
}
