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
 * @file test_jps3d_artificial.cpp
 * @brief Standalone (no-ROS) benchmark comparing JPS3D and A* on ArtificialMap.
 *
 * ArtificialMap geometry:
 *   Volume  : x=[0,10], y=[0,5], z=[0,3] m, resolution=0.05 m
 *   Wall    : x=[4.5,5.5], y=[0,2], z=[0,3] — passage above y=2
 *   Frame   : "earth"
 *
 * Test cases:
 *   1. (0,0,0.5) → (3,3,0.5)   — no obstacle between start and goal
 *   2. (1,1,0.5) → (7,1,0.5)   — must navigate around the wall
 *   3. start inside obstacle    — must return START_OCCUPIED
 */

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "as2_3d_map_server_plugin/artificial_map.hpp"
#include "astar3d.hpp"
#include "jps3d_planner.hpp"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static const char * astar_result_str(Astar3D::Result r)
{
  static const char * s[] = {
    "SUCCESS", "NO_PATH", "START_OCCUPIED", "GOAL_OCCUPIED",
    "OUT_OF_BOUNDS", "MAX_ITER_REACHED"
  };
  return s[static_cast<int>(r)];
}

static const char * jps_result_str(Jps3DPlanner::Result r)
{
  switch (r) {
    case Jps3DPlanner::Result::SUCCESS:        return "SUCCESS";
    case Jps3DPlanner::Result::NO_PATH:        return "NO_PATH";
    case Jps3DPlanner::Result::START_OCCUPIED: return "START_OCCUPIED";
    case Jps3DPlanner::Result::GOAL_OCCUPIED:  return "GOAL_OCCUPIED";
    case Jps3DPlanner::Result::OUT_OF_BOUNDS:  return "OUT_OF_BOUNDS";
  }
  return "UNKNOWN";
}

static void print_path_summary(const std::vector<Eigen::Vector3d> & path)
{
  if (path.empty()) {
    std::cout << "    (no path)\n";
    return;
  }
  double dist = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    dist += (path[i] - path[i - 1]).norm();
  }
  std::cout << "    waypoints=" << path.size()
            << "  length=" << std::fixed << std::setprecision(2) << dist << " m";
  std::cout << "  first=" << path.front().transpose().format(
    Eigen::IOFormat(Eigen::StreamPrecision, 0, ",", ",", "", "", "[", "]"));
  std::cout << "  last=" << path.back().transpose().format(
    Eigen::IOFormat(Eigen::StreamPrecision, 0, ",", ",", "", "", "[", "]"));
  std::cout << "\n";
}

struct TestCase {
  Eigen::Vector3d start, goal;
  std::string     description;
};

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main()
{
  // Create ArtificialMap (shared between both planners — no copying).
  auto map = std::make_shared<as2_3d_map_server::ArtificialMap>();

  // Create planners (no inflation for clean comparison; wall avoidance depends
  // purely on occupancy, not on a safety margin).
  Astar3DParams apar;
  apar.inflation_radius = 0.0f;
  apar.allow_diagonal   = true;
  apar.w_heuristic      = 1.0f;

  Jps3DParams jpar;
  jpar.inflation_radius = 0.0f;
  jpar.map_update_rate  = 0;  // build once, reuse

  std::cout << "=== Building JPS3D collision map from ArtificialMap...\n";
  const auto t_build0 = std::chrono::steady_clock::now();
  Jps3DPlanner jps(map, jpar);
  const auto t_build1 = std::chrono::steady_clock::now();
  const double ms_build =
    std::chrono::duration<double, std::milli>(t_build1 - t_build0).count();
  std::cout << "    Map built in " << std::fixed << std::setprecision(1)
            << ms_build << " ms\n\n";

  Astar3D astar(map, apar);

  // Test cases
  const std::vector<TestCase> tests = {
    {Eigen::Vector3d{0.0, 0.0, 0.5},  Eigen::Vector3d{3.0, 3.0, 0.5},
     "Free path: (0,0,0.5) → (3,3,0.5)"},
    {Eigen::Vector3d{1.0, 1.0, 0.5},  Eigen::Vector3d{7.0, 1.0, 0.5},
     "Around wall: (1,1,0.5) → (7,1,0.5)"},
    {Eigen::Vector3d{5.0, 1.0, 1.5},  Eigen::Vector3d{8.0, 4.0, 1.5},
     "Start occupied: (5,1,1.5) inside wall"},
  };

  int failed = 0;

  for (size_t i = 0; i < tests.size(); ++i) {
    const auto & tc = tests[i];
    std::cout << "=== Test " << (i + 1) << ": " << tc.description << " ===\n";
    std::cout << "  Start: " << tc.start.transpose() << "\n";
    std::cout << "  Goal:  " << tc.goal.transpose() << "\n";

    // --- JPS3D ---
    const auto t_jps0 = std::chrono::steady_clock::now();
    auto path_jps = jps.plan(tc.start, tc.goal);
    const auto t_jps1 = std::chrono::steady_clock::now();
    const double ms_jps =
      std::chrono::duration<double, std::milli>(t_jps1 - t_jps0).count();

    std::cout << "  JPS3D [" << jps_result_str(jps.lastResult()) << "]  "
              << std::fixed << std::setprecision(2) << ms_jps << " ms\n";
    print_path_summary(path_jps);

    // --- A* ---
    const auto t_as0 = std::chrono::steady_clock::now();
    auto path_astar = astar.plan(tc.start, tc.goal);
    const auto t_as1 = std::chrono::steady_clock::now();
    const double ms_astar =
      std::chrono::duration<double, std::milli>(t_as1 - t_as0).count();

    std::cout << "  A*    [" << astar_result_str(astar.lastResult()) << "]  "
              << std::fixed << std::setprecision(2) << ms_astar << " ms\n";
    print_path_summary(path_astar);

    // Speed ratio (only meaningful when both succeed)
    if (jps.lastResult() == Jps3DPlanner::Result::SUCCESS &&
        astar.lastResult() == Astar3D::Result::SUCCESS && ms_astar > 0.001)
    {
      std::cout << "  Speed ratio  JPS3D/A* = "
                << std::setprecision(1) << ms_astar / ms_jps << "x\n";
    }

    // Validation for tests 1 and 2
    if (i < 2) {
      if (jps.lastResult() != Jps3DPlanner::Result::SUCCESS) {
        std::cerr << "  FAIL: JPS3D expected SUCCESS\n";
        ++failed;
      }
      if (astar.lastResult() != Astar3D::Result::SUCCESS) {
        std::cerr << "  FAIL: A* expected SUCCESS\n";
        ++failed;
      }
    }
    // Test 3: expect START_OCCUPIED
    if (i == 2) {
      if (jps.lastResult() != Jps3DPlanner::Result::START_OCCUPIED) {
        std::cerr << "  FAIL: JPS3D expected START_OCCUPIED, got "
                  << jps_result_str(jps.lastResult()) << "\n";
        ++failed;
      }
      if (astar.lastResult() != Astar3D::Result::START_OCCUPIED) {
        std::cerr << "  FAIL: A* expected START_OCCUPIED, got "
                  << astar_result_str(astar.lastResult()) << "\n";
        ++failed;
      }
    }
    std::cout << "\n";
  }

  if (failed == 0) {
    std::cout << "All tests passed.\n";
    return 0;
  } else {
    std::cerr << failed << " test(s) FAILED.\n";
    return 1;
  }
}
