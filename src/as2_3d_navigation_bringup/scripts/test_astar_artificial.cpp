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
 * @file test_astar_artificial.cpp
 * @brief Standalone (no-ROS) test for Astar3D using ArtificialMap.
 *
 * Map geometry (ArtificialMap):
 *   Volume  : x=[0,10], y=[0,5], z=[0,3] m
 *   Wall    : x=[4.5,5.5], y=[0,2], z=[0,3]  — passage at y=[2,5]
 *   Frame   : "earth", resolution 0.05 m
 *
 * Run (after building the workspace):
 *   ./install/as2_3d_navigation_bringup/lib/as2_3d_navigation_bringup/test_astar_artificial
 */

#include <iostream>
#include <iomanip>
#include <memory>
#include <vector>

#include <Eigen/Core>

// Header-only — no ROS runtime needed
#include "as2_3d_map_server_plugin/artificial_map.hpp"
#include "astar3d.hpp"

static const char * result_str(Astar3D::Result r)
{
  switch (r) {
    case Astar3D::Result::SUCCESS:          return "SUCCESS";
    case Astar3D::Result::NO_PATH:          return "NO_PATH";
    case Astar3D::Result::START_OCCUPIED:   return "START_OCCUPIED";
    case Astar3D::Result::GOAL_OCCUPIED:    return "GOAL_OCCUPIED";
    case Astar3D::Result::OUT_OF_BOUNDS:    return "OUT_OF_BOUNDS";
    case Astar3D::Result::MAX_ITER_REACHED: return "MAX_ITER_REACHED";
  }
  return "UNKNOWN";
}

static void print_path(const std::vector<Eigen::Vector3d> & path)
{
  if (path.empty()) {
    std::cout << "  (empty)\n";
    return;
  }
  const size_t max_shown = 8;
  for (size_t i = 0; i < std::min(path.size(), max_shown); ++i) {
    std::cout << std::fixed << std::setprecision(2)
              << "  [" << i << "] ("
              << path[i].x() << ", "
              << path[i].y() << ", "
              << path[i].z() << ")\n";
  }
  if (path.size() > max_shown) {
    std::cout << "  ... (" << (path.size() - max_shown) << " more waypoints)\n";
  }
}

int main()
{
  auto map = std::make_shared<as2_3d_map_server::ArtificialMap>();

  // -------------------------------------------------------------------
  // Test 1: (0,0,0.5) → (3,3,0.5)
  // No obstacle between start and goal — A* must find a path.
  // inflation_radius=0 to avoid out-of-bounds treatment at map edge.
  // -------------------------------------------------------------------
  {
    Astar3D::Params p;
    p.max_iterations   = 500000;
    p.inflation_radius = 0.0f;   // zero inflation: boundary-safe for corners
    p.allow_diagonal   = true;
    p.w_heuristic      = 1.0f;

    Astar3D planner(map, p);

    Eigen::Vector3d start{0.0, 0.0, 0.5};
    Eigen::Vector3d goal{3.0, 3.0, 0.5};

    std::cout << "\n=== Test 1: free path (no obstacle) ===\n";
    std::cout << "  Start : (0.00, 0.00, 0.50)\n";
    std::cout << "  Goal  : (3.00, 3.00, 0.50)\n";
    std::cout << "  Inflation: 0.00 m\n";

    auto path = planner.plan(start, goal);

    std::cout << "  Result  : " << result_str(planner.lastResult()) << "\n";
    std::cout << "  Waypoints (" << path.size() << "):\n";
    print_path(path);

    if (planner.lastResult() != Astar3D::Result::SUCCESS) {
      std::cerr << "  FAIL: expected SUCCESS\n";
      return 1;
    }
  }

  // -------------------------------------------------------------------
  // Test 2: (1,1,0.5) → (7,1,0.5) — must navigate around the wall
  // Wall at x=[4.5,5.5], y=[0,2] blocks the straight line.
  // Path must detour through y > 2.
  // -------------------------------------------------------------------
  {
    Astar3D::Params p;
    p.max_iterations   = 500000;
    p.inflation_radius = 0.10f;
    p.allow_diagonal   = true;
    p.w_heuristic      = 1.0f;

    Astar3D planner(map, p);

    Eigen::Vector3d start{1.0, 1.0, 0.5};
    Eigen::Vector3d goal{7.0, 1.0, 0.5};

    std::cout << "\n=== Test 2: detour around wall ===\n";
    std::cout << "  Start : (1.00, 1.00, 0.50)\n";
    std::cout << "  Goal  : (7.00, 1.00, 0.50)\n";
    std::cout << "  Wall  : x=[4.5,5.5], y=[0,2], z=[0,3]\n";
    std::cout << "  Inflation: 0.10 m\n";

    auto path = planner.plan(start, goal);

    std::cout << "  Result  : " << result_str(planner.lastResult()) << "\n";
    std::cout << "  Waypoints (" << path.size() << "):\n";
    print_path(path);

    if (planner.lastResult() != Astar3D::Result::SUCCESS) {
      std::cerr << "  FAIL: expected SUCCESS\n";
      return 1;
    }
  }

  // -------------------------------------------------------------------
  // Test 3: start inside the wall → must return START_OCCUPIED
  // -------------------------------------------------------------------
  {
    Astar3D::Params p;
    p.max_iterations   = 500000;
    p.inflation_radius = 0.0f;
    p.allow_diagonal   = true;
    p.w_heuristic      = 1.0f;

    Astar3D planner(map, p);

    Eigen::Vector3d start{5.0, 1.0, 1.5};   // well inside the wall
    Eigen::Vector3d goal{8.0, 4.0, 1.5};

    std::cout << "\n=== Test 3: start inside obstacle → START_OCCUPIED ===\n";
    std::cout << "  Start : (5.00, 1.00, 1.50) — inside wall\n";
    std::cout << "  Goal  : (8.00, 4.00, 1.50)\n";

    auto path = planner.plan(start, goal);

    std::cout << "  Result  : " << result_str(planner.lastResult()) << "\n";
    std::cout << "  Expected: START_OCCUPIED\n";

    if (planner.lastResult() != Astar3D::Result::START_OCCUPIED) {
      std::cerr << "  FAIL: expected START_OCCUPIED\n";
      return 1;
    }
  }

  std::cout << "\nAll tests passed.\n";
  return 0;
}
