/**
 * @file test_jps3d_real_map.cpp
 * @brief Standalone (no-ROS) profiling of Jps3DPlanner against the real arena
 *        OctoMap, using production parameters (inflation=0.30, map_update_rate=0).
 *
 * Scans the map for two free points ~4m apart, then calls plan() twice with
 * the SAME start/goal to check whether repeated calls cost the same as the
 * first (diagnosing the "buildMap only logs once but every call still takes
 * seconds" symptom).
 */

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>

#include <Eigen/Core>

#include "as2_3d_path_planner_plugin/octomap_planner_map.hpp"
#include "jps3d_planner.hpp"

using Clock = std::chrono::steady_clock;

static double ms_since(const Clock::time_point & t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

int main(int argc, char ** argv)
{
  const std::string map_file =
    argc > 1 ? argv[1] : "/home/pabloochoa/Desktop/TFM/map/arena_map_BUENA2.bt";

  auto map = std::make_shared<as2_3d_path_planner::OctomapPlannerMap>();
  const auto t_load0 = Clock::now();
  if (!map->loadFromFile(map_file)) {
    std::cerr << "Failed to load map file: " << map_file << "\n";
    return 1;
  }
  std::cout << "Map loaded in " << ms_since(t_load0) << " ms | nodes="
            << map->getNodeCount() << " res=" << map->getResolution() << "\n";

  const auto b = map->getBounds();
  std::cout << "Padded bounds x[" << b.x_min << "," << b.x_max << "] y["
            << b.y_min << "," << b.y_max << "] z[" << b.z_min << "," << b.z_max << "]\n";

  // Production params (from path_planner_params.yaml)
  Jps3DParams jpar;
  jpar.inflation_radius = 0.30f;
  jpar.map_update_rate  = 0;

  std::cout << "\n=== Constructing Jps3DPlanner (production params) ===\n";
  const auto t_ctor0 = Clock::now();
  Jps3DPlanner jps(map, jpar);
  std::cout << "Constructor wall time: " << ms_since(t_ctor0) << " ms\n";

  // Scan for two FREE points ~4m apart, well inside the real IMAV arena
  // (x[-3.5,3.5] y[-7,7] z[0,4]), at a safe altitude.
  auto findFree = [&](double x, double y, double z) -> bool {
    return map->getVoxelState(x, y, z) == as2_3d_map_interface::VoxelState::FREE ||
           map->getVoxelState(x, y, z) == as2_3d_map_interface::VoxelState::UNKNOWN;
  };

  Eigen::Vector3d start, goal;
  bool start_found = false, goal_found = false;
  for (double x = -3.0; x <= 3.0 && !start_found; x += 0.25) {
    for (double y = -3.0; y <= 3.0 && !start_found; y += 0.25) {
      if (findFree(x, y, 1.0)) {
        start = Eigen::Vector3d(x, y, 1.0);
        start_found = true;
      }
    }
  }
  for (double x = 3.0; x >= -3.0 && !goal_found; x -= 0.25) {
    for (double y = 3.0; y >= -3.0 && !goal_found; y -= 0.25) {
      const double dist = (Eigen::Vector3d(x, y, 1.0) - start).norm();
      if (dist > 3.0 && findFree(x, y, 1.0)) {
        goal = Eigen::Vector3d(x, y, 1.0);
        goal_found = true;
      }
    }
  }

  if (!start_found || !goal_found) {
    std::cerr << "Could not find free start/goal points automatically.\n";
    return 1;
  }

  std::cout << "\nStart: " << start.transpose() << "\nGoal:  " << goal.transpose() << "\n";

  for (int i = 1; i <= 3; ++i) {
    std::cout << "\n=== plan() call #" << i << " ===\n";
    const auto t0 = Clock::now();
    auto path = jps.plan(start, goal);
    const double ms = ms_since(t0);
    std::cout << "  wall time (outside function): " << std::fixed
               << std::setprecision(2) << ms << " ms | waypoints=" << path.size()
               << " | result=" << static_cast<int>(jps.lastResult()) << "\n";
  }

  return 0;
}
