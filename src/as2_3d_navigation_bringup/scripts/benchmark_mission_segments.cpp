/**
 * @file benchmark_mission_segments.cpp
 * @brief Standalone (no-ROS) benchmark of Astar3D vs Jps3DPlanner over
 *        REAL mission waypoints (not uniformly random points), testing
 *        consecutive segments plus random re-orderings of the same point
 *        set to reach a larger, still physically meaningful sample.
 *
 * Rationale: benchmark_planners samples uniformly random points across
 * the whole arena volume, which can include physically nonsensical
 * start/goal points (e.g. flush against a wall). This tool instead uses
 * a fixed list of REAL mission waypoints (known to be reachable, chosen
 * by the operator), and evaluates:
 *   - Block 0: the 15 consecutive segments in the ORIGINAL mission order.
 *   - Blocks 1..N: the same 15 segments, but visiting the 16 waypoints in
 *     a randomly shuffled order (fixed seed per block, reproducible).
 * Total segments are trimmed to exactly 50.
 *
 * Waypoints are hardcoded below — edit WAYPOINTS_17 to match the mission
 * (the last point is dropped automatically, per the 16-point requirement).
 *
 * Usage:
 *   benchmark_mission_segments <map.bt> [inflation_radius] [total_segments]
 * Defaults: inflation_radius=0.20, total_segments=50
 */
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "as2_3d_path_planner_plugin/octomap_planner_map.hpp"
#include "jps3d_planner.hpp"
#include "astar3d.hpp"

using Clock = std::chrono::steady_clock;

static double ms_since(const Clock::time_point & t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

static double pathLength(const std::vector<Eigen::Vector3d> & path)
{
  double len = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    len += (path[i] - path[i - 1]).norm();
  }
  return len;
}

// -----------------------------------------------------------------------
// Original 17-point mission list. The LAST point is dropped automatically
// (per requirement: 17 given -> drop last -> 16 points used).
// -----------------------------------------------------------------------
static const std::vector<Eigen::Vector3d> WAYPOINTS_17 = {
  {-0.45, -6.5, 1.25},
  {-0.45, -5.2, 1.25},
  {-0.45, -4.8, 2.0},
  {-0.45, -4.2, 2.0},
  {-0.45, -3.8, 0.6},
  {0.2, -2.2, 0.6},
  {0.2, -1.0, 1.25},
  {-0.45, -0.2, 1.25},
  {-0.45, 0.8, 1.25},
  {-2.85, 4.0, 1.35},
  {-2.85, 5.5, 1.35},
  {-2.85, 4.0, 1.35},
  {-0.5, 4.0, 1.35},
  {0.5, 6.0, 1.35},
  {2.0, 6.0, 0.6},
  {2.5, -5.5, 1.5},
  {2.5, -5.5, 0.4},   // <- dropped
};

struct SegResult
{
  int block;
  size_t from_idx, to_idx;
  Eigen::Vector3d from, to;
  double dist;
  bool jps_ok; double jps_ms; size_t jps_wp; double jps_len;
  bool astar_ok; double astar_ms; size_t astar_wp; double astar_len;
};

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <map.bt> [inflation_radius] [total_segments]\n";
    return 1;
  }
  const std::string map_file = argv[1];
  const float inflation = argc > 2 ? std::stof(argv[2]) : 0.20f;
  const int total_segments = argc > 3 ? std::stoi(argv[3]) : 50;

  std::vector<Eigen::Vector3d> waypoints16(
    WAYPOINTS_17.begin(), WAYPOINTS_17.end() - 1);  // drop last point

  auto map = std::make_shared<as2_3d_path_planner::OctomapPlannerMap>();
  if (!map->loadFromFile(map_file)) {
    std::cerr << "Failed to load map file: " << map_file << "\n";
    return 1;
  }

  std::cout << "=== MISSION SEGMENT BENCHMARK (real waypoints + shuffles) ===\n";
  std::cout << "Map file          : " << map_file << "\n";
  std::cout << "Map nodes         : " << map->getNodeCount() << "\n";
  std::cout << "Inflation radius  : " << inflation << " m\n";
  std::cout << "Base waypoints    : " << waypoints16.size()
            << " (last of original 17 dropped)\n";
  std::cout << "Segments per block: " << (waypoints16.size() - 1) << "\n";
  std::cout << "Target segments   : " << total_segments << "\n\n";

  Jps3DParams jpar;
  jpar.inflation_radius = inflation;
  jpar.map_update_rate  = 0;
  Astar3D::Params apar;
  apar.inflation_radius = inflation;

  Jps3DPlanner jps(map, jpar);
  Astar3D astar(map, apar);

  auto stateName = [&](const Eigen::Vector3d & p) -> std::string {
    const auto s = map->getVoxelState(p.x(), p.y(), p.z());
    switch (s) {
      case as2_3d_map_interface::VoxelState::FREE: return "FREE";
      case as2_3d_map_interface::VoxelState::OCCUPIED: return "OCCUPIED";
      case as2_3d_map_interface::VoxelState::UNKNOWN: return "UNKNOWN";
      default: return "OUT_OF_BOUNDS";
    }
  };

  std::cout << "=== WAYPOINT STATES (occupancy check, base 16 points) ===\n";
  for (size_t i = 0; i < waypoints16.size(); ++i) {
    const auto & p = waypoints16[i];
    std::cout << "  wp" << i << " (" << p.x() << "," << p.y() << "," << p.z()
              << ") -> " << stateName(p) << "\n";
  }
  std::cout << "\n";

  // Build the ordered sequence of point-orderings: block 0 = original
  // order; blocks 1.. = random shuffles with fixed, reproducible seeds.
  std::vector<std::vector<Eigen::Vector3d>> blocks;
  blocks.push_back(waypoints16);  // block 0: original order

  const int segs_per_block = static_cast<int>(waypoints16.size()) - 1;
  int block_idx = 1;
  while (static_cast<int>(blocks.size()) * segs_per_block < total_segments) {
    std::vector<Eigen::Vector3d> shuffled = waypoints16;
    std::mt19937 rng(1000u + block_idx);  // reproducible seed per block
    std::shuffle(shuffled.begin(), shuffled.end(), rng);
    blocks.push_back(shuffled);
    ++block_idx;
  }

  std::vector<SegResult> results;
  results.reserve(total_segments);

  std::cout << "=== SEGMENT RESULTS ===\n";
  std::cout << "blk | seg |  dist |  JPS3D ms | wp | len(m) | ok |   A* ms | wp | len(m) | ok\n";
  std::cout << "----|-----|-------|-----------|----|--------|----| --------|----|--------|----\n";

  for (size_t b = 0; b < blocks.size() && static_cast<int>(results.size()) < total_segments; ++b) {
    const auto & pts = blocks[b];
    for (size_t i = 0; i + 1 < pts.size() && static_cast<int>(results.size()) < total_segments; ++i) {
      SegResult r{};
      r.block = static_cast<int>(b);
      r.from = pts[i];
      r.to = pts[i + 1];
      r.dist = (r.to - r.from).norm();

      const auto t_j = Clock::now();
      auto jpath = jps.plan(r.from, r.to);
      r.jps_ms = ms_since(t_j);
      r.jps_ok = !jpath.empty();
      r.jps_wp = jpath.size();
      r.jps_len = pathLength(jpath);

      const auto t_a = Clock::now();
      auto apath = astar.plan(r.from, r.to);
      r.astar_ms = ms_since(t_a);
      r.astar_ok = !apath.empty();
      r.astar_wp = apath.size();
      r.astar_len = pathLength(apath);

      results.push_back(r);

      std::cout << std::setw(3) << b << " |"
                << std::setw(4) << (results.size() - 1) << " |"
                << std::setw(6) << std::fixed << std::setprecision(2) << r.dist << " |"
                << std::setw(10) << r.jps_ms << " |"
                << std::setw(3) << r.jps_wp << " |"
                << std::setw(7) << r.jps_len << " |"
                << std::setw(3) << (r.jps_ok ? "Y" : "N") << " |"
                << std::setw(8) << r.astar_ms << " |"
                << std::setw(3) << r.astar_wp << " |"
                << std::setw(7) << r.astar_len << " |"
                << std::setw(3) << (r.astar_ok ? "Y" : "N")
                << (!r.jps_ok ? "   <-- JPS3D FAILED" : "")
                << (!r.astar_ok ? "   <-- A* FAILED" : "")
                << "\n";
    }
  }

  int jps_success = 0, astar_success = 0;
  double jps_time_sum = 0.0, astar_time_sum = 0.0;
  for (const auto & r : results) {
    if (r.jps_ok) {jps_success++; jps_time_sum += r.jps_ms;}
    if (r.astar_ok) {astar_success++; astar_time_sum += r.astar_ms;}
  }

  std::cout << "\n=== SUMMARY ===\n";
  std::cout << "Total segments : " << results.size()
            << " (across " << blocks.size() << " orderings: 1 original + "
            << (blocks.size() - 1) << " random shuffles)\n";
  std::cout << "JPS3D success  : " << jps_success << "/" << results.size()
            << " (" << (100.0 * jps_success / results.size()) << "%)"
            << " | mean time (successful): "
            << (jps_success ? jps_time_sum / jps_success : 0.0) << " ms\n";
  std::cout << "A*    success  : " << astar_success << "/" << results.size()
            << " (" << (100.0 * astar_success / results.size()) << "%)"
            << " | mean time (successful): "
            << (astar_success ? astar_time_sum / astar_success : 0.0) << " ms\n";

  if (jps_success < static_cast<int>(results.size())) {
    std::cout << "\n=== JPS3D FAILURES DETAIL ===\n";
    for (const auto & r : results) {
      if (!r.jps_ok) {
        std::cout << "  block " << r.block << ": ("
                  << r.from.x() << "," << r.from.y() << "," << r.from.z()
                  << ") -> (" << r.to.x() << "," << r.to.y() << "," << r.to.z()
                  << ") dist=" << r.dist << "m\n";
      }
    }
  }

  return 0;
}
