/**
 * @file benchmark_chunk_size.cpp
 * @brief Standalone (no-ROS) sweep of the snap.chunk_size parameter over
 *        the real mission waypoints, replicating the exact chunking +
 *        collision-check pipeline used in Plugin::on_activate().
 *
 * For each chunk_size value, generates the full discrete JPS3D path for
 * the whole mission (all segments concatenated, per the multi-waypoint
 * logic), then applies chunked Minimum-Snap smoothing exactly as
 * production code does, reporting how many chunks fall back to discrete.
 *
 * Usage:
 *   benchmark_chunk_size <map.bt> [inflation_radius]
 * Default inflation_radius = 0.20
 */
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "as2_3d_path_planner_plugin/octomap_planner_map.hpp"
#include "jps3d_planner.hpp"
#include "astar3d.hpp"
#include "minimum_snap.hpp"


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
    const double cx = traj[idx].x();
    const double cy = traj[idx].y();
    const double cz = traj[idx].z();
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dz = -r; dz <= r; ++dz) {
          if (dx * dx + dy * dy + dz * dz > r2) {continue;}
          const double wx = cx + dx * res;
          const double wy = cy + dy * res;
          const double wz = cz + dz * res;
          const auto s = map->getVoxelState(wx, wy, wz);
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

// -----------------------------------------------------------------------
// Real 17-point mission (same as used in run_mission.py / benchmark_mission_segments)
// -----------------------------------------------------------------------
static const std::vector<Eigen::Vector3d> MISSION_WAYPOINTS = {
  {-0.45, -6.5, 1.25},
  {-0.45, -5.2, 1.25},
  {-0.45, -4.8, 2.0},
  {-0.45, -4.2, 2.0},
  {-0.45, -3.8, 0.5},
  {0.2, -2.2, 0.6},
  {0.2, -1.0, 1.5},
  {-0.45, -0.2, 1.25},
  {-0.45, 0.8, 1.25},
  {-2.85, 4.0, 1.35},
  {-2.85, 5.5, 1.35},
  {-2.85, 4.0, 1.35},
  {-0.5, 4.0, 1.35},
  {0.5, 6.0, 1.35},
  {2.0, 6.0, 0.6},
  {2.5, -5.5, 1.5},
  {2.5, -5.5, 0.4},
};

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <map.bt> [inflation_radius]\n";
    return 1;
  }
  const std::string map_file = argv[1];
  const float inflation = argc > 2 ? std::stof(argv[2]) : 0.20f;

  auto map = std::make_shared<as2_3d_path_planner::OctomapPlannerMap>();
  if (!map->loadFromFile(map_file)) {
    std::cerr << "Failed to load map file: " << map_file << "\n";
    return 1;
  }

  Jps3DParams jpar;
  jpar.inflation_radius = inflation;
  jpar.map_update_rate  = 0;
  Astar3D::Params apar;
  apar.inflation_radius = inflation;
  Jps3DPlanner jps(map, jpar);
  Astar3D astar(map, apar);

  // --- Step 1: full JPS3D discrete path across all mission segments ---
  std::vector<Eigen::Vector3d> waypoints;
  for (size_t seg = 0; seg + 1 < MISSION_WAYPOINTS.size(); ++seg) {
    const auto & s = MISSION_WAYPOINTS[seg];
    const auto & g = MISSION_WAYPOINTS[seg + 1];
    auto seg_wp = jps.plan(s, g);
    if (seg_wp.empty()) {seg_wp = astar.plan(s, g);}
    if (seg_wp.empty()) {
      std::cerr << "Segment " << seg << " failed to plan — aborting.\n";
      return 1;
    }
    const size_t skip = (seg == 0) ? 0u : 1u;
    waypoints.insert(waypoints.end(), seg_wp.begin() + skip, seg_wp.end());
  }

  std::cout << "=== CHUNK SIZE SWEEP ===\n";
  std::cout << "Map file        : " << map_file << "\n";
  std::cout << "Inflation radius: " << inflation << " m\n";
  std::cout << "Total discrete waypoints (whole mission): " << waypoints.size() << "\n\n";

  const std::vector<int> chunk_sizes = {4, 6, 8, 10, 12, 16, 20};

  std::cout << "chunk_size | n_chunks | smooth | discrete | %discrete | samples\n";
  std::cout << "-----------|----------|--------|----------|-----------|--------\n";

  MinimumSnap::Params snap_params;
  // Match production defaults from path_planner_params.yaml
  snap_params.total_time = 10.0;
  snap_params.sample_dt  = 0.05;
  snap_params.max_vel    = 1.5;
  snap_params.max_acc    = 2.0;

  for (int chunk_size : chunk_sizes) {
    MinimumSnap snap(snap_params);
    const int n_wp = static_cast<int>(waypoints.size());
    int chunk_idx = 0, smooth_chunks = 0, discrete_chunks = 0, total_samples = 0;

    for (int chunk_start = 0; chunk_start < n_wp - 1; ) {
      const int chunk_end = std::min(chunk_start + chunk_size - 1, n_wp - 1);
      const std::vector<Eigen::Vector3d> chunk_wps(
        waypoints.cbegin() + chunk_start, waypoints.cbegin() + chunk_end + 1);

      bool use_discrete = false;
      if (chunk_wps.size() >= 2) {
        const auto sampled = snap.generate(chunk_wps);
        if (sampled.size() < 2) {
          use_discrete = true;
        } else {
          int unsafe_idx = -1;
          const bool safe = trajectoryIsSafe(sampled, map, inflation, unsafe_idx);
          if (safe) {
            ++smooth_chunks;
            total_samples += static_cast<int>(sampled.size());
          } else {
            use_discrete = true;
          }
        }
      } else {
        use_discrete = true;
      }

      if (use_discrete) {
        ++discrete_chunks;
        total_samples += static_cast<int>(chunk_wps.size());
      }

      ++chunk_idx;
      chunk_start = chunk_end;
      if (chunk_start >= n_wp - 1) {break;}
    }

    const double pct_discrete = 100.0 * discrete_chunks / chunk_idx;
    std::cout << std::setw(10) << chunk_size << " |"
              << std::setw(9) << chunk_idx << " |"
              << std::setw(7) << smooth_chunks << " |"
              << std::setw(9) << discrete_chunks << " |"
              << std::setw(10) << std::fixed << std::setprecision(1) << pct_discrete << "% |"
              << std::setw(8) << total_samples << "\n";
  }

  return 0;
}
