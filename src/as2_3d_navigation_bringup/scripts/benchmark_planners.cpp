/**
 * @file benchmark_planners.cpp
 * @brief Standalone (no-ROS) comparative benchmark of Astar3D vs Jps3DPlanner
 *        on the real arena OctoMap.
 *
 * Covers experiments (a) A* vs JPS3D benchmark, (f) global planning success
 * rate, and (g) latency scaling vs distance, in a single run.
 *
 * Usage:
 *   benchmark_planners <map.bt> [n_cases] [inflation_radius] [seed]
 * Defaults: n_cases=50, inflation_radius=0.20, seed=42 (reproducible)
 */
#include <algorithm>
#include <chrono>
#include <cmath>
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

struct CaseResult
{
  double distance_straight;
  bool   jps_ok;
  double jps_ms;
  size_t jps_wp;
  double jps_len;
  int    jps_result_code;
  bool   astar_ok;
  double astar_ms;
  size_t astar_wp;
  double astar_len;
  int    astar_result_code;
};

static void printStats(const std::string & label, std::vector<double> v)
{
  if (v.empty()) {
    std::cout << "  " << label << ": (no samples)\n";
    return;
  }
  std::sort(v.begin(), v.end());
  double sum = 0.0;
  for (double x : v) {sum += x;}
  const double mean = sum / v.size();
  const double median = v[v.size() / 2];
  double var = 0.0;
  for (double x : v) {var += (x - mean) * (x - mean);}
  const double stdev = std::sqrt(var / v.size());
  std::cout << "  " << label
            << ": n=" << v.size()
            << " mean=" << std::fixed << std::setprecision(2) << mean
            << " median=" << median
            << " min=" << v.front()
            << " max=" << v.back()
            << " stdev=" << stdev << "\n";
}

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <map.bt> [n_cases] [inflation_radius] [seed]\n";
    return 1;
  }
  const std::string map_file = argv[1];
  const int n_cases = argc > 2 ? std::stoi(argv[2]) : 50;
  const float inflation = argc > 3 ? std::stof(argv[3]) : 0.20f;
  const unsigned seed = argc > 4 ? static_cast<unsigned>(std::stoul(argv[4])) : 42u;

  auto map = std::make_shared<as2_3d_path_planner::OctomapPlannerMap>();
  const auto t_load0 = Clock::now();
  if (!map->loadFromFile(map_file)) {
    std::cerr << "Failed to load map file: " << map_file << "\n";
    return 1;
  }
  const double load_ms = ms_since(t_load0);

  const auto b = map->getBounds();
  std::cout << "=== BENCHMARK CONFIGURATION ===\n";
  std::cout << "Map file        : " << map_file << "\n";
  std::cout << "Map load time   : " << std::fixed << std::setprecision(2)
            << load_ms << " ms\n";
  std::cout << "Map nodes       : " << map->getNodeCount() << "\n";
  std::cout << "Map resolution  : " << map->getResolution() << " m\n";
  std::cout << "Padded bounds   : x[" << b.x_min << "," << b.x_max
            << "] y[" << b.y_min << "," << b.y_max
            << "] z[" << b.z_min << "," << b.z_max << "]\n";
  std::cout << "Test cases      : " << n_cases << "\n";
  std::cout << "Inflation radius: " << inflation << " m\n";
  std::cout << "RNG seed        : " << seed << " (reproducible)\n\n";

  Jps3DParams jpar;
  jpar.inflation_radius = inflation;
  jpar.map_update_rate  = 0;

  Astar3D::Params apar;
  apar.inflation_radius = inflation;

  const auto t_jps_ctor = Clock::now();
  Jps3DPlanner jps(map, jpar);
  const double jps_ctor_ms = ms_since(t_jps_ctor);

  const auto t_astar_ctor = Clock::now();
  Astar3D astar(map, apar);
  const double astar_ctor_ms = ms_since(t_astar_ctor);

  std::cout << "JPS3D construction : " << jps_ctor_ms << " ms\n";
  std::cout << "A*    construction : " << astar_ctor_ms << " ms\n\n";

  const double AX_MIN = -3.4, AX_MAX = 3.4;
  const double AY_MIN = -6.9, AY_MAX = 6.9;
  const double AZ_MIN = 0.5,  AZ_MAX = 2.5;
  const double MIN_PAIR_DIST = 2.0;

  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dx(AX_MIN, AX_MAX);
  std::uniform_real_distribution<double> dy(AY_MIN, AY_MAX);
  std::uniform_real_distribution<double> dz(AZ_MIN, AZ_MAX);

  auto isUsable = [&](const Eigen::Vector3d & p) {
    const auto s = map->getVoxelState(p.x(), p.y(), p.z());
    return s == as2_3d_map_interface::VoxelState::FREE ||
           s == as2_3d_map_interface::VoxelState::UNKNOWN;
  };

  std::vector<CaseResult> results;
  results.reserve(n_cases);

  std::cout << "=== PER-CASE RESULTS ===\n";
  std::cout << "case |   dist |  JPS3D ms |  wp | len(m) | ok |   A* ms |  wp | len(m) | ok\n";
  std::cout << "-----|--------|-----------|-----|--------|----|---------|-----|--------|----\n";

  int generated = 0, attempts = 0;
  const int MAX_ATTEMPTS = n_cases * 200;

  while (generated < n_cases && attempts < MAX_ATTEMPTS) {
    ++attempts;
    Eigen::Vector3d s(dx(rng), dy(rng), dz(rng));
    Eigen::Vector3d g(dx(rng), dy(rng), dz(rng));
    if ((g - s).norm() < MIN_PAIR_DIST) {continue;}
    if (!isUsable(s) || !isUsable(g)) {continue;}

    CaseResult r{};
    r.distance_straight = (g - s).norm();

    const auto t_j = Clock::now();
    auto jpath = jps.plan(s, g);
    r.jps_ms = ms_since(t_j);
    r.jps_wp = jpath.size();
    r.jps_len = pathLength(jpath);
    r.jps_ok = !jpath.empty();
    r.jps_result_code = static_cast<int>(jps.lastResult());

    const auto t_a = Clock::now();
    auto apath = astar.plan(s, g);
    r.astar_ms = ms_since(t_a);
    r.astar_wp = apath.size();
    r.astar_len = pathLength(apath);
    r.astar_ok = !apath.empty();
    r.astar_result_code = static_cast<int>(astar.lastResult());

    results.push_back(r);
    ++generated;

    std::cout << std::setw(4) << generated << " |"
              << std::setw(7) << std::fixed << std::setprecision(2) << r.distance_straight << " |"
              << std::setw(10) << r.jps_ms << " |"
              << std::setw(4) << r.jps_wp << " |"
              << std::setw(7) << r.jps_len << " |"
              << std::setw(3) << (r.jps_ok ? "Y" : "N") << " |"
              << std::setw(8) << r.astar_ms << " |"
              << std::setw(4) << r.astar_wp << " |"
              << std::setw(7) << r.astar_len << " |"
              << std::setw(3) << (r.astar_ok ? "Y" : "N") << "\n";
  }

  if (results.empty()) {
    std::cerr << "\nNo valid test cases generated after " << attempts
              << " attempts. Check arena bounds vs map contents.\n";
    return 1;
  }

  std::vector<double> jps_times, astar_times, jps_wps, astar_wps;
  std::vector<double> jps_lens, astar_lens, speedups, len_ratios;
  int jps_success = 0, astar_success = 0, both_success = 0;

  for (const auto & r : results) {
    if (r.jps_ok) {
      ++jps_success;
      jps_times.push_back(r.jps_ms);
      jps_wps.push_back(static_cast<double>(r.jps_wp));
      jps_lens.push_back(r.jps_len);
    }
    if (r.astar_ok) {
      ++astar_success;
      astar_times.push_back(r.astar_ms);
      astar_wps.push_back(static_cast<double>(r.astar_wp));
      astar_lens.push_back(r.astar_len);
    }
    if (r.jps_ok && r.astar_ok) {
      ++both_success;
      if (r.jps_ms > 0.0) {speedups.push_back(r.astar_ms / r.jps_ms);}
      if (r.astar_len > 0.0) {len_ratios.push_back(r.jps_len / r.astar_len);}
    }
  }

  std::cout << "\n=== AGGREGATE RESULTS (n=" << results.size() << " cases) ===\n\n";
  std::cout << "Success rate:\n";
  std::cout << "  JPS3D: " << jps_success << "/" << results.size()
            << " (" << std::setprecision(1)
            << (100.0 * jps_success / results.size()) << "%)\n";
  std::cout << "  A*   : " << astar_success << "/" << results.size()
            << " (" << (100.0 * astar_success / results.size()) << "%)\n";
  std::cout << "  Both : " << both_success << "/" << results.size()
            << " (" << (100.0 * both_success / results.size()) << "%)\n\n";

  std::cout << std::setprecision(2);
  std::cout << "Planning time (ms):\n";
  printStats("JPS3D", jps_times);
  printStats("A*   ", astar_times);
  std::cout << "\nWaypoint count:\n";
  printStats("JPS3D", jps_wps);
  printStats("A*   ", astar_wps);
  std::cout << "\nPath length (m):\n";
  printStats("JPS3D", jps_lens);
  printStats("A*   ", astar_lens);
  std::cout << "\nJPS3D speedup over A* (per case, both succeeded):\n";
  printStats("ratio", speedups);
  std::cout << "\nJPS3D/A* path length ratio (1.0 = identical):\n";
  printStats("ratio", len_ratios);

  std::cout << "\n=== LATENCY SCALING VS DISTANCE (experiment g) ===\n";
  std::cout << "Distance bucket | n  | JPS3D mean ms | A* mean ms\n";
  std::cout << "----------------|----|---------------|------------\n";
  const double buckets[] = {2.0, 4.0, 6.0, 8.0, 10.0, 1e9};
  const char * labels[] = {"2-4 m ", "4-6 m ", "6-8 m ", "8-10 m", ">10 m "};
  for (int bi = 0; bi < 5; ++bi) {
    const double lo = buckets[bi], hi = buckets[bi + 1];
    double sj = 0.0, sa = 0.0;
    int nj = 0, na = 0;
    for (const auto & r : results) {
      if (r.distance_straight < lo || r.distance_straight >= hi) {continue;}
      if (r.jps_ok)   {sj += r.jps_ms;   ++nj;}
      if (r.astar_ok) {sa += r.astar_ms; ++na;}
    }
    std::cout << "  " << labels[bi] << "        |"
              << std::setw(3) << nj << " |"
              << std::setw(14) << (nj ? sj / nj : 0.0) << " |"
              << std::setw(11) << (na ? sa / na : 0.0) << "\n";
  }

  std::cout << "\nGeneration stats: " << generated << " cases from "
            << attempts << " sampling attempts\n";
  return 0;
}
