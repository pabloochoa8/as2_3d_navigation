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
 * @file jps3d_planner.hpp
 * @brief Thin wrapper around KumarRobotics/jps3d exposing the same interface
 *        as Astar3D so the two are drop-in interchangeable.
 *
 * API differences vs the task description assumption:
 *   - The 3D map type is JPS::VoxelMapUtil (= JPS::MapUtil<3>), NOT OccMapUtil.
 *   - Inflation is done via MapUtil::dilate(sphere_offsets) once after populating
 *     the map, not cell-by-cell during search.
 *   - JPS::MapUtil::floatToInt uses round(x - 0.5), so the grid origin must be
 *     set to bounds_min - 0.5*res to avoid off-by-one at boundary coordinates.
 *
 * @author Pablo Ochoa Izaguirre <p.ochoaizaguirre@alumnos.upm.es>
 */

#ifndef JPS3D_PLANNER_HPP_
#define JPS3D_PLANNER_HPP_

#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <as2_3d_map_interface/map_interface.hpp>

// Forward declarations to avoid pulling JPS3D headers into every translation unit.
template<int Dim> class JPSPlanner;
namespace JPS { template<int Dim> class MapUtil; }

/**
 * @brief Tuning knobs for Jps3DPlanner — top-level to avoid CWG nested-struct
 *        default-argument issue (same pattern as Astar3DParams).
 */
struct Jps3DParams
{
  float inflation_radius = 0.10f;  ///< spherical safety margin around obstacles [m]
  int   map_update_rate  = 1;      ///< rebuild collision map every N calls (0 = only once)
};

class Jps3DPlanner
{
public:
  using Params = Jps3DParams;

  explicit Jps3DPlanner(
    std::shared_ptr<as2_3d_map_interface::MapInterface> map,
    const Params & params = Params{});

  ~Jps3DPlanner();

  /**
   * @brief Plan a collision-free path from start to goal.
   * @return Waypoints in world coordinates (map frame). Empty on failure.
   */
  std::vector<Eigen::Vector3d> plan(
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & goal);

  enum class Result
  {
    SUCCESS, NO_PATH, START_OCCUPIED, GOAL_OCCUPIED, OUT_OF_BOUNDS
  };

  Result lastResult() const;

private:
  void buildMap();

  std::shared_ptr<as2_3d_map_interface::MapInterface> map_;
  Params params_;
  Result last_result_{Result::NO_PATH};

  std::shared_ptr<JPS::MapUtil<3>> map_util_;
  std::shared_ptr<JPSPlanner<3>>  planner_;

  double res_{0.05};
  int inflation_r_{2};
  int call_count_{0};
  bool map_built_{false};
};

#endif  // JPS3D_PLANNER_HPP_
