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
 * @file astar3d.hpp
 * @brief Pure-C++ 3D A* path planner with spherical inflation, 26-connectivity,
 *        and octile-distance heuristic. No ROS dependency.
 *
 * @author Pablo Ochoa Izaguirre <p.ochoaizaguirre@alumnos.upm.es>
 */

#ifndef ASTAR3D_HPP_
#define ASTAR3D_HPP_

#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <as2_3d_map_interface/map_interface.hpp>

/**
 * @brief Tuning knobs for Astar3D — defined at file scope to avoid CWG issues
 *        with nested struct default member initializers (fixed in C++17 but
 *        silently broken by some older compilers in nested contexts).
 */
struct Astar3DParams
{
  int   max_iterations   = 500000;
  float inflation_radius = 0.15f;   ///< spherical safety margin around obstacles [m]
  bool  allow_diagonal   = true;    ///< 26-connectivity when true, 6-connectivity otherwise
  float w_heuristic      = 1.0f;    ///< 1.0 = admissible A*; >1.0 = weighted (faster, suboptimal)
};

class Astar3D
{
public:
  using Params = Astar3DParams;

  explicit Astar3D(
    std::shared_ptr<as2_3d_map_interface::MapInterface> map,
    const Params & params = Params{});

  /**
   * @brief Compute a collision-free path from start to goal.
   * @return Sequence of waypoints in world coordinates (map frame), including
   *         start and goal. Empty if planning fails.
   */
  std::vector<Eigen::Vector3d> plan(
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & goal);

  enum class Result
  {
    SUCCESS,
    NO_PATH,
    START_OCCUPIED,
    GOAL_OCCUPIED,
    OUT_OF_BOUNDS,
    MAX_ITER_REACHED
  };

  Result lastResult() const;

private:
  // Returns true if the voxel at (ix,iy,iz) and all voxels within inflation_r_
  // are free (no obstacle and no out-of-bounds).
  bool isInflatedFree(int ix, int iy, int iz) const;

  // Octile 3D distance heuristic (consistent, admissible for 26-connectivity).
  float heuristic(int ix, int iy, int iz, int gx, int gy, int gz) const;

  // Unique integer key for voxel (ix, iy, iz).
  int64_t key(int ix, int iy, int iz) const;

  void worldToVoxel(double x, double y, double z, int & ix, int & iy, int & iz) const;
  Eigen::Vector3d voxelToWorld(int ix, int iy, int iz) const;

  std::shared_ptr<as2_3d_map_interface::MapInterface> map_;
  Params params_;
  Result last_result_{Result::NO_PATH};

  double res_;
  double x_min_, y_min_, z_min_;
  int nx_, ny_, nz_;
  int inflation_r_;   ///< inflation radius expressed in voxels
};

#endif  // ASTAR3D_HPP_
