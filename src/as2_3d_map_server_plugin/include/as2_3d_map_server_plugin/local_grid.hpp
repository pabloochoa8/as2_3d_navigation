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
 * @file local_grid.hpp
 * @brief Dense 3D voxel grid centered on the drone for reactive obstacle
 *        avoidance. Slides with the drone and is cleared on every center update.
 *
 * Default window: 4×4×2.5 m at 3 cm resolution (~1.47 M cells, ~183 KB packed).
 *
 * @author Pablo Ochoa Izaguirre <p.ochoaizaguirre@alumnos.upm.es>
 */

#ifndef AS2_3D_MAP_SERVER_PLUGIN__LOCAL_GRID_HPP_
#define AS2_3D_MAP_SERVER_PLUGIN__LOCAL_GRID_HPP_

#include <mutex>
#include <optional>
#include <vector>

#include <Eigen/Core>

namespace as2_3d_map_server
{

class LocalGrid
{
public:
  /**
   * @param size_x  Window width  along X [m]
   * @param size_y  Window depth  along Y [m]
   * @param size_z  Window height along Z [m]
   * @param resolution  Voxel edge length [m]
   */
  LocalGrid(double size_x, double size_y, double size_z, double resolution);

  /** Move the window to a new drone position and clear all occupancy data. */
  void updateCenter(double cx, double cy, double cz);

  /** Mark voxels occupied by the given points (already in map/earth frame). */
  void insertPoints(const std::vector<Eigen::Vector3d> & points);

  /**
   * @brief Query occupancy of a single point.
   * @return true/false if the point is inside the window; nullopt otherwise.
   */
  std::optional<bool> isOccupied(double x, double y, double z) const;

  /** Returns true if the point falls inside the current window. */
  bool isInWindow(double x, double y, double z) const;

  /** Reset every cell to unoccupied. */
  void clear();

  double getCenterX() const;
  double getCenterY() const;
  double getCenterZ() const;

private:
  const double size_x_, size_y_, size_z_;
  const double resolution_;
  const int nx_, ny_, nz_;

  double cx_{0.0}, cy_{0.0}, cz_{0.0};

  // Packed bit vector: true = occupied
  std::vector<bool> grid_;
  mutable std::mutex mutex_;

  /** World coordinate → cell index. Returns false if out of window. */
  bool toIndex(double x, double y, double z,
               int & ix, int & iy, int & iz) const;

  int flatIndex(int ix, int iy, int iz) const;
};

}  // namespace as2_3d_map_server

#endif  // AS2_3D_MAP_SERVER_PLUGIN__LOCAL_GRID_HPP_
