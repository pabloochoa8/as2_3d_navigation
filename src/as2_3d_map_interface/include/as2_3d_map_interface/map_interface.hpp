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
 * @file map_interface.hpp
 * @brief Abstract interface between the 3D volumetric map and the path planner.
 *
 * Implementations are provided by as2_3d_map_server_plugin. Consumers (planners,
 * safety checkers) depend only on this header — never on the concrete backend.
 *
 * @author Pablo Ochoa Izaguirre <p.ochoaizaguirre@alumnos.upm.es>
 */

#ifndef AS2_3D_MAP_INTERFACE__MAP_INTERFACE_HPP_
#define AS2_3D_MAP_INTERFACE__MAP_INTERFACE_HPP_

#include <cstdint>
#include <string>

namespace as2_3d_map_interface
{

/**
 * @brief Occupancy classification of a single voxel.
 *
 * UNKNOWN and OUT_OF_BOUNDS are both treated as non-navigable by isOccupied()
 * to guarantee conservative behaviour: if we don't know, we don't go.
 */
enum class VoxelState : uint8_t
{
  FREE = 0,         ///< Voxel observed as free
  OCCUPIED = 1,     ///< Voxel observed as occupied
  UNKNOWN = 2,      ///< Voxel not yet observed
  OUT_OF_BOUNDS = 3 ///< Query point lies outside the map volume
};

/**
 * @brief Axis-aligned bounding box of the navigable map volume (metres, map frame).
 */
struct MapBounds
{
  double x_min{0.0};
  double x_max{0.0};
  double y_min{0.0};
  double y_max{0.0};
  double z_min{0.0};
  double z_max{0.0};
};

/**
 * @brief Abstract query interface for a 3D volumetric map.
 *
 * Concrete implementations (OctoMap, local voxel grid, …) inherit this class
 * and are loaded at runtime via pluginlib by as2_3d_map_server_plugin.
 * All coordinates are in metres in the frame returned by getFrameId().
 */
class MapInterface
{
public:
  virtual ~MapInterface() = default;

  // ---------------------------------------------------------------------------
  // Pure-virtual primitives — must be implemented by every backend
  // ---------------------------------------------------------------------------

  /**
   * @brief Return the occupancy state of the voxel that contains (x, y, z).
   * @param x X coordinate in the map frame [m]
   * @param y Y coordinate in the map frame [m]
   * @param z Z coordinate in the map frame [m]
   * @return VoxelState classification for that voxel
   */
  virtual VoxelState getVoxelState(double x, double y, double z) const = 0;

  /**
   * @brief Check whether (x, y, z) lies within the map volume.
   * @return true if the point is inside the map AABB
   */
  virtual bool isInBounds(double x, double y, double z) const = 0;

  /**
   * @brief Return the axis-aligned bounding box of the map in the map frame.
   */
  virtual MapBounds getBounds() const = 0;

  /**
   * @brief TF frame in which all coordinates are expressed (e.g. "earth").
   */
  virtual std::string getFrameId() const = 0;

  /**
   * @brief Nominal voxel edge length [m].
   */
  virtual double getResolution() const = 0;

  // ---------------------------------------------------------------------------
  // Convenience wrappers — implemented here, not overridable
  // ---------------------------------------------------------------------------

  /**
   * @brief Return true if the planner must treat this cell as an obstacle.
   *
   * Conservative: UNKNOWN and OUT_OF_BOUNDS both block navigation.
   * Only FREE is passable.
   */
  bool isOccupied(double x, double y, double z) const
  {
    return getVoxelState(x, y, z) != VoxelState::FREE;
  }

  /**
   * @brief Return true if the voxel has never been observed.
   *
   * Planners can use this to distinguish "blocked by obstacle" from
   * "blocked by lack of information" when building exploration strategies.
   */
  bool isUnknown(double x, double y, double z) const
  {
    return getVoxelState(x, y, z) == VoxelState::UNKNOWN;
  }
};

}  // namespace as2_3d_map_interface

#endif  // AS2_3D_MAP_INTERFACE__MAP_INTERFACE_HPP_
