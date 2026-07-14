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
 * @file octomap_planner_map.hpp
 * @brief Lightweight MapInterface wrapping an octomap::OcTree loaded from a
 *        binary .bt file. Designed for the A* global path planner — no ROS,
 *        no write operations after initialisation.
 *
 * Design decisions:
 *  - UNKNOWN voxels (unmapped space) are exposed as UNKNOWN, NOT as occupied.
 *    The A* planner treats UNKNOWN as FREE so it can navigate through areas
 *    that were not observed during the mapping scan (optimistic planning).
 *  - isInBounds uses the OctoMap extent plus a configurable padding margin so
 *    the drone can plan to positions slightly outside the observed area without
 *    hitting an immediate OUT_OF_BOUNDS rejection.
 *  - OUT_OF_BOUNDS is returned only for positions beyond the padded bounds —
 *    the A* treats these as hard walls (arena boundary).
 *
 * @author Pablo Ochoa Izaguirre <p.ochoaizaguirre@alumnos.upm.es>
 */

#ifndef AS2_3D_PATH_PLANNER_PLUGIN__OCTOMAP_PLANNER_MAP_HPP_
#define AS2_3D_PATH_PLANNER_PLUGIN__OCTOMAP_PLANNER_MAP_HPP_

#include <memory>
#include <string>

#include <octomap/OcTree.h>
#include <as2_3d_map_interface/map_interface.hpp>

namespace as2_3d_path_planner
{

class OctomapPlannerMap : public as2_3d_map_interface::MapInterface
{
public:
  // Padding added to the OcToMap metric bounds on each side [m].
  static constexpr double kBoundsPadding = 1.0;

  // Voxels at z < kFloorClearance are returned as UNKNOWN, not OCCUPIED.
  // This prevents the mapped floor plane (z≈0) from blocking the inflation
  // sphere of a drone flying close to the ground.
  static constexpr double kFloorClearance = 0.15;

  OctomapPlannerMap()
  : octree_(std::make_unique<octomap::OcTree>(0.05))
  {}

  /**
   * @brief Load a binary OctoMap (.bt) file.
   * @return true if successful and tree contains at least one leaf node.
   */
  bool loadFromFile(const std::string & filepath)
  {
    if (!octree_->readBinary(filepath)) {
      return false;
    }
    if (octree_->getNumLeafNodes() == 0) {
      return false;
    }
    // Cache OcToMap bounds (non-const operation — do once here, not in const queries)
    double xmin, ymin, zmin, xmax, ymax, zmax;
    octree_->getMetricMin(xmin, ymin, zmin);
    octree_->getMetricMax(xmax, ymax, zmax);

    octomap_bounds_.x_min = xmin;
    octomap_bounds_.x_max = xmax;
    octomap_bounds_.y_min = ymin;
    octomap_bounds_.y_max = ymax;
    octomap_bounds_.z_min = zmin;
    octomap_bounds_.z_max = zmax;

    // Padded bounds: the A* planning domain is larger than the observed area
    padded_bounds_.x_min = xmin - kBoundsPadding;
    padded_bounds_.x_max = xmax + kBoundsPadding;
    padded_bounds_.y_min = ymin - kBoundsPadding;
    padded_bounds_.y_max = ymax + kBoundsPadding;
    padded_bounds_.z_min = 0.0;                    // floor is always at 0
    padded_bounds_.z_max = zmax + kBoundsPadding;

    loaded_ = true;
    return true;
  }

  size_t getNodeCount() const
  {
    return octree_ ? octree_->getNumLeafNodes() : 0u;
  }

  // ---------------------------------------------------------------------------
  // MapInterface
  // ---------------------------------------------------------------------------

  /**
   * @brief Returns the voxel state for a world point (x, y, z).
   *
   * OUT_OF_BOUNDS  — beyond padded arena bounds (hard wall for A*)
   * UNKNOWN        — within padded bounds but not observed in the OcToMap scan
   * OCCUPIED / FREE — as stored in the OcToMap tree
   */
  as2_3d_map_interface::VoxelState getVoxelState(
    double x, double y, double z) const override
  {
    if (!loaded_) {
      return as2_3d_map_interface::VoxelState::UNKNOWN;
    }
    if (!isInPaddedBounds(x, y, z)) {
      return as2_3d_map_interface::VoxelState::OUT_OF_BOUNDS;
    }
    if (!isInOctomapBounds(x, y, z)) {
      return as2_3d_map_interface::VoxelState::UNKNOWN;
    }
    // Floor plane voxels block the inflation sphere for near-ground starts.
    // Treat them as UNKNOWN so the drone can plan at any altitude >= padded z_min.
    if (z < kFloorClearance) {
      return as2_3d_map_interface::VoxelState::UNKNOWN;
    }
    const octomap::OcTreeNode * node = octree_->search(x, y, z);
    if (!node) {
      return as2_3d_map_interface::VoxelState::UNKNOWN;
    }
    return node->getOccupancy() > 0.5
      ? as2_3d_map_interface::VoxelState::OCCUPIED
      : as2_3d_map_interface::VoxelState::FREE;
  }

  /**
   * @brief Returns true if point is within the padded planning domain.
   * Positions outside the padded domain are treated as hard walls.
   */
  bool isInBounds(double x, double y, double z) const override
  {
    return loaded_ && isInPaddedBounds(x, y, z);
  }

  /**
   * @brief Returns the padded bounding box used by the A* grid.
   */
  as2_3d_map_interface::MapBounds getBounds() const override
  {
    return padded_bounds_;
  }

  std::string getFrameId() const override {return "earth";}

  double getResolution() const override
  {
    return octree_ ? octree_->getResolution() : 0.05;
  }

private:
  bool isInPaddedBounds(double x, double y, double z) const
  {
    return x >= padded_bounds_.x_min && x <= padded_bounds_.x_max &&
           y >= padded_bounds_.y_min && y <= padded_bounds_.y_max &&
           z >= padded_bounds_.z_min && z <= padded_bounds_.z_max;
  }

  bool isInOctomapBounds(double x, double y, double z) const
  {
    return x >= octomap_bounds_.x_min && x <= octomap_bounds_.x_max &&
           y >= octomap_bounds_.y_min && y <= octomap_bounds_.y_max &&
           z >= octomap_bounds_.z_min && z <= octomap_bounds_.z_max;
  }

  std::unique_ptr<octomap::OcTree> octree_;
  as2_3d_map_interface::MapBounds octomap_bounds_{};
  as2_3d_map_interface::MapBounds padded_bounds_{};
  bool loaded_{false};
};

}  // namespace as2_3d_path_planner

#endif  // AS2_3D_PATH_PLANNER_PLUGIN__OCTOMAP_PLANNER_MAP_HPP_
