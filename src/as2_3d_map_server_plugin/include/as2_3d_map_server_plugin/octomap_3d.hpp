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
 * @file octomap_3d.hpp
 * @brief OctoMap-backed implementation of MapInterface.
 *        Inserts point clouds (already in earth frame) and answers occupancy
 *        queries via the standard VoxelState API.
 *
 * @author Pablo Ochoa Izaguirre <p.ochoaizaguirre@alumnos.upm.es>
 */

#ifndef AS2_3D_MAP_SERVER_PLUGIN__OCTOMAP_3D_HPP_
#define AS2_3D_MAP_SERVER_PLUGIN__OCTOMAP_3D_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <octomap/OcTree.h>

#include <as2_3d_map_interface/map_interface.hpp>

namespace as2_3d_map_server
{

class OctomapMap : public as2_3d_map_interface::MapInterface
{
public:
  explicit OctomapMap(double resolution = 0.05);
  ~OctomapMap() override = default;

  // ---------- MapInterface ----------

  as2_3d_map_interface::VoxelState getVoxelState(double x, double y, double z) const override;
  bool isInBounds(double x, double y, double z) const override;
  as2_3d_map_interface::MapBounds getBounds() const override;
  std::string getFrameId() const override;
  double getResolution() const override;

  // ---------- Update / persistence ----------

  /**
   * @brief Insert a point cloud into the octree.
   * @param points  Endpoints already transformed to the map frame (earth).
   * @param sensor_origin  Sensor position in the map frame (earth).
   */
  void insertPointCloud(
    const std::vector<Eigen::Vector3d> & points,
    const Eigen::Vector3d & sensor_origin);

  /** Load a binary OctoMap (.bt) file. Returns true on success. */
  bool loadFromFile(const std::string & filepath);

  /** Write the current tree to a binary OctoMap (.bt) file. Returns true on success. */
  bool saveToFile(const std::string & filepath) const;

  /** Number of leaf nodes currently in the tree. */
  size_t getNodeCount() const;

  /** Raw pointer to the internal octree — used for serialisation only. */
  octomap::OcTree * getOcTree() const
  {
    return octree_.get();
  }

private:
  std::shared_ptr<octomap::OcTree> octree_;
  double resolution_;
  std::string frame_id_{"earth"};
  mutable std::mutex mutex_;
};

}  // namespace as2_3d_map_server

#endif  // AS2_3D_MAP_SERVER_PLUGIN__OCTOMAP_3D_HPP_
