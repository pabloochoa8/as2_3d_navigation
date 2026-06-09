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

#include "as2_3d_map_server_plugin/octomap_3d.hpp"

#include <octomap/Pointcloud.h>

namespace as2_3d_map_server
{

OctomapMap::OctomapMap(double resolution)
: resolution_(resolution)
{
  octree_ = std::make_shared<octomap::OcTree>(resolution_);
}

// ---------------------------------------------------------------------------
// MapInterface
// ---------------------------------------------------------------------------

as2_3d_map_interface::VoxelState OctomapMap::getVoxelState(
  double x, double y, double z) const
{
  if (!isInBounds(x, y, z)) {
    return as2_3d_map_interface::VoxelState::OUT_OF_BOUNDS;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const octomap::OcTreeNode * node = octree_->search(x, y, z);

  if (node == nullptr) {
    return as2_3d_map_interface::VoxelState::UNKNOWN;
  }

  return node->getOccupancy() > 0.5
    ? as2_3d_map_interface::VoxelState::OCCUPIED
    : as2_3d_map_interface::VoxelState::FREE;
}

bool OctomapMap::isInBounds(double x, double y, double z) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (octree_->getNumLeafNodes() == 0) {
    return false;
  }

  double xmin, ymin, zmin, xmax, ymax, zmax;
  octree_->getMetricMin(xmin, ymin, zmin);
  octree_->getMetricMax(xmax, ymax, zmax);

  return x >= xmin && x <= xmax &&
         y >= ymin && y <= ymax &&
         z >= zmin && z <= zmax;
}

as2_3d_map_interface::MapBounds OctomapMap::getBounds() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  as2_3d_map_interface::MapBounds b;
  if (octree_->getNumLeafNodes() == 0) {
    return b;  // zero-initialised
  }

  octree_->getMetricMin(b.x_min, b.y_min, b.z_min);
  octree_->getMetricMax(b.x_max, b.y_max, b.z_max);
  return b;
}

std::string OctomapMap::getFrameId() const {return frame_id_;}

double OctomapMap::getResolution() const {return resolution_;}

// ---------------------------------------------------------------------------
// Update / persistence
// ---------------------------------------------------------------------------

void OctomapMap::insertPointCloud(
  const std::vector<Eigen::Vector3d> & points,
  const Eigen::Vector3d & sensor_origin)
{
  octomap::Pointcloud pc;
  pc.reserve(points.size());

  for (const auto & p : points) {
    pc.push_back(octomap::point3d(
        static_cast<float>(p.x()),
        static_cast<float>(p.y()),
        static_cast<float>(p.z())));
  }

  const octomap::point3d origin(
    static_cast<float>(sensor_origin.x()),
    static_cast<float>(sensor_origin.y()),
    static_cast<float>(sensor_origin.z()));

  std::lock_guard<std::mutex> lock(mutex_);
  // Scan is already in global (earth) frame — use the global-frame overload
  octree_->insertPointCloud(pc, origin, -1.0, false, false);
  octree_->updateInnerOccupancy();
}

bool OctomapMap::loadFromFile(const std::string & filepath)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return octree_->readBinary(filepath);
}

bool OctomapMap::saveToFile(const std::string & filepath) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return octree_->writeBinary(filepath);
}

size_t OctomapMap::getNodeCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return octree_->getNumLeafNodes();
}

}  // namespace as2_3d_map_server
