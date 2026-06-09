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

#include "as2_3d_map_server_plugin/local_grid.hpp"

#include <cmath>

namespace as2_3d_map_server
{

LocalGrid::LocalGrid(
  double size_x, double size_y, double size_z, double resolution)
: size_x_(size_x), size_y_(size_y), size_z_(size_z),
  resolution_(resolution),
  nx_(static_cast<int>(std::round(size_x / resolution))),
  ny_(static_cast<int>(std::round(size_y / resolution))),
  nz_(static_cast<int>(std::round(size_z / resolution)))
{
  grid_.assign(static_cast<size_t>(nx_ * ny_ * nz_), false);
}

// ---------------------------------------------------------------------------

void LocalGrid::updateCenter(double cx, double cy, double cz)
{
  std::lock_guard<std::mutex> lock(mutex_);
  cx_ = cx;
  cy_ = cy;
  cz_ = cz;
  // Slide the window: previous occupancy is no longer valid
  std::fill(grid_.begin(), grid_.end(), false);
}

void LocalGrid::insertPoints(const std::vector<Eigen::Vector3d> & points)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & p : points) {
    int ix, iy, iz;
    if (toIndex(p.x(), p.y(), p.z(), ix, iy, iz)) {
      grid_[static_cast<size_t>(flatIndex(ix, iy, iz))] = true;
    }
  }
}

std::optional<bool> LocalGrid::isOccupied(double x, double y, double z) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  int ix, iy, iz;
  if (!toIndex(x, y, z, ix, iy, iz)) {
    return std::nullopt;
  }
  return grid_[static_cast<size_t>(flatIndex(ix, iy, iz))];
}

bool LocalGrid::isInWindow(double x, double y, double z) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return std::abs(x - cx_) < size_x_ * 0.5 &&
         std::abs(y - cy_) < size_y_ * 0.5 &&
         std::abs(z - cz_) < size_z_ * 0.5;
}

void LocalGrid::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::fill(grid_.begin(), grid_.end(), false);
}

double LocalGrid::getCenterX() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return cx_;
}

double LocalGrid::getCenterY() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return cy_;
}

double LocalGrid::getCenterZ() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return cz_;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

bool LocalGrid::toIndex(
  double x, double y, double z,
  int & ix, int & iy, int & iz) const
{
  // Origin of the grid (lower-left-bottom corner in map frame)
  const double ox = cx_ - size_x_ * 0.5;
  const double oy = cy_ - size_y_ * 0.5;
  const double oz = cz_ - size_z_ * 0.5;

  ix = static_cast<int>(std::floor((x - ox) / resolution_));
  iy = static_cast<int>(std::floor((y - oy) / resolution_));
  iz = static_cast<int>(std::floor((z - oz) / resolution_));

  return ix >= 0 && ix < nx_ &&
         iy >= 0 && iy < ny_ &&
         iz >= 0 && iz < nz_;
}

int LocalGrid::flatIndex(int ix, int iy, int iz) const
{
  return ix + nx_ * (iy + ny_ * iz);
}

}  // namespace as2_3d_map_server
