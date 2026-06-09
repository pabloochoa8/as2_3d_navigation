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
 * @file artificial_map.hpp
 * @brief Hardcoded 3D corridor map implementing MapInterface, used for planner
 *        verification before a real OctoMap backend is integrated.
 *
 * Geometry:
 *   Volume  : x=[0,10], y=[0,5], z=[0,3] m, resolution=0.05 m
 *   Obstacle: wall at x=[4.5,5.5], y=[0,2], z=[0,3]  (passage via y=[2,5])
 *   Frame   : "earth"
 *
 * @author Pablo Ochoa Izaguirre <p.ochoaizaguirre@alumnos.upm.es>
 */

#ifndef AS2_3D_MAP_SERVER_PLUGIN__ARTIFICIAL_MAP_HPP_
#define AS2_3D_MAP_SERVER_PLUGIN__ARTIFICIAL_MAP_HPP_

#include <string>

#include <as2_3d_map_interface/map_interface.hpp>

namespace as2_3d_map_server
{

class ArtificialMap : public as2_3d_map_interface::MapInterface
{
public:
  ArtificialMap() = default;
  ~ArtificialMap() override = default;

  as2_3d_map_interface::VoxelState getVoxelState(double x, double y, double z) const override
  {
    if (!isInBounds(x, y, z)) {
      return as2_3d_map_interface::VoxelState::OUT_OF_BOUNDS;
    }
    if (x >= 4.5 && x <= 5.5 && y >= 0.0 && y <= 2.0 && z >= 0.0 && z <= 3.0) {
      return as2_3d_map_interface::VoxelState::OCCUPIED;
    }
    return as2_3d_map_interface::VoxelState::FREE;
  }

  bool isInBounds(double x, double y, double z) const override
  {
    return x >= 0.0 && x <= 10.0 &&
           y >= 0.0 && y <= 5.0 &&
           z >= 0.0 && z <= 3.0;
  }

  as2_3d_map_interface::MapBounds getBounds() const override
  {
    as2_3d_map_interface::MapBounds b;
    b.x_min = 0.0;  b.x_max = 10.0;
    b.y_min = 0.0;  b.y_max = 5.0;
    b.z_min = 0.0;  b.z_max = 3.0;
    return b;
  }

  std::string getFrameId() const override {return "earth";}

  double getResolution() const override {return 0.05;}
};

}  // namespace as2_3d_map_server

#endif  // AS2_3D_MAP_SERVER_PLUGIN__ARTIFICIAL_MAP_HPP_
