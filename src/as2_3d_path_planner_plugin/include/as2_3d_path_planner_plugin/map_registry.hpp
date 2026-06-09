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
 * @file map_registry.hpp
 * @brief Process-local singleton that couples the map server plugin with the
 *        path planner plugin when both are loaded in the same process.
 *
 * Usage:
 *   Map server (on initialize):
 *     MapRegistry::instance().registerMap(my_map_ptr);
 *
 *   Path planner (on_activate):
 *     auto map = MapRegistry::instance().getMap();
 *     if (!map) { ... handle no-map case ... }
 *
 * NOTE: only works within a single OS process. If the map server and the
 * path planner run in separate processes (separate ROS 2 nodes launched
 * independently), use a different IPC mechanism.
 *
 * @author Pablo Ochoa Izaguirre <p.ochoaizaguirre@alumnos.upm.es>
 */

#ifndef AS2_3D_PATH_PLANNER_PLUGIN__MAP_REGISTRY_HPP_
#define AS2_3D_PATH_PLANNER_PLUGIN__MAP_REGISTRY_HPP_

#include <memory>
#include <as2_3d_map_interface/map_interface.hpp>

namespace as2_3d_path_planner
{

class MapRegistry
{
public:
  static MapRegistry & instance()
  {
    static MapRegistry inst;
    return inst;
  }

  void registerMap(std::shared_ptr<as2_3d_map_interface::MapInterface> map)
  {
    map_ = std::move(map);
  }

  std::shared_ptr<as2_3d_map_interface::MapInterface> getMap() const
  {
    return map_;
  }

private:
  MapRegistry() = default;
  std::shared_ptr<as2_3d_map_interface::MapInterface> map_;
};

}  // namespace as2_3d_path_planner

#endif  // AS2_3D_PATH_PLANNER_PLUGIN__MAP_REGISTRY_HPP_
