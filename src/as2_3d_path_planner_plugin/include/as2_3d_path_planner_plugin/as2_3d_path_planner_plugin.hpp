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
 * @file as2_3d_path_planner_plugin.hpp
 * @brief Skeleton 3D path planner plugin for Aerostack2 PathPlannerBehavior.
 *
 * Inherits as2_behaviors_path_planning::PluginBase and provides stub
 * implementations of all pure-virtual methods. Real planning logic
 * (JPS3D / EGO-Planner / Minimum-Snap) is added in later phases.
 *
 * @author Pablo Ochoa Izaguirre <p.ochoaizaguirre@alumnos.upm.es>
 */

#ifndef AS2_3D_PATH_PLANNER_PLUGIN__AS2_3D_PATH_PLANNER_PLUGIN_HPP_
#define AS2_3D_PATH_PLANNER_PLUGIN__AS2_3D_PATH_PLANNER_PLUGIN_HPP_

#include <memory>
#include <vector>

#include <as2_behaviors_path_planning/path_planner_plugin_base.hpp>
#include <as2_3d_map_interface/map_interface.hpp>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "as2_msgs/action/navigate_to_point.hpp"

namespace as2_3d_path_planner
{

class Plugin : public as2_behaviors_path_planning::PluginBase
{
public:
  // ----- Lifecycle methods (pure-virtual in PluginBase) -----

  void initialize(
    as2::Node * node_ptr,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;

  bool on_activate(
    geometry_msgs::msg::PoseStamped drone_pose,
    as2_msgs::action::NavigateToPoint::Goal goal) override;

  bool on_deactivate() override;
  bool on_modify() override;
  bool on_pause() override;
  bool on_resume() override;
  void on_execution_end() override;
  as2_behavior::ExecutionStatus on_run() override;

  // ----- Map query helpers (plugin-internal, not part of PluginBase) -----
  // Will delegate to map_interface_ once the map plugin is integrated.

  bool is_occupied(const geometry_msgs::msg::PointStamped & point);
  bool is_path_traversable(const std::vector<geometry_msgs::msg::PointStamped> & path);

private:
  // Shared pointer to the map backend; assigned during initialize() once
  // as2_3d_map_server_plugin publishes its handle (Phase 2).
  std::shared_ptr<as2_3d_map_interface::MapInterface> map_interface_;
};

}  // namespace as2_3d_path_planner

#endif  // AS2_3D_PATH_PLANNER_PLUGIN__AS2_3D_PATH_PLANNER_PLUGIN_HPP_
