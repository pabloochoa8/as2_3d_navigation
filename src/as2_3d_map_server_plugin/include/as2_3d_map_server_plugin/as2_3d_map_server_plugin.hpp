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
 * @file as2_3d_map_server_plugin.hpp
 * @brief Map server plugin: subscribes to a PointCloud2, transforms points to
 *        the map frame via TF, feeds an OctomapMap, and exposes occupancy
 *        queries through the as2_behaviors_path_planning::PluginBase interface.
 *
 * @author Pablo Ochoa Izaguirre <p.ochoaizaguirre@alumnos.upm.es>
 */

#ifndef AS2_3D_MAP_SERVER_PLUGIN__AS2_3D_MAP_SERVER_PLUGIN_HPP_
#define AS2_3D_MAP_SERVER_PLUGIN__AS2_3D_MAP_SERVER_PLUGIN_HPP_

#include <memory>
#include <string>

#include <as2_behaviors_path_planning/path_planner_plugin_base.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <std_srvs/srv/trigger.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

#include "as2_3d_map_server_plugin/octomap_3d.hpp"
#include "as2_3d_map_server_plugin/local_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "as2_msgs/action/navigate_to_point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace as2_3d_map_server
{

class Plugin : public as2_behaviors_path_planning::PluginBase
{
public:
  // ----- PluginBase lifecycle -----
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

  // ----- Map query (public for path planner integration) -----
  bool is_occupied(const geometry_msgs::msg::PointStamped & point);

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  std::shared_ptr<OctomapMap> octomap_map_;
  std::shared_ptr<LocalGrid> local_grid_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  // tf_buffer_ is inherited from PluginBase; we replace it with a fresh one
  // that has its own listener so the map server controls its own TF context.
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string pointcloud_topic_;
  std::string pose_topic_;
  std::string map_frame_;
  std::string map_save_path_;
  double resolution_{0.05};

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::TimerBase::SharedPtr octomap_pub_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace as2_3d_map_server

#endif  // AS2_3D_MAP_SERVER_PLUGIN__AS2_3D_MAP_SERVER_PLUGIN_HPP_
