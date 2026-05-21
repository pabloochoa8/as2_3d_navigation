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

#include "as2_3d_path_planner_plugin/as2_3d_path_planner_plugin.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace as2_3d_path_planner
{

void Plugin::initialize(
  as2::Node * node_ptr,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  node_ptr_ = node_ptr;
  tf_buffer_ = tf_buffer;
  RCLCPP_INFO(node_ptr_->get_logger(), "as2_3d_path_planner_plugin initialized");
}

bool Plugin::on_activate(
  geometry_msgs::msg::PoseStamped drone_pose,
  as2_msgs::action::NavigateToPoint::Goal goal)
{
  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "on_activate: goal (%.2f, %.2f, %.2f) from drone (%.2f, %.2f, %.2f)",
    goal.point.point.x, goal.point.point.y, goal.point.point.z,
    drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z);

  // Stub path: direct line to goal (no collision checking yet).
  path_.clear();
  geometry_msgs::msg::Point goal_point;
  goal_point.x = goal.point.point.x;
  goal_point.y = goal.point.point.y;
  goal_point.z = goal.point.point.z;
  path_.push_back(goal_point);

  return true;
}

bool Plugin::on_deactivate()
{
  return true;
}

bool Plugin::on_modify()
{
  return true;
}

bool Plugin::on_pause()
{
  return true;
}

bool Plugin::on_resume()
{
  return true;
}

void Plugin::on_execution_end()
{
}

as2_behavior::ExecutionStatus Plugin::on_run()
{
  return as2_behavior::ExecutionStatus::SUCCESS;
}

bool Plugin::is_occupied(const geometry_msgs::msg::PointStamped & /*point*/)
{
  // Stub: assume free until map_interface_ is connected in Phase 2.
  return false;
}

bool Plugin::is_path_traversable(
  const std::vector<geometry_msgs::msg::PointStamped> & /*path*/)
{
  // Stub: assume traversable until map_interface_ is connected in Phase 2.
  return true;
}

}  // namespace as2_3d_path_planner

PLUGINLIB_EXPORT_CLASS(as2_3d_path_planner::Plugin, as2_behaviors_path_planning::PluginBase)
