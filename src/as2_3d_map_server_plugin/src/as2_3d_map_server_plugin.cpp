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

#include "as2_3d_map_server_plugin/as2_3d_map_server_plugin.hpp"
#include "as2_3d_map_server_plugin/octomap_3d.hpp"

#include <cmath>
#include <vector>

#include <Eigen/Geometry>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/time.h>
#include <tf2_eigen/tf2_eigen/tf2_eigen.hpp>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace as2_3d_map_server
{

// ---------------------------------------------------------------------------
// initialize
// ---------------------------------------------------------------------------

void Plugin::initialize(
  as2::Node * node_ptr,
  std::shared_ptr<tf2_ros::Buffer> /*tf_buffer*/)
{
  node_ptr_ = node_ptr;

  // Parameters
  node_ptr_->declare_parameter(
    "pointcloud_topic", "/drone0/sensor_measurements/front_camera/points");
  node_ptr_->declare_parameter("map_frame", "earth");
  node_ptr_->declare_parameter("resolution", 0.05);
  node_ptr_->declare_parameter("map_file", "");

  pointcloud_topic_ =
    node_ptr_->get_parameter("pointcloud_topic").as_string();
  map_frame_ = node_ptr_->get_parameter("map_frame").as_string();
  resolution_ = node_ptr_->get_parameter("resolution").as_double();
  const std::string map_file =
    node_ptr_->get_parameter("map_file").as_string();

  // Create OctoMap backend
  octomap_map_ = std::make_shared<OctomapMap>(resolution_);

  // Local reactive layer (4×4×2.5 m window, 3 cm resolution)
  local_grid_ = std::make_shared<LocalGrid>(4.0, 4.0, 2.5, 0.03);

  // Pose topic for sliding the local window
  node_ptr_->declare_parameter(
    "pose_topic", "/drone0/self_localization/pose");
  pose_topic_ = node_ptr_->get_parameter("pose_topic").as_string();

  pose_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, rclcpp::QoS(10).best_effort(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      local_grid_->updateCenter(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z);
    });

  if (!map_file.empty()) {
    if (octomap_map_->loadFromFile(map_file)) {
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "Loaded map from %s (%zu nodes)", map_file.c_str(),
        octomap_map_->getNodeCount());
    } else {
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "Could not load map from %s — starting empty", map_file.c_str());
    }
  }

  // Own TF buffer + listener (independent of the one passed by PluginBase)
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_ptr_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // PointCloud2 subscription
  pc_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_,
    rclcpp::QoS(10).reliable(),
    std::bind(&Plugin::pointcloud_callback, this, std::placeholders::_1));

  // Save-map service
  node_ptr_->declare_parameter("map_save_path", "/tmp/arena_map.bt");
  map_save_path_ = node_ptr_->get_parameter("map_save_path").as_string();

  save_map_srv_ = node_ptr_->create_service<std_srvs::srv::Trigger>(
    "~/save_map",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response)
    {
      bool success = octomap_map_->saveToFile(map_save_path_);
      response->success = success;
      response->message = success ?
        "Map saved to " + map_save_path_ :
        "Failed to save map to " + map_save_path_;
      RCLCPP_INFO(node_ptr_->get_logger(),
        "[map_server] %s", response->message.c_str());
    });

  RCLCPP_INFO(node_ptr_->get_logger(),
    "[map_server] Save map service available at ~/save_map");

  // Timer de diagnóstico — imprime estado del octree cada 5s
  diagnostic_timer_ = node_ptr_->create_wall_timer(
    std::chrono::seconds(5),
    [this]() {
      size_t nodes = octomap_map_->getNodeCount();
      if (nodes == 0) {
        RCLCPP_INFO(node_ptr_->get_logger(),
          "[map_server] Octree empty — waiting for pointcloud data");
        return;
      }
      auto bounds = octomap_map_->getBounds();
      RCLCPP_INFO(node_ptr_->get_logger(),
        "[map_server] OctoMap: %zu nodes | bounds: x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f]",
        nodes,
        bounds.x_min, bounds.x_max,
        bounds.y_min, bounds.y_max,
        bounds.z_min, bounds.z_max);
      RCLCPP_INFO(node_ptr_->get_logger(),
        "[map_server] LocalGrid: window 4x4x2.5m @ (%.2f, %.2f, %.2f)",
        local_grid_->getCenterX(),
        local_grid_->getCenterY(),
        local_grid_->getCenterZ());
    });

  // OctoMap publisher for RViz (latched, 2 s period)
  octomap_pub_ = node_ptr_->create_publisher<octomap_msgs::msg::Octomap>(
    "~/octomap_full", rclcpp::QoS(1).reliable().transient_local());

  octomap_pub_timer_ = node_ptr_->create_wall_timer(
    std::chrono::seconds(2),
    [this]() {
      if (octomap_map_->getNodeCount() == 0) {return;}

      octomap_msgs::msg::Octomap msg;
      msg.header.frame_id = octomap_map_->getFrameId();
      msg.header.stamp = node_ptr_->get_clock()->now();

      octomap::OcTree * tree = octomap_map_->getOcTree();
      if (tree && octomap_msgs::fullMapToMsg(*tree, msg)) {
        octomap_pub_->publish(msg);
      }
    });

  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "as2_3d_map_server_plugin initialized | topic: %s | frame: %s | res: %.3f m",
    pointcloud_topic_.c_str(), map_frame_.c_str(), resolution_);
}

// ---------------------------------------------------------------------------
// pointcloud_callback
// ---------------------------------------------------------------------------

void Plugin::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(),
    *node_ptr_->get_clock(), 2000,
    "[map_server] pointcloud_callback called | frame: %s | stamp: %d.%d",
    msg->header.frame_id.c_str(),
    msg->header.stamp.sec,
    msg->header.stamp.nanosec);

  // 1. Look up transform: map_frame ← cloud frame
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(
      map_frame_,
      msg->header.frame_id,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      node_ptr_->get_logger(),
      *node_ptr_->get_clock(), 2000,
      "TF not available: %s", e.what());
    return;
  }

  // 2. Sensor origin in map frame
  RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(),
    *node_ptr_->get_clock(), 2000,
    "[map_server] TF OK | origin: (%.2f, %.2f, %.2f)",
    tf_stamped.transform.translation.x,
    tf_stamped.transform.translation.y,
    tf_stamped.transform.translation.z);

  const Eigen::Vector3d sensor_origin(
    tf_stamped.transform.translation.x,
    tf_stamped.transform.translation.y,
    tf_stamped.transform.translation.z);

  // 3. Build rotation matrix from the TF quaternion
  const auto & q = tf_stamped.transform.rotation;
  const Eigen::Quaterniond rot(q.w, q.x, q.y, q.z);
  const Eigen::Matrix3d R = rot.toRotationMatrix();

  // 4. Iterate PointCloud2, transform each point, apply filters
  std::vector<Eigen::Vector3d> points;
  points.reserve(msg->width * msg->height);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const float lx = *iter_x, ly = *iter_y, lz = *iter_z;

    // Discard NaN / Inf
    if (!std::isfinite(lx) || !std::isfinite(ly) || !std::isfinite(lz)) {
      continue;
    }

    // Distance filter (> 5 m from sensor origin in local frame)
    const float dist2 = lx * lx + ly * ly + lz * lz;
    if (dist2 > 25.0f) {
      continue;
    }

    // Transform to map frame
    const Eigen::Vector3d p_earth =
      R * Eigen::Vector3d(lx, ly, lz) + sensor_origin;

    // Discard points below ground
    if (p_earth.z() < 0.0) {
      continue;
    }

    points.push_back(p_earth);
  }

  if (points.empty()) {
    return;
  }

  // 5. Insert into OctoMap global layer and local reactive layer
  octomap_map_->insertPointCloud(points, sensor_origin);
  local_grid_->insertPoints(points);
}

// ---------------------------------------------------------------------------
// PluginBase stubs
// ---------------------------------------------------------------------------

bool Plugin::on_activate(
  geometry_msgs::msg::PoseStamped /*drone_pose*/,
  as2_msgs::action::NavigateToPoint::Goal /*goal*/)
{
  return true;
}

bool Plugin::on_deactivate() {return true;}
bool Plugin::on_modify() {return true;}
bool Plugin::on_pause() {return true;}
bool Plugin::on_resume() {return true;}
void Plugin::on_execution_end() {}

as2_behavior::ExecutionStatus Plugin::on_run()
{
  return as2_behavior::ExecutionStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// Map query
// ---------------------------------------------------------------------------

bool Plugin::is_occupied(const geometry_msgs::msg::PointStamped & point)
{
  const double x = point.point.x;
  const double y = point.point.y;
  const double z = point.point.z;

  // Local layer has priority: higher resolution, up-to-date
  const auto local_result = local_grid_->isOccupied(x, y, z);
  if (local_result.has_value()) {
    return local_result.value();
  }

  // Fallback: global OctoMap
  return octomap_map_->isOccupied(x, y, z);
}

}  // namespace as2_3d_map_server

PLUGINLIB_EXPORT_CLASS(as2_3d_map_server::Plugin, as2_behaviors_path_planning::PluginBase)
