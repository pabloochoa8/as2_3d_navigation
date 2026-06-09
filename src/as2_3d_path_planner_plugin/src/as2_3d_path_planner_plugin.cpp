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
#include "as2_3d_path_planner_plugin/map_registry.hpp"
#include "as2_3d_path_planner_plugin/octomap_planner_map.hpp"

#include <cmath>
#include <Eigen/Core>
#include <pluginlib/class_list_macros.hpp>

namespace as2_3d_path_planner
{

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static const char * jps3d_result_str(Jps3DPlanner::Result r)
{
  switch (r) {
    case Jps3DPlanner::Result::SUCCESS:        return "SUCCESS";
    case Jps3DPlanner::Result::NO_PATH:        return "NO_PATH";
    case Jps3DPlanner::Result::START_OCCUPIED: return "START_OCCUPIED";
    case Jps3DPlanner::Result::GOAL_OCCUPIED:  return "GOAL_OCCUPIED";
    case Jps3DPlanner::Result::OUT_OF_BOUNDS:  return "OUT_OF_BOUNDS";
  }
  return "UNKNOWN";
}

static const char * astar_result_str(Astar3D::Result r)
{
  static const char * s[] = {
    "SUCCESS", "NO_PATH", "START_OCCUPIED", "GOAL_OCCUPIED",
    "OUT_OF_BOUNDS", "MAX_ITER_REACHED"
  };
  return s[static_cast<int>(r)];
}

/**
 * @brief Check whether all sampled trajectory points are collision-free.
 *
 * Uses the same spherical inflation logic as Astar3D::isInflatedFree but
 * operates on world coordinates directly.  OUT_OF_BOUNDS cells block
 * (drone must stay inside the arena); UNKNOWN cells pass (optimistic).
 *
 * @return true if the trajectory is safe; false otherwise.
 * @param[out] first_unsafe_idx  Index of the first unsafe sample (-1 if safe).
 */
static bool trajectoryIsSafe(
  const std::vector<Eigen::Vector3d> & traj,
  std::shared_ptr<as2_3d_map_interface::MapInterface> map,
  float inflation_radius,
  int & first_unsafe_idx)
{
  first_unsafe_idx = -1;

  const double res = map->getResolution();
  const int r = static_cast<int>(
    std::ceil(static_cast<double>(inflation_radius) / res - 1e-6));
  const int r2 = r * r;

  for (int idx = 0; idx < static_cast<int>(traj.size()); ++idx) {
    const double cx = traj[idx].x();
    const double cy = traj[idx].y();
    const double cz = traj[idx].z();

    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dz = -r; dz <= r; ++dz) {
          if (dx * dx + dy * dy + dz * dz > r2) {continue;}
          const double wx = cx + dx * res;
          const double wy = cy + dy * res;
          const double wz = cz + dz * res;
          const auto s = map->getVoxelState(wx, wy, wz);
          if (s == as2_3d_map_interface::VoxelState::OCCUPIED ||
            s == as2_3d_map_interface::VoxelState::OUT_OF_BOUNDS)
          {
            first_unsafe_idx = idx;
            return false;
          }
        }
      }
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// initialize
// ---------------------------------------------------------------------------

void Plugin::initialize(
  as2::Node * node_ptr,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  node_ptr_ = node_ptr;
  tf_buffer_ = tf_buffer;

  // --- A* parameters ---
  node_ptr_->declare_parameter("astar.max_iterations", 500000);
  node_ptr_->declare_parameter("astar.inflation_radius", 0.10);
  node_ptr_->declare_parameter("astar.allow_diagonal", true);
  node_ptr_->declare_parameter("astar.w_heuristic", 1.0);

  astar_params_.max_iterations =
    static_cast<int>(node_ptr_->get_parameter("astar.max_iterations").as_int());
  astar_params_.inflation_radius = static_cast<float>(
    node_ptr_->get_parameter("astar.inflation_radius").as_double());
  astar_params_.allow_diagonal =
    node_ptr_->get_parameter("astar.allow_diagonal").as_bool();
  astar_params_.w_heuristic = static_cast<float>(
    node_ptr_->get_parameter("astar.w_heuristic").as_double());

  // --- JPS3D parameters ---
  node_ptr_->declare_parameter("jps.inflation_radius", 0.10);
  node_ptr_->declare_parameter("jps.map_update_rate", 1);
  node_ptr_->declare_parameter("planner.use_jps3d", true);

  jps_params_.inflation_radius = static_cast<float>(
    node_ptr_->get_parameter("jps.inflation_radius").as_double());
  jps_params_.map_update_rate =
    static_cast<int>(node_ptr_->get_parameter("jps.map_update_rate").as_int());
  use_jps3d_ = node_ptr_->get_parameter("planner.use_jps3d").as_bool();

  // --- Minimum-Snap parameters ---
  node_ptr_->declare_parameter("snap.enabled",    true);
  node_ptr_->declare_parameter("snap.total_time", 10.0);
  node_ptr_->declare_parameter("snap.sample_dt",  0.05);
  node_ptr_->declare_parameter("snap.max_vel",    1.5);
  node_ptr_->declare_parameter("snap.max_acc",    2.0);

  snap_enabled_                = node_ptr_->get_parameter("snap.enabled").as_bool();
  snap_params_.total_time      = node_ptr_->get_parameter("snap.total_time").as_double();
  snap_params_.sample_dt       = node_ptr_->get_parameter("snap.sample_dt").as_double();
  snap_params_.max_vel         = node_ptr_->get_parameter("snap.max_vel").as_double();
  snap_params_.max_acc         = node_ptr_->get_parameter("snap.max_acc").as_double();

  if (snap_enabled_) {
    snap_ = std::make_unique<MinimumSnap>(snap_params_);
  }

  // --- Load map from file (primary) ---
  node_ptr_->declare_parameter("map_file", "");
  const std::string map_file =
    node_ptr_->get_parameter("map_file").as_string();

  if (!map_file.empty()) {
    auto octomap = std::make_shared<OctomapPlannerMap>();
    if (octomap->loadFromFile(map_file)) {
      map_interface_ = octomap;
      const auto b = map_interface_->getBounds();
      RCLCPP_INFO(node_ptr_->get_logger(),
        "[path_planner] OctoMap loaded: %zu nodes | res=%.3f m | "
        "padded bounds x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f]",
        octomap->getNodeCount(), map_interface_->getResolution(),
        b.x_min, b.x_max, b.y_min, b.y_max, b.z_min, b.z_max);
    } else {
      RCLCPP_ERROR(node_ptr_->get_logger(),
        "[path_planner] Failed to load map from '%s'", map_file.c_str());
    }
  }

  // --- Fallback: in-process registry ---
  if (!map_interface_) {
    auto map = MapRegistry::instance().getMap();
    if (map) {
      map_interface_ = map;
      RCLCPP_INFO(node_ptr_->get_logger(),
        "[path_planner] Map obtained from MapRegistry");
    } else {
      RCLCPP_WARN(node_ptr_->get_logger(),
        "[path_planner] No map_file and no MapRegistry entry — will retry at on_activate");
    }
  }

  // --- Instantiate planners ---
  if (map_interface_) {
    astar_ = std::make_unique<Astar3D>(map_interface_, astar_params_);
    if (use_jps3d_) {
      RCLCPP_INFO(node_ptr_->get_logger(),
        "[path_planner] Building JPS3D collision map...");
      jps_ = std::make_unique<Jps3DPlanner>(map_interface_, jps_params_);
      RCLCPP_INFO(node_ptr_->get_logger(),
        "[path_planner] JPS3D ready | inflation=%.2f m",
        jps_params_.inflation_radius);
    }
  }

  RCLCPP_INFO(node_ptr_->get_logger(),
    "[path_planner] initialised | jps=%s | snap=%s | inflation=%.2f m",
    use_jps3d_ ? "on" : "off",
    snap_enabled_ ? "on" : "off",
    astar_params_.inflation_radius);
}

// ---------------------------------------------------------------------------
// on_activate
// ---------------------------------------------------------------------------

bool Plugin::on_activate(
  geometry_msgs::msg::PoseStamped drone_pose,
  as2_msgs::action::NavigateToPoint::Goal goal)
{
  RCLCPP_INFO(node_ptr_->get_logger(),
    "[path_planner] on_activate: goal (%.2f, %.2f, %.2f) "
    "from drone (%.2f, %.2f, %.2f)",
    goal.point.point.x, goal.point.point.y, goal.point.point.z,
    drone_pose.pose.position.x, drone_pose.pose.position.y,
    drone_pose.pose.position.z);

  // Late-init if map was not available at initialize time
  if (!map_interface_) {
    auto map = MapRegistry::instance().getMap();
    if (!map) {
      RCLCPP_ERROR(node_ptr_->get_logger(),
        "[path_planner] No map available — cannot plan");
      return false;
    }
    map_interface_ = map;
    astar_ = std::make_unique<Astar3D>(map_interface_, astar_params_);
    if (use_jps3d_) {
      jps_ = std::make_unique<Jps3DPlanner>(map_interface_, jps_params_);
    }
    if (snap_enabled_) {
      snap_ = std::make_unique<MinimumSnap>(snap_params_);
    }
  }

  const Eigen::Vector3d start{
    drone_pose.pose.position.x,
    drone_pose.pose.position.y,
    drone_pose.pose.position.z};

  const Eigen::Vector3d goal_pos{
    goal.point.point.x,
    goal.point.point.y,
    goal.point.point.z};

  // -------------------------------------------------------------------------
  // Step 1: Global path (JPS3D with A* fallback)
  // -------------------------------------------------------------------------
  std::vector<Eigen::Vector3d> waypoints;

  if (use_jps3d_ && jps_) {
    waypoints = jps_->plan(start, goal_pos);
    if (waypoints.empty()) {
      RCLCPP_WARN(node_ptr_->get_logger(),
        "[path_planner] JPS3D failed (%s) — falling back to A*",
        jps3d_result_str(jps_->lastResult()));
    }
  }

  if (waypoints.empty()) {
    waypoints = astar_->plan(start, goal_pos);
    if (waypoints.empty()) {
      RCLCPP_ERROR(node_ptr_->get_logger(),
        "[path_planner] A* also failed (%s) — no path",
        astar_result_str(astar_->lastResult()));
      return false;
    }
  }

  const std::string planner_used =
    (use_jps3d_ && jps_ && jps_->lastResult() == Jps3DPlanner::Result::SUCCESS)
    ? "JPS3D" : "A*";

  RCLCPP_INFO(node_ptr_->get_logger(),
    "[path_planner] %s: %zu discrete waypoints", planner_used.c_str(), waypoints.size());

  // -------------------------------------------------------------------------
  // Step 2: Minimum-Snap trajectory generation
  // -------------------------------------------------------------------------
  std::vector<Eigen::Vector3d> final_pts;  // what we eventually put in path_

  if (snap_enabled_ && snap_ && waypoints.size() >= 2) {
    const auto sampled = snap_->generate(waypoints);

    if (sampled.size() < 2) {
      RCLCPP_WARN(node_ptr_->get_logger(),
        "[path_planner] Minimum-Snap returned empty trajectory — using discrete waypoints");
      final_pts = waypoints;
    } else {
      // -----------------------------------------------------------------------
      // Step 3: Collision check on the smooth trajectory
      // -----------------------------------------------------------------------
      int unsafe_idx = -1;
      const bool safe = trajectoryIsSafe(
        sampled, map_interface_,
        astar_params_.inflation_radius,   // reuse A* inflation
        unsafe_idx);

      if (safe) {
        RCLCPP_INFO(node_ptr_->get_logger(),
          "[path_planner] Smooth trajectory: %zu samples, %.1f s — SAFE",
          sampled.size(), snap_->totalTime());
        final_pts = sampled;
      } else {
        RCLCPP_WARN(node_ptr_->get_logger(),
          "[path_planner] Smooth trajectory UNSAFE at sample %d (%.2f, %.2f, %.2f) "
          "— falling back to discrete waypoints",
          unsafe_idx,
          sampled[unsafe_idx].x(), sampled[unsafe_idx].y(), sampled[unsafe_idx].z());
        final_pts = waypoints;
      }
    }
  } else {
    final_pts = waypoints;
  }

  // -------------------------------------------------------------------------
  // Fill path_
  // -------------------------------------------------------------------------
  path_.clear();
  path_.reserve(final_pts.size());
  for (const auto & pt : final_pts) {
    geometry_msgs::msg::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    path_.push_back(p);
  }

  RCLCPP_INFO(node_ptr_->get_logger(),
    "[path_planner] path_ filled with %zu points (%s)",
    path_.size(),
    (final_pts.size() == waypoints.size()) ? "discrete" : "smooth");
  return true;
}

bool Plugin::on_deactivate() {return true;}
bool Plugin::on_modify()     {return true;}
bool Plugin::on_pause()      {return true;}
bool Plugin::on_resume()     {return true;}
void Plugin::on_execution_end() {}

as2_behavior::ExecutionStatus Plugin::on_run()
{
  return as2_behavior::ExecutionStatus::SUCCESS;
}

bool Plugin::is_occupied(const geometry_msgs::msg::PointStamped & point)
{
  if (!map_interface_) {return false;}
  return map_interface_->isOccupied(point.point.x, point.point.y, point.point.z);
}

bool Plugin::is_path_traversable(
  const std::vector<geometry_msgs::msg::PointStamped> & path)
{
  for (const auto & p : path) {
    if (is_occupied(p)) {return false;}
  }
  return true;
}

}  // namespace as2_3d_path_planner

PLUGINLIB_EXPORT_CLASS(as2_3d_path_planner::Plugin, as2_behaviors_path_planning::PluginBase)
