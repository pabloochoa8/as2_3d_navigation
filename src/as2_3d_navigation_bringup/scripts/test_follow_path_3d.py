#!/usr/bin/env python3
"""
Test script: send a FollowPath action with waypoints at different heights.

This script:
  1. Sets the drone to offboard
  2. Arms it
  3. Takes off to 1.0 m
  4. Sends a 3D FollowPath goal with variable z

Expected behavior:
  - WP0: (0.0, 0.0, 1.0)
  - WP1: (1.0, 0.0, 1.0)
  - WP2: (1.0, 0.0, 2.0)
  - WP3: (2.0, 0.0, 2.0)
  - WP4: (2.0, 0.0, 1.0)
  - WP5: (0.0, 0.0, 1.0)

Usage:
  python3 test_follow_path_3d.py --namespace drone0
"""

import sys
import argparse
from time import sleep

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from as2_msgs.action import FollowPath
from as2_msgs.msg import PoseWithID
from geometry_msgs.msg import Pose

from as2_python_api.drone_interface import DroneInterface


WAYPOINTS = [
    (0.0, 0.0, 1.0),
    (1.0, 0.0, 1.0),
    (1.0, 0.0, 2.0),
    (2.0, 0.0, 2.0),
    (2.0, 0.0, 1.0),
    (0.0, 0.0, 1.0),
]


class FollowPath3DTest(Node):
    def __init__(self, namespace: str = "drone0"):
        super().__init__("follow_path_3d_test")
        self.namespace = namespace
        action_name = f"/{namespace}/FollowPathBehavior"
        self._client = ActionClient(self, FollowPath, action_name)
        self.get_logger().info(f"Connecting to action server: {action_name}")

    def send_goal(self) -> bool:
        self.get_logger().info("Waiting for FollowPath action server...")

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "Action server not available after 10 s. Is AS2 running?"
            )
            return False

        goal = FollowPath.Goal()
        goal.header.frame_id = "earth"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.max_speed = 0.5

        self.get_logger().info("Path to execute:")

        for i, (x, y, z) in enumerate(WAYPOINTS):
            pid = PoseWithID()
            pid.id = str(i)
            pid.pose = Pose()
            pid.pose.position.x = x
            pid.pose.position.y = y
            pid.pose.position.z = z
            pid.pose.orientation.w = 1.0

            goal.path.append(pid)

            self.get_logger().info(
                f"  WP{i}: x={x:.2f}  y={y:.2f}  z={z:.2f}"
            )

        self.get_logger().info("Sending FollowPath goal...")

        future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb,
        )

        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error("Goal handle is None")
            return False

        if not goal_handle.accepted:
            self.get_logger().error("Goal REJECTED")
            return False

        self.get_logger().info("Goal ACCEPTED — executing 3D path")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f"Result: success={result.follow_path_success}")

        return result.follow_path_success

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"[feedback] speed={fb.actual_speed:.2f} m/s  "
            f"dist_to_next={fb.actual_distance_to_next_waypoint:.2f} m  "
            f"remaining_wp={fb.remaining_waypoints}  "
            f"next_id={fb.next_waypoint_id}",
            throttle_duration_sec=1.0,
        )


def prepare_drone(namespace: str):
    print("Creating DroneInterface")
    drone = DroneInterface(namespace, verbose=True, use_sim_time=True)

    print("Offboard")
    drone.offboard()
    sleep(2.0)

    print("Arm")
    drone.arm()
    sleep(2.0)

    print("Takeoff to 1.0 m")
    drone.takeoff(1.0, speed=0.5)
    print("Takeoff done")

    print("Stabilizing before FollowPath")
    sleep(3.0)

    return drone


def main():
    parser = argparse.ArgumentParser(
        description="Test 3D FollowPath with z-variable waypoints"
    )
    parser.add_argument(
        "--namespace",
        default="drone0",
        help="Drone namespace, default: drone0",
    )
    parser.add_argument(
        "--no-takeoff",
        action="store_true",
        help="Skip offboard/arm/takeoff if the drone is already flying",
    )

    args, _ = parser.parse_known_args()

    rclpy.init()

    drone = None

    try:
        if not args.no_takeoff:
            drone = prepare_drone(args.namespace)

        node = FollowPath3DTest(namespace=args.namespace)
        success = node.send_goal()
        node.destroy_node()

    finally:
        if drone is not None:
            drone.shutdown()

        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
