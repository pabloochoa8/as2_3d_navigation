#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names
#      of its contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch the AS2 path-planner behavior loaded with the 3D map-server plugin."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory('as2_3d_navigation_bringup')
    default_config = os.path.join(bringup_dir, 'config', 'map_server_params.yaml')

    args = [
        DeclareLaunchArgument(
            'namespace',
            default_value='drone0',
            description='Drone namespace'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),
        DeclareLaunchArgument(
            'plugin_name',
            default_value='as2_3d_map_server',
            description='pluginlib class name for the path-planner / map-server plugin'),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS 2 log level'),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Full path to the node parameters YAML file'),
    ]

    node = Node(
        package='as2_behaviors_path_planning',
        executable='as2_behaviors_path_planning_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'plugin_name': LaunchConfiguration('plugin_name'),
            },
            LaunchConfiguration('config_file'),
        ],
    )

    return LaunchDescription(args + [node])
