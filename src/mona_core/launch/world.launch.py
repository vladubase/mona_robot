#!/usr/bin/env python3

# Copyright 2026 vladubase
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Launch description for the simulation infrastructure (World).

This module initializes the Gazebo physics engine and the Foxglove
telemetry bridge. It serves as the foundational infrastructure required
prior to the instantiation of any robotic agents. No robot-specific
nodes or namespaces are declared within this scope.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # --- VARIABLES AND PATHS --- #
    pkg_mona_description = get_package_share_directory("mona_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_file = os.path.join(pkg_mona_description, "worlds", "warehouse.sdf")

    # --- LAUNCH ARGUMENTS --- #
    arg_headless = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="Run in headless mode (NO Rviz, NO Gazebo GUI)",
    )
    headless = LaunchConfiguration("headless")

    arg_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock")
    use_sim_time = LaunchConfiguration("use_sim_time")

    arg_foxglove_address = DeclareLaunchArgument(
        "foxglove_address",
        default_value="127.0.0.1",
        description="Interface of the Foxglove bridge. 127.0.0.1 = this machine only; "
        "0.0.0.0 exposes the bridge to the network.",
    )
    foxglove_address = LaunchConfiguration("foxglove_address")

    # --- ENVIRONMENT SETTINGS --- #
    env_resource = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[os.path.dirname(pkg_mona_description), ":", pkg_mona_description],
    )

    # Forced highlighting of ROS 2 logs
    env_color = SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1")

    # --- INFRASTRUCTURE NODES --- #
    # 1. Gazebo Server
    gz_args = [
        PythonExpression(["'-s -r ' if '", headless, "' == 'true' else '-r '"]),
        world_file,
    ]
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    # 2. Foxglove WebSocket Bridge (Global Telemetry)
    # Access policy: read-only telemetry, no client publishing (no access to
    # hardware/* topics), only the E-STOP and E-STOP reset services, and no
    # parameter access to the safety node and the FDIR manager.
    bridge_foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[
            {
                "port": 8765,
                "address": foxglove_address,
                "send_buffer_limit": 100000000,
                "use_sim_time": use_sim_time,
                "capabilities": [
                    "parameters",
                    "parametersSubscribe",
                    "services",
                    "connectionGraph",
                    "assets",
                ],
                "client_topic_whitelist": ["(?!)"],  # matches nothing
                "service_whitelist": [
                    "^/mona_[0-9]{3}/emergency_stop$",
                    "^/mona_[0-9]{3}/emergency_stop_reset$",
                ],
                "param_whitelist": ["^(?!/mona_[0-9]{3}/(safety_node|mona_fdir_manager)\\.).*$"],
            }
        ],
        output="screen",
    )

    # ==============================================================================
    # CRITICAL ARCHITECTURAL NOTE: Late Joiner QoS Mitigation for Foxglove
    # ==============================================================================
    # The Foxglove bridge is deliberately wrapped in a TimerAction to delay its execution.
    #
    # Rationale: Our C++ IPC component container initializes the robot's hardware
    # interfaces and TF tree extremely fast (in milliseconds). The `/tf_static` topic,
    # which contains the robot's structural frames, relies on the `Transient Local`
    # QoS policy.
    #
    # If the Foxglove bridge is launched simultaneously with the robot spawn inside
    # a Dockerized FastDDS network, a known Race Condition occurs during topology
    # discovery. The bridge drops the initial `/tf_static` broadcast, causing the
    # Foxglove Studio UI to cache an empty TF tree ("No options" in Fixed Frame)
    # and fail to sync with `/clock`.
    #
    # By delaying the bridge by 7 seconds, we guarantee it joins the ROS 2 graph
    # ONLY after the network topology is fully stabilized and latched topics are
    # populated, ensuring a flawless telemetry sync.
    # ==============================================================================
    delayed_foxglove = TimerAction(period=7.0, actions=[bridge_foxglove])

    # --- LAUNCH --- #
    return LaunchDescription(
        [
            env_resource,
            env_color,
            arg_headless,
            arg_use_sim_time,
            arg_foxglove_address,
            gz_sim,
            delayed_foxglove,
        ]
    )
