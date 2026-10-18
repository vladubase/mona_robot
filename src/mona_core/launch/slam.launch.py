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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Path definitions
    mona_core_dir = get_package_share_directory("mona_core")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    slam_params_file = LaunchConfiguration(
        "slam_params_file",
        default=os.path.join(mona_core_dir, "configs", "mapper_params_localization.yaml"),
    )

    # Argument declarations
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        "slam_params_file",
        default_value=slam_params_file,
        description="Full path to the ROS2 parameters file to use for the slam_toolbox node",
    )

    # Integration of the standard slam_toolbox launch file
    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, "launch", "localization_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
        }.items(),
    )

    # Launch description generation
    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            declare_slam_params_file_cmd,
            start_slam_toolbox_cmd,
        ]
    )
