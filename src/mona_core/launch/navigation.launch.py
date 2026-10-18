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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    params_file = LaunchConfiguration("params_file")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_params_file = DeclareLaunchArgument("params_file")

    # ISOLATE SetRemap within an empty (dummy) group using PushRosNamespace('')
    # This prevents the SetRemap directive from leaking into the parent robot.launch.py scope
    nav2_group = GroupAction(
        actions=[
            PushRosNamespace(""),
            SetRemap(src="cmd_vel", dst="cmd_nav"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                }.items(),
            ),
        ]
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_params_file,
            nav2_group,
        ]
    )
