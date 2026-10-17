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
Launch description for a single robotic agent within a swarm architecture.

This module provisions the navigation stack, sensor drivers, perception
algorithms, and the Gazebo spawner for an individual robot. All instantiated
nodes are encapsulated within a dynamic namespace derived from the
'namespace' launch argument, ensuring strict isolation between swarm replicas.
"""

from nav2_common.launch import RewrittenYaml
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    ExecuteProcess,
    DeclareLaunchArgument,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace, SetRemap
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # --- VARIABLES AND PATHS --- #
    pkg_mona_core = get_package_share_directory("mona_core")
    pkg_mona_description = get_package_share_directory("mona_description")
    pkg_mona_perception = get_package_share_directory("mona_perception")

    file_fdir_policy = os.path.join(pkg_mona_core, "configs", "fdir_policy.yaml")
    file_safety_params = os.path.join(pkg_mona_core, "configs", "safety_params.yaml")
    file_ekf_config = os.path.join(pkg_mona_core, "configs", "ekf.yaml")

    file_xacro = os.path.join(pkg_mona_description, "urdf", "mona.urdf.xacro")
    file_rviz_config = os.path.join(pkg_mona_description, "rviz", "mona.rviz")

    # file_map_serialized = os.path.join(pkg_mona_core, "maps", "warehouse_map_serialized")
    # file_slam_params = os.path.join(pkg_mona_core, "configs", "mapper_params_online_async.yaml")
    # file_nav2_params = os.path.join(pkg_mona_core, "configs", "nav2_params.yaml")

    # --- LAUNCH ARGUMENTS --- #
    arg_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="mona_001",
        description="Top-level namespace for the robot (e.g. mona_001)",
    )
    namespace = LaunchConfiguration("namespace")

    arg_headless = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="Run in headless mode (NO Rviz, NO Gazebo GIU)",
    )
    headless = LaunchConfiguration("headless")

    arg_use_gamepad = DeclareLaunchArgument(
        "use_gamepad",
        default_value="false",
        description="Set to true to enable DualSense teleop.",
    )
    use_gamepad = LaunchConfiguration("use_gamepad")

    arg_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use sim time or real time"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    arg_spawn_x = DeclareLaunchArgument(
        "spawn_x", default_value="0.0", description="Spawn X coordinate"
    )
    spawn_x = LaunchConfiguration("spawn_x")
    arg_spawn_y = DeclareLaunchArgument(
        "spawn_y", default_value="0.0", description="Spawn Y coordinate"
    )
    spawn_y = LaunchConfiguration("spawn_y")
    arg_spawn_yaw = DeclareLaunchArgument(
        "spawn_yaw", default_value="0.0", description="Spawn Yaw angle"
    )
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    # --- ENVIRONMENT SETTINGS --- #
    env_resource = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[os.path.dirname(pkg_mona_description), ":", pkg_mona_description],
    )

    # Forced highlighting of ROS 2 logs
    env_color = SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1")

    # --- URDF PROCESSING --- #
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", file_xacro, " namespace:=", namespace]),
            value_type=str,
        )
    }

    # --- NODES OF THE ROBOT --- #
    # 1. Robot State Publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time, "frame_prefix": [namespace, "/"]},
        ],
    )

    # 2. Spawn a robot with a dynamic name and position
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            namespace,
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-Y",
            spawn_yaw,
            "-z",
            "0.01",
        ],
        output="screen",
    )

    # 3. BRIDGE ROS <-> IGNITION
    bridge_gazebo_ign = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            [
                "/",
                namespace,
                "/hardware/motor_cmd@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            ],
            ["/", namespace, "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry"],
            ["/", namespace, "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"],
            [
                "/",
                namespace,
                "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
            ],
            # "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",  # /clock remains global
            [
                "/",
                namespace,
                "/lidar_front/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            ],
            [
                "/",
                namespace,
                "/lidar_back/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            ],
            [
                "/",
                namespace,
                "/lidar_left/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            ],
            [
                "/",
                namespace,
                "/lidar_right/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            ],
            ["/", namespace, "/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU"],
            [
                "/",
                namespace,
                "/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            ],
            [
                "/",
                namespace,
                "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            ],
        ],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "qos_overrides./imu/data.subscriber.reliability": "best_effort",
            }
        ],
        output="screen",
    )

    # 4. Rviz2
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", file_rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=UnlessCondition(headless),
    )

    # 5. HIGH-BANDWIDTH CONTAINER: Zero-copy environment for perception and control
    # Expendable layer: If this crashes, the isolated safety layer will halt the robot.
    container_sensor_control = ComposableNodeContainer(
        name="sensor_control_executor",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="mona_perception",
                plugin="mona_perception::LidarMergerNode",
                name="mona_lidar_merger",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "target_frame": PythonExpression(
                            ["'", namespace, "/base_link'"]
                        ),
                    }
                ],
            ),
            ComposableNode(
                package="mona_control",
                plugin="mona_control::TwistMuxNode",
                name="twist_mux_node",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ],
    )

    # 6. ISOLATED PROCESS: Hardware Safety Core
    # Runs in its own OS process to survive perception/control crashes.
    process_safety_core = ComposableNodeContainer(
        name="isolated_safety_executor",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="mona_safety",
                plugin="mona_safety::SafetyNode",
                name="safety_node",
                parameters=[{"use_sim_time": use_sim_time}, file_safety_params],
            )
        ],
    )

    # 7. ISOLATED PROCESS: FDIR Watchdog
    # Runs in its own OS process. If this dies, the hardware PLC watchdog trips.
    process_fdir_watchdog = ComposableNodeContainer(
        name="isolated_fdir_executor",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="mona_core",
                plugin="mona_core::FdirManagerNode",
                name="mona_fdir_manager",
                parameters=[{"use_sim_time": use_sim_time}, file_fdir_policy],
            )
        ],
    )

    # 8. Perception
    launch_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mona_perception, "launch", "perception.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time, "namespace": namespace}.items(),
    )

    # 9. EKF (Robot Localization)
    ekf_param_rewrites = {
        "use_sim_time": use_sim_time,
        "map_frame": "/map",
        "odom_frame": [namespace, "/odom"],
        "base_link_frame": [namespace, "/base_footprint"],
        "world_frame": [namespace, "/odom"],
    }

    ekf_configured_params = RewrittenYaml(
        source_file=file_ekf_config,
        root_key=namespace,
        param_rewrites=ekf_param_rewrites,
        convert_types=True,
    )

    node_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_configured_params],
        remappings=[("odometry/filtered", "odom_filtered")],
    )

    # # 10. SLAM (Localization mode)
    # map_file_path = os.path.join(pkg_mona_core, "maps", "warehouse_map_serialized")

    # slam_param_rewrites = {
    #     "use_sim_time": use_sim_time,
    #     "odom_frame": [namespace, "/odom"],
    #     "map_frame": "map",  # The map frame is always global
    #     "base_frame": [namespace, "/base_footprint"],
    #     "scan_topic": "scan",
    #     "map_file_name": map_file_path,  # Specify the path to the map file
    #     # 'map_start_pose': [spawn_x, spawn_y, spawn_yaw] # Dynamic coord spawning
    # }

    # slam_configured_params = RewrittenYaml(
    #     source_file=slam_params_file,
    #     root_key=namespace,
    #     param_rewrites=slam_param_rewrites,
    #     convert_types=True,
    # )

    # launch_slam = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_mona_core, "launch", "slam.launch.py")
    #     ),
    #     launch_arguments={
    #         "use_sim_time": use_sim_time,
    #         "slam_params_file": slam_configured_params,
    #     }.items(),
    # )

    # # 11. Autonomous navigation Nav2
    # nav2_param_rewrites = {
    #     "use_sim_time": use_sim_time,
    #     "robot_base_frame": [namespace, "/base_footprint"],
    #     "odom_frame": [namespace, "/odom"],
    #     "global_frame": "map",  # Nav2 maps are built relative to the global 'map' frame
    # }

    # nav2_configured_params = RewrittenYaml(
    #     source_file=nav2_params_file,
    #     root_key="",
    #     param_rewrites=nav2_param_rewrites,
    #     convert_types=True,
    # )

    # launch_navigation = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_mona_core, "launch", "navigation.launch.py")
    #     ),
    #     launch_arguments={
    #         "use_sim_time": use_sim_time,
    #         "params_file": nav2_configured_params,
    #     }.items(),
    # )

    # 12. Manual control (Gamepad)
    launch_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mona_core, "launch", "teleop.launch.py")
        ),
        condition=IfCondition(use_gamepad),
    )

    # 13. Rosbag
    rosbag_record = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--use-sim-time",
            "-o",
            ["mona_flight_data_", namespace],
            ["/", namespace, "/cmd_vel"],
            ["/", namespace, "/cmd_vel_teleop"],
            ["/", namespace, "/cmd_vel_nav"],
            ["/", namespace, "/robot_status"],
            ["/", namespace, "/diagnostics"],
        ],
        output="screen",
    )

    # --- GROUPING A ROBOT IN NAMESPACE --- #
    robot_group = GroupAction(
        [
            PushRosNamespace(namespace),
            SetRemap(src="tf", dst="/tf"),
            SetRemap(src="tf_static", dst="/tf_static"),
            node_robot_state_publisher,
            spawn_entity,
            bridge_gazebo_ign,
            node_rviz,
            container_sensor_control,
            process_safety_core,
            process_fdir_watchdog,
            launch_perception,
            node_ekf,
            launch_teleop,
            # launch_slam,
            # launch_navigation,
            rosbag_record,
        ]
    )

    # --- LAUNCH --- #
    return LaunchDescription(
        [
            env_resource,
            env_color,
            arg_namespace,
            arg_headless,
            arg_use_gamepad,
            arg_use_sim_time,
            arg_spawn_x,
            arg_spawn_y,
            arg_spawn_yaw,
            robot_group,
        ]
    )
