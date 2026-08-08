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

cmake_minimum_required(VERSION 3.8)
project(mona_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake      REQUIRED)
find_package(rclcpp           REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs   REQUIRED)
find_package(geometry_msgs    REQUIRED)
find_package(std_msgs         REQUIRED)
find_package(sensor_msgs      REQUIRED)
find_package(tf2_ros          REQUIRED)
find_package(tf2_sensor_msgs  REQUIRED)
find_package(laser_geometry   REQUIRED)
find_package(message_filters  REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

# ------------------------------------------------------------------------------
# 1. Build the Component Library
# ------------------------------------------------------------------------------
add_library(lidar_merger_component SHARED
  src/lidar_merger_node.cpp
)

target_include_directories(lidar_merger_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(lidar_merger_component
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  laser_geometry
  message_filters
)

# Register the plugin within the ROS 2 component ecosystem
rclcpp_components_register_node(lidar_merger_component
  PLUGIN "mona_perception::LidarMergerNode"
  EXECUTABLE lidar_merger_node
)

# ------------------------------------------------------------------------------
# 2. Install Targets
# ------------------------------------------------------------------------------
install(
  TARGETS lidar_merger_component lidar_merger_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY   include/
  DESTINATION include
)

install(
  DIRECTORY   launch
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# 3. Testing and Linting
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  # Use manual package discovery for linters (instead of ament_lint_auto)
  # to guarantee strict compliance with enterprise QA standards.
  find_package(ament_cmake_gtest      REQUIRED)
  find_package(ament_cmake_cppcheck   REQUIRED)
  find_package(ament_cmake_cpplint    REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint    REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_copyright  REQUIRED)
  find_package(ament_cmake_flake8     REQUIRED)

  # Build GTest suite
  set(TEST_DEPS
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
    geometry_msgs
    std_msgs
    sensor_msgs
    tf2_ros
  )

  ament_add_gtest(test_lidar_merger_lifecycle test/test_lidar_merger_lifecycle.cpp)
  target_link_libraries(test_lidar_merger_lifecycle lidar_merger_component)
  ament_target_dependencies(test_lidar_merger_lifecycle ${TEST_DEPS})

  # Execute Linters
  # C++ static analysis and style formatting
  ament_cppcheck()
  ament_cpplint()
  ament_uncrustify(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../configs/uncrustify.cfg")

  # Copyright verification
  ament_copyright()

  # Python linter with custom configuration
  ament_flake8(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../configs/setup.cfg")

  # XML syntax validation (package, launch, xacro)
  ament_xmllint()

  # CMake syntax formatting
  ament_lint_cmake()
endif()

ament_package()
