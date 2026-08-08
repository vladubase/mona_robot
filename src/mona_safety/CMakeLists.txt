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
project(mona_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake      REQUIRED)
find_package(rclcpp           REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs    REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(nav_msgs         REQUIRED)
find_package(std_msgs         REQUIRED)
find_package(std_srvs         REQUIRED)
find_package(mona_msgs        REQUIRED)

include_directories(include)

# ------------------------------------------------------------------------------
# 1. Build the Component Library
# ------------------------------------------------------------------------------
add_library(safety_component SHARED
  src/safety_node.cpp
)

target_include_directories(safety_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(safety_component
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  geometry_msgs
  diagnostic_msgs
  diagnostic_updater
  nav_msgs
  std_msgs
  std_srvs
  mona_msgs
)

# Register the plugin within the ROS 2 component ecosystem
rclcpp_components_register_node(safety_component
  PLUGIN "mona_safety::SafetyNode"
  EXECUTABLE safety_node
)

# ------------------------------------------------------------------------------
# 2. Install Targets
# ------------------------------------------------------------------------------
install(
  TARGETS safety_component safety_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY   include/
  DESTINATION include
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
    geometry_msgs
    nav_msgs
    std_msgs
    std_srvs
    mona_msgs
  )

  ament_add_gtest(test_safety_lifecycle test/test_safety_lifecycle.cpp)
  target_link_libraries(test_safety_lifecycle safety_component)
  ament_target_dependencies(test_safety_lifecycle ${TEST_DEPS})

  ament_add_gtest(test_safety_logic test/test_safety_logic.cpp)
  target_link_libraries(test_safety_logic safety_component)
  ament_target_dependencies(test_safety_logic ${TEST_DEPS})

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
