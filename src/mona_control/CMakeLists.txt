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
project(mona_control)

# Language standard, strict warnings and runtime checks shared by all MONA C++ packages
include("${CMAKE_CURRENT_SOURCE_DIR}/../../configs/compiler_settings.cmake")

find_package(ament_cmake      REQUIRED)
find_package(rclcpp           REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action    REQUIRED)
find_package(geometry_msgs    REQUIRED)
find_package(std_msgs         REQUIRED)
find_package(nav2_msgs        REQUIRED)
find_package(mona_msgs        REQUIRED)

include_directories(include)

# ------------------------------------------------------------------------------
# 1. Build the Component Library
# ------------------------------------------------------------------------------
add_library(twist_mux_component SHARED
	src/twist_mux_node.cpp
)
mona_enable_strict_warnings(twist_mux_component)

target_include_directories(twist_mux_component PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(twist_mux_component SYSTEM
	rclcpp
	rclcpp_lifecycle
	rclcpp_components
	rclcpp_action
	geometry_msgs
	std_msgs
	nav2_msgs
	mona_msgs
)

# Register the plugin within the ROS 2 component ecosystem
rclcpp_components_register_node(twist_mux_component
	PLUGIN "mona_control::TwistMuxNode"
	EXECUTABLE twist_mux_node
)

# ------------------------------------------------------------------------------
# 2. Install Targets
# ------------------------------------------------------------------------------
install(
	TARGETS twist_mux_component twist_mux_node
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
		mona_msgs
	)

	ament_add_gtest(test_twist_mux_lifecycle test/test_twist_mux_lifecycle.cpp)
	mona_enable_strict_warnings(test_twist_mux_lifecycle)
	target_link_libraries(test_twist_mux_lifecycle twist_mux_component)
	ament_target_dependencies(test_twist_mux_lifecycle SYSTEM ${TEST_DEPS})

	ament_add_gtest(test_twist_mux_safety test/test_twist_mux_safety.cpp)
	mona_enable_strict_warnings(test_twist_mux_safety)
	target_link_libraries(test_twist_mux_safety twist_mux_component)
	ament_target_dependencies(test_twist_mux_safety SYSTEM ${TEST_DEPS})

	# Execute Linters
	# C++ static analysis and style formatting
	ament_cppcheck()
	# Tab indentation and trailing-comment spacing follow the project style, enforced by uncrustify
	ament_cpplint(FILTERS "-whitespace/tab" "-whitespace/comments" MAX_LINE_LENGTH 120)
	ament_uncrustify(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../configs/uncrustify.cfg")

	# Copyright verification
	ament_copyright()

	# Python linter with custom configuration
	ament_flake8(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../configs/setup.cfg")

	# XML syntax validation (package, launch, xacro)
	ament_xmllint()

	# CMake syntax formatting
	# CMake files are indented with tabs (project code style)
	ament_lint_cmake("--filters=-whitespace/tabs" MAX_LINE_LENGTH 120)
endif()

ament_package()
