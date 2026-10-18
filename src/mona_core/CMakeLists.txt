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
project(mona_core)

# Language standard, strict warnings and runtime checks shared by all MONA C++ packages
include("${CMAKE_CURRENT_SOURCE_DIR}/../../configs/compiler_settings.cmake")

find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(rclcpp_components  REQUIRED)
find_package(rclcpp_lifecycle   REQUIRED)
find_package(lifecycle_msgs     REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(mona_msgs          REQUIRED)
find_package(geometry_msgs      REQUIRED)

include_directories(include)

# ------------------------------------------------------------------------------
# 1. Build the Component Library (FDIR Manager)
# ------------------------------------------------------------------------------
add_library(fdir_manager_component SHARED
	src/fdir_manager_node.cpp
)
mona_enable_strict_warnings(fdir_manager_component)

target_include_directories(fdir_manager_component PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(fdir_manager_component SYSTEM
	rclcpp
	rclcpp_components
	rclcpp_lifecycle
	lifecycle_msgs
	std_msgs
	mona_msgs
	geometry_msgs
)

# Register the plugin within the ROS 2 component ecosystem
rclcpp_components_register_node(fdir_manager_component
	PLUGIN "mona_core::FdirManagerNode"
	EXECUTABLE fdir_manager_node
)

# ------------------------------------------------------------------------------
# 2. Install Targets & Directories
# ------------------------------------------------------------------------------
install(
	TARGETS fdir_manager_component fdir_manager_node
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY   include/
	DESTINATION include
)

# Install configuration, launch, map, and script directories
install(
	DIRECTORY launch configs maps
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
		std_msgs
		mona_msgs
	)

	ament_add_gtest(test_fdir_manager test/test_fdir_manager.cpp)
	mona_enable_strict_warnings(test_fdir_manager)
	target_link_libraries(test_fdir_manager fdir_manager_component)
	ament_target_dependencies(test_fdir_manager SYSTEM ${TEST_DEPS})

	ament_add_gtest(test_fdir_tier_escalation test/test_fdir_tier_escalation.cpp)
	mona_enable_strict_warnings(test_fdir_tier_escalation)
	target_link_libraries(test_fdir_tier_escalation fdir_manager_component)
	ament_target_dependencies(test_fdir_tier_escalation SYSTEM ${TEST_DEPS})

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
