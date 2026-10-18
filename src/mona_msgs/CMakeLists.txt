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
project(mona_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake      REQUIRED)
find_package(std_msgs         REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ------------------------------------------------------------------------------
# Message Generation
# ------------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/FdirState.msg"
	"msg/SafetyDiagnostics.msg"
	"msg/SafetyState.msg"
	"msg/TwistMuxState.msg"
	DEPENDENCIES std_msgs
)

# ------------------------------------------------------------------------------
# Testing
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
	# Use manual package discovery for linters (instead of ament_lint_auto)
	# to guarantee strict compliance with enterprise QA standards.
	find_package(ament_cmake_xmllint    REQUIRED)
	find_package(ament_cmake_lint_cmake REQUIRED)
	find_package(ament_cmake_copyright  REQUIRED)

	# Copyright verification
	ament_copyright()

	# XML syntax validation (package, launch, xacro)
	ament_xmllint()

	# CMake syntax formatting
	# CMake files are indented with tabs (project code style)
	ament_lint_cmake("--filters=-whitespace/tabs" MAX_LINE_LENGTH 120)
endif()

ament_package()
