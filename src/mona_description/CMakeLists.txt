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
project(mona_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake  REQUIRED)
find_package(urdf         REQUIRED)
find_package(xacro        REQUIRED)

install(
	DIRECTORY urdf meshes rviz worlds
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
	# 1. Use manual package discovery for specific linters (instead of ament_lint_auto)
	find_package(ament_cmake_xmllint    REQUIRED)
	find_package(ament_cmake_lint_cmake REQUIRED)
	find_package(ament_cmake_copyright  REQUIRED)
	find_package(ament_cmake_flake8     REQUIRED)

	# 2. Execute each linter manually
	# Copyright verification
	ament_copyright()

	# Python linter with custom configuration
	ament_flake8(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../configs/setup.cfg")

	# XML syntax validation (package, launch, xacro)
	ament_xmllint()

	# CMake style validation
	# CMake files are indented with tabs (project code style)
	ament_lint_cmake("--filters=-whitespace/tabs" MAX_LINE_LENGTH 120)
endif()

ament_package()