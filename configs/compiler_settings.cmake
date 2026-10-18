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

# ==============================================================================
# compiler_settings.cmake
# Purpose: Single source of truth for the C++ language standard, compiler
#          warnings and runtime checks of all MONA C++ packages.
# Usage:   include("${CMAKE_CURRENT_SOURCE_DIR}/../../configs/compiler_settings.cmake")
#          directly after project(), then call
#          mona_enable_strict_warnings(<target>) for every MONA library and test.
#          Warnings are applied per target, so third-party sources compiled in the
#          same directory (e.g. GoogleTest via ament_add_gtest) are not affected.
#          Third-party headers must be added as SYSTEM includes
#          (ament_target_dependencies(<target> SYSTEM ...)).
# ==============================================================================

# --- Language standard --- #
# CMake recognizes CXX_STANDARD 23 from version 3.20 onwards; older versions
# would silently fall back to an earlier standard.
if(CMAKE_VERSION VERSION_LESS "3.20")
	message(FATAL_ERROR "compiler_settings.cmake requires CMake >= 3.20 (found ${CMAKE_VERSION}).")
endif()

# C++23 without GNU extensions. The available C++23 library features depend on
# the toolchain (GCC 11 on Ubuntu 22.04 implements only part of C++23).
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Compilation database --- #
# Writes build/<package>/compile_commands.json with the exact
# flags and include paths of every translation unit.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Compiler warnings (every warning is an error) --- #
set(MONA_STRICT_WARNINGS "")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
	list(APPEND MONA_STRICT_WARNINGS
		-Wall
		-Wextra
		-Wpedantic
		-Werror
		# Control flow: every switch handles every enumerator and has a default branch
		-Wswitch-enum
		-Wswitch-default
		-Wimplicit-fallthrough
		# Implicit conversions that may lose data or change sign
		-Wconversion
		-Wsign-conversion
		-Wdouble-promotion
		-Wfloat-equal
		# Casts and object model
		-Wold-style-cast
		-Wcast-qual
		-Wcast-align
		-Wnon-virtual-dtor
		-Woverloaded-virtual
		-Wzero-as-null-pointer-constant
		# Correctness and readability
		-Wshadow
		-Wnull-dereference
		-Wformat=2
		-Wundef
		-Wextra-semi
		-Wmissing-declarations
		-Wunused
	)
endif()

# GCC-specific diagnostics without a Clang equivalent of the same name
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	list(APPEND MONA_STRICT_WARNINGS
		-Wduplicated-cond
		-Wduplicated-branches
		-Wlogical-op
		-Wuseless-cast
		-Wsuggest-override
	)
endif()

# An unsupported compiler must fail the configuration instead of silently
# building without the warning set.
if(NOT MONA_STRICT_WARNINGS)
	message(FATAL_ERROR "No strict warning set is defined for compiler '${CMAKE_CXX_COMPILER_ID}'.")
endif()

# Applies the strict warning set to a single MONA target.
function(mona_enable_strict_warnings target)
	target_compile_options(${target} PRIVATE ${MONA_STRICT_WARNINGS})
endfunction()

# --- Runtime checks --- #
# Enables precondition checks in the C++ standard library (e.g. out-of-range
# element access, front() on an empty container). A violation terminates the
# process with a diagnostic instead of continuing with undefined behaviour.
add_compile_definitions(_GLIBCXX_ASSERTIONS)
