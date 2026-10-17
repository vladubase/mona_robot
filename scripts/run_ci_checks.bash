#!/bin/bash
source /opt/ros/humble/setup.bash

# ==============================================================================
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

# ament_cppcheck skips cppcheck 2.x by default (reported as "skipped", not failed)
# because of performance issues on large code bases. MONA requires cppcheck to run.
export AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

# Define colors for output clarity
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Force clean FastDDS shared memory to prevent deadlocks
rm -rf /dev/shm/fastrtps* /dev/shm/ros2* /dev/shm/env_shared* 2>/dev/null || true

echo -e "${BLUE}[INFO] Starting Local Continuous Integration sequence...${NC}"

# ------------------------------------------------------------------------------
# 0. Pre-flight Check
# ------------------------------------------------------------------------------
if [ ! -d "src" ]; then
    echo -e "${RED}[ERROR] Directory 'src' not found.${NC}"
    echo "Please run this script from the root of your ROS 2 workspace."
    exit 1
fi

# ------------------------------------------------------------------------------
# 1. Static Analysis (Linting Only - No Reformatting)
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[1/4] Running Static Analysis...${NC}"

# Python Linting (Flake8)
echo -e "${BLUE}[CI] Running Flake8 linter...${NC}"
if python3 -m flake8 --version &> /dev/null; then
    # Check for critical errors (syntax, undefined variables); any finding fails the pipeline
    if ! python3 -m flake8 src/ scripts/ --count --select=E9,F63,F7,F82 --show-source --statistics; then
        echo -e "${RED}[ERROR] Python linting failed (Flake8).${NC}"
        exit 1
    fi
    echo -e "${GREEN}[OK] Python linting passed (Flake8).${NC}"
else
    echo -e "${RED}[ERROR] Flake8 linter not found. Run 'pip install flake8' locally.${NC}"
    exit 1
fi

# ------------------------------------------------------------------------------
# 2. Clean Workspace
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[2/4] Cleaning workspace (build, install, log)...${NC}"

# Check the success of cleaning directly
if ! rm -rf build/* install/* log/*; then
    echo -e "${RED}[ERROR] Failed to clean directories.${NC}"
    exit 1
fi

# ------------------------------------------------------------------------------
# 3. Build Packages
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[3/4] Building packages (with coverage enabled)...${NC}"

if ! colcon build --cmake-clean-cache --symlink-install \
  --cmake-args -DCMAKE_CXX_FLAGS="--coverage" -DCMAKE_C_FLAGS="--coverage" -DCMAKE_EXE_LINKER_FLAGS="--coverage"; then
    echo -e "${RED}[ERROR] Build failed.${NC}"
    exit 1
fi

# ------------------------------------------------------------------------------
# 4. Run Tests
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[4/4] Running tests...${NC}"

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo -e "${RED}[ERROR] install/setup.bash not found. Build might have failed silently.${NC}"
    exit 1
fi

echo -e "${BLUE}[CI] Executing colcon test...${NC}"
colcon test || echo -e "${YELLOW}[WARN] Some tests failed during execution. Proceeding to report generation...${NC}"

# ------------------------------------------------------------------------------
# 5. Generate Coverage Report
# ------------------------------------------------------------------------------
echo -e "${BLUE}[CI] Generating Test Coverage Report...${NC}"

if command -v lcov &> /dev/null; then
    lcov --capture --directory build --output-file build/coverage.info > /dev/null 2>&1
    lcov --remove build/coverage.info '/usr/*' '/opt/ros/*' '*/install/*' '*/test/*' '*/build/*' --output-file build/coverage.info > /dev/null 2>&1
    
    echo -e "${GREEN}[OK] Coverage report generated at build/coverage.info${NC}"
    
    echo -e "${YELLOW}\n================================================================================${NC}"
    echo -e "${YELLOW}                             TEST COVERAGE SUMMARY                              ${NC}"
    echo -e "${YELLOW}================================================================================${NC}"
    lcov --list build/coverage.info
    echo -e "${YELLOW}================================================================================\n${NC}"
else
    echo -e "${RED}[WARN] lcov not found. Skipping coverage report.${NC}"
fi

# ------------------------------------------------------------------------------
# 6. Validate Results and Exit
# ------------------------------------------------------------------------------
echo -e "${BLUE}[CI] Validating XML Test Results...${NC}"

if colcon test-result --all --verbose; then
    echo -e "${GREEN}====================================================${NC}"
    echo -e "${GREEN}[SUCCESS] All checks passed. Build and Tests are OK.${NC}"
    echo -e "${GREEN}====================================================${NC}"
    
    echo -e "\n${BLUE}Recommended next steps:${NC}"
    echo -e "  1. Create a new feature branch: ${YELLOW}git checkout -b <branch-name>${NC}"
    echo -e "  2. Commit your changes:         ${YELLOW}git commit -am \"your message\"${NC}"
    echo -e "  3. Push to origin:              ${YELLOW}git push origin <branch-name>${NC}"
    echo -e "  4. Open a Pull Request on GitHub."
    exit 0
else
    echo -e "${RED}=================================================${NC}"
    echo -e "${RED}[FAILURE] One or more tests failed. See details above.${NC}"
    echo -e "${RED}=================================================${NC}"
    
    echo -e "\n${YELLOW}[!] Please fix the logic errors before committing.${NC}"
    exit 1
fi
