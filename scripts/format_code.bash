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

# Define colors for output clarity
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}[INFO] Starting Code Formatting sequence...${NC}"

# 0. Pre-flight Check
if [ ! -d "src" ]; then
    echo -e "${RED}[ERROR] Directory 'src' not found.${NC}"
    echo "Please run this script from the root of your ROS 2 workspace."
    exit 1
fi

echo -e "${YELLOW}[1/1] Auto-formatting code to match standards...${NC}"

# C++ Formatting (uncrustify)
if command -v ament_uncrustify &> /dev/null; then
    ament_uncrustify -c configs/uncrustify.cfg --reformat src/ > /dev/null 2>&1
    echo -e "${GREEN}[OK] C++ formatted (ament_uncrustify).${NC}"
else
    echo -e "${RED}[WARN] ament_uncrustify not found, skipping C++ format.${NC}"
fi

# Python Formatting (Black)
echo -e "${BLUE}[CI] Running Black formatter...${NC}"
if python3 -m black --version &> /dev/null; then
    python3 -m black --line-length 120 src/ scripts/
    echo -e "${GREEN}[OK] Python formatted (Black).${NC}"
else
    echo -e "${RED}[ERROR] Black formatter not found. Run 'pip install black' locally.${NC}"
    exit 1
fi

# Strip trailing whitespace from CMakeLists.txt
find src -name "CMakeLists.txt" -exec sed -i 's/[[:space:]]*$//' {} +

echo -e "${GREEN}[SUCCESS] All files formatted.${NC}"
