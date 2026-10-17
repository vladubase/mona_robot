#!/bin/bash

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

# ==============================================================================
# Script:   start_world.bash
# Purpose:  Launch the simulation infrastructure (Gazebo Server, Clock, RViz2).
# ==============================================================================

# Terminal output colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Ensure graceful cleanup on any exit signal
cleanup() {
    echo -e "\n${YELLOW}[INFO] Shutting down infrastructure...${NC}"
    
    # Force clean FastDDS shared memory to prevent deadlocks
    rm -rf /dev/shm/fastrtps* /dev/shm/ros2* /dev/shm/env_shared* 2>/dev/null || true
    
    docker rm -f global_clock_bridge global_rviz >/dev/null 2>&1
    docker compose down --remove-orphans >/dev/null 2>&1
    exit
}
trap cleanup EXIT INT TERM

# Detect host-specific values (GPU vendor, /dev/dri group IDs) for Docker Compose.
source "$(dirname "$0")/detect_host.bash"

echo -e "${BLUE}[INFO] Cleaning up the environment...${NC}"
# Force clean FastDDS shared memory to prevent deadlocks
rm -rf /dev/shm/fastrtps* /dev/shm/ros2* /dev/shm/env_shared* 2>/dev/null || true
docker rm -f global_clock_bridge global_rviz >/dev/null 2>&1
docker compose down --remove-orphans >/dev/null 2>&1

echo -e "${BLUE}[INFO] Starting Gazebo Physics Server (Headless Mode)...${NC}"
docker compose up -d world
sleep 5

# Bridge Gazebo ignition.msgs.Clock to ROS 2 rosgraph_msgs/msg/Clock
echo -e "${BLUE}[INFO] Starting Global Time Server (/clock bridge)...${NC}"
docker compose run -d --no-deps --rm --name global_clock_bridge mona-robot \
    bash -c "ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"

# Launch a single RViz2 instance to monitor the entire fleet and the global /map
echo -e "${BLUE}[INFO] Starting Global RViz2 Interface...${NC}"
docker compose run -d --no-deps --rm --name global_rviz mona-robot \
    bash -c "ros2 run rviz2 rviz2 -d src/mona_description/rviz/mona.rviz --ros-args -p use_sim_time:=true"

echo -e "${GREEN}[INFO] Infrastructure is running. Gazebo is ready to accept robots.${NC}"
echo -e "${YELLOW}[INFO] Press Ctrl+C to terminate.${NC}"

# Keep the script active to catch termination signals
tail -f /dev/null &
wait $!