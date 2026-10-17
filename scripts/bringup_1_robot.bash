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
# Script:   bringup_1_robot.bash
# Purpose:  Launch a single robot instance with Gazebo, RViz2, and Foxglove.
#           Useful for testing teleoperation (gamepad) and standalone behaviors.
# ==============================================================================

# Terminal output colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Ensure graceful cleanup on any exit signal (Ctrl+C, error, or normal termination)
cleanup() {
    echo -e "\n${YELLOW}[INFO] Shutting down simulation and cleaning up...${NC}"
    
    # Force clean FastDDS shared memory to prevent deadlocks
    rm -rf /dev/shm/fastrtps* /dev/shm/ros2* /dev/shm/env_shared* 2>/dev/null || true
    
    docker rm -f global_clock_bridge >/dev/null 2>&1
    docker compose down --remove-orphans >/dev/null 2>&1
    exit
}
trap cleanup EXIT INT TERM

# Detect host-specific values (GPU vendor, /dev/dri group IDs) for Docker Compose.
source "$(dirname "$0")/detect_host.bash"

echo -e "${BLUE}[INFO] Cleaning up the environment...${NC}"
# Force clean FastDDS shared memory to prevent deadlocks
rm -rf /dev/shm/fastrtps* /dev/shm/ros2* /dev/shm/env_shared* 2>/dev/null || true
docker rm -f global_clock_bridge >/dev/null 2>&1
docker compose down --remove-orphans >/dev/null 2>&1

echo -e "${BLUE}[INFO] Starting the simulation (Gazebo)...${NC}"
docker compose up -d world
sleep 5

echo -e "${BLUE}[INFO] Starting Global Time Server (/clock bridge)...${NC}"
docker compose run -d --no-deps --rm --name global_clock_bridge mona-robot \
    bash -c "ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"

echo -e "${GREEN}[INFO] Robot is booting! Monitor telemetry via Foxglove.${NC}"
echo -e "${YELLOW}[INFO] Press Ctrl+C to terminate the simulation.${NC}"
echo -e "${BLUE}[INFO] Spawning robot mona_001...${NC}"

# Run synchronously. Pressing Ctrl+C will terminate the node, exit the script, and trigger the cleanup trap.
docker compose run --rm --name mona_001 mona-robot ros2 launch mona_core robot.launch.py \
    namespace:=mona_001 \
    use_gamepad:=true \
    headless:=false \
    use_sim_time:=true