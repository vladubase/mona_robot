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
# Script:   start_fleet.bash
# Purpose:  Spawn a scalable fleet utilizing Docker Compose to inherit volumes and envs.
# Usage:    ./start_fleet.bash [number_of_robots]
# ==============================================================================

# Terminal output colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Ensure graceful cleanup on any exit signal
cleanup() {
    echo -e "\n${YELLOW}[INFO] Shutting down the fleet gracefully...${NC}"
    
    # Force clean FastDDS shared memory to prevent deadlocks
    rm -rf /dev/shm/fastrtps* /dev/shm/ros2* /dev/shm/env_shared* 2>/dev/null || true
    
    docker stop -t 10 $(docker ps -q --filter name="mona_0*") >/dev/null 2>&1
    exit
}
trap cleanup EXIT INT TERM

# Detect host-specific values (GPU vendor, /dev/dri group IDs) for Docker Compose.
source "$(dirname "$0")/detect_host.bash"

# Force clean FastDDS shared memory to prevent deadlocks
rm -rf /dev/shm/fastrtps* /dev/shm/ros2* /dev/shm/env_shared* 2>/dev/null || true

ROBOT_COUNT=${1:-1}

echo -e "${BLUE}[INFO] Spawning a fleet of $ROBOT_COUNT robot(s)...${NC}"

for (( i=1; i<=ROBOT_COUNT; i++ )); do
    ROBOT_ID=$(printf "mona_%03d" $i)
    OFFSET_Y=$(awk "BEGIN {print ($i - 1) * 1.5}")
    DELAY=$(( (i - 1) * 20 ))

    echo -e "${BLUE}[INFO] [$i/$ROBOT_COUNT] Initializing $ROBOT_ID at Y_offset=$OFFSET_Y (Boot delay: ${DELAY}s)...${NC}"

    # The '&' operator pushes the process to the background, streaming logs directly to the console
    docker compose run --no-deps --rm --name "$ROBOT_ID" mona-robot \
        bash -c "sleep $DELAY && \
            ros2 launch mona_core robot.launch.py \
            namespace:=$ROBOT_ID \
            use_gamepad:=true \
            headless:=true \
            use_sim_time:=true \
            spawn_x:=0.0 \
            spawn_y:=$OFFSET_Y" &
done

echo -e "${GREEN}[INFO] Fleet successfully deployed.${NC}"
echo -e "${YELLOW}[INFO] Streaming telemetry logs... Press Ctrl+C to terminate all robots.${NC}"

# Block execution to keep background log streams active
wait