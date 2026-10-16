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
# Script:   detect_host.bash
# Purpose:  Detect host-specific values (user IDs, GPU vendor, /dev/dri group IDs)
#           so that docker-compose.yml contains no values that differ between hosts.
# Usage:    source scripts/detect_host.bash   -> exports the variables into this shell
#           ./scripts/detect_host.bash        -> prints KEY=VALUE lines (Makefile)
# Override: a variable already set in the environment always wins,
#           e.g. GPU_VENDOR=none MESA_PPA= make rebuild
# ==============================================================================

# User mapping: bind-mounted files must belong to the host user 
HOST_UID="${HOST_UID:-$(id -u)}"
HOST_GID="${HOST_GID:-$(id -g)}"

# GPU device group IDs (numeric, as seen by the host kernel)
# Group names ("video", "render") resolve to different numeric IDs across
# distributions (e.g. Arch, Ubuntu, Fedora), so the container receives the
# host's numeric IDs instead of the names.
detect_gid() {
    local dev
    for dev in "$@"; do
        if [ -e "$dev" ]; then
            stat -c '%g' "$dev"
            return
        fi
    done
}
HOST_VIDEO_GID="${HOST_VIDEO_GID:-$(detect_gid /dev/dri/card*)}"
HOST_RENDER_GID="${HOST_RENDER_GID:-$(detect_gid /dev/dri/renderD*)}"

# GPU vendor from PCI vendor IDs in sysfs
# PCI vendor IDs: 0x10de = NVIDIA, 0x1002 = AMD, 0x8086 = Intel.
# On hybrid-graphics systems, the first match in this order is selected.
detect_gpu_vendor() {
    local vendors
    vendors=$(cat /sys/class/drm/card*/device/vendor 2>/dev/null)
    if grep -q 0x10de <<< "$vendors"; then
        echo "nvidia"
    elif grep -q 0x1002 <<< "$vendors"; then
        echo "amd"
    elif grep -q 0x8086 <<< "$vendors"; then
        echo "intel"
    else
        echo "none"
    fi
}
GPU_VENDOR="${GPU_VENDOR:-$(detect_gpu_vendor)}"

# Mesa source for the image (Mesa-driven GPUs only: Intel, AMD)
# The Mesa release of the base distribution can predate the host GPU
# (Ubuntu 22.04 ships Mesa 23.2, which does not support Intel Arrow Lake).
# 'make doctor' verifies that hardware acceleration is active.
# "${MESA_PPA+x}" distinguishes an unset variable from an explicitly empty one,
# so "MESA_PPA= make rebuild" forces the distribution Mesa.
if [ -z "${MESA_PPA+x}" ]; then
    case "$GPU_VENDOR" in
        intel | amd) MESA_PPA="ppa:kisak/turtle" ;;
        *) MESA_PPA="" ;;
    esac
fi

HOST_VARS=(HOST_UID HOST_GID HOST_VIDEO_GID HOST_RENDER_GID GPU_VENDOR MESA_PPA)

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
    # Sourced: export the values to Docker Compose commands started from this shell
    export "${HOST_VARS[@]}"
else
    # Executed: print KEY=VALUE lines for the Makefile
    for var in "${HOST_VARS[@]}"; do
        echo "$var=${!var}"
    done
fi
