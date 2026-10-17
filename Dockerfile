FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# 1. User Mapping Configuration
ARG USERNAME=mona_dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# 2. System and ROS 2 Dependencies Installation
# Optional Mesa PPA for GPUs that are newer than the Mesa release of the base
# distribution. scripts/detect_host.bash selects it per host GPU vendor; an
# empty value keeps the distribution Mesa.
ARG MESA_PPA=""
RUN apt-get update && apt-get install -y \
    software-properties-common curl sudo git \
    && if [ -n "$MESA_PPA" ]; then add-apt-repository -y "$MESA_PPA"; fi \
    && apt-get update && apt-get install -y \
    build-essential cmake python3-colcon-common-extensions \
    # Quality Assurance and Static Code Analysis Tools
    clang-tidy clang-format-14 cppcheck lcov \
    ros-humble-ament-cmake-clang-tidy \
    ros-humble-ament-cmake-cppcheck \
    ros-humble-ament-lint-auto \
    ros-humble-ament-cmake-uncrustify \
    ros-humble-ament-cmake-flake8 \
    # Robotic Stack and Simulation Dependencies
    ros-humble-xacro ros-humble-rviz2 ros-humble-ros-gz \
    ros-humble-foxglove-bridge \
    ros-humble-laser-geometry ros-humble-tf2-sensor-msgs \
    ros-humble-rqt-graph ros-humble-rqt-common-plugins \
    ros-humble-rqt-robot-monitor \
    ros-humble-rqt-runtime-monitor \
    ros-humble-robot-localization \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-joy ros-humble-teleop-twist-joy \
    python3-pip \
    libgl1-mesa-glx libgl1-mesa-dri mesa-utils \
    mesa-vulkan-drivers vulkan-tools \
    # Ogre3D/Mesa Bug Patch: Dynamic detection and syntax correction of the fragment shader
    && SHADER_FILE=$(find /opt/ros/humble -name "indexed_8bit_image.frag" | head -n 1) \
    && if [ -n "$SHADER_FILE" ]; then \
        sed -i '/#version/a #extension GL_ARB_shading_language_420pack : enable' "$SHADER_FILE" && \
        sed -i 's/uniform sampler2D alpha_image;/layout(binding = 0) uniform sampler2D alpha_image;/g' "$SHADER_FILE" && \
        sed -i 's/uniform sampler1D palette;/layout(binding = 1) uniform sampler1D palette;/g' "$SHADER_FILE"; \
        echo "Shader patched successfully at $SHADER_FILE"; \
    fi \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir --upgrade pip \
    # Deployed Black formatter alongside Flake8 for Enterprise-grade Python linting
    && python3 -m pip install --no-cache-dir "pycodestyle<2.9.0" "flake8<5.0.0" "black"

# 3. User Creation and Privilege Assignment
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -aG video,audio $USERNAME \
    && (getent group render && usermod -aG render $USERNAME || true)

# 4. Workspace Initialization
USER $USERNAME
WORKDIR /home/$USERNAME/mona_ws

# 5. Source Code Migration
COPY --chown=$USERNAME:$USER_GID src src
COPY --chown=$USERNAME:$USER_GID configs configs
COPY --chown=$USERNAME:$USER_GID scripts scripts

# 6. Pre-compilation of the ROS 2 Workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# 7. Environment Sourcing
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /home/${USERNAME}/mona_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

# 8. Entrypoint Configuration and Privilege Reversion
# Temporarily elevate privileges to install the entrypoint script system-wide
USER root
COPY scripts/entrypoint.bash /usr/local/bin/entrypoint.bash
RUN chmod +x /usr/local/bin/entrypoint.bash
# Revert to the unprivileged user to ensure runtime security
USER ${USERNAME}

ENTRYPOINT ["/usr/local/bin/entrypoint.bash"]

# Default execution command (dynamically overridden by Docker Compose configurations)
CMD ["ros2", "launch", "mona_core", "robot.launch.py", \
     "headless:=true", \
     "use_gamepad:=false", \
     "use_sim_time:=true"]