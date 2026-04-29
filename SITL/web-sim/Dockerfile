# syntax=docker/dockerfile:1.7
FROM ubuntu:24.04@sha256:c4a8d5503dfb2a3eb8ab5f807da5bc69a85730fb49b5cfca2330194ebcc41c7b

ENV DEBIAN_FRONTEND=noninteractive
ARG ROS_APT_SOURCE_VERSION=1.2.0
ARG ROS_DEV_TOOLS_VERSION=1.0.1
ARG ROS_JAZZY_DESKTOP_VERSION=0.11.0-1noble.20260412.072512
ARG ROS_JAZZY_MAVROS_VERSION=2.14.0-1noble.20260412.051838
ARG ROS_JAZZY_MAVROS_EXTRAS_VERSION=2.14.0-1noble.20260412.054445
ARG ROS_JAZZY_GEOGRAPHIC_MSGS_VERSION=1.0.6-2noble.20260412.035916
ARG ROS_JAZZY_ROS_GZ_BRIDGE_VERSION=1.0.22-1noble.20260412.043437
ARG ROS_JAZZY_ROS_GZ_IMAGE_VERSION=1.0.22-1noble.20260412.051928

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y \
    bash \
    ca-certificates \
    curl \
    git \
    gnupg \
    locales \
    lsb-release \
    openssh-server \
    python3-pip \
    sudo \
    tmux \
    tigervnc-standalone-server \
    tigervnc-common \
    openbox \
    wmctrl \
    x11-utils \
    xterm \
    websockify \
    novnc \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8

# ROS 2 Jazzy apt source + desktop install
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb && \
    apt-get update && apt-get install -y \
    ros-dev-tools=${ROS_DEV_TOOLS_VERSION} \
    ros-jazzy-desktop=${ROS_JAZZY_DESKTOP_VERSION} && \
    rm -rf /var/lib/apt/lists/*

RUN useradd -m -s /bin/bash -G sudo sim && \
    echo "sim ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# PX4 Autopilot + simulator dependencies.
# Pin PX4 to a known commit for reproducible builds.
# Update PX4_REF intentionally (and together with test/validation) when upgrading.
ARG PX4_REF=fc9e4e8844f1b792cf2db01b1f0eaaa0fb8dd199
RUN git clone --depth 1 --recursive --shallow-submodules https://github.com/PX4/PX4-Autopilot.git /PX4-Autopilot && \
    cd /PX4-Autopilot && \
    git fetch --depth 1 origin "${PX4_REF}" && \
    git checkout --detach FETCH_HEAD && \
    bash /PX4-Autopilot/Tools/setup/ubuntu.sh && \
    chown -R sim:sim /PX4-Autopilot

# Pre-compile PX4 SITL firmware only. Do NOT add gz_x500 here — that is a run target
# (launches the sim and will hang `docker build` forever). Runtime runs `cmake --build … -- gz_x500` in entrypoint.
RUN su - sim -c "cd /PX4-Autopilot && make px4_sitl"

# MAVROS for bridge between PX4 SITL and ROS
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y \
    dbus-x11 \
    ros-jazzy-mavros=${ROS_JAZZY_MAVROS_VERSION} \
    ros-jazzy-mavros-extras=${ROS_JAZZY_MAVROS_EXTRAS_VERSION} \
    ros-jazzy-geographic-msgs=${ROS_JAZZY_GEOGRAPHIC_MSGS_VERSION} \
    ros-jazzy-ros-gz-bridge=${ROS_JAZZY_ROS_GZ_BRIDGE_VERSION} \
    ros-jazzy-ros-gz-image=${ROS_JAZZY_ROS_GZ_IMAGE_VERSION} && \
    /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh && \
    rm -rf /var/lib/apt/lists/*

# Bring repo scripts + ROS workspace into container at Pi-compatible path.
RUN mkdir -p /home/sim/drone_workspace/drone-2026
COPY start_drone.sh /home/sim/drone_workspace/drone-2026/start_drone.sh
COPY start_recording.sh /home/sim/drone_workspace/drone-2026/start_recording.sh
COPY start_mission_stack.sh /home/sim/drone_workspace/drone-2026/start_mission_stack.sh
COPY start_ros.sh /home/sim/drone_workspace/drone-2026/start_ros.sh
COPY ros_workspace /home/sim/drone_workspace/drone-2026/ros_workspace
COPY SITL/custom_assets /home/sim/drone_workspace/drone-2026/SITL/custom_assets
COPY SITL/web-sim/spawn_random_target.sh /home/sim/drone_workspace/drone-2026/SITL/web-sim/spawn_random_target.sh
COPY SITL/web-sim/spawn_sim_gimbal_camera.sh /home/sim/drone_workspace/drone-2026/SITL/web-sim/spawn_sim_gimbal_camera.sh
COPY SITL/web-sim/start_rviz_image_gui.sh /home/sim/drone_workspace/drone-2026/SITL/web-sim/start_rviz_image_gui.sh
COPY SITL/web-sim/run_rviz_inner.sh /home/sim/drone_workspace/drone-2026/SITL/web-sim/run_rviz_inner.sh
COPY SITL/web-sim/rviz /home/sim/drone_workspace/drone-2026/SITL/web-sim/rviz
COPY SITL/web-sim/tile_sitl_desktop.sh /home/sim/drone_workspace/drone-2026/SITL/web-sim/tile_sitl_desktop.sh
COPY SITL/web-sim/vnc/xstartup /home/sim/.vnc/xstartup
RUN sed -i 's/\r$//' /home/sim/drone_workspace/drone-2026/start_*.sh && \
    sed -i 's/\r$//' /home/sim/drone_workspace/drone-2026/SITL/web-sim/spawn_random_target.sh && \
    sed -i 's/\r$//' /home/sim/drone_workspace/drone-2026/SITL/web-sim/spawn_sim_gimbal_camera.sh && \
    sed -i 's/\r$//' /home/sim/drone_workspace/drone-2026/SITL/web-sim/start_rviz_image_gui.sh && \
    sed -i 's/\r$//' /home/sim/drone_workspace/drone-2026/SITL/web-sim/run_rviz_inner.sh && \
    sed -i 's/\r$//' /home/sim/drone_workspace/drone-2026/SITL/web-sim/tile_sitl_desktop.sh && \
    sed -i 's/\r$//' /home/sim/.vnc/xstartup && \
    chmod +x /home/sim/drone_workspace/drone-2026/start_*.sh && \
    chmod +x /home/sim/drone_workspace/drone-2026/SITL/web-sim/spawn_random_target.sh /home/sim/drone_workspace/drone-2026/SITL/web-sim/spawn_sim_gimbal_camera.sh /home/sim/drone_workspace/drone-2026/SITL/web-sim/start_rviz_image_gui.sh /home/sim/drone_workspace/drone-2026/SITL/web-sim/run_rviz_inner.sh /home/sim/drone_workspace/drone-2026/SITL/web-sim/tile_sitl_desktop.sh && \
    chmod +x /home/sim/.vnc/xstartup && \
    chown -R sim:sim /home/sim/drone_workspace && \
    chown -R sim:sim /home/sim/.vnc

USER sim
RUN /bin/bash -lc "source /opt/ros/jazzy/setup.bash && cd ~/drone_workspace/drone-2026/ros_workspace && colcon build --symlink-install"
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/sim/.bashrc && \
    echo "source ~/drone_workspace/drone-2026/ros_workspace/install/setup.bash" >> /home/sim/.bashrc

USER root
RUN mkdir -p /var/run/sshd && \
    ssh-keygen -A && \
    sed -i 's/^#\?PasswordAuthentication .*/PasswordAuthentication yes/' /etc/ssh/sshd_config && \
    sed -i 's/^#\?PermitRootLogin .*/PermitRootLogin no/' /etc/ssh/sshd_config

COPY SITL/web-sim/entrypoint.sh /entrypoint.sh
RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh

WORKDIR /home/sim
EXPOSE 2222 5900 6080 14540/udp 14550/udp
ENTRYPOINT ["/entrypoint.sh"]
