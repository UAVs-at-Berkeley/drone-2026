# syntax=docker/dockerfile:1.7
FROM simulator-base

ENV DEBIAN_FRONTEND=noninteractive
ARG ROS_APT_SOURCE_VERSION=1.2.0
ARG ROS_DEV_TOOLS_VERSION=1.0.1
ARG ROS_JAZZY_DESKTOP_VERSION=0.11.0-1noble.20260412.072512
ARG ROS_JAZZY_MAVROS_VERSION=2.14.0-1noble.20260412.051838
ARG ROS_JAZZY_MAVROS_EXTRAS_VERSION=2.14.0-1noble.20260412.054445
ARG ROS_JAZZY_GEOGRAPHIC_MSGS_VERSION=1.0.6-2noble.20260412.035916
ARG ROS_JAZZY_ROS_GZ_BRIDGE_VERSION=1.0.22-1noble.20260412.043437
ARG ROS_JAZZY_ROS_GZ_IMAGE_VERSION=1.0.22-1noble.20260412.051928

# ROS 2 Jazzy and bridge packages live in the robot layer because they bind the simulator to the ROS workspace.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb && \
    apt-get update && apt-get install -y \
    ros-dev-tools=${ROS_DEV_TOOLS_VERSION} \
    ros-jazzy-desktop=${ROS_JAZZY_DESKTOP_VERSION} \
    ros-jazzy-mavros=${ROS_JAZZY_MAVROS_VERSION} \
    ros-jazzy-mavros-extras=${ROS_JAZZY_MAVROS_EXTRAS_VERSION} \
    ros-jazzy-geographic-msgs=${ROS_JAZZY_GEOGRAPHIC_MSGS_VERSION} \
    ros-jazzy-ros-gz-bridge=${ROS_JAZZY_ROS_GZ_BRIDGE_VERSION} \
    ros-jazzy-ros-gz-image=${ROS_JAZZY_ROS_GZ_IMAGE_VERSION} && \
    /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh && \
    rm -rf /var/lib/apt/lists/*

# Bring button scripts, custom sim assets, and ROS workspace into the project runtime path.
RUN mkdir -p /home/sim/drone_workspace/drone-2026
COPY buttons/scripts /home/sim/drone_workspace/drone-2026/buttons/scripts
COPY ros_workspace /home/sim/drone_workspace/drone-2026/ros_workspace
COPY sim/custom_assets /home/sim/drone_workspace/drone-2026/sim/custom_assets
COPY sim/scripts/spawn_random_target.sh /home/sim/drone_workspace/drone-2026/sim/scripts/spawn_random_target.sh
COPY sim/scripts/spawn_sim_gimbal_camera.sh /home/sim/drone_workspace/drone-2026/sim/scripts/spawn_sim_gimbal_camera.sh
COPY sim/scripts/start_rviz_image_gui.sh /home/sim/drone_workspace/drone-2026/sim/scripts/start_rviz_image_gui.sh
COPY sim/scripts/run_rviz_inner.sh /home/sim/drone_workspace/drone-2026/sim/scripts/run_rviz_inner.sh
COPY sim/scripts/rviz /home/sim/drone_workspace/drone-2026/sim/scripts/rviz
COPY sim/scripts/tile_sitl_desktop.sh /home/sim/drone_workspace/drone-2026/sim/scripts/tile_sitl_desktop.sh
COPY sim/scripts/vnc/xstartup /home/sim/.vnc/xstartup
RUN sed -i 's/\r$//' /home/sim/drone_workspace/drone-2026/buttons/scripts/*.sh && \
    sed -i 's/\r$//' /home/sim/drone_workspace/drone-2026/sim/scripts/spawn_random_target.sh && \
    sed -i 's/\r$//' /home/sim/drone_workspace/drone-2026/sim/scripts/spawn_sim_gimbal_camera.sh && \
    sed -i 's/\r$//' /home/sim/drone_workspace/drone-2026/sim/scripts/start_rviz_image_gui.sh && \
    sed -i 's/\r$//' /home/sim/drone_workspace/drone-2026/sim/scripts/run_rviz_inner.sh && \
    sed -i 's/\r$//' /home/sim/drone_workspace/drone-2026/sim/scripts/tile_sitl_desktop.sh && \
    sed -i 's/\r$//' /home/sim/.vnc/xstartup && \
    chmod +x /home/sim/drone_workspace/drone-2026/buttons/scripts/*.sh && \
    chmod +x /home/sim/drone_workspace/drone-2026/sim/scripts/spawn_random_target.sh /home/sim/drone_workspace/drone-2026/sim/scripts/spawn_sim_gimbal_camera.sh /home/sim/drone_workspace/drone-2026/sim/scripts/start_rviz_image_gui.sh /home/sim/drone_workspace/drone-2026/sim/scripts/run_rviz_inner.sh /home/sim/drone_workspace/drone-2026/sim/scripts/tile_sitl_desktop.sh && \
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

COPY sim/scripts/entrypoint.sh /entrypoint.sh
RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh

WORKDIR /home/sim
EXPOSE 2222 5900 6080 14540/udp 14550/udp
ENTRYPOINT ["/entrypoint.sh"]
