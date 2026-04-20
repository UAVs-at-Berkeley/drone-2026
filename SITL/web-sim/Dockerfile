FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
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
    xterm \
    websockify \
    novnc \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8

# ROS 2 Jazzy apt source + desktop install
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | awk -F\" '/tag_name/{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb && \
    apt-get update && apt-get install -y ros-dev-tools ros-jazzy-desktop && \
    rm -rf /var/lib/apt/lists/*

RUN useradd -m -s /bin/bash -G sudo sim && \
    echo "sim ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# PX4 Autopilot + simulator dependencies
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive /PX4-Autopilot && \
    bash /PX4-Autopilot/Tools/setup/ubuntu.sh && \
    chown -R sim:sim /PX4-Autopilot

# MAVROS for bridge between PX4 SITL and ROS
RUN apt-get update && apt-get install -y \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    ros-jazzy-geographic-msgs && \
    /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh && \
    rm -rf /var/lib/apt/lists/*

# Bring repo scripts + ROS workspace into container at Pi-compatible path.
RUN mkdir -p /home/sim/drone_workspace/drone-2026
COPY start_drone.sh /home/sim/drone_workspace/drone-2026/start_drone.sh
COPY start_recording.sh /home/sim/drone_workspace/drone-2026/start_recording.sh
COPY start_mission_stack.sh /home/sim/drone_workspace/drone-2026/start_mission_stack.sh
COPY start_ros.sh /home/sim/drone_workspace/drone-2026/start_ros.sh
COPY ros_workspace /home/sim/drone_workspace/drone-2026/ros_workspace
RUN sed -i 's/\r$//' /home/sim/drone_workspace/drone-2026/start_*.sh && \
    chmod +x /home/sim/drone_workspace/drone-2026/start_*.sh && \
    chown -R sim:sim /home/sim/drone_workspace

USER sim
RUN /bin/bash -lc "source /opt/ros/jazzy/setup.bash && cd ~/drone_workspace/drone-2026/ros_workspace && colcon build --symlink-install"
RUN mkdir -p /home/sim/.vnc && \
    printf '%s\n' '#!/bin/bash' 'exec openbox-session' > /home/sim/.vnc/xstartup && \
    chmod +x /home/sim/.vnc/xstartup
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
