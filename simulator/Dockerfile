# Use the official Ubuntu base image
FROM ubuntu:24.04

# Install all dependencies for PX4 SITL and QGroundControl
RUN apt-get update && apt-get install -y \
    curl \
    vim \
    git \
    lsb-release \
    sudo \
    wget \
    ca-certificates \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libfuse2 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-cursor-dev \
    libgl1 \
    libglx0 \
    libglu1-mesa \
    libxrender1 \
    libxrandr2 \
    libxss1 \
    x11-apps \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user for QGroundControl (QGC refuses to run as root)
# Also add user to sudo group so they can use sudo if needed for PX4 operations
# Create user early so we can run operations as that user and avoid slow chown operations
RUN useradd -m -s /bin/bash -G sudo qgcuser && \
    echo "qgcuser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Clone PX4 source code
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive /PX4-Autopilot

# Set working directory for PX4 setup
WORKDIR /PX4-Autopilot

# Run PX4 install script
# Note: The install script implicitly installs a bunch of other stuff, like python, pip, make, etc.
RUN ./Tools/setup/ubuntu.sh

# Chown PX4 directory after setup script completes (setup script creates files as root)
RUN chown -R qgcuser:qgcuser /PX4-Autopilot

# Set build arguments for architecture detection and QGC version
ARG TARGETARCH
ARG QGC_VERSION=v5.0.0

# Detect architecture and install QGroundControl
# For x86_64: Use AppImage (extracted to avoid FUSE requirements)
# For arm64: Use flatpak (AppImage not available)
RUN ARCH=${TARGETARCH:-$(dpkg --print-architecture)} && \
    if [ "$ARCH" = "amd64" ] || [ "$ARCH" = "x86_64" ]; then \
        echo "Installing QGroundControl for x86_64 via AppImage..." && \
        wget -q https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage -O /tmp/QGroundControl.AppImage && \
        chmod +x /tmp/QGroundControl.AppImage && \
        cd /opt && \
        /tmp/QGroundControl.AppImage --appimage-extract && \
        mv squashfs-root qgroundcontrol && \
        rm /tmp/QGroundControl.AppImage && \
        ln -s /opt/qgroundcontrol/AppRun /usr/local/bin/qgroundcontrol && \
        chown -R qgcuser:qgcuser /opt/qgroundcontrol && \
        echo "QGroundControl extracted successfully to /opt/qgroundcontrol"; \
    elif [ "$ARCH" = "arm64" ] || [ "$ARCH" = "aarch64" ]; then \
        echo "Skipping QGroundControl installation for arm64" \
    else \
        echo "Unsupported architecture: $ARCH" && exit 1; \
    fi


# Update locale to English (US) and install ROS2 dependencies
RUN apt-get update && apt-get install -y \
    locales \
    software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

# Set locale environment variables
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install ROS2 apt source
RUN apt-get update && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN apt-get update && apt-get install -y \
    ros-dev-tools \
    ros-jazzy-desktop \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS environment in qgcuser's .bashrc so it's available in interactive shells
# Note: We need to do this before switching to qgcuser since we're still root here
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/qgcuser/.bashrc && \
    chown qgcuser:qgcuser /home/qgcuser/.bashrc

# Install colcon and build dependencies for DDS client
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Switch to non-root user
USER qgcuser

# Create workspace and clone Micro-XRCE-DDS-Agent
RUN mkdir -p ~/px4_ros_uxrce_dds_ws/src && \
    cd ~/px4_ros_uxrce_dds_ws/src && \
    git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git

# Build the workspace (source ROS in the same RUN command)
WORKDIR /home/qgcuser/px4_ros_uxrce_dds_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Add workspace setup to .bashrc so it's available in interactive shells
RUN echo "source ~/px4_ros_uxrce_dds_ws/install/local_setup.bash" >> ~/.bashrc

# Create PX4 ROS2 workspace and clone required repositories
RUN mkdir -p ~/px4_ros_com_ws/src && \
    cd ~/px4_ros_com_ws/src && \
    git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/PX4/px4_ros_com.git

# Build the PX4 ROS2 workspace
WORKDIR /home/qgcuser/px4_ros_com_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Add PX4 ROS2 workspace setup to .bashrc
RUN echo "source ~/px4_ros_com_ws/install/local_setup.bash" >> ~/.bashrc

# Set working directory to PX4-Autopilot (user can change this if needed)
WORKDIR /PX4-Autopilot

# Set default command (runs when container starts)
# Sources ROS environments, starts MicroXRCEAgent in background, then starts tmux
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source ~/px4_ros_uxrce_dds_ws/install/local_setup.bash && source ~/px4_ros_com_ws/install/local_setup.bash && MicroXRCEAgent udp4 -p 8888 & exec tmux"]

