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
# For arm64: Build from source using official QGroundControl build process
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
        echo "Building QGroundControl from source for arm64 using official build process..." && \
        echo "Installing build dependencies..." && \
        apt-get update && apt-get install -y --no-install-recommends \
            ninja-build \
            cmake \
            unzip \
            && rm -rf /var/lib/apt/lists/* && \
        echo "Downloading and running QGroundControl's official setup scripts..." && \
        mkdir -p /tmp/qt && \
        wget -q https://raw.githubusercontent.com/mavlink/qgroundcontrol/master/tools/setup/install-dependencies-debian.sh -O /tmp/qt/install-dependencies-debian.sh && \
        wget -q https://raw.githubusercontent.com/mavlink/qgroundcontrol/master/tools/setup/install-qt-debian.sh -O /tmp/qt/install-qt-debian.sh && \
        chmod +x /tmp/qt/*.sh && \
        /tmp/qt/install-dependencies-debian.sh && \
        echo "Patching install script to use Qt 6.8.3 instead of latest..." && \
        sed -i 's/6\.10\.1/6.8.3/g' /tmp/qt/install-qt-debian.sh && \
        sed -i 's/6\.10/6.8/g' /tmp/qt/install-qt-debian.sh || true && \
        /tmp/qt/install-qt-debian.sh && \
        rm -rf /tmp/qt && \
        echo "Finding Qt installation..." && \
        QT_DIR=$(find /opt/Qt/6.8.3 -type d -name "gcc_*" 2>/dev/null | head -1) && \
        if [ -z "$QT_DIR" ]; then \
            QT_DIR=$(find /opt/Qt -type d \( -name "gcc_*" -o -name "gcc_arm64" \) 2>/dev/null | head -1); \
        fi && \
        if [ -z "$QT_DIR" ] || [ ! -d "$QT_DIR" ]; then \
            echo "ERROR: Qt 6.8.3 installation not found! Found: $(ls -la /opt/Qt 2>/dev/null || echo 'nothing')" && exit 1; \
        fi && \
        echo "Using Qt installation at: $QT_DIR" && \
        export QT_ROOT_DIR=$QT_DIR && \
        export PATH=$QT_ROOT_DIR/bin:$PATH && \
        echo "Cloning and building QGroundControl..." && \
        cd /tmp && \
        git clone --recursive --depth 1 --branch ${QGC_VERSION} https://github.com/mavlink/qgroundcontrol.git qgc-build && \
        cd qgc-build && \
        mkdir build && \
        cd build && \
        $QT_ROOT_DIR/bin/qt-cmake -S .. -B . -G Ninja -DCMAKE_BUILD_TYPE=Release && \
        cmake --build . --target all --parallel && \
        cmake --install . --config Release --prefix /opt/qgroundcontrol && \
        cd / && \
        rm -rf /tmp/qgc-build && \
        ln -s /opt/qgroundcontrol/bin/QGroundControl /usr/local/bin/qgroundcontrol || \
        ln -s /opt/qgroundcontrol/QGroundControl /usr/local/bin/qgroundcontrol || \
        (find /opt/qgroundcontrol -name "QGroundControl" -type f -executable | head -1 | xargs -I {} ln -s {} /usr/local/bin/qgroundcontrol) && \
        chown -R qgcuser:qgcuser /opt/qgroundcontrol && \
        echo "QGroundControl built and installed successfully to /opt/qgroundcontrol"; \
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

