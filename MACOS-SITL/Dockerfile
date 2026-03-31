# Base OS
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive

# Basic tools
RUN apt-get update && apt-get install -y \
    curl \
    git \
    sudo \
    python3-pip \
    cmake \
    tmux \
    make \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
# Add user to sudo group so they can use sudo if needed for PX4 operations
RUN useradd -m -s /bin/bash -G sudo user && \
    echo "user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ROS 2 Jazzy
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*
ENV LC_ALL=en_US.UTF-8 \
    LANG=en_US.UTF-8
RUN apt-get update && \
    apt-get install -y software-properties-common && add-apt-repository universe && \
    ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb && \
    apt-get update && apt-get install -y ros-dev-tools ros-jazzy-desktop && \
    rm -rf /var/lib/apt/lists/*
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/user/.bashrc

# PX4 SITL (whose set up script also installs Gazebo)
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive /PX4-Autopilot && \
    bash /PX4-Autopilot/Tools/setup/ubuntu.sh && \
    chown -R user:user /PX4-Autopilot

# VNC server for Gazebo GUI access
RUN apt-get update && apt-get install -y \
    tigervnc-standalone-server \
    tigervnc-common \
    openbox \
    xterm \
    && rm -rf /var/lib/apt/lists/*
# VNC startup configuration
RUN mkdir -p /home/user/.vnc && \
    echo "#!/bin/bash" > /home/user/.vnc/xstartup && \
    echo "exec openbox-session" >> /home/user/.vnc/xstartup && \
    chmod +x /home/user/.vnc/xstartup && \
    chown -R user:user /home/user/.vnc

# Set up MAVROS to allow for PX4 and ROS 2 integration
RUN apt-get update && apt-get install -y ros-jazzy-mavros  ros-jazzy-mavros-extras && \
    /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh && \
    rm -rf /var/lib/apt/lists/*

# Build PX4 ROS 2 workspace
USER user
RUN pip install --user -U --break-system-packages empy==3.3.4 pyros-genmsg setuptools && \
    mkdir -p ~/ros2_workspace/src && \
    cd ~/ros2_workspace/src && \
    git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/PX4/px4_ros_com.git && \
    cd ~/ros2_workspace && \
    /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build" && \
    echo "source ~/ros2_workspace/install/local_setup.bash" >> ~/.bashrc

# Container startup command
# 1. Starts up a VNC server inside the container, which will then forward the Gazebo UI display to host machine's VNC client
# 2. Starts tmux
EXPOSE 5900
EXPOSE 14550/udp
WORKDIR /PX4-Autopilot
RUN echo "export DISPLAY=:0" >> ~/.bashrc
CMD ["/bin/bash", "-c", "vncserver :0 -geometry 1280x720 -SecurityTypes None -localhost no --I-KNOW-THIS-IS-INSECURE & exec tmux"]   
