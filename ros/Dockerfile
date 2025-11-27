# Use the official Ubuntu base image
FROM ubuntu:24.04

# Update package lists and install basic tools
RUN apt-get update && apt-get install -y \
    curl \
    vim \
    sudo \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Update locale to English (US)
RUN apt-get update && apt-get install -y \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS environment in .bashrc so it's available in interactive shells
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Set default command (runs when container starts)
CMD ["bash"]