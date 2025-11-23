# Use the official Ubuntu base image
FROM ubuntu:24.04

# Update package lists and install basic tools
RUN apt-get update && apt-get install -y \
    curl \
    vim \
    && rm -rf /var/lib/apt/lists/*
    

# Install dependencies for PX4
RUN apt-get update && apt-get install -y \
    git \
    lsb-release \
    sudo \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Clone PX4 source code
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive /PX4-Autopilot

# Set working directory
WORKDIR /PX4-Autopilot

# Run PX4 install script
# Note: The install script implicitly installs a bunch of other stuff, like python, pip, make, etc.
RUN ./Tools/setup/ubuntu.sh

# Install x11-apps
RUN apt-get update && apt-get install -y x11-apps && rm -rf /var/lib/apt/lists/*

# Install QGroundControl
RUN sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y && \
    sudo apt install libfuse2 -y && \
    sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y



# Set default command (runs when container starts)
CMD ["bash"]