# Use the official Ubuntu base image
FROM ubuntu:24.04

# Install required dependencies for QGroundControl
RUN apt-get update && apt-get install -y \
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
    curl \
    wget \
    ca-certificates \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Set build arguments for architecture detection and QGC version
ARG TARGETARCH
ARG QGC_VERSION=v5.0.0

# Install Qt 6 and build dependencies for QGroundControl (needed for arm64 build)
# These are installed early so they're available for both architectures
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    qt6-base-dev \
    qt6-base-dev-tools \
    qt6-multimedia-dev \
    qt6-positioning-dev \
    qt6-tools-dev \
    qt6-declarative-dev \
    qt6-svg-dev \
    qt6-serialport-dev \
    qt6-quick3d-dev \
    qt6-quickcontrols2-dev \
    qt6-shadertools-dev \
    qt6-3d-dev \
    qt6-connectivity-dev \
    qt6-sensors-dev \
    qt6-websockets-dev \
    qt6-webchannel-dev \
    qt6-webengine-dev \
    qt6-networkauth-dev \
    qt6-remoteobjects-dev \
    qt6-scxml-dev \
    qt6-lottie-dev \
    qt6-imageformats-dev \
    qt6-virtualkeyboard-dev \
    qt6-datavisualization-dev \
    qt6-charts-dev \
    qt6-quicktimeline-dev \
    qt6-quick3dphysics-dev \
    qt6-quick3dassetimport-dev \
    qt6-quick3dparticles-dev \
    qt6-quick3deffects-dev \
    qt6-quick3dextras-dev \
    libsdl2-dev \
    speech-dispatcher \
    libudev-dev \
    patchelf \
    && rm -rf /var/lib/apt/lists/*

# Detect architecture and install QGroundControl
# For x86_64: Use AppImage (extracted to avoid FUSE requirements)
# For arm64: Build from source
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
        echo "Building QGroundControl from source for arm64..." && \
        cd /tmp && \
        git clone --recursive --depth 1 --branch ${QGC_VERSION} https://github.com/mavlink/qgroundcontrol.git qgc-build && \
        cd qgc-build && \
        mkdir build && \
        cd build && \
        cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release && \
        ninja && \
        mkdir -p /opt/qgroundcontrol && \
        cp -r staging/* /opt/qgroundcontrol/ && \
        cd / && \
        rm -rf /tmp/qgc-build && \
        ln -s /opt/qgroundcontrol/QGroundControl /usr/local/bin/qgroundcontrol && \
        chown -R qgcuser:qgcuser /opt/qgroundcontrol && \
        echo "QGroundControl built and installed successfully to /opt/qgroundcontrol"; \
    else \
        echo "Unsupported architecture: $ARCH" && exit 1; \
    fi


# Create a non-root user for QGroundControl (QGC refuses to run as root)
RUN useradd -m -s /bin/bash qgcuser && \
    chown -R qgcuser:qgcuser /opt/qgroundcontrol

# Set working directory to the user's home directory
WORKDIR /home/qgcuser

# Switch to non-root user
USER qgcuser

# Set default command (runs when container starts)
CMD ["bash"]
