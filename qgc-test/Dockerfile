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

# Detect architecture, download and extract the correct QGroundControl AppImage
# QGroundControl AppImages are available for x86_64 (amd64) and aarch64 (arm64)
# We extract the AppImage during build to avoid FUSE requirements at runtime
RUN ARCH=${TARGETARCH:-$(dpkg --print-architecture)} && \
    if [ "$ARCH" = "amd64" ] || [ "$ARCH" = "x86_64" ]; then \
        QGC_ARCH="x86_64"; \
    elif [ "$ARCH" = "arm64" ] || [ "$ARCH" = "aarch64" ]; then \
        QGC_ARCH="arm64"; \
    else \
        echo "Unsupported architecture: $ARCH" && exit 1; \
    fi && \
    wget -q https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-${QGC_ARCH}.AppImage -O /tmp/QGroundControl.AppImage && \
    chmod +x /tmp/QGroundControl.AppImage && \
    cd /opt && \
    /tmp/QGroundControl.AppImage --appimage-extract && \
    mv squashfs-root qgroundcontrol && \
    rm /tmp/QGroundControl.AppImage && \
    ln -s /opt/qgroundcontrol/AppRun /usr/local/bin/qgroundcontrol && \
    echo "QGroundControl extracted successfully to /opt/qgroundcontrol"

# Create a non-root user for QGroundControl (QGC refuses to run as root)
RUN useradd -m -s /bin/bash qgcuser && \
    chown -R qgcuser:qgcuser /opt/qgroundcontrol

# Set working directory to the user's home directory
WORKDIR /home/qgcuser

# Switch to non-root user
USER qgcuser

# Set default command (runs when container starts)
CMD ["bash"]
