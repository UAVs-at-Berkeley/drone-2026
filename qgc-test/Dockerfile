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

# Create a non-root user for QGroundControl (QGC refuses to run as root)
# Create user early so we can use it during the build process
RUN useradd -m -s /bin/bash qgcuser

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

# Ensure qgcuser owns QGroundControl installation (if it exists)
RUN if [ -d /opt/qgroundcontrol ]; then chown -R qgcuser:qgcuser /opt/qgroundcontrol; fi

# Set working directory to the user's home directory
WORKDIR /home/qgcuser

# Switch to non-root user
USER qgcuser

# Set default command (runs when container starts)
CMD ["bash"]
