# syntax=docker/dockerfile:1.7
FROM ubuntu:24.04@sha256:c4a8d5503dfb2a3eb8ab5f807da5bc69a85730fb49b5cfca2330194ebcc41c7b

ENV DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y \
    bash \
    ca-certificates \
    curl \
    dbus-x11 \
    git \
    gnupg \
    locales \
    lsb-release \
    openssh-server \
    python3-pip \
    software-properties-common \
    sudo \
    tmux \
    tigervnc-standalone-server \
    tigervnc-common \
    openbox \
    wmctrl \
    x11-utils \
    xterm \
    websockify \
    novnc \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN useradd -m -s /bin/bash -G sudo sim && \
    echo "sim ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# PX4 Autopilot + Gazebo simulator dependencies.
# Pin PX4 to a known commit for reproducible builds.
# Update PX4_REF intentionally and together with validation when upgrading.
ARG PX4_REF=fc9e4e8844f1b792cf2db01b1f0eaaa0fb8dd199
RUN git clone --depth 1 --recursive --shallow-submodules https://github.com/PX4/PX4-Autopilot.git /PX4-Autopilot && \
    cd /PX4-Autopilot && \
    git fetch --depth 1 origin "${PX4_REF}" && \
    git checkout --detach FETCH_HEAD && \
    bash /PX4-Autopilot/Tools/setup/ubuntu.sh && \
    chown -R sim:sim /PX4-Autopilot

# Pre-compile PX4 SITL firmware only. Do not add gz_x500 here because that is a run target.
RUN su - sim -c "cd /PX4-Autopilot && make px4_sitl"

WORKDIR /home/sim
EXPOSE 2222 5900 6080 14540/udp 14550/udp
