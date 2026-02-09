FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Basic dev tooling for ROS 2 workspaces
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    git \
    libcamera-dev \
    libcamera-ipa \
    libcamera0.2 \
    nano \
    ros-jazzy-camera-ros \
    ros-jazzy-cv-bridge \
    sudo \
    v4l-utils \
    python3-opencv \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-dev-tools \
  && rm -rf /var/lib/apt/lists/*

# Set a persistent password for the default ubuntu user (override via build arg)
ARG UBUNTU_PASSWORD=qwerty
RUN echo "ubuntu:${UBUNTU_PASSWORD}" | chpasswd

# Ensure ubuntu can use sudo
RUN usermod -aG sudo ubuntu \
  && echo "ubuntu ALL=(ALL) ALL" >/etc/sudoers.d/ubuntu \
  && chmod 0440 /etc/sudoers.d/ubuntu

# rosdep (idempotent)
RUN rosdep init || true && rosdep update

# --- Python venv for pip packages (PEP 668 safe) ---
RUN apt-get update && apt-get install -y python3-venv && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv /opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH="/opt/venv/bin:$PATH"
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir ultralytics==8.4.12 ncnn==1.0.20260114
