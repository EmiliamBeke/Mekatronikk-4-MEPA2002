FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Basic dev tooling for ROS 2 workspaces
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools \
    nano \
    ros-jazzy-cv-bridge \
    ros-dev-tools \
    v4l-utils \
    python3-opencv \
    python3-pip \
    python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

# Set a persistent password for the default ubuntu user (override via build arg)
ARG UBUNTU_PASSWORD=qwerty
RUN echo "ubuntu:${UBUNTU_PASSWORD}" | chpasswd

# Ensure ubuntu can use sudo
RUN usermod -aG sudo ubuntu \
  && echo "ubuntu ALL=(ALL) ALL" >/etc/sudoers.d/ubuntu \
  && chmod 0440 /etc/sudoers.d/ubuntu

# --- Python venv for pip packages (PEP 668 safe) ---
RUN apt-get update && apt-get install -y python3-venv && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv /opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH="/opt/venv/bin:$PATH"
RUN pip install --upgrade pip && \
  pip install numpy==1.26.4 catkin_pkg==1.0.0 ultralytics==8.4.12 ncnn==1.0.20260114
