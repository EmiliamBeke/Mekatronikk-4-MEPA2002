# syntax=docker/dockerfile:1
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# --- Python venv for pip packages (PEP 668 safe) ---
# Install venv first so pip layer can be cached independently from apt changes
RUN apt-get update && apt-get install -y python3-venv && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv /opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH="/opt/venv/bin:$PATH"
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir --prefer-binary \
      numpy==1.26.4 \
      catkin_pkg==1.0.0 \
      pyserial==3.5 \
      ncnn==1.0.20260114 \
      lgpio==0.2.2.0 \
      adafruit-blinka==9.0.2 \
      adafruit-circuitpython-bno08x==1.3.1 \
      adafruit-extended-bus \
      matplotlib \
      opencv-python==4.11.0.86 \
      pillow \
      pyyaml \
      requests \
      scipy \
      psutil \
      polars \
      ultralytics-thop && \
    pip install --no-cache-dir --no-deps ultralytics==8.4.12

# Basic dev tooling for ROS 2 workspaces.
# This image is intended for headless SSH use on the Pi, so no GUI tools are included.
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools \
    nano \
    libgl1 \
    ros-jazzy-navigation2 \
    ros-jazzy-cv-bridge \
    ros-jazzy-robot-localization \
    ros-dev-tools \
    build-essential \
    i2c-tools \
    liblgpio-dev \
    liblgpio1 \
    v4l-utils \
    python3-opencv \
    python3-dev \
    python3-pip \
    python3-colcon-common-extensions \
    swig \
  && rm -rf /var/lib/apt/lists/*

# Set a persistent password for the default ubuntu user (override via build arg)
ARG UBUNTU_PASSWORD=qwerty
RUN echo "ubuntu:${UBUNTU_PASSWORD}" | chpasswd

# Ensure ubuntu can use sudo
RUN usermod -aG sudo ubuntu \
  && echo "ubuntu ALL=(ALL) ALL" >/etc/sudoers.d/ubuntu \
  && chmod 0440 /etc/sudoers.d/ubuntu
