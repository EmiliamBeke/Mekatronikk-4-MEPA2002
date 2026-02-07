FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Basic dev tooling for ROS 2 workspaces
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    git \
    nano \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-dev-tools \
  && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --break-system-packages \
    ultralytics==8.4.12 \
    ncnn==1.0.20260114
# rosdep (idempotent)
RUN rosdep init || true && rosdep update
