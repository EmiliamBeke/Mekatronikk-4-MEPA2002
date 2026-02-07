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

# rosdep (idempotent)
RUN rosdep init || true && rosdep update
