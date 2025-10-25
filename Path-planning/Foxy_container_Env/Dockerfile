# Use official ROS 2 Foxy image
FROM ros:foxy-ros-base

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Update & install Python and ROS tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Set Python3 as default
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update || true

# Set workspace
ENV ROS_WS=/home/vscode/ros2_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

# Source ROS 2 Foxy
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
ENV ROS_DISTRO=foxy
