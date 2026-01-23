#!/usr/bin/env bash
set -eox pipefail

echo 'installing ros2'

apt update && apt install locales -y
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

apt-get update -y && apt-get upgrade -y
apt-get update
apt-get install -y --no-install-recommends \
	gnupg2 \
	lsb-release \
	ca-certificates \
	software-properties-common \
	curl

add-apt-repository universe -y
apt-get update
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
dpkg -i /tmp/ros2-apt-source.deb

apt-get update && apt-get upgrade -y
apt-get install ros-humble-desktop python3-argcomplete ros-dev-tools -y
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
printenv | grep -i ROS

apt-get install -y --no-install-recommends \
	ros-humble-rosbridge-suite \
	python3-serial \
	python3-rosdep \
	ros-humble-cv-bridge

rosdep init && rosdep update
apt-get clean 
rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
ros2 doctor