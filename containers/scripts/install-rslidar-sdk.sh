#!/usr/bin/env bash
set -eox pipefail

echo 'installing rslidar sdk'

source /opt/ros/humble/setup.bash

mkdir rslidar_build && cd rslidar_build

mkdir src && cd src

git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd rslidar_sdk
git submodule init
git submodule update

cd ..

git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git

cd ..

colcon build