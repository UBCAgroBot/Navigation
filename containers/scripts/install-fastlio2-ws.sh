#!/usr/bin/env bash
set -eox pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

TARGET_HOME="${TARGET_HOME:-/home/vscode/workspace}"
mkdir -p "$TARGET_HOME/fastlio2-build/src"
cd "$TARGET_HOME/fastlio2-build/src"

# livox_ros_driver2
if [ ! -d livox_ros_driver2 ]; then
  git clone https://github.com/Livox-SDK/livox_ros_driver2.git
fi
cd livox_ros_driver2
cp package_ROS2.xml package.xml
sed -i '/LIVOX_INTERFACES_INCLUDE_DIRECTORIES/s/^/#/' CMakeLists.txt

cd "$TARGET_HOME/fastlio2-build"
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y

colcon build \
  --packages-select livox_ros_driver2 \
  --symlink-install \
  --cmake-args \
    -DROS_EDITION=ROS2 \
    -DHUMBLE_ROS=humble \
    -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3

# FAST_LIO_ROS2
cd "$TARGET_HOME/fastlio2-build/src"
if [ ! -d FAST_LIO_ROS2 ]; then
  git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive
fi

cd "$TARGET_HOME/fastlio2-build"
rosdep install --from-paths src --ignore-src -r -y

colcon build \
  --parallel-workers "$(nproc)" \
  --symlink-install \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
