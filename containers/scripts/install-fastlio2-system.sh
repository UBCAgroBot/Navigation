#!/usr/bin/env bash
set -euox pipefail
export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y \
  cmake libpcl-dev libeigen3-dev \
  ros-humble-pcl-ros \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  python3-rosdep python3-dev

# Build & install Livox-SDK2 system-wide
cd /tmp
rm -rf Livox-SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
find . -name "CMakeLists.txt" -exec sed -i -E 's/VERSION ([0-2]\.[0-9]+|3\.[0-4])/VERSION 3.5/g' {} +
mkdir -p build && cd build
cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j"$(nproc)"
make install
ldconfig

# rosdep init is system-level (writes /etc)
rosdep init || true
