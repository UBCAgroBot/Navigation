#!/usr/bin/env bash
set -ex

echo 'Installing standard Ubuntu 22.04 CMake for ROS 2 Humble'
export DEBIAN_FRONTEND=noninteractive

# Remove any custom Kitware repositories if they were added
rm -f /etc/apt/sources.list.d/kitware.list

apt-get update
# Install the stable version guaranteed to work with Humble
apt-get install -y --no-install-recommends cmake

# Cleanup
apt-get clean
rm -rf /var/lib/apt/lists/*

cmake --version