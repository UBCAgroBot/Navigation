#!/usr/bin/env bash
set -euox pipefail

echo 'installing opencv'

apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
apt-get install -y libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev
apt-get install -y libv4l-dev v4l-utils qv4l2
apt-get install -y curl

# grab dbpkg from github repository and install