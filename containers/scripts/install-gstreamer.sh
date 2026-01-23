#!/usr/bin/env bash
set -euox pipefail

echo 'installing gstreamer'

apt update && apt install -y libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-libav \
    gstreamer1.0-tools

apt install -y libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly