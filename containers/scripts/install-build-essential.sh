#!/usr/bin/env bash
set -euox pipefail

echo 'installing build-essential'

apt-get update -y && apt-get upgrade -y
apt-get install -y --no-install-recommends \
    sudo \
    build-essential \
    software-properties-common \
    apt-transport-https \
    ca-certificates \
    lsb-release \
    pkg-config \
    gnupg \
    git \
    gdb \
    wget \
    curl \
    nano \
    zip \
    unzip \
    time \
    sshpass \
    ssh-client \
    plocate \
    xz-utils \
    libnss3-tools \
    libx11-dev \
    xauth \
    apt-utils \
    dialog \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    locales \
    libpcap-dev

apt-get clean 
rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
gcc --version 
g++ --version