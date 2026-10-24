#!/bin/bash
# ============================================================================
#  AgroBot Setup Script
#  Installs ROS 2 Humble + Ignition Gazebo Fortress inside WSL2 (Ubuntu 22.04)
#
#  Run inside WSL2 Ubuntu 22.04:
#    chmod +x scripts/setup_wsl.sh
#    ./scripts/setup_wsl.sh
# ============================================================================

set -eo pipefail  # Exit on error; fail pipeline on first command error

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log()  { echo -e "${GREEN}[SETUP]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }

# ─────────────────────────────────────────────
#  1. Set up UTF-8 Locale
# ─────────────────────────────────────────────
log "Setting up UTF-8 locale..."
sudo apt-get update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq locales > /dev/null
sudo locale-gen en_US en_US.UTF-8 > /dev/null
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ─────────────────────────────────────────────
#  2. Add ROS 2 Apt Repository
# ─────────────────────────────────────────────
log "Adding ROS 2 apt repository..."
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq curl gnupg2 lsb-release software-properties-common > /dev/null
sudo add-apt-repository universe -y > /dev/null 2>&1

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ─────────────────────────────────────────────
#  3. Install ROS 2 Humble Desktop & Tools
# ─────────────────────────────────────────────
log "Installing ROS 2 Humble Desktop (this takes 10-20 min)..."
sudo apt-get update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-argcomplete \
  ros-dev-tools

# ─────────────────────────────────────────────
#  4. Initialize rosdep
# ─────────────────────────────────────────────
log "Initializing rosdep..."
sudo rosdep init 2>/dev/null || true
rosdep update

# ─────────────────────────────────────────────
#  5. Install Gazebo Fortress & Bridge Packages
# ─────────────────────────────────────────────
log "Installing Ignition Gazebo Fortress + ROS-Gazebo bridge..."
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ackermann-msgs \
  ros-humble-nav2-msgs \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher

# ─────────────────────────────────────────────
#  6. Configure Environment & WSL2 Settings
# ─────────────────────────────────────────────
log "Configuring ~/.bashrc with ROS 2 and WSL2 stability variables..."

if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
  echo "" >> ~/.bashrc
  echo "# ROS 2 Humble" >> ~/.bashrc
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  
  echo "# WSL2 ROS 2 Network Isolation (prevents dropped terminal discovery)" >> ~/.bashrc
  echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
  
  echo "# WSLg GUI Rendering Fixes (prevents Gazebo black screens & crashes)" >> ~/.bashrc
  echo "export QT_QPA_PLATFORM=xcb" >> ~/.bashrc
  echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
fi

# Source for current shell session
# Temporarily disable unbound variable checking if active, as ROS setup files rely on empty vars
set +u 2>/dev/null || true
source /opt/ros/humble/setup.bash

# ─────────────────────────────────────────────
#  7. Verification
# ─────────────────────────────────────────────
log "Verifying installations..."
if command -v ros2 &> /dev/null; then
  log "✓ ROS 2 Humble installed"
else
  warn "✗ ros2 command not found — check apt installation"
  exit 1
fi

if command -v ign &> /dev/null; then
  IGN_VERSION=$(ign gazebo --version 2>/dev/null | head -1 || echo "unknown")
  log "✓ Ignition Gazebo installed (${IGN_VERSION})"
else
  warn "ign command not found — installing ignition-fortress directly..."
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y ignition-fortress
fi

echo ""
echo "============================================"
log "✓ Setup complete!"
echo "============================================"
echo ""
echo "  ROS 2 Humble ............. installed"
echo "  Ignition Gazebo Fortress . installed"
echo "  ROS-Gazebo bridge ........ installed"
echo "  WSL2 GUI/Network flags ... configured"
echo ""
echo "  Next steps:"
echo "    1. Reload your shell: source ~/.bashrc"
echo "    2. Proceed to Phase 4 of docs/SETUP_WSL.md (workspace build)"
echo ""