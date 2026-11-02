# Complete Setup Guide
### Gazebo + ROS 2 Simulation via WSL2

---

## Why WSL2 with Ubuntu 22.04?

- **ROS 2 Humble** only supports **Ubuntu 22.04 (Jammy)**
- Your existing WSL distro may be a different Ubuntu version
- WSL2 supports **multiple distros side by side** — install 22.04 without touching your existing setup
- WSL2 includes **WSLg** — GUI apps (like Gazebo) appear directly as Windows windows

> [!IMPORTANT]
> ROS 2 and Gazebo are **not** downloaded from a website.
> They're installed via `apt` from official ROS repositories — like an app store for Linux.

---
---

## Phase 1 — Install Ubuntu 22.04 on WSL2

### 1.1) Install the distro

Open **PowerShell** and run:

```powershell
wsl --update
wsl --install Ubuntu-22.04
```

This installs Ubuntu 22.04 as a **new, separate** WSL2 distro. Your existing distros stay untouched.

---

### 1.2) Verify

```powershell
wsl -l -v
```

You should see `Ubuntu-22.04` with VERSION `2`:

```
  NAME              STATE       VERSION
  Ubuntu-22.04      Running     2
```

---

### 1.3) Enter the distro

```powershell
wsl -d Ubuntu-22.04
```

> Everything from here runs **inside WSL2 Ubuntu 22.04**.

---

### 1.4) Confirm Ubuntu version

```bash
lsb_release -a
```

Should show `Jammy` and `22.04`.

---
---

## Phase 2 — Clone the Repo

Clone **inside the WSL filesystem** (not under `/mnt/c/`) — builds are much faster and symlinks work.
`--recurse-submodules` pulls in the RoboSense LiDAR driver.

```bash
git clone --recurse-submodules git@github.com:UBCAgroBot/Navigation.git ~/Navigation
cd ~/Navigation
```

Should show `src/`, `scripts/`, `docs/`, etc.

> [!NOTE]
> Already cloned without submodules? Run `git submodule update --init --recursive`.

---
---

## Phase 3 — Install ROS 2 Humble + Gazebo

> [!TIP]
> **This entire phase is automated.** Run the setup script:
> ```bash
> chmod +x ~/Navigation/scripts/setup_wsl.sh
> ~/Navigation/scripts/setup_wsl.sh
> ```
> Then skip ahead to **Phase 4**. The steps below are the manual version if you prefer.

### 3.1) Set up locale

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

---

### 3.2) Add the ROS 2 apt repository

```bash
sudo apt update
sudo apt install -y curl gnupg2 lsb-release software-properties-common
sudo add-apt-repository universe -y
```

> [!WARNING]
> Skip this step if ROS 2 is already installed (`ls /etc/apt/sources.list.d/ | grep ros`).
> Adding the repo twice breaks `apt` with *"Conflicting values set for option Signed-By"* —
> fix that with `sudo rm /etc/apt/sources.list.d/ros2.list`.

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

---

### 3.3) Install ROS 2 Humble Desktop

```bash
sudo apt update
sudo apt install -y ros-humble-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-argcomplete \
  ros-dev-tools
```

> [!NOTE]
> Takes **10–20 minutes**. Downloads ~1.5 GB.

---

### 3.4) Install Gazebo + bridge packages

```bash
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ackermann-msgs \
  ros-humble-nav2-msgs \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher
```

> [!WARNING]
> **Gazebo Classic vs Ignition Gazebo** — two different simulators:
> - `gazebo` (v11) = **Gazebo Classic** — old, **not used by this codebase**
> - `ign gazebo` (v6) = **Ignition Gazebo Fortress** — **this is what we use**
>
> The `gz sim` command only exists in newer versions (Garden+). On Humble, always use `ign gazebo`.

---

### 3.5) Initialize rosdep + auto-source ROS 2

```bash
sudo rosdep init || true
rosdep update

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 3.6) Verify

```bash
ros2 --help
ign gazebo --version
```

Should show ROS 2 commands and Gazebo version `6.x.x`.

---
---

## Phase 4 — Build the ROS 2 Workspace

The repo **is** the workspace — every package lives under `src/`.

### 4.1) Install dependencies + build

```bash
cd ~/Navigation
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

---

### 4.2) Source workspace + set Gazebo resource path

```bash
echo "source ~/Navigation/install/setup.bash" >> ~/.bashrc
echo 'export IGN_GAZEBO_RESOURCE_PATH=$HOME/.ros/virtual_maize_field:$IGN_GAZEBO_RESOURCE_PATH' >> ~/.bashrc
source ~/.bashrc
```

> [!IMPORTANT]
> Sourcing `install/setup.bash` adds the maize crop models (`model://maize_01`, etc.) to
> `IGN_GAZEBO_RESOURCE_PATH` automatically. The extra line adds the folder holding the
> generated world's assets. Without both, the world file fails to load.

---
---

## Phase 5 — Generate the World

```bash
ros2 run virtual_maize_field generate_world
```

This creates the world file + supporting files at `~/.ros/virtual_maize_field/generated.world`.

---
---

## Phase 6 — Launch the Simulation

### 6.1) Start Gazebo + navigation

```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch my_robot_description simulation.launch.py
```

> [!NOTE]
> `LIBGL_ALWAYS_SOFTWARE=1` forces software rendering for WSLg compatibility.
> For headless mode (no GUI, faster): add `headless:=True`

> [!IMPORTANT]
> Wait for Gazebo to fully load before spawning the robot.
> Look for output from `[ign gazebo-1]` confirming the world loaded.

This starts:
- Gazebo with the generated maize field
- ROS↔Gazebo bridges (clock, IMU, LiDAR, odometry, cmd_vel, steering)
- Gap follower node (default navigation algorithm)

---

### 6.2) Spawn the robot *(separate terminal)*

```bash
wsl -d Ubuntu-22.04
ros2 launch my_robot_description spawn_sdf_robot.launch.py
```

---

### 6.3) Verify topics *(separate terminal)*

```bash
wsl -d Ubuntu-22.04
ros2 topic list
```

```bash
ros2 topic echo /scan --once
```

```bash
ros2 topic echo /odom --once
```

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

If you see data → everything works. 🎉

---
---

## Quick Reference

| Task | Command |
|------|---------|
| Enter WSL | `wsl -d Ubuntu-22.04` |
| List distros | `wsl -l -v` |
| Rebuild | `cd ~/Navigation && colcon build --symlink-install` |
| Sim (GUI) | `export LIBGL_ALWAYS_SOFTWARE=1 && ros2 launch my_robot_description simulation.launch.py` |
| Sim (headless) | `ros2 launch my_robot_description simulation.launch.py headless:=True` |
| Spawn robot | `ros2 launch my_robot_description spawn_sdf_robot.launch.py` |
| List topics | `ros2 topic list` |
| Inspect topic | `ros2 topic echo /scan` |

---
---

## What's Next?

1. **Run existing controllers** — gap follower, wall follower
2. **Create a Gymnasium RL env** — wrap ROS 2 topics into `step()` / `reset()`
3. **Train an RL agent** — PPO, SAC, etc. with Stable-Baselines3

> [!TIP]
> For faster training, set `<real_time_factor>0.0</real_time_factor>` in your
> `generated.world` file to uncap simulation speed.
