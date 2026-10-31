# 🌽 AgroBot Navigation

Gazebo simulation + navigation nodes for autonomous row traversal in a virtual maize field.  
Built with **ROS 2 Humble**.

---

## ✅ What You Need

- [Docker Desktop](https://www.docker.com/products/docker-desktop/)
- [VS Code](https://code.visualstudio.com/) with the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension
- An **X11 server** for GUI windows (VcXsrv on Windows, XQuartz on macOS)
- *(Optional)* NVIDIA GPU + [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) — only needed for the ZED camera container

---

## 🚀 Quick Start

### Step 1 — Clone + Open the Dev Container

```bash
git clone --recurse-submodules git@github.com:UBCAgroBot/Navigation.git
```

> Already cloned? Run `git submodule update --init --recursive` to pull in
> the RoboSense LiDAR driver.

Open this folder in VS Code. It will prompt you to reopen in a container.

| Pick this container | When you want to… |
|---|---|
| **ROS** | Run the simulation (no GPU needed) |
| **ZED_ROS** | Work with the physical ZED camera (needs NVIDIA GPU) |

> **Linux users:** allow X11 access before opening the container:
> ```bash
> xhost +local:docker
> ```

---

### Step 2 — Build

```bash
cd /home/vscode/workspace
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> **Tip:** add the source command to your shell so you don't have to run it every time:
> ```bash
> echo "source /home/vscode/workspace/install/setup.bash" >> ~/.bashrc
> ```

---

### Step 3 — Generate a World

```bash
ros2 run virtual_maize_field generate_world
```

This creates a maize field world file at `~/.ros/virtual_maize_field/generated.world`.

---

### Step 4 — Launch the Simulation

**Terminal 1** — start Gazebo + navigation:

```bash
ros2 launch my_robot_description simulation.launch.py
```

**Terminal 2** — spawn the robot:

```bash
ros2 launch my_robot_description spawn_sdf_robot.launch.py
```

That's it — you should see the robot in the field. 🎉

To spawn a newer robot revision instead:

```bash
ros2 launch rev4_description spawn_rev4.launch.py
ros2 launch rev3_description spawn.launch.py
```

<details>
<summary><b>Optional launch arguments</b></summary>

```bash
ros2 launch my_robot_description simulation.launch.py paused:=True headless:=True
```

| Argument | Default | What it does |
|---|---|---|
| `use_sim_time` | `True` | Use the simulation clock |
| `paused` | `False` | Start Gazebo paused |
| `headless` | `False` | Run without the Gazebo GUI |
| `world_path` | `~/.ros/virtual_maize_field/` | Folder with the `.world` file |
| `world_name` | `generated.world` | World filename |

</details>

---

## 🧭 Running Navigation Nodes

The simulation launch already starts the **gap follower** by default.  
To run other nodes instead, use the commands below.

### Wall Follower (LiDAR)

Follows the left wall using a PD controller on raw LiDAR scans.

```bash
ros2 launch nav_algorithms wall_follower_launch.py
```

| Argument | Default | What it does |
|---|---|---|
| `type` | `lidar` | `lidar` runs `wall_follower_lidar`; `map` runs the map-based `wall_follower` *(WIP)* |
| `fake_scan` | `false` | `true` also starts `demo_lidar` to publish synthetic `/scan` data — only use without Gazebo |

### Gap Follower

Finds the largest open gap in the scan and steers toward it.

```bash
ros2 launch nav_algorithms gap_follower_launch.py
```

### Single Node (no launch file)

```bash
ros2 run nav_algorithms wall_follower_lidar
ros2 run nav_algorithms gap_follower_node
ros2 run nav_algorithms gap_follower_ackermann   # PD + safety-bubble gap follower (Ackermann /drive)
ros2 run nav_algorithms demo_lidar               # fake LiDAR data for testing without Gazebo
ros2 run nav_algorithms explorer                 # frontier exploration (needs Nav2 + a map)
```

---

## 📦 Node Reference

| Node | What it does | Publishes to |
|---|---|---|
| `wall_follower_lidar` | PD wall follower on raw LiDAR | `/drive` (Ackermann) |
| `gap_follower_node` | Reactive gap finder | `/cmd_vel` (Twist) + `/steer_fl`, `/steer_fr` |
| `gap_follower_ackermann` | Gap finder with safety bubbles, disparity extension, PD steering | `/drive` (Ackermann) |
| `demo_lidar` | Synthetic LiDAR publisher for offline tests | `/scan` |
| `explorer` | Frontier-based exploration via Nav2 | Nav2 action goals |
| `wall_follower` | Map-based wall follower *(WIP — not functional yet)* | Nav2 action goals |
| `motor_bridge` | Sends drive commands to the STM32 over serial and checks its acks | serial (`/dev/ttyACM0`) |

---

## 🔌 Talking to the STM32

`motor_bridge` forwards `/cmd_vel` + `/steer_fl`/`/steer_fr` (what `gap_follower_node` publishes)
to the STM32 at 20 Hz over the Nucleo-G474RE's ST-LINK USB serial port. The STM32 answers every
command with an ack echoing the values it applied. If commands stop for 0.5 s the bridge sends
zero speed; the firmware also stops the drive motor after 300 ms without a command.

The firmware is in `firmware/motor_control` (STM32CubeIDE project for the Nucleo-G474RE; protocol
in `Core/Inc/cmd_link.h`). It ships with `CMD_LINK_DRIVE_MOTORS 0`: commands are acknowledged but
nothing moves. Import that folder into STM32CubeIDE to build and flash it. Don't regenerate code
from the `.ioc` file: `main.c` has hand edits that CubeMX would overwrite.

```bash
# STM32 plugged in by USB and flashed
ros2 run motor_bridge motor_bridge                                    # T1
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"   # T2
```

T1 should print `STM32 link up` and then `ack seq=… speed=500mm/s …`. Parameters: `port`
(default `/dev/ttyACM0`), `baud`, `rate_hz`, `cmd_timeout`, `command_source`
(`twist_steer` or `ackermann` for `/drive`), `dry_run` (log lines instead of opening the port).

Message format (ASCII, XOR checksum of the bytes between `$` and `*`):

| Direction | Line |
|---|---|
| Jetson → STM32 | `$C,<seq>,<speed_mm_s>,<steer_mrad>*<CS>` |
| STM32 → Jetson | `$A,<seq>,<speed_mm_s>,<steer_mrad>,<flags>*<CS>` (flags: bit 0 motors enabled, bit 1 clamped) |
| STM32 → Jetson | `$E,<code>*<CS>` (1 bad checksum, 2 bad format, 3 line too long) |

---

## 📂 Project Layout

```
Navigation/
├── .devcontainer/
│   ├── ROS/                      ← simulation-only container
│   └── ZED_ROS/                  ← ZED camera + GPU container
├── containers/                   ← ROS-Dev.Dockerfile + install scripts
├── docs/SETUP_WSL.md             ← native WSL2 setup (no Docker)
├── scripts/setup_wsl.sh          ← installs ROS 2 Humble + Gazebo Fortress
├── firmware/
│   └── motor_control/            ← STM32G474 motor control firmware (STM32CubeIDE project)
└── src/                          ← colcon workspace: every ROS package lives here
    ├── my_robot_description/     ← sim launch + original four-wheel robot model
    ├── rev3_description/         ← rev3 robot model + spawn launch
    ├── rev4_description/         ← rev4 robot model + spawn launch
    ├── nav_algorithms/           ← navigation nodes (Python)
    ├── motor_bridge/             ← ROS → STM32 serial link
    ├── virtual_maize_field/      ← maize field world generator (vendored, see below)
    └── drivers/
        ├── rslidar_sdk/          ← submodule: RoboSense LiDAR driver
        └── rslidar_msg/          ← submodule: RoboSense message types
```

### Vendored `virtual_maize_field`

`src/virtual_maize_field` is a copy of [FieldRobotEvent/virtual_maize_field](https://github.com/FieldRobotEvent/virtual_maize_field)
(`ros2-ign` branch, GPLv3), added with `git subtree --squash` so we can edit it in place.
Our changes are marked with `AgroBot:` comments. To pull upstream changes:

```bash
git subtree pull --prefix=src/virtual_maize_field --squash \
  https://github.com/FieldRobotEvent/virtual_maize_field.git ros2-ign
```

This replaces the old `UBCAgroBot/navigation_gazebo` fork.

### Other branches

Some remote branches hold separate projects rather than feature work on this code:

| Branch | Contents |
|---|---|
| `Motorcontrols` | STM32G474 motor control firmware — now in `firmware/motor_control` |
| `sensor_fusion_main`, `sensor_fusion_4`, `sensor_fusion_fastlio`, `sensor-fusion3` | FAST-LIO / EKF sensor fusion workspaces |
| `path-planning` | Early Nav2 experiments (superseded by `src/nav_algorithms`) |

---

## 📡 Key ROS Topics

| Topic | Message Type | Used by |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | wall follower, gap follower, demo lidar |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | wall follower (LiDAR) |
| `/cmd_vel` | `geometry_msgs/Twist` | gap follower |
| `/steer_fl`, `/steer_fr` | `std_msgs/Float64` | gap follower |
| `/map` | `nav_msgs/OccupancyGrid` | explorer, map-based wall follower |
| `/odom` | `nav_msgs/Odometry` | bridged from Gazebo |
| `/imu` | `sensor_msgs/Imu` | bridged from Gazebo |
| `/clock` | `rosgraph_msgs/Clock` | simulation time sync |