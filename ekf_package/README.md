# Simulating straight-line motion with EKF from `robot_localization`

This package (`ekf_test`) sets up a simple **sensor fusion pipeline** in ROS 2 using the
[`robot_localization`](https://github.com/cra-ros-pkg/robot_localization) package.

It creates **fake IMU and odometry data**, fuses them with an **Extended Kalman Filter (EKF)**,
and visualizes the fused pose in **RViz**.  
The goal is to verify that the EKF output follows a **perfect straight line** when given
ideal sensor inputs.

---

## Task Overview

1. Install and configure the `robot_localization` ROS 2 package.
2. Create an EKF parameter YAML file that:
   - Subscribes to `/imu` and `/odom`.
   - Uses IMU for orientation / angular velocity.
   - Uses odometry for forward velocity / position.
3. Implement a fake sensor publisher node (`ekf_publisher.cpp`) that:
   - Publishes IMU messages at 100 Hz.
   - Publishes odometry messages at a fixed linear speed in a straight line.
4. Run the EKF node from `robot_localization` with the YAML config.
5. Visualize `/odometry/filtered` in RViz and verify straight-line motion.

---

## Dependencies

- ROS 2 (Galactic / Humble / Iron, etc. depending on your environment)
- `robot_localization` package
- `rviz2`

Install `robot_localization` (inside your ROS 2 environment):

```bash
sudo apt install ros-${ROS_DISTRO}-robot-localization
```

You don’t need to add robot_localization to this repo; it’s installed system-wide.

## Package Structure

Approximate layout of the ekf_test package:
```bash
ekf_test/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── ekf.yaml
├── launch/
│   └── ekf_test.launch.py
└── src/
    └── ekf_publisher.cpp
```

---

## EKF Configuration (ekf.yaml)

The EKF configuration file:

- Defines two inputs:
  - `odom0` – Odometry input (e.g., `/odom`)
  - `imu0` – IMU input (e.g., `/imu`)

- Specifies which fields to trust from each sensor:
  - IMU: orientation + angular velocity (no linear acceleration in this simple test).
  - Odometry: forward linear velocity + pose in the x direction.
- Ensures the topics match what the publisher node is using.

High-level idea (not full YAML):

```bash
ekf_filter_node:
  ros__parameters:
    frequency: 50.0

    # Odometry input
    odom0: /odom
    odom0_config: [true,  true,  false,
                   false, false, true,
                   true,  true,  false,
                   false, false, false,
                   false, false]
    odom0_differential: false
    odom0_queue_size: 10

    # IMU input
    imu0: /imu
    imu0_config: [false, false, false,
                  true,  true,  true,
                  false, false, false,
                  false, false, false,
                  false, false]
    imu0_differential: false
    imu0_queue_size: 10

    # Frames (example)
    base_link_frame: base_link
    odom_frame: odom
    world_frame: odom
```
Adjust the actual *_config arrays to match your exact setup – this is just a sketch.

---

## Fake Sensor Publisher (ekf_publisher.cpp)

The ekf_publisher node simulates a robot moving in a straight line at a constant speed.

Behavior

- IMU (/imu topic):
 - Published at 100 Hz.
 - Orientation: maintains a constant heading.
 - Angular velocity: 0 (no rotation).
 - Linear acceleration: 0 (constant velocity, so no acceleration).
- Odometry (/odom topic):

Published at a lower but steady rate (e.g., 30–50 Hz).
- Linear velocity: constant (e.g., 1.0 m/s along +x).
- Angular velocity: 0.
- Pose: x increases linearly over time, y = 0, heading constant.

The node typically:
1. Stores a start time.
2. Computes position as x = v * t.
3. Fills out nav_msgs/msg/Odometry and sensor_msgs/msg/Imu messages.
4. Publishes them on /odom and /imu.

---

## Building the Package

From your workspace root (e.g., ~/workspace/ekf_package):

```bash 
colcon build
source install/setup.bash

```
--- 

## Running the System
## 1. Run the Fake Sensor Publisher
```ros2 run ekf_test ekf_publisher```


This publishes:
```bash
/odom
/imu
```
## 2. Run robot_localization EKF Node with YAML Parameters

Using an absolute path example:
```bash
ros2 run robot_localization ekf_node \
  --ros-args \
  --params-file /home/vscode/workspace/ekf_package/ekf_test/config/ekf.yaml
```

Or (once your package is properly installed):
```bash
ros2 run robot_localization ekf_node \
  --ros-args \
  --params-file $(ros2 pkg prefix ekf_test)/share/ekf_test/config/ekf.yaml
```
## 3. Visualize in RViz

Start RViz:

```rviz2```


## In RViz:

1. Add a display for Odometry or Pose and set the topic to:
- ```/odometry/filtered```

2. Set:
- Global Frame: odom
- Reference / Fixed Frame: odom (since that’s the frame used by the EKF).

3. For the camera:
- Set the Target Frame to base_link so the camera tracks the robot arrow.

---


## Validation: Straight-Line Check

With everything running:
- The ```/odometry/filtered``` topic should show a pose that moves in a straight line along the x-axis.
- The arrow in RViz should:
- Move forward at constant speed.
- Maintain constant orientation (no rotation, no drift in y).
If the output drifts or curves:
- Check that:
    - The IMU and odometry topics and frame IDs match what’s in ekf.yaml.
    - The correct fields are enabled in odom0_config and imu0_config.
    - The timestamps are being set correctly in your publisher.

---

## Useful ROS 2 Commands

List all topics:
```ros2 topic list```

Echo the fused output:
```ros2 topic echo /odometry/filtered```

Check the fake sensor topics:
```bash
ros2 topic echo /odom
ros2 topic echo /imu```
