# Navigation


# EKF Test with `robot_localization`

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


You don’t need to add robot_localization to this repo; it’s installed system-wide.

Package Structure

Approximate layout of the ekf_test package:

ekf_test/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── ekf.yaml
├── launch/
│   └── ekf_test.launch.py
└── src/
    └── ekf_publisher.cpp