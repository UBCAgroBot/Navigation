from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("sensor_fusion_launch"),
        "config",
        "ekf.yaml"
    ])

    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[config],
            remappings=[
                # only needed if your topics differ
                # ("/wheel/odometry", "/your/odom/topic"),
                # ("/imu/data", "/your/imu/topic"),
            ],
        )
    ])
