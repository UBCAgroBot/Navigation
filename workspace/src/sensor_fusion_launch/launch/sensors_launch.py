import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Lidar
    lidar_package = 'rslidar_sdk'
    lidar_launch = 'start.py'


    lidar_launch_path = PathJoinSubstitution([
        FindPackageShare(lidar_package),
        'launch',
        lidar_launch
    ])


    lidar_config = PathJoinSubstitution([
        FindPackageShare(lidar_package),
        'config_files',
        'rslidar_config.yaml'
    ])


    # IMU
    imu_package = 'phidgets_spatial'
    imu_launch = 'spatial-launch.py'

    imu_launch_path = PathJoinSubstitution([
        FindPackageShare(imu_package),
        'launch',
        imu_launch
    ])


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path),
            # You can also pass arguments to the included launch file
            launch_arguments={
                'config_file': lidar_config
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch_path),
            # You can also pass arguments to the included launch file
            launch_arguments={
                
            }.items()
        ),
    ])