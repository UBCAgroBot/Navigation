import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    lidar_package = 'rslidar_sdk'
    lidar_launch = 'start.py'


    other_launch_file_path = PathJoinSubstitution([
        FindPackageShare(lidar_package),
        'launch',
        lidar_launch
    ])


    lidar_config = PathJoinSubstitution([
        FindPackageShare(lidar_package),
        'config_files',
        'rslidar_config.yaml'
    ])


    return LaunchDescription([
         IncludeLaunchDescription(
            PythonLaunchDescriptionSource(other_launch_file_path),
            # You can also pass arguments to the included launch file
            launch_arguments={
                'config_file': lidar_config
            }.items()
        ),
    ])