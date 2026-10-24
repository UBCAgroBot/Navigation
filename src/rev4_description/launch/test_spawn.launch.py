from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from virtual_maize_field import get_spawner_launch_file

SDF_PATH = os.path.join(
    get_package_share_directory('rev4_description'), 'models', 'rev4', 'rev4.sdf'
)

with open(SDF_PATH, 'r') as infp:
    robot_desc = infp.read()

def generate_launch_description() -> LaunchDescription:
    robot_spawner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_spawner_launch_file()]),
        launch_arguments={"robot_name": "<<rev4>>", "z": "{{1.0}}"}.items(),
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False, 'robot_description': robot_desc}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        robot_spawner_launch])
