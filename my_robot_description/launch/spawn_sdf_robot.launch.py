from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    sdf_default = os.path.expanduser(
        '~/ros2_ws/src/my_robot_description/models/my_robot/model.sdf'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sdf_path',
            default_value=sdf_default,
            description='Absolute path to robot SDF file'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', LaunchConfiguration('sdf_path'),
                '-name', 'my_robot',
                '-z', '0.5',
                '-x', '0',
                '-y', '-11.25'
            ],
            output='screen'
        )
    ])

