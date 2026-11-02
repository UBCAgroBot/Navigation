from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

SDF_PATH = os.path.join(
    get_package_share_directory('rev4_description'), 'models', 'rev4', 'rev4.sdf'
)

with open(SDF_PATH, 'r') as infp:
    robot_desc = infp.read()

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'sdf_path',
            default_value=SDF_PATH,
            description='Absolute path to robot SDF file'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', LaunchConfiguration('sdf_path'),
                '-name', 'rev4',
                '-z', '1',
                '-x', '6.9',
                '-y', '-8',
                '-R', '0.0',
                '-P', '0.0',
                '-Y', '1.5708'
            ],
            output='screen'
        ),

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
        )
    ])
