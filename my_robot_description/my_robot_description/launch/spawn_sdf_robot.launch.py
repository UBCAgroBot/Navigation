from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

with open('/home/lucas/ros2_ws/src/my_robot_description/models/my_robot/model.sdf', 'r') as infp:
    robot_desc = infp.read()

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
                '-z', '1',
                '-x', '-1.2',
                '-y', '-11',
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
