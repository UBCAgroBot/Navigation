from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    type = LaunchConfiguration('type')

    return LaunchDescription([
        DeclareLaunchArgument(
            'type',
            default_value='lidar',
        ),
        Node(
            package='tasks12',
            executable='wall_follower',
            name='wall_follower',
            output='screen',
            condition=IfCondition(PythonExpression(["'", type, "'", " == 'map'"])),
        ),
        Node(
            package='tasks12',
            executable='wall_follower_lidar',
            name='wall_follower_lidar',
            output='screen',
            condition=IfCondition(PythonExpression(["'", type, "'", " == 'lidar'"])),
        ),
        Node(
            package='tasks12',
            executable='demo_lidar',
            name='demo_lidar',
            output='screen',
            condition=IfCondition(PythonExpression(["'", type, "'", " == 'lidar'"])),
        ),
    ])
