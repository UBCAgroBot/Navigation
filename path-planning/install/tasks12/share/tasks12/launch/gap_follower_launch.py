from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    type = LaunchConfiguration('type')

    return LaunchDescription([
        Node(
            package='tasks12',
            executable='gap_follower_node',
            name='gap_follower_node',
            output='screen',
        ),
    ])