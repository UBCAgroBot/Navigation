from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    type = LaunchConfiguration('type')
    fake_scan = LaunchConfiguration('fake_scan')

    return LaunchDescription([
        DeclareLaunchArgument(
            'type',
            default_value='lidar',
            description="Wall follower to run: 'lidar' (PD on raw /scan) or 'map' (Nav2 goals from /map, WIP)",
        ),
        DeclareLaunchArgument(
            'fake_scan',
            default_value='false',
            description='Also start demo_lidar to publish synthetic /scan data (for testing without Gazebo)',
        ),
        Node(
            package='nav_algorithms',
            executable='wall_follower',
            name='wall_follower',
            output='screen',
            condition=IfCondition(PythonExpression(["'", type, "'", " == 'map'"])),
        ),
        Node(
            package='nav_algorithms',
            executable='wall_follower_lidar',
            name='wall_follower_lidar',
            output='screen',
            condition=IfCondition(PythonExpression(["'", type, "'", " == 'lidar'"])),
        ),
        Node(
            package='nav_algorithms',
            executable='demo_lidar',
            name='demo_lidar',
            output='screen',
            condition=IfCondition(fake_scan),
        ),
    ])
