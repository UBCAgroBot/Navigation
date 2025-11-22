from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ekf_test',
            executable='ekf_publisher',
            name='ekf_publisher'
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=['config/ekf.yaml']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', ''],
            output='screen'
        )
    ])