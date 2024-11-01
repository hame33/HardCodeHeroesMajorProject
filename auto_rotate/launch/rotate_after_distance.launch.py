from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auto_rotate',
            executable='rotate_after_distance',
            name='rotate_after_distance',
            output='screen'
        )
    ])
