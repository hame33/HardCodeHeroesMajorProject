from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auto_rotate',
            executable='rotate_after_distance',
            name='rotate_after_distance',
            output='screen',
            parameters=[{'target_distance': 2.0, 'rotation_speed': 0.5}]
        )
    ])
