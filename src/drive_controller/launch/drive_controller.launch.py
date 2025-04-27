from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drive_controller',
            executable='drive_controller',
            name='drive_controller',
            output='screen'
        )
    ])
