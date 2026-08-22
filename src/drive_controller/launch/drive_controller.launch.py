import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('drive_controller'),
        'config',
        'drive_controller.yaml',
    )
    return LaunchDescription([
        Node(
            package='drive_controller',
            executable='drive_controller',
            name='drive_controller',
            output='screen',
            parameters=[params_file],
        )
    ])
