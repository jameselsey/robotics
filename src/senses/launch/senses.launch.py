from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    senses_share_dir = get_package_share_directory('senses')
    eyes_launch_path = os.path.join(senses_share_dir, 'launch', 'eyes.launch.py')


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(eyes_launch_path)
        ),
        Node(
            package='senses',
            executable='ears',
            name='ears'
        ),
        Node(
            package='senses',
            executable='mouth',
            name='mouth'
        ),
        Node(
            package='senses',
            executable='brain',
            name='brain'
        ),
    ])
