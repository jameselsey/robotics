from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the joystick launch file
    joystick_launch_path = os.path.join(
        get_package_share_directory('joystick'),
        'launch',
        'joystick.launch.py'
    )

    # Create an inclusion action for the joystick launch
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joystick_launch_path)
    )

    # Define the foxglove_bridge node
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'port': 8765}]
    )

    # Drive controller node
    drive_controller_node = Node(
        package='drive_controller',
        executable='drive_controller',
        name='drive_controller',
        output='screen'
    )

    return LaunchDescription([
        joystick_launch,
        foxglove_bridge_node,
        drive_controller_node
    ])
