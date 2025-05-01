from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
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

    # robot description node to get the urdf onto /robot_description
    # foxglove will need this if you want the model available
    urdf_path = os.path.join(
        get_package_share_directory('tank_description'),
        'urdf',
        'robot.urdf.xacro'
    )
    robot_description_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'xacro ',
                PathJoinSubstitution([
                    FindPackageShare('tank_description'),
                    'urdf',
                    'robot.urdf.xacro'
                ])
            ])
        }]
    )

    return LaunchDescription([
        joystick_launch,
        foxglove_bridge_node,
        drive_controller_node,
        robot_description_node
    ])
