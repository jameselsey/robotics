import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("bringup")
    default_params = os.path.join(bringup_dir, "config", "slam_toolbox.yaml")

    slam_params_file = LaunchConfiguration("slam_params_file")
    laser_x = LaunchConfiguration("laser_x")
    laser_y = LaunchConfiguration("laser_y")
    laser_z = LaunchConfiguration("laser_z")
    laser_yaw = LaunchConfiguration("laser_yaw")

    return LaunchDescription([
        DeclareLaunchArgument("slam_params_file", default_value=default_params),
        DeclareLaunchArgument("laser_x", default_value="0.0"),
        DeclareLaunchArgument("laser_y", default_value="0.0"),
        DeclareLaunchArgument("laser_z", default_value="0.16"),
        DeclareLaunchArgument("laser_yaw", default_value="0.0"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser_tf",
            arguments=[
                "--x", laser_x,
                "--y", laser_y,
                "--z", laser_z,
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", laser_yaw,
                "--frame-id", "base_link",
                "--child-frame-id", "laser",
            ],
            output="screen",
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params_file],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_slam",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["slam_toolbox"],
            }],
        ),
    ])
