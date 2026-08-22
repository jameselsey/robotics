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
    return LaunchDescription([
        DeclareLaunchArgument("slam_params_file", default_value=default_params),
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
