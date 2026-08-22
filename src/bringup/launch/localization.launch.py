"""Start Nav2 map-server and AMCL localization for a saved occupancy map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("bringup")
    nav2_dir = get_package_share_directory("nav2_bringup")

    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    pose_file = LaunchConfiguration("pose_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            description="Absolute path to the saved occupancy-map YAML file",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(bringup_dir, "config", "nav2_params.yaml"),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "pose_file",
            default_value=os.path.expanduser("~/.ros/robopi/localization_pose.json"),
        ),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, "launch", "localization_launch.py")
            ),
            launch_arguments={
                "map": map_file,
                "params_file": params_file,
                "use_sim_time": use_sim_time,
                "autostart": "true",
            }.items(),
        ),
        Node(
            package="bringup",
            executable="localization_pose_manager.py",
            name="localization_pose_manager",
            output="screen",
            parameters=[{
                "pose_file": pose_file,
                "initial_pose_x": LaunchConfiguration("initial_pose_x"),
                "initial_pose_y": LaunchConfiguration("initial_pose_y"),
                "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
            }],
        ),
    ])
