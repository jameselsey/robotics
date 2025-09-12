from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    image_in   = LaunchConfiguration("image_in")
    image_viz  = LaunchConfiguration("image_viz")
    image_rate = LaunchConfiguration("image_viz_rate_hz")

    return LaunchDescription([
        # Declare args (must be part of the LaunchDescription)
        DeclareLaunchArgument("image_in",          default_value="/image_raw/compressed"),
        DeclareLaunchArgument("image_viz",         default_value="/image_viz/compressed"),
        DeclareLaunchArgument("image_viz_rate_hz", default_value="3.0"),

        # Camera
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="v4l2_camera_node",
        ),

        # Your node
        Node(
            package="senses",
            executable="eyes",
            name="eyes",
        ),

        # Throttle <mode> <in> <rate> <out>
        Node(
            package="topic_tools",
            executable="throttle",
            name="image_throttle",
            arguments=["messages", image_in, image_rate, image_viz],
            output="screen",
        ),
    ])
