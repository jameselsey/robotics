from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    image_in = LaunchConfiguration("image_in")
    image_viz = LaunchConfiguration("image_viz")
    image_rate = LaunchConfiguration("image_viz_rate_hz")

    return LaunchDescription(
        [
            # Declare args (must be part of the LaunchDescription)
            DeclareLaunchArgument("image_in", default_value="/image_raw/compressed"),
            DeclareLaunchArgument("image_viz", default_value="/image_viz/compressed"),
            DeclareLaunchArgument("image_viz_rate_hz", default_value="3.0"),
            DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
            # Camera
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="v4l2_camera_node",
                parameters=[
                    {
                        "video_device": ParameterValue(
                            LaunchConfiguration("camera_device"), value_type=str
                        )
                    }
                ],
            ),
            # Throttle <mode> <in> <rate> <out>
            Node(
                package="topic_tools",
                executable="throttle",
                name="image_throttle",
                arguments=["messages", image_in, image_rate, image_viz],
                output="screen",
            ),
        ]
    )
