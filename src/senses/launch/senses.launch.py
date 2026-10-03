import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    senses_share_dir = get_package_share_directory("senses")
    eyes_launch_path = os.path.join(senses_share_dir, "launch", "eyes.launch.py")
    default_rooms_config = os.path.join(senses_share_dir, "config", "rooms.yaml")

    channel_type = LaunchConfiguration("channel_type", default="serial")
    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyUSB0")
    serial_baudrate = LaunchConfiguration("serial_baudrate", default="460800")
    frame_id = LaunchConfiguration("frame_id", default="laser")
    inverted = LaunchConfiguration("inverted", default="false")
    angle_compensate = LaunchConfiguration("angle_compensate", default="true")
    scan_mode = LaunchConfiguration("scan_mode", default="Standard")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "audio_device_name",
                default_value="Brio",
                description="Audio input device name to search for (case-insensitive substring match)",
            ),
            DeclareLaunchArgument(
                "voice_params_file",
                default_value=os.path.join(senses_share_dir, "config", "voice_agent.yaml"),
            ),
            DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
            DeclareLaunchArgument("audio_output_device_name", default_value=""),
            DeclareLaunchArgument("audio_input_device_index", default_value="-1"),
            DeclareLaunchArgument("audio_output_device_index", default_value="-1"),
            DeclareLaunchArgument("oww_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("oww_port", default_value="10400"),
            DeclareLaunchArgument("aws_profile", default_value="default"),
            DeclareLaunchArgument("aws_region", default_value="us-east-1"),
            DeclareLaunchArgument("nova_model_id", default_value="amazon.nova-2-sonic-v1:0"),
            DeclareLaunchArgument("nova_voice", default_value="amy"),
            DeclareLaunchArgument("vision_enabled", default_value="true"),
            DeclareLaunchArgument("vision_topic", default_value="/image_viz/compressed"),
            DeclareLaunchArgument("vision_model_id", default_value="amazon.nova-lite-v1:0"),
            DeclareLaunchArgument(
                "rooms_config_path",
                default_value=default_rooms_config,
                description="Semantic room polygons and reviewed navigation poses",
            ),
            DeclareLaunchArgument("endpointing_sensitivity", default_value="LOW"),
            DeclareLaunchArgument("idle_timeout_seconds", default_value="45.0"),
            # Bedrock closes inactive bidirectional streams after about 295 seconds.
            # End our session first so audio and LED resources are cleaned up locally.
            DeclareLaunchArgument("max_session_seconds", default_value="240.0"),
            DeclareLaunchArgument(
                "enable_lidar",
                default_value="true",
                description="Start the sllidar node. Set false for desk testing.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(eyes_launch_path),
                launch_arguments={"camera_device": LaunchConfiguration("camera_device")}.items(),
            ),
            Node(
                package="senses",
                executable="voice_agent",
                name="voice_agent",
                output="screen",
                parameters=[
                    LaunchConfiguration("voice_params_file"),
                    {
                        "output_device_name": ParameterValue(
                            LaunchConfiguration("audio_output_device_name"), value_type=str
                        ),
                        "input_device_index": ParameterValue(
                            LaunchConfiguration("audio_input_device_index"), value_type=int
                        ),
                        "output_device_index": ParameterValue(
                            LaunchConfiguration("audio_output_device_index"), value_type=int
                        ),
                        "oww_host": ParameterValue(LaunchConfiguration("oww_host"), value_type=str),
                        "oww_port": ParameterValue(LaunchConfiguration("oww_port"), value_type=int),
                        "input_device_name": ParameterValue(
                            LaunchConfiguration("audio_device_name", default="Brio"), value_type=str
                        ),
                        "aws_profile": ParameterValue(
                            LaunchConfiguration("aws_profile"), value_type=str
                        ),
                        "aws_region": ParameterValue(
                            LaunchConfiguration("aws_region"), value_type=str
                        ),
                        "nova_model_id": ParameterValue(
                            LaunchConfiguration("nova_model_id"), value_type=str
                        ),
                        "nova_voice": ParameterValue(
                            LaunchConfiguration("nova_voice"), value_type=str
                        ),
                        "vision_enabled": ParameterValue(
                            LaunchConfiguration("vision_enabled"), value_type=bool
                        ),
                        "vision_topic": ParameterValue(
                            LaunchConfiguration("vision_topic"), value_type=str
                        ),
                        "vision_model_id": ParameterValue(
                            LaunchConfiguration("vision_model_id"), value_type=str
                        ),
                        "rooms_config_path": ParameterValue(
                            LaunchConfiguration("rooms_config_path"), value_type=str
                        ),
                        "endpointing_sensitivity": ParameterValue(
                            LaunchConfiguration("endpointing_sensitivity"), value_type=str
                        ),
                        "idle_timeout_seconds": ParameterValue(
                            LaunchConfiguration("idle_timeout_seconds"), value_type=float
                        ),
                        "max_session_seconds": ParameterValue(
                            LaunchConfiguration("max_session_seconds"), value_type=float
                        ),
                    },
                ],
            ),
            Node(
                package="senses",
                executable="room_markers",
                name="room_marker_publisher",
                output="screen",
                parameters=[
                    {
                        "rooms_config_path": ParameterValue(
                            LaunchConfiguration("rooms_config_path"), value_type=str
                        ),
                    }
                ],
            ),
            DeclareLaunchArgument(
                "channel_type",
                default_value=channel_type,
                description="Specifying channel type of lidar",
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value=serial_port,
                description="Specifying usb port to connected lidar",
            ),
            DeclareLaunchArgument(
                "serial_baudrate",
                default_value=serial_baudrate,
                description="Specifying usb port baudrate to connected lidar",
            ),
            DeclareLaunchArgument(
                "frame_id", default_value=frame_id, description="Specifying frame_id of lidar"
            ),
            DeclareLaunchArgument(
                "inverted",
                default_value=inverted,
                description="Specifying whether or not to invert scan data",
            ),
            DeclareLaunchArgument(
                "angle_compensate",
                default_value=angle_compensate,
                description="Specifying whether or not to enable angle_compensate of scan data",
            ),
            DeclareLaunchArgument(
                "scan_mode", default_value=scan_mode, description="Specifying scan mode of lidar"
            ),
            Node(
                package="sllidar_ros2",
                executable="sllidar_node",
                name="sllidar_node",
                respawn=True,
                respawn_delay=2.0,
                condition=IfCondition(LaunchConfiguration("enable_lidar")),
                parameters=[
                    {
                        "channel_type": ParameterValue(channel_type, value_type=str),
                        "serial_port": ParameterValue(serial_port, value_type=str),
                        "serial_baudrate": ParameterValue(serial_baudrate, value_type=int),
                        "frame_id": ParameterValue(frame_id, value_type=str),
                        "inverted": ParameterValue(inverted, value_type=bool),
                        "angle_compensate": ParameterValue(angle_compensate, value_type=bool),
                        "scan_mode": ParameterValue(scan_mode, value_type=str),
                    }
                ],
                output="screen",
            ),
        ]
    )
