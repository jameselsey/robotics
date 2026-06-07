from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    senses_share_dir = get_package_share_directory('senses')
    eyes_launch_path = os.path.join(senses_share_dir, 'launch', 'eyes.launch.py')

    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='false')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    return LaunchDescription([
        DeclareLaunchArgument(
            'audio_device_name',
            default_value='Brio',
            description='Audio input device name to search for (case-insensitive substring match)'
        ),
        DeclareLaunchArgument('aws_profile', default_value='default'),
        DeclareLaunchArgument('aws_region', default_value='us-east-1'),
        DeclareLaunchArgument('nova_model_id', default_value='amazon.nova-2-sonic-v1:0'),
        DeclareLaunchArgument('nova_voice', default_value='amy'),
        DeclareLaunchArgument('vision_enabled', default_value='true'),
        DeclareLaunchArgument('vision_topic', default_value='/image_viz/compressed'),
        DeclareLaunchArgument('vision_model_id', default_value='amazon.nova-lite-v1:0'),
        DeclareLaunchArgument('endpointing_sensitivity', default_value='LOW'),
        DeclareLaunchArgument('idle_timeout_seconds', default_value='45.0'),
        DeclareLaunchArgument('max_session_seconds', default_value='420.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(eyes_launch_path)
        ),
        Node(
            package='senses',
            executable='voice_agent',
            name='voice_agent',
            output='screen',
            parameters=[{
                'input_device_name': LaunchConfiguration('audio_device_name', default='Brio'),
                'aws_profile': LaunchConfiguration('aws_profile'),
                'aws_region': LaunchConfiguration('aws_region'),
                'nova_model_id': LaunchConfiguration('nova_model_id'),
                'nova_voice': LaunchConfiguration('nova_voice'),
                'vision_enabled': LaunchConfiguration('vision_enabled'),
                'vision_topic': LaunchConfiguration('vision_topic'),
                'vision_model_id': LaunchConfiguration('vision_model_id'),
                'endpointing_sensitivity': LaunchConfiguration('endpointing_sensitivity'),
                'idle_timeout_seconds': LaunchConfiguration('idle_timeout_seconds'),
                'max_session_seconds': LaunchConfiguration('max_session_seconds'),
            }]
        ),
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),
        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),
        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type': channel_type,
                         'serial_port': serial_port,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                         'scan_mode': scan_mode}],
            output='screen'),
    ])
