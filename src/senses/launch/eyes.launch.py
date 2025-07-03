from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the v4l2_camera_node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            # parameters=[{
            #     'video_device': '/dev/video0',
            #     'pixel_format': 'YUYV',
            #     'output_encoding': 'rgb8',
            #     'image_size': [640, 480],
            # }],
            # remappings=[
            #     ('image_raw', '/eyes/image_raw')
            # ]
        ),
        # Launch the eyes node
        Node(
            package='senses',
            executable='eyes',
            name='eyes'
        )
    ])
