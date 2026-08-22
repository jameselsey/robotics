from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_yaw = LaunchConfiguration('laser_yaw')

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
        parameters=[{
            'port': 8765,
            'use_compression': True,
            'max_qos_depth': 1,
            'send_buffer_limit_bytes':67108864,
            # Expose only these topics to Foxglove (ECMAScript regex)
            "topic_whitelist": [
                r"^(.*/)?camera_info$",                                
                r"^(.*/)?cmd_vel$",
                r"^(.*/)?image_viz/compressed$",
                r"^(.*/)?joint_states$",
                r"^(.*/)?goal_pose$",
                r"^(.*/)?amcl_pose$",
                r"^(.*/)?particle_cloud$",
                r"^(.*/)?plan$",
                r"^(.*/)?local_costmap/.*$",
                r"^(.*/)?global_costmap/.*$",
                r"^(.*/)?slam_toolbox/.*$",
                r"^(.*/)?map_metadata$",
                r"^(.*/)?map$",
                r"^(.*/)?joy$",
                r"^(.*/)?odom$",
                r"^(.*/)?robot_description$",
                r"^(.*/)?scan$",
                r"^(.*/)?speech_input$",
                r"^(.*/)?speech_output$",
                r"^(.*/)?tf$",
                r"^(.*/)?tf_static$",
                r"^(.*/)?visualization_marker_array$",
            ],
        }]
    )

    # Drive controller node
    drive_controller_params = os.path.join(
        get_package_share_directory('drive_controller'), 'config', 'drive_controller.yaml'
    )
    drive_controller_node = Node(
        package='drive_controller',
        executable='drive_controller',
        name='drive_controller',
        output='screen',
        parameters=[drive_controller_params],
    )

    # robot description node to get the urdf onto /robot_description
    # foxglove will need this if you want the model available
    urdf_path = os.path.join(
        get_package_share_directory('tank_description'),
        'urdf',
        'robot.urdf'
    )
    robot_description_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_path).read()
        }]
    )
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '--x', laser_x,
            '--y', laser_y,
            '--z', laser_z,
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', laser_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
        output='screen',
    )

    senses_share_dir = get_package_share_directory('senses')
    eyes_launch_path = os.path.join(senses_share_dir, 'launch', 'senses.launch.py')
    bringup_share_dir = get_package_share_directory('bringup')
    slam_launch_path = os.path.join(bringup_share_dir, 'launch', 'slam.launch.py')
    nav2_launch_path = os.path.join(bringup_share_dir, 'launch', 'nav2.launch.py')
    localization_launch_path = os.path.join(
        bringup_share_dir, 'launch', 'localization.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_navigation',
            default_value='false',
            description='Start Nav2 navigation servers as part of all.launch.py',
        ),
        DeclareLaunchArgument(
            'use_saved_map',
            default_value='false',
            description='Use map-server and AMCL instead of online SLAM mapping',
        ),
        DeclareLaunchArgument(
            'saved_map_file',
            default_value=os.path.expanduser('~/robotics/maps/house.yaml'),
            description='Occupancy-map YAML used when use_saved_map is true',
        ),
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.16'),
        DeclareLaunchArgument('laser_yaw', default_value='3.141592653589793'),
        DeclareLaunchArgument(
            'localization_pose_file',
            default_value=os.path.expanduser('~/.ros/robopi/localization_pose.json'),
            description='Runtime file used to restore the last AMCL pose',
        ),
        DeclareLaunchArgument('initial_pose_x', default_value='-0.819'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.823'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='1.221'),
        joystick_launch,
        foxglove_bridge_node,
        drive_controller_node,
        robot_description_node,
        joint_state_pub,
        base_to_laser_tf,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(eyes_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            condition=UnlessCondition(LaunchConfiguration('use_saved_map')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch_path),
            condition=IfCondition(LaunchConfiguration('use_saved_map')),
            launch_arguments={
                'map': LaunchConfiguration('saved_map_file'),
                'pose_file': LaunchConfiguration('localization_pose_file'),
                'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
            }.items(),
        ),
        TimerAction(
            period=8.0,
            condition=IfCondition(LaunchConfiguration('enable_navigation')),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_path),
            )],
        ),
    ])
