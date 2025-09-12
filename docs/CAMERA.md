# Cameras
I'm using the Logitech BRIO 100 camera, it's an inexpensive USB camera and includes an on-board mic.

The camera is compatible with the [v4l drivers](https://index.ros.org/r/v4l2_camera/#jazzy)

## Performance Improvements
If you install this package, it will automatically create a new topic called `/image_raw/compressed`

```
sudo apt-get install ros-${ROS_DISTRO}-image-transport-plugins
```

If we publish this as is, then we'll likely see this output on the foxglove bridge
```
foxglove_bridge-3] [WARN] [1757674243.951852798] [foxglove_bridge]: [WS] 192.168.0.18:59453: Send buffer limit reached
[foxglove_bridge-3] [WARN] [1757674248.558986138] [foxglove_bridge]: [WS] 192.168.0.18:59453: Send buffer limit reached
[foxglove_bridge-3] [WARN] [1757674251.067843415] [foxglove_bridge]: [WS] 192.168.0.18:59453: Send buffer limit reached
```

This is likely because the image data is too large and becomes a bottle neck.

We can work around this creating a new topic that is a throttled version of the `/image_raw/compressed` topic, like this

First install this if you havent
```
sudo apt install ros-jazzy-topic-tools
```

Then you'll be able to use the throttle nodes

```
        # Declare args (must be part of the LaunchDescription)
        DeclareLaunchArgument("image_in",          default_value="/image_raw/compressed"),
        DeclareLaunchArgument("image_viz",         default_value="/image_viz/compressed"),
        DeclareLaunchArgument("image_viz_rate_hz", default_value="3.0"),

        # Throttle <mode> <in> <rate> <out>
        Node(
            package="topic_tools",
            executable="throttle",
            name="image_throttle",
            arguments=["messages", image_in, image_rate, image_viz],
            output="screen",
        ),
```        

Then, in the foxglove bridge, we can use the whitelist to include the `/image_viz/compressed` topic and leave out the other `/image_raw` topics, like this

```
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
                r"^(.*/)?image_viz/compressed$",
            ],
        }]
    )
```