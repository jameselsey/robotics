# Mapping and Room Annotation

This robot uses the standard ROS 2 stack for mapping and navigation:

- `slam_toolbox` builds and maintains the `map` frame from `/scan`, `/odom`, and TF.
- Nav2 accepts navigation goals through the `navigate_to_pose` action.
- `senses/config/rooms.yaml` is the robot-specific semantic layer that gives human names to areas in the SLAM map.

The important frame chain is:

```text
map -> odom -> base_link -> laser
```

`odom -> base_link` comes from the drive controller, `map -> odom` comes from SLAM Toolbox, and `base_link -> laser` is published by `bringup/launch/slam.launch.py`.

## 1. Start Mapping on the Robot

On the Pi:

```bash
cd ~/robotics
source /opt/ros/jazzy/setup.bash
source install/setup.bash
make launch
```

`make launch` starts the normal robot stack plus SLAM Toolbox. In Foxglove or RViz, you should see:

- `/scan`
- `/map`
- `/map_metadata`
- `/tf`
- `/tf_static`
- `/odom`

Drive the robot slowly around the house so SLAM Toolbox can build up the occupancy grid. Loop closures improve if you return through areas it has already seen.

## 2. Save the Occupancy Map

When the map looks good, save it from the Pi:

```bash
mkdir -p ~/robotics/maps
ros2 run nav2_map_server map_saver_cli -f ~/robotics/maps/home
```

That creates:

```text
~/robotics/maps/home.yaml
~/robotics/maps/home.pgm
```

These files are the occupancy map. They are not the room labels yet.

## 3. Open RViz on the Mac

RViz is easiest for choosing exact map coordinates because it can publish clicked points.

On macOS, install and start XQuartz, enable **Allow connections from network clients** in XQuartz settings, restart XQuartz, then allow local X11 clients:

```bash
xhost +localhost
```

Docker Desktop may also need host networking enabled for ROS 2 DDS traffic. If `make rviz` complains about host networking, enable it in Docker Desktop settings, or temporarily remove `network_mode: host` from the `rviz` service and try again on the same Wi-Fi network.

From your Mac checkout of this repo:

```bash
make build-rviz
make rviz
```

In RViz:

1. Set `Fixed Frame` to `map`.
2. Add a `Map` display and set topic to `/map`.
3. Add `LaserScan` for `/scan` if you want to see live lidar.
4. Add `TF` if you want to inspect frames.
5. Use the toolbar's `Publish Point` tool to click room corners.

If RViz opens but shows no ROS topics, check that the Mac can resolve `robopi` and that the robot and Mac are on the same network. The Docker RViz container uses Cyclone DDS with `robopi` as its explicit peer.

## 4. Capture Room Polygon Points

On the Pi, echo clicked points from RViz:

```bash
ros2 topic echo /clicked_point
```

Then in RViz, click the corners of a room in order around the perimeter. Each click prints a `point.x` and `point.y` in the `map` frame. Copy those coordinates into `senses/config/rooms.yaml`.

Example:

```yaml
frame_id: map
base_frame: base_link
rooms:
  bedroom:
    polygon:
      - [1.20, -0.40]
      - [3.80, -0.40]
      - [3.80, 2.10]
      - [1.20, 2.10]
    navigate_pose:
      x: 2.50
      y: 0.85
      yaw: 0.0
  kitchen:
    polygon:
      - [-2.10, 0.20]
      - [0.80, 0.20]
      - [0.80, 2.50]
      - [-2.10, 2.50]
    navigate_pose:
      x: -0.60
      y: 1.30
      yaw: 1.57
```

The `polygon` is used for questions like "what room are you in?". The optional `navigate_pose` is used for commands like "navigate to the bedroom".

After editing room labels, rebuild or use the installed config path directly, then ask the agent to reload labels:

```bash
colcon build --packages-select senses --symlink-install
source install/setup.bash
```

Then say: "reload room labels".

## 5. Start Nav2 Navigation

Nav2 is installed but not started by default. Start it when you are ready to send autonomous goals:

```bash
make launch ARGS="enable_navigation:=true"
```

If your current Makefile does not pass `ARGS`, use the direct command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source ~/vendor_ws/install/setup.bash
ros2 launch bringup all.launch.py enable_navigation:=true
```

Once Nav2 is running and a room has a `navigate_pose`, the voice agent can call `navigate_to_room` for requests like:

```text
Can you navigate to the bedroom?
```

## Troubleshooting

If `what room are you in?` says there is no `map -> base_link` transform, SLAM has not published a map transform yet. Check `/scan`, `/odom`, and TF first.

If Nav2 says the action server is not running, start with `enable_navigation:=true` and wait for the Nav2 lifecycle nodes to activate.

If room detection gives the wrong label, inspect the clicked polygon order and make sure the points enclose the room without crossing over themselves.
