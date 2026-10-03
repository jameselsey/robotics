# Mapping and room annotation

This page covers occupancy maps and semantic room labels. For encoder calibration,
coordinate conventions, and SLAM diagnostics, use the
[encoder-to-SLAM guide](slam-mapping-guide.md). For saved-map localization,
reviewed navigation goals, and cancellation, use [NAVIGATION](NAVIGATION.md).

The launch commands below describe the retained native Pi workflow. They are not
yet a VENTUNO deployment procedure. See [migration status](VENTUNO_MIGRATION.md)
and run robot processes yourself from your terminal once hardware is connected.

## ROS data and frame ownership

```text
map -> odom -> base_link -> laser
```

- SLAM Toolbox publishes `map -> odom` while mapping; AMCL owns it in saved-map mode.
- The drive controller publishes encoder `/odom` and `odom -> base_link`.
- `bringup/launch/all.launch.py` publishes the static `base_link -> laser` transform.
- LiDAR publishes `/scan`; Foxglove shows scans, TF, `/map`, and room markers.

Do not run SLAM and AMCL as competing `map -> odom` publishers.

## Build and save an occupancy map

In an existing native ROS installation, `make launch` starts the mapping stack.
Drive slowly through the area, revisiting distinctive locations for loop closure.
Check `/scan`, `/odom`, `/tf`, `/tf_static`, `/map`, and `/map_metadata` in Foxglove.

When satisfied, use:

```bash
make save-map MAP_NAME=house
```

This saves `maps/house.yaml` and `maps/house.pgm`. Override `MAP_NAME` or `MAP_DIR`
for another map. The map YAML and image describe occupancy, not room names.

## Annotate named rooms

The default labels live in `src/senses/config/rooms.yaml`. A room combines a
polygon for location questions with an explicitly reviewed pose for navigation:

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
```

Polygons alone support room identification; navigation requires `navigate_pose`.
Choose it on checked free space with clearance for the robot. The agent does not
use a polygon centroid as an automatic navigation goal.

Install updated labels in the existing native workspace with:

```bash
colcon build --packages-select senses --symlink-install
source install/setup.bash
```

Then ask the agent to reload room labels. Room markers start with senses and
publish `/visualization_marker_array` in the `map` frame. Use
`make publish-room-markers ROOMS_CONFIG=maps/house.rooms.yaml` to publish an
alternate labels file manually.

## Navigation and troubleshooting

`make launch-navigation` enables Nav2 with the live SLAM map.
`make launch-localized` loads the saved map with AMCL and Nav2 instead.
Follow [NAVIGATION](NAVIGATION.md) for initialization and first-goal checks.

If room lookup lacks `map -> base_link`, inspect scan, odometry, localization,
and TF before changing labels. If the Nav2 action server is unavailable, verify
its lifecycle activation. If labels are wrong, check polygon coordinates and
edge ordering against the selected occupancy map.
