# From wheel encoders to a ROS 2 SLAM map

This guide documents the working RoboPi mapping stack and the calibration path used to reach it. It is also structured as a practical outline for a YouTube walkthrough.

## What the finished system does

The robot accepts velocity commands on `/cmd_vel`, drives two tracks, measures wheel movement with quadrature encoders, publishes odometry and TF, publishes a 2D LiDAR scan, and uses SLAM Toolbox to build an occupancy grid.

The important data flow is:

```text
Xbox controller
  -> joy_node -> teleop_twist_joy -> /cmd_vel
  -> drive_controller -> motors
                       -> encoders -> /odom
                                   -> odom -> base_link TF

RPLIDAR -> /scan in laser frame
URDF    -> robot_state_publisher
static TF: base_link -> laser

/scan + map -> odom -> base_link -> laser TF
  -> slam_toolbox -> /map + map -> odom TF
  -> foxglove_bridge -> Foxglove
```

## Coordinate-frame contract

ROS mobile robots conventionally use `+X` as forward, `+Y` as left, and `+Z` as up.

For this robot:

- The red drive wheels are at the physical front and must have positive X positions in the URDF.
- The green idlers are at the physical rear and must have negative X positions.
- The tall enclosure is at the rear and must have a negative X position.
- A positive `/cmd_vel.linear.x` must move the physical robot toward the red wheels.
- Positive encoder travel must increase odometry X when both tracks move forward.
- The LiDAR transform must make an obstacle physically in front appear at the red-wheel end of the model.

The RPLIDAR is mounted 180 degrees relative to the corrected `base_link` axis, so `slam.launch.py` uses a laser yaw of π radians. This does not reverse scan data arbitrarily; it describes the real sensor mounting relative to the ROS frame.

A mismatch here can look deceptively plausible in a robot-relative scan panel while destroying the map. SLAM will try to reconcile scans that imply motion in one direction with odometry that claims the opposite direction, producing large `map -> odom` corrections, duplicated walls, or apparent backward travel.

## 1. Verify the URDF before driving

The source description is:

```text
src/tank_description/urdf/robot.urdf.xacro
```

Foxglove currently consumes the generated URDF:

```text
src/tank_description/urdf/robot.urdf
```

Generate and validate it with:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run xacro xacro \
  src/tank_description/urdf/robot.urdf.xacro \
  -o src/tank_description/urdf/robot.urdf
check_urdf src/tank_description/urdf/robot.urdf
```

The workspace `make build` target also regenerates the file before invoking `colcon build`.

Check visually that:

- Red wheels are at the front.
- Green idlers and the tall enclosure are at the rear.
- Tracks and wheels sit symmetrically around `base_link`.
- The model moves red-end-first when odometry X increases.

## 2. Establish the TF tree

SLAM Toolbox needs this complete chain at the timestamp of every scan:

```text
map -> odom -> base_link -> laser
```

Ownership is deliberately split:

- SLAM Toolbox publishes `map -> odom`.
- `drive_controller` publishes `odom -> base_link`.
- `slam.launch.py` publishes the static `base_link -> laser` transform.

Useful checks:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_monitor map laser
```

An initial TF warning or one dropped scan while nodes start is usually harmless. Repeated message-filter drops indicate a timestamp, TF, CPU, or queue problem that needs investigation.

## 3. Verify the LiDAR independently

The LiDAR driver publishes `/scan` as `sensor_msgs/msg/LaserScan` in the `laser` frame.

Check its health:

```bash
ros2 topic info /scan --verbose
ros2 topic hz /scan
ros2 topic echo --once /scan
```

The current scanner runs at about 10 Hz in `Standard` mode with angle compensation enabled.

Before debugging SLAM, perform a physical direction test:

1. Put a board immediately in front of the red-wheel end.
2. Display `/scan` and the URDF together using `base_link` as the fixed frame.
3. Confirm the board appears directly in front of the red wheels.
4. Repeat on the left and right if sensor orientation is uncertain.

If the raw scan geometry is wrong, correct the physical `base_link -> laser` transform. Do not tune SLAM to compensate for a bad transform.

## 4. Verify encoder signs

The controller publishes JSON diagnostics on `/wheel_diagnostics`:

```bash
ros2 topic echo /wheel_diagnostics --field data
```

With both tracks commanded forward:

- The calibrated left and right distances must both be positive internally.
- Their magnitudes should be similar during a straight run.
- The diagnostics `warnings` list should remain empty.

The left encoder is physically wired in the opposite counting direction, so its configured inversion is intentional.

## 5. Calibrate distance

Use a measured floor line and always measure the same reference point on the chassis at the start and finish.

1. Restart the stack so odometry starts at zero.
2. Place the chosen chassis reference on the zero mark.
3. Drive approximately straight to a known distance.
4. Read `/odom` and `/wheel_diagnostics`.
5. Calculate:

```text
new_scale = old_scale * physical_distance / odometry_distance
```

The calibration run used here produced approximately 1,740 counts per side. With a wheel radius of 0.026 m, 230 counts per revolution, and a scale of `0.81`, that corresponds to approximately 1.00 m.

Current calibrated values live in:

```text
src/drive_controller/config/drive_controller.yaml
```

Both sides currently use the same distance scale because the reference run differed by only two encoder counts. Do not introduce separate left/right distance scales unless repeated straight tests show a consistent imbalance.

## 6. Calibrate heading and wheelbase

Distance scale and heading calibration are separate.

- Equal scale error changes distance.
- Left/right scale mismatch produces drift during straight travel.
- Incorrect `wheel_base_m` changes the measured angle during turns.
- Motor trim affects open-loop physical straightness but should not be used to falsify encoder distance.

For a useful heading test:

1. Mark the starting orientation.
2. Command a slow 360-degree in-place rotation.
3. Compare physical and odometry headings.
4. Repeat in both directions.
5. Adjust wheelbase only from repeatable angular error.

## 7. Start SLAM Toolbox

The bringup launch includes:

- joystick and teleoperation
- drive controller and odometry
- robot and joint state publishers
- LiDAR and camera/senses nodes
- static laser TF
- asynchronous SLAM Toolbox
- lifecycle management
- Foxglove bridge

Build and launch:

```bash
source /opt/ros/jazzy/setup.bash
source ros_venv/bin/activate
make build
make launch
```

SLAM configuration is in:

```text
src/bringup/config/slam_toolbox.yaml
```

Confirm the node and topics:

```bash
ros2 lifecycle get /slam_toolbox
ros2 topic info /map --verbose
ros2 topic hz /scan
ros2 topic hz /odom
ros2 node list
```

Expected results:

- `/slam_toolbox` is `active`.
- `/scan` has one LiDAR publisher and a SLAM subscriber.
- `/odom` publishes at about 50 Hz.
- `/map` has SLAM Toolbox as its publisher.

Camera calibration errors in the combined startup log do not imply that SLAM failed. Always identify the node name attached to an error.

## 8. Configure Foxglove

Connect to:

```text
ws://robopi.local:8765
```

Recommended views:

- Robot-relative scan: fixed frame `base_link`, showing `/scan` and the URDF.
- Global mapping view: fixed frame `map`, showing `/map`, `/scan`, and the URDF.

If the entire map rotates while driving, check whether the panel is following the robot pose. The occupancy grid itself publishes an identity orientation; camera-follow behavior can make a healthy grid look as if it is rotating.

## 9. Drive a good mapping route

- Start with the robot stationary for a few seconds.
- Drive slowly enough to limit scan motion distortion and track slip.
- Prefer smooth turns over abrupt spins.
- Revisit distinctive areas so loop closure has useful geometry.
- Avoid moving furniture or people close to the scanner during the run.
- Expect unknown space and ray-shaped free-space patches early in a map.

SLAM maps everything currently visible, not only the floor directly traversed by the robot. A large map can appear immediately if the LiDAR sees distant walls.

## 10. Save and inspect a map

The repository provides:

```bash
make save-map MAP_NAME=house
```

This writes `maps/house.yaml` and `maps/house.pgm` after waiting for `/map`.

Before trusting a map, inspect:

- parallel walls remain parallel
- revisited walls overlap instead of duplicating
- robot direction agrees in the scan and map views
- `map -> odom` corrections are plausible rather than cancelling most travel
- encoder distance remains close to a known physical measurement

## Troubleshooting order

Use this order to avoid tuning one subsystem around another subsystem’s fault:

1. Physical front/rear convention and URDF.
2. Laser direction and static transform.
3. Encoder presence and sign.
4. Encoder distance scale.
5. Wheelbase and heading.
6. Scan and odometry rates/timestamps.
7. TF availability.
8. SLAM Toolbox lifecycle and configuration.
9. Foxglove fixed frame and follow settings.
10. Only then tune scan-matcher parameters.

## Suggested YouTube structure

1. Show the completed map first.
2. Explain `/cmd_vel`, encoders, `/odom`, `/scan`, `/map`, and TF using the data-flow diagram.
3. Introduce REP-style robot axes and the red-front/green-rear visual convention.
4. Demonstrate the board-in-front LiDAR test.
5. Demonstrate the one-metre encoder calibration and formula.
6. Show `odom -> base_link` and `map -> odom` in the terminal.
7. Show the failure mode caused by reversed model coordinates.
8. Explain why the robot-relative scan could look correct while the global map was wrong.
9. Launch SLAM, drive a loop, and watch the map unfold.
10. Finish with automated tests that prevent the front/rear regression.

## Further reading

- [ROS 2 Jazzy package development](https://docs.ros.org/en/jazzy/How-To-Guides/Developing-a-ROS-2-Package.html)
- [ROS 2 launch-file packaging](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-system.html)
- [URDF parser documentation](https://docs.ros.org/en/jazzy/p/urdf/index.html)
- [SLAM Toolbox documentation](https://docs.ros.org/en/jazzy/p/slam_toolbox/)
