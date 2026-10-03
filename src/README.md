# ROS 2 packages

| Package | Responsibility | Build type |
| --- | --- | --- |
| `bringup` | Robot launch orchestration, Foxglove, SLAM, localization, Nav2, and health/pose helpers | `ament_cmake` |
| `drive_controller` | `/cmd_vel` motor commands, measured encoder odometry, TF, wheel diagnostics, and calibration | `ament_python` |
| `joystick` | Xbox input, deadman teleoperation, and voice-session controls | `ament_cmake` |
| `senses` | Nova voice agent, audio feedback, camera launch, vision tools, room markers, and movement/navigation tools | `ament_python` |
| `tank_description` | Robot URDF/Xacro and description resources | `ament_cmake` |

CMake packages can install launch, YAML, URDF, and Python helper scripts without
containing C or C++ nodes. The Python executable packages use `setup.py` and
console entry points.

`eyes.launch.py` runs the external `v4l2_camera` driver and `topic_tools` image
throttle; no custom camera node is needed. Foxglove is launched by `bringup`, not
by a separate workspace package. The LiDAR driver currently comes from the
legacy external vendor workspace.

Pi GPIO remains in the drive controller and voice LED adapter until phase 4.
Package cleanup, containerization, and local inference integration are tracked
in the [VENTUNO migration](../docs/VENTUNO_MIGRATION.md).
