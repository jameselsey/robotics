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
by a separate workspace package. The LiDAR driver is built from the pinned `vendor.repos` inside the ROS image.

Pi GPIO remains in the drive controller and voice LED adapter until phase 4.
Package cleanup, containerization, and local inference integration are tracked
in the [VENTUNO migration](../docs/VENTUNO_MIGRATION.md).

## Build and voice boundaries

`tank_description` owns the canonical `urdf/robot.urdf.xacro`. Its CMake target
creates `build/tank_description/urdf/robot.urdf` and installs both files into the
package share directory. Builds work with read-only source; generated URDF is no
longer tracked. The removed `tank.urdf.xacro` was an unused older description;
the empty view launch file had no behavior.

Within `senses`, `voice_agent.py` remains the ROS/wake-word adapter.
`voice_config.py` declares and normalizes settings, `nova_backend.py` constructs
the Nova model and holds the unchanged prompt, `conversation_session.py` owns
async task lifetimes, and `audio_devices.py` selects input/output devices.
Existing audio processing, vision, movement, and semantic navigation modules
retain their responsibilities. Local inference and the MCU adapter arrive in
phase 4. Both acknowledgement sound effects remain installed.

Python packages install ROS resources through `data_files`; ROS dependencies
belong in `package.xml`, while the PyPI voice libraries remain in the root
`requirements.txt`. CMake resource packages need no conversion to Python builds.
`make lint` checks the selected files incrementally in the test image with Ruff
0.15.1; `make test` runs package/deployment tests and production import/resource
checks without hardware. See [container operation](../docs/CONTAINERS.md).

## Deployment configuration

Launch arguments are visible through `ros2 launch bringup all.launch.py --show-args`
without starting nodes. Included launch arguments are available from full bringup.

| Setting | Default / purpose |
| --- | --- |
| `drive_params_file` | Installed drive YAML; preserves current calibration |
| `foxglove_port` | `8765` |
| `camera_device` | `/dev/video0`; can use a stable device symlink |
| `serial_port`, `serial_baudrate` | `/dev/ttyUSB0`, `460800`; LiDAR |
| `audio_device_name` | `Brio`; case-insensitive input name substring |
| `audio_output_device_name` | Empty; system-default output |
| `audio_input_device_index`, `audio_output_device_index` | `-1`; optional numeric fallback after name lookup |
| `oww_host`, `oww_port` | `127.0.0.1`, `10400`; wake-word service |
| `voice_params_file` | Installed `senses/config/voice_agent.yaml` |
| `rooms_config_path` | Installed rooms YAML with reviewed navigation poses |
| `saved_map_file` | `ROBOT_MAP_FILE` environment value, or `maps/house.yaml` relative to launch working directory |
| `localization_pose_file` | `$ROS_HOME/robopi/localization_pose.json`, falling back to `~/.ros` |

The voice YAML exposes buffer sizes, gate/AEC settings, wake settings, and the
historical Pi LED settings. Named launch parameters override corresponding YAML
values; YAML overrides node defaults for other settings. Use launch arguments for
AWS profile/region, model selection, devices, and endpoints. Credentials stay in
the AWS credential provider, never YAML. `navigation_log_path` can be set in the
voice YAML; its default follows `$ROS_HOME/robopi/navigation_events.jsonl`.

The Make targets now use Compose and the installed overlays, without host ROS
or a vendor workspace. Runtime settings and state paths are defined in `.env`
and Compose; see the container guide. Pi BCM numbers are historical configuration, not
VENTUNO pin assignments; do not wire the VENTUNO using them.
