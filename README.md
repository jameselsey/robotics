# robotics
Robotic experiments!

Joystick
----
Launch using `ros2 launch robotics joystick.launch.py`

Foxglove
----
Make sure you have foxglove installed. You can install it using the following command:
```bash
sudo apt install ros-jazzy-foxglove-bridge
```

Then run the foxglove bridge itself using `make launch-bridge`, this will just run the bridge and
you can then connect like `ws://pi5:8765`

It isn't particularly useful on it's own, so run the "bringup" which will launch all components, so you can control it and see sensor data etc

Pi Setup
---
When setting this up on a fresh pi, you'll need to install a bunch of packages
```bash
sudo apt install -y \
  ros-dev-tools \
  ros-jazzy-desktop \
  ros-jazzy-ros-base \
  '~nros-jazzy-rqt*' \
  ros-jazzy-joy \
  ros-jazzy-teleop-twist-joy \
  tree \
  curl
```