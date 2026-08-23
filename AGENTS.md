# RoboPi operating rule

- Never start, stop, restart, or relaunch the robot's ROS 2/robotics launch processes.
- Code changes, builds, tests, and read-only runtime diagnostics over SSH are allowed.
- When changes require a relaunch, tell the user and let them run it from their own SSH terminal so they retain Ctrl-C control.
