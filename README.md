# robotics
A collection of robotics projects that I have been tinkering with to learn more.

![R2](img/r2.jpg)

# How to install
(this will become much easier when I find the time to Docker-ise it, but for now, we'll just install directly onto the host system)

The instructions are quite long, see the [INSTALL.md](INSTALL.md) for the full instructions.


# How to run
I've made it very easy to run the robot:
```bash
make build
make launch
```

You can also run individual packages such as `make launch-senses` if you wanted to run a single subsystem.

It'll run the foxglove bridge too, so point that at `ws://pi5:8765` and you'll see this

![Foxglove](img/foxglove.png)

It isn't particularly useful on it's own, so run the "bringup" which will launch all components, so you can control it and see sensor data etc

# Parts List
- Xiaor geek tank chassis
- Raspberry Pi 5
- Logitech Brio100 webcam
- Cheap USB speakers from AliExpress
- Xbox controller
- Cheap Buck converters from AliExpress to step down the 18v to 12v for the motors and 5v for the pi
- L298N H bridge motor driver
- Various bits of scrap wood and screws to hold it all together
- Googly eyes


# Why am I building this?
Mostly curiosity, and I enjoy tinkering with things and solving hard problems. Robotics gives me a fantastic platform to explore and learn:
* Electronics, hardware design, and soldering
* Software, ros2, python, c++
* Computer vision, machine learning, and AI
* SLAM, navigation, and autonomy
* Visualisation and simulators
* CAD and 3d printing (for when R2 eventually gets a new body)
