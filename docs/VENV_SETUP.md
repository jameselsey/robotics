# Python dependencies in containers

VENTUNO uses `/opt/robot-venv` inside the ROS image with distribution ROS libraries
exposed through system site-packages. Application dependencies are pinned in
[requirements.txt](../requirements.txt) and fully hash-locked for ARM64/Python 3.12
in [requirements.lock](../docker/ros/requirements.lock). No host venv is required.

`make build`, `make test`, and `make lint` use containers. See
[container operation](CONTAINERS.md) for dependency updates and runtime limits.
Historical native Pi setup remains at the
[Pi baseline](https://github.com/jameselsey/robotics/blob/8100627c087d5cc25e0c40bdf27b5d8a42e131bf/docs/VENV_SETUP.md).
