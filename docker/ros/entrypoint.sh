#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/robot-venv/bin/activate
source /opt/vendor/install/setup.bash
source /opt/robot/install/setup.bash
export PYTHONUNBUFFERED=1
exec "$@"
