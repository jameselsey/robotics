#!/usr/bin/env bash
set -eo pipefail
cd /workspace
# Each command is hardware-independent. Import smoke runs in a separate process
# so unit-test SDK/ROS stubs cannot hide broken runtime imports.
python3 -m pip check
python3 tools/smoke.py
python3 -m colcon --log-base /opt/robot/log test --base-paths /opt/robot/src \
  --build-base /opt/robot/build --install-base /opt/robot/install --merge-install
python3 -m colcon test-result --test-result-base /opt/robot/build --verbose
python3 -m pytest -p no:cacheprovider -q test
make lint-code RUFF=ruff
