# Installation status

## VENTUNO Q

The migration is in progress. The board runs Ubuntu 24.04 ARM64 and already has
Docker, Compose, and Arduino tooling, but this repository does not yet provide
a complete VENTUNO runtime. The robot hardware is not connected.

Follow [VENTUNO migration progress](VENTUNO_MIGRATION.md) and the
[container workflow](CONTAINERS.md). Phase 3 provides build/tests, pinned vendor
code, model setup and Compose service configuration. Phase 4 must supply MCU and
local voice adapters and verified wiring before robot startup is enabled.
Do not install the old Pi ROS/Python dependencies on the VENTUNO host.

The host will still supply kernel drivers, device permissions, Docker, Arduino's
Router service, and the board's accelerator firmware/runtime interfaces. ROS,
application Python packages, and custom dependency builds will live in containers.

## Raspberry Pi 5 archive

The original host installation instructions are available at the immutable
[Pi baseline INSTALL document](https://github.com/jameselsey/robotics/blob/8100627c087d5cc25e0c40bdf27b5d8a42e131bf/docs/INSTALL.md).
Use that revision's source, requirements, and instructions together when
reproducing the Pi version. The planned `pi5-final` tag points to the same commit;
see [the tag handoff](VENTUNO_MIGRATION.md#preserve-the-pi-baseline).

The Makefile now runs the container workflow. Historical native environment/vendor
setup remains available at the Pi baseline; [VENV_SETUP](VENV_SETUP.md) and
[VENDOR_OVERLAY](VENDOR_OVERLAY.md) point to the current container replacements.
Mapping/navigation documents retain the robot's ROS behavior and safeguards.
