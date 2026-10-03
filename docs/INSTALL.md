# Installation status

## VENTUNO Q

The migration is in progress. The board runs Ubuntu 24.04 ARM64 and already has
Docker, Compose, and Arduino tooling, but this repository does not yet provide
a complete VENTUNO runtime. The robot hardware is not connected.

Follow [VENTUNO migration progress](VENTUNO_MIGRATION.md). Phase 3 will provide
the container build, pinned vendor dependencies, model setup, device configuration,
and foreground startup workflow. Phase 4 will provide verified wiring and MCU
firmware instructions. Do not install the old Pi ROS/Python dependencies on the
VENTUNO host to work around the unfinished migration.

The host will still supply kernel drivers, device permissions, Docker, Arduino's
Router service, and the board's accelerator firmware/runtime interfaces. ROS,
application Python packages, and custom dependency builds will live in containers.

## Raspberry Pi 5 archive

The original host installation instructions are available at the immutable
[Pi baseline INSTALL document](https://github.com/jameselsey/robotics/blob/8100627c087d5cc25e0c40bdf27b5d8a42e131bf/docs/INSTALL.md).
Use that revision's source, requirements, and instructions together when
reproducing the Pi version. The planned `pi5-final` tag points to the same commit;
see [the tag handoff](VENTUNO_MIGRATION.md#preserve-the-pi-baseline).

The legacy Makefile, Python environment, and [vendor overlay](VENDOR_OVERLAY.md)
remain temporarily for the retained native ROS launch workflow. They are not
VENTUNO installation instructions. [VENV_SETUP](VENV_SETUP.md) summarizes the
remaining Python dependencies; mapping and navigation guides describe the ROS
behavior that the migration must preserve.
