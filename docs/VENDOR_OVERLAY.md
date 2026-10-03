# Container vendor overlay

The ROS image imports [vendor.repos](../vendor.repos) and builds `sllidar_ros2`
inside `/opt/vendor`. Its commit is immutable; no `~/vendor_ws` or host dependency
installation is required. The image entrypoint sources vendor and robot overlays.

Run `make build` and `make test`; see [container operation](CONTAINERS.md).
Update the source SHA deliberately and validate the new driver before accepting it.
Historical Pi workspace instructions are available in the
[Pi baseline](https://github.com/jameselsey/robotics/blob/8100627c087d5cc25e0c40bdf27b5d8a42e131bf/docs/VENDOR_OVERLAY.md).
