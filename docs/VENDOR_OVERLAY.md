# Legacy Pi vendor overlay

> This is the existing Raspberry Pi setup, not a VENTUNO installation step.
> Phase 3 of the [migration](VENTUNO_MIGRATION.md) will build pinned vendor
> dependencies inside the ROS container instead of requiring `~/vendor_ws`.

A vendor overlay gives us the ability to build ros2 packages from source and make them available to our main (underlay)

Currently we just have sllidar in the vendor overlay, but we could add more in the future

## Create and build the vendor overlay

```
mkdir -p ~/vendor_ws/src
cd ~/vendor_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git

cd ~/vendor_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y -r
colcon build --symlink-install
```
