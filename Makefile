.PHONY: clean build launch-joystick

clean:
	rm -rf build/ install/ log/

build: clean
	colcon build --symlink-install
	@echo "✅ Build complete."

launch-joystick:
	@bash -c "source install/setup.bash && ros2 launch joystick joystick.launch.py"

launch-bridge:
	@bash -c "source install/setup.bash && ros2 launch foxglove_bridge_launcher bridge.launch.py"

launch:
	@bash -c "source install/setup.bash && ros2 launch bringup all.launch.py"
