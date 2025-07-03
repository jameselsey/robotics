.PHONY: clean build launch-joystick

clean:
	rm -rf build/ install/ log/

build: clean
	# we have to compile the urdf file from the xacro, because urdf is what foxglove requires
	ros2 run xacro xacro src/tank_description/urdf/robot.urdf.xacro > src/tank_description/urdf/robot.urdf
	colcon build --symlink-install
	@echo "✅ Build complete."

launch-joystick:
	@bash -c "source install/setup.bash && ros2 launch joystick joystick.launch.py"

launch-bridge:
	@bash -c "source install/setup.bash && ros2 launch foxglove_bridge_launcher bridge.launch.py"

launch-drive:
	@bash -c "source install/setup.bash && ros2 launch drive_controller drive_controller.launch.py"

launch-senses:
	@bash -c "source install/setup.bash && ros2 launch senses senses.launch.py"

launch:
	@bash -c "source install/setup.bash && ros2 launch bringup all.launch.py"
