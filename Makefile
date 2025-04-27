.PHONY: build

build:
	colcon build
	@echo "✅ Build complete."

launch-joystick:
	source install/setup.bash && ros2 launch joystick joystick.launch.py
