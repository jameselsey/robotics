.PHONY: clean build launch-joystick

clean:
	rm -rf build/ install/ log/

install-deps:
	PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install -yr --from-paths .

install-oww:
	sudo -H --preserve-env=PIP_BREAK_SYSTEM_PACKAGES python3 -m pip install -U --no-deps "openwakeword>=0.6.0"
	sudo -H --preserve-env=PIP_BREAK_SYSTEM_PACKAGES python3 -m pip install -U onnxruntime numpy


build: clean
	# we have to compile the urdf file from the xacro, because urdf is what foxglove requires
	ros2 run xacro xacro src/tank_description/urdf/robot.urdf.xacro > src/tank_description/urdf/robot.urdf
	colcon build --symlink-install
	@echo "✅ Build complete."

launch-joystick:
	@bash -c "source install/setup.bash && source ~/vendor_ws/install/setup.bash && ros2 launch joystick joystick.launch.py"

launch-drive:
	@bash -c "source install/setup.bash && source ~/vendor_ws/install/setup.bash && ros2 launch drive_controller drive_controller.launch.py"

launch-senses:
	@bash -c "source install/setup.bash && source ~/vendor_ws/install/setup.bash && ros2 launch senses senses.launch.py"

launch:
	@bash -c "source install/setup.bash && source ~/vendor_ws/install/setup.bash && ros2 launch bringup all.launch.py"

docker:
	# You may need to do these first, if it complains about permissions errors
	#   sudo usermod -aG docker $USER
	#   newgrp docker
	docker compose up -d
	sleep 5
	docker exec -it ollama ollama pull tinyllama

build-docker:
	docker compose build --no-cache

connect:
	docker compose exec hailo /bin/bash

# voice:
# 	@msg="$(strip $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS)))"; \
# 	[ -n "$$msg" ] || msg="This is a test message"; \
# 	ros2 topic pub --once /speech_output std_msgs/msg/String "data: '$$msg'"

# # Swallow extra words after 'voice' so make doesn't treat them as targets
# $(filter-out voice,$(MAKECMDGOALS)):
# 	@: