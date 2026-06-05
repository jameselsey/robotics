.PHONY: clean build launch-joystick venv rviz build-rviz save-map publish-room-markers

# Virtual environment setup
VENV_DIR = ros_venv
VENV_PYTHON = $(VENV_DIR)/bin/python3
VENV_PIP = $(VENV_DIR)/bin/pip
VENV_ACTIVATE = . $(VENV_DIR)/bin/activate
ARGS ?=
MAP_NAME ?= house
MAP_DIR ?= maps
MAP_TOPIC ?= /map
MAP_WAIT_TIMEOUT ?= 15
MAP_SAVE_TIMEOUT ?= 10.0
ROOMS_CONFIG ?= src/senses/config/rooms.yaml

# ROS2 log formatting - human-readable timestamps
export RCUTILS_CONSOLE_OUTPUT_FORMAT=[{severity}] [{time}] [{name}]: {message}
export RCUTILS_COLORIZED_OUTPUT=1

venv:
	@if [ ! -d "$(VENV_DIR)" ]; then \
		echo "🔧 Creating virtual environment with system site packages..."; \
		python3 -m venv --system-site-packages $(VENV_DIR); \
		touch $(VENV_DIR)/COLCON_IGNORE; \
		$(VENV_PIP) install -U pip wheel setuptools; \
		echo "📦 Installing Python dependencies from requirements.txt..."; \
		$(VENV_PIP) install -r requirements.txt; \
		echo "✅ Virtual environment ready."; \
	else \
		echo "✅ Virtual environment already exists."; \
	fi

clean:
	rm -rf build/ install/ log/

clean-venv:
	rm -rf $(VENV_DIR)
	@echo "🗑️  Virtual environment removed."

install-deps: venv
	@echo "📦 Installing ROS2 system dependencies via rosdep..."
	PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install -yr --from-paths . --as-root pip:false

install-oww:
	@echo "📦 Installing openWakeWord in venv..."
	$(VENV_PIP) install -U --no-deps "openwakeword>=0.6.0"
	$(VENV_PIP) install -U onnxruntime numpy

build: 
	# we have to compile the urdf file from the xacro, because urdf is what foxglove requires
	ros2 run xacro xacro src/tank_description/urdf/robot.urdf.xacro > src/tank_description/urdf/robot.urdf
	@echo "🔨 Building with venv activated..."
	@bash -c "colcon build --symlink-install"
	@echo "✅ Build complete."

launch-joystick:
	@bash -c "$(VENV_ACTIVATE) && source install/setup.bash && source ~/vendor_ws/install/setup.bash && ros2 launch joystick joystick.launch.py"

launch-drive:
	@bash -c "$(VENV_ACTIVATE) && source install/setup.bash && source ~/vendor_ws/install/setup.bash && ros2 launch drive_controller drive_controller.launch.py"

launch-senses:
	@bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && source ~/vendor_ws/install/setup.bash && ros2 launch senses senses.launch.py"

launch:
	@bash -c "$(VENV_ACTIVATE) && source install/setup.bash && source ~/vendor_ws/install/setup.bash && ros2 launch bringup all.launch.py $(ARGS)"

save-map:
	@mkdir -p $(MAP_DIR)
	@echo "Saving SLAM map to $(MAP_DIR)/$(MAP_NAME).yaml and $(MAP_DIR)/$(MAP_NAME).pgm"
	@bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 lifecycle set /slam_toolbox configure >/dev/null 2>&1 || true && ros2 lifecycle set /slam_toolbox activate >/dev/null 2>&1 || true && timeout $(MAP_WAIT_TIMEOUT) bash -c 'until ros2 topic echo $(MAP_TOPIC) --once >/dev/null 2>&1; do sleep 1; done' && ros2 run nav2_map_server map_saver_cli -t $(MAP_TOPIC) -f $(MAP_DIR)/$(MAP_NAME) --ros-args -p save_map_timeout:=$(MAP_SAVE_TIMEOUT)"

publish-room-markers:
	@bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run senses room_markers --ros-args -p rooms_config_path:=$(ROOMS_CONFIG)"

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


build-rviz:
	docker compose build rviz

rviz:
	@echo "Starting RViz in Docker. On macOS, start XQuartz and run: xhost +localhost"
	DISPLAY=$${DISPLAY:-host.docker.internal:0} docker compose --profile tools run --rm rviz
