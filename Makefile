.PHONY: clean build launch-joystick venv

# Virtual environment setup
VENV_DIR = ros_venv
VENV_PYTHON = $(VENV_DIR)/bin/python3
VENV_PIP = $(VENV_DIR)/bin/pip
VENV_ACTIVATE = . $(VENV_DIR)/bin/activate

# ROS2 log formatting - human-readable timestamps
export RCUTILS_CONSOLE_OUTPUT_FORMAT=[{severity}] [{time:%Y-%m-%d %H:%M:%S.%f}] [{name}]: {message}
export RCUTILS_COLORIZED_OUTPUT=1

venv:
	@if [ ! -d "$(VENV_DIR)" ]; then \
		echo "🔧 Creating virtual environment with system site packages..."; \
		python3 -m venv --system-site-packages $(VENV_DIR); \
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
	rosdep install -yr --from-paths . --skip-keys "python3-numpy python3-soundfile python3-pyaudio python3-openai-whisper-pip python3-playsound-pip python3-pyttsx3-pip python3-piper-tts-pip python3-sounddevice-pip python3-pvporcupine-pip"

install-oww:
	@echo "📦 Installing openWakeWord in venv..."
	$(VENV_PIP) install -U --no-deps "openwakeword>=0.6.0"
	$(VENV_PIP) install -U onnxruntime numpy


build: venv clean
	# we have to compile the urdf file from the xacro, because urdf is what foxglove requires
	ros2 run xacro xacro src/tank_description/urdf/robot.urdf.xacro > src/tank_description/urdf/robot.urdf
	@echo "🔨 Building with venv activated..."
	@bash -c "source $(VENV_DIR)/bin/activate && echo 'Using Python:' && which python3 && colcon build --symlink-install"
	@echo "✅ Build complete."
	@echo "🔍 Verifying Python interpreter in executables..."
	@head -1 install/senses/lib/senses/brain || echo "Brain executable not found"

launch-joystick:
	@bash -c "$(VENV_ACTIVATE) && source install/setup.bash && source ~/vendor_ws/install/setup.bash && ros2 launch joystick joystick.launch.py"

launch-drive:
	@bash -c "$(VENV_ACTIVATE) && source install/setup.bash && source ~/vendor_ws/install/setup.bash && ros2 launch drive_controller drive_controller.launch.py"

launch-senses:
	@bash -c "$(VENV_ACTIVATE) && source install/setup.bash && source ~/vendor_ws/install/setup.bash && ros2 launch senses senses.launch.py"

launch:
	@bash -c "$(VENV_ACTIVATE) && source install/setup.bash && source ~/vendor_ws/install/setup.bash && ros2 launch bringup all.launch.py"

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