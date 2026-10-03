.DEFAULT_GOAL := help
# Share the already-built legacy cache on this storage-limited board.
# Override with DOCKER_BUILDKIT=1 on a host with room for a fresh BuildKit cache.
export DOCKER_BUILDKIT ?= 0
COMPOSE ?= docker compose
BASE = -f docker-compose.yml
HARDWARE = $(BASE) -f compose.hardware.yaml
NOVA = $(HARDWARE) -f compose.nova.yaml
ROS_SERVICE ?= ros
ROS_EXEC = $(COMPOSE) $(BASE) exec $(ROS_SERVICE) /opt/robot/bin/entrypoint.sh
ARGS ?=
MAP_NAME ?= house
MAP_TOPIC ?= /map
MAP_SAVE_TIMEOUT ?= 10.0
ROOMS_CONFIG ?= /config/senses/rooms.yaml

.PHONY: help prepare preflight config build build-docker test lint lint-code \
	setup-models inference launch launch-navigation launch-localized launch-nova \
	status logs health shell save-map publish-room-markers navigation-log calibrate-angular \
	build-rviz rviz

help:
	@echo "prepare/config/build/test: hardware-independent setup and checks"
	@echo "setup-models: download ASR/TTS into project runtime folders (internet required)"
	@echo "inference: local services in foreground; Ctrl-C stops this stack"
	@echo "launch[-navigation|-localized|-nova]: foreground robot (gated until phase 4)"
	@echo "status/logs/health/shell: diagnostics; rviz: optional Linux GUI tool"

prepare:
	python3 tools/prepare.py

preflight:
	python3 tools/preflight.py --mode build

config:
	$(COMPOSE) $(BASE) --profile '*' config --quiet
	$(COMPOSE) $(NOVA) --profile '*' config --quiet

build: preflight config
	$(COMPOSE) $(BASE) build ros
	$(COMPOSE) $(BASE) build checks

build-docker: build

test:
	$(COMPOSE) $(BASE) run --rm --no-deps checks

lint:
	$(COMPOSE) $(BASE) run --rm --no-deps checks make lint-code

# Internal test-image target; requires no host Python/ROS environment.
RUFF ?= ruff
LINT_FILES = src/senses/senses/voice_agent.py src/senses/senses/voice_config.py \
	src/senses/senses/nova_backend.py src/senses/senses/conversation_session.py \
	src/senses/senses/audio_devices.py src/senses/test/test_voice_config.py \
	src/senses/test/test_conversation_session.py src/senses/launch \
	src/bringup/launch/all.launch.py src/bringup/launch/localization.launch.py \
	src/bringup/test src/tank_description/test src/senses/setup.py \
	src/drive_controller/setup.py tools test docker/ros/robot-start.py
lint-code:
	$(RUFF) check $(LINT_FILES)
	$(RUFF) format --check $(LINT_FILES)

setup-models: prepare
	$(COMPOSE) $(BASE) run --rm --no-deps download-asr
	$(COMPOSE) $(BASE) run --rm --no-deps download-tts

inference:
	python3 tools/preflight.py --mode local
	$(COMPOSE) $(BASE) --profile local-voice up

launch:
	python3 tools/preflight.py --mode robot
	ROS_LAUNCH_ARGS="$(ARGS)" $(COMPOSE) $(HARDWARE) --profile robot up --abort-on-container-exit --exit-code-from ros

launch-navigation:
	@$(MAKE) launch ARGS="enable_navigation:=true $(ARGS)"

launch-localized:
	@test -f "runtime/maps/$(MAP_NAME).yaml" || (echo "Missing runtime/maps/$(MAP_NAME).yaml" && exit 1)
	@$(MAKE) launch ARGS="use_saved_map:=true enable_navigation:=true saved_map_file:=/maps/$(MAP_NAME).yaml $(ARGS)"

launch-nova:
	python3 tools/preflight.py --mode nova
	ROS_LAUNCH_ARGS="$(ARGS)" $(COMPOSE) $(NOVA) --profile nova up --abort-on-container-exit --exit-code-from ros-nova

status:
	$(COMPOSE) $(BASE) --profile '*' ps

logs:
	$(COMPOSE) $(BASE) --profile '*' logs --tail 100

health:
	$(ROS_EXEC) timeout 8 ros2 topic echo /foxglove_health --once --full-length

shell:
	$(ROS_EXEC) bash

# Diagnostics/calibration reuse the existing ROS container, never launch another robot.
save-map:
	$(ROS_EXEC) ros2 run nav2_map_server map_saver_cli -t $(MAP_TOPIC) -f /maps/$(MAP_NAME) --ros-args -p save_map_timeout:=$(MAP_SAVE_TIMEOUT)

publish-room-markers:
	$(ROS_EXEC) ros2 run senses room_markers --ros-args -p rooms_config_path:=$(ROOMS_CONFIG)

navigation-log:
	@tail -n 100 -f runtime/state/ros/robopi/navigation_events.jsonl

calibrate-angular:
	$(ROS_EXEC) ros2 run drive_controller calibrate_angular --config /config/drive/drive_controller.yaml

build-rviz:
	$(COMPOSE) $(BASE) build rviz

rviz:
	$(COMPOSE) $(BASE) run --rm --no-deps rviz
