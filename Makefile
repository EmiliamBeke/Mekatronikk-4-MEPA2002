SHELL := /bin/bash

.PHONY: build shell up down ws lidar-setup lidar-test mega-upload mega-test mega-motor-test mega-terminal mega-keyboard mega-calibrate pc-mega-keyboard pc-ros-keyboard sim-build sim-stop sim sim-headless sim-topics sim-nav2 plotjuggler pi-bringup pc-camera-rviz pc-teddy-rviz camera-stop camera-reload

MEGA_UPLOAD_DEFAULT_SKETCH := mega_keyboard_drive
MEGA_UPLOAD_SKETCH := $(firstword $(filter-out mega-upload,$(MAKECMDGOALS)))

build:
	@echo "Building docker image with buildx (uses local cache)."
	$(MAKE) docker-buildx-build

shell:
	docker compose run --rm ros

up:
	docker compose up -d

down:
	docker compose down

# Docker Buildx helpers (use on Pi for faster cacheable builds)
.PHONY: docker-buildx-setup docker-buildx-build docker-buildx-clean

docker-buildx-setup:
	@echo "Setting up buildx builder (idempotent)"
	-docker buildx create --name mekk4-builder --use 2>/dev/null || true
	-docker buildx inspect --bootstrap 2>/dev/null || true
	@mkdir -p $(HOME)/.buildx-cache

docker-buildx-build: docker-buildx-setup
	@echo "Building image with buildx and local cache at $(HOME)/.buildx-cache"
	DOCKER_BUILDKIT=1 docker buildx build \
	  --builder mekk4-builder \
	  --cache-to=type=local,dest=$(HOME)/.buildx-cache \
	  --cache-from=type=local,src=$(HOME)/.buildx-cache \
	  --load \
	  -t mekk4/ros2-jazzy-dev:local \
	  docker/

docker-buildx-clean:
	@echo "Removing buildx builder and cache (safe to run)"
	-docker buildx rm mekk4-builder 2>/dev/null || true
	-rm -rf $(HOME)/.buildx-cache 2>/dev/null || true

# Build ROS workspace + fix console_script shebang to venv python (PEP668-safe)
ws:
	docker compose run --rm ros bash -lc '/ws/scripts/ws_build.sh'

lidar-setup:
	docker compose run --rm ros bash -lc '/ws/scripts/setup_ldlidar_driver.sh'

lidar-test:
	docker compose run --rm ros bash -lc '/ws/scripts/lidar_smoketest.sh'

mega-upload:
	bash ./scripts/mega_upload.sh "$(if $(MEGA_UPLOAD_SKETCH),$(MEGA_UPLOAD_SKETCH),$(if $(MEGA_SKETCH),$(MEGA_SKETCH),$(MEGA_UPLOAD_DEFAULT_SKETCH)))"

ifneq ($(filter mega-upload,$(MAKECMDGOALS)),)
$(filter-out mega-upload,$(MAKECMDGOALS)):
	@:
endif

mega-test:
	bash ./scripts/mega_smoketest.sh

mega-motor-test:
	bash ./scripts/mega_motor_test.sh

mega-terminal:
	bash ./scripts/mega_serial_terminal.sh

mega-keyboard:
	bash ./scripts/mega_keyboard_teleop.sh

mega-calibrate:
	bash ./scripts/mega_calibrate.sh $(ARGS)

pc-mega-keyboard:
	bash ./scripts/pc_mega_serial_keyboard.sh "$(if $(PI_HOST),$(PI_HOST),gruppe5@gruppe5pi5)"

pc-ros-keyboard:
	bash ./scripts/pc_ros_keyboard.sh "$(if $(PI_HOST),$(PI_HOST),gruppe5pi5)" $(ARGS)

# Native (non-Docker) simulation helpers for developer machines
sim-build:
	bash -lc 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install'

sim-stop:
	bash ./scripts/sim_stop.sh

sim: sim-stop
	bash -lc 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch robot_bringup minimal_all.launch.py use_teddy:=$(if $(WITH_TEDDY),$(WITH_TEDDY),true)'

sim-headless: sim-stop
	bash -lc 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch robot_bringup minimal_all.launch.py headless:=true rviz:=true use_teddy:=$(if $(WITH_TEDDY),$(WITH_TEDDY),true)'

sim-topics:
	bash -lc 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 topic list | grep -E "/clock|/odom|/lidar|/cmd_vel"'

plotjuggler:
	bash ./scripts/plotjuggler_sim_monitor.sh
	
pi-bringup:
	bash ./scripts/pi_bringup.sh

pc-camera-rviz:
	bash ./scripts/pc_udp_camera_rviz.sh "$(if $(PI_HOST),$(PI_HOST),gruppe5pi5)"

pc-teddy-rviz:
	bash ./scripts/pc_teddy_rviz.sh "$(if $(PI_HOST),$(PI_HOST),gruppe5pi5)"

camera-stop:
	bash ./scripts/camera_stop.sh

camera-reload:
	bash ./scripts/camera_reload.sh
