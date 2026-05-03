SHELL := /bin/bash

.PHONY: build shell up down ws lidar-setup lidar-test mega-upload mega-test mega-motor-test mega-terminal mega-keyboard mega-calibrate pc-mega-keyboard pc-ros-keyboard sim-build sim-stop sim sim-headless sim-topics sim-nav2 pi-bringup pc-teddy-rviz camera-stop camera-reload

MEGA_UPLOAD_DEFAULT_SKETCH := mega_keyboard_drive
MEGA_UPLOAD_SKETCH := $(firstword $(filter-out mega-upload,$(MAKECMDGOALS)))

build:
	docker compose build

shell:
	docker compose run --rm ros

up:
	docker compose up -d

down:
	docker compose down

# Short aliases for convenience
.PHONY: bd pb

bd: build
	@:

pb: pi-bringup
	@:

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
	bash -lc 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch robot_bringup minimal_all.launch.py use_teddy:=$(if $(WITH_TEDDY),$(WITH_TEDDY),true) use_teddy_approach:=$(if $(WITH_TEDDY_APPROACH),$(WITH_TEDDY_APPROACH),true)'

sim-headless: sim-stop
	bash -lc 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch robot_bringup minimal_all.launch.py headless:=true rviz:=true use_teddy:=$(if $(WITH_TEDDY),$(WITH_TEDDY),true) use_teddy_approach:=$(if $(WITH_TEDDY_APPROACH),$(WITH_TEDDY_APPROACH),true)'

sim-topics:
	bash -lc 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 topic list | grep -E "/clock|/odom|/lidar|/cmd_vel"'

sim-nav2:
	bash -lc 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch robot_bringup nav2_stack.launch.py use_sim_time:=true params_file:=$$PWD/config/nav2_params.yaml'
	
pi-bringup:
	bash ./scripts/pi_bringup.sh

pc-teddy-rviz:
	bash ./scripts/pc_teddy_rviz.sh "$(if $(PI_HOST),$(PI_HOST),gruppe5pi5)"

camera-stop:
	bash ./scripts/camera_stop.sh

camera-reload:
	bash ./scripts/camera_reload.sh
