SHELL := /bin/bash

.PHONY: build shell up down ws vision lidar-setup lidar-test sim-build sim sim-headless sim-topics

build:
	docker compose build

shell:
	docker compose run --rm ros

up:
	docker compose up -d

down:
	docker compose down
# Build ROS workspace + fix console_script shebang to venv python (PEP668-safe)
ws:
	docker compose run --rm ros bash -lc '/ws/scripts/ws_build.sh'

vision:
	./scripts/vision_stream.sh

lidar-setup:
	docker compose run --rm ros bash -lc '/ws/scripts/setup_ldlidar_driver.sh'

lidar-test:
	docker compose run --rm ros bash -lc '/ws/scripts/lidar_smoketest.sh'

# Native (non-Docker) simulation helpers for developer machines
sim-build:
	bash -lc 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install'

sim:
	bash -lc 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch robot_bringup minimal_all.launch.py'

sim-headless:
	bash -lc 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch robot_bringup minimal_all.launch.py headless:=true'

sim-topics:
	bash -lc 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 topic list | grep -E "/clock|/odom|/lidar|/cmd_vel"'

