# AGENTS

## Purpose
This file helps coding agents work effectively in this repository without guessing project conventions.

## Working Style (User Preference)
- Be direct and technical. Avoid filler.
- Ground claims in repository evidence: code, configs, launch files, scripts, or logs.
- State uncertainty explicitly and include how to verify it.
- Distinguish clearly between:
  - implemented now
  - implied but not implemented
  - planned only in comments/TODOs
- Prefer minimal, high-confidence changes before larger refactors.
- Preserve architecture unless there is a clear technical reason to change it.
- Prioritize practical, reproducible Linux commands.
- Treat hardware-facing and safety-relevant behavior conservatively.
- Prefer deterministic, explicit paths/devices/topics over auto-discovery when the correct endpoint is known.

## Project Baseline
- ROS distribution: Jazzy.
- Workspace type: ROS2 colcon workspace with source packages in [src](src).
- Pi workflow: headless SSH operation is normal.
- Containerized robot runtime is defined in [compose.yml](compose.yml) and [docker/Dockerfile](docker/Dockerfile).

## Canonical Commands
Use these first unless a task requires something else.

### Docker + workspace
- Build image: `make build`
- Build workspace in container: `make ws`
- Open container shell: `make shell`

Reference: [Makefile](Makefile), [scripts/ws_build.sh](scripts/ws_build.sh)

### Local simulation (host)
- Build host workspace: `make sim-build`
- Sim with GUI: `make sim`
- Sim headless: `make sim-headless`
- Run Nav2 in sim: `make sim-nav2`

Reference: [Makefile](Makefile), [src/robot_bringup/launch/minimal_all.launch.py](src/robot_bringup/launch/minimal_all.launch.py)

### Physical robot bringup (Pi)
- Standard bringup: `make pi-bringup`
- Common toggles: `WITH_NAV2=0`, `WITH_TEDDY=0/1`, `WITH_IMU=0/1`, `WITH_MEGA_DRIVER=0/1`, `WITH_EKF=0/1`
- Host override when discovery is wrong: `PC_HOST=<pc-ip> make pi-bringup`

Reference: [scripts/pi_bringup.sh](scripts/pi_bringup.sh), [README.md](README.md)

### Pi Repository Setup (Sparse Checkout)
Pi runs from `main` branch using **git sparse checkout** to exclude simulation-only code.

**Key fact:** Pi clones only these directories:
- `src/mekk4_bringup`, `src/mekk4_perception`, `src/robot_bringup`, `src/robot_description`
- `config`, `scripts`, `docker`

**Pi does NOT clone** (by design):
- `src/robot_gz` (Gazebo world/models)
- `src/robot_sim_control` (sim adapter)
- `models/` (sim asset models)
- `arduino/` (not needed at runtime)
- `docs/wiki/` (documentation accessed on PC)

When adding new files/directories:
- Hardware and control code: will auto-appear on Pi via `git pull`.
- Simulation-only code: won't appear on Pi unless added to sparse-checkout list.

Setup reference: [scripts/pi_sparse_checkout_setup.sh](scripts/pi_sparse_checkout_setup.sh)

## Architecture Boundaries
Primary packages in [src](src):
- [src/robot_bringup](src/robot_bringup): launch orchestration.
- [src/mekk4_bringup](src/mekk4_bringup): hardware-oriented nodes (Mega, IMU, cmd mux, teleop).
- [src/mekk4_perception](src/mekk4_perception): teddy detector and UDP camera bridge.
- [src/robot_description](src/robot_description): URDF and meshes.
- [src/robot_gz](src/robot_gz): Gazebo models/world.
- [src/robot_sim_control](src/robot_sim_control): simulation-only tracked-drive command adapter.

Treat [build](build), [install](install), and [log](log) as generated artifacts.

## Interface Contracts To Preserve
Before changing launch/config code, check these files:
- Launch wiring: [src/robot_bringup/launch/pi_robot.launch.py](src/robot_bringup/launch/pi_robot.launch.py)
- Sim wiring: [src/robot_bringup/launch/minimal_all.launch.py](src/robot_bringup/launch/minimal_all.launch.py)
- EKF fusion assumptions: [config/ekf.yaml](config/ekf.yaml)
- Nav2 frames/topics: [config/nav2_params.yaml](config/nav2_params.yaml)
- Camera/YOLO transport assumptions: [config/camera_params.yaml](config/camera_params.yaml)

Notable current assumptions from code/config:
- Nav2 and costmaps are configured around odom/base_link and /lidar in [config/nav2_params.yaml](config/nav2_params.yaml).
- EKF currently fuses wheel odometry velocity and IMU yaw (not full pose fusion) in [config/ekf.yaml](config/ekf.yaml).
- Pi bringup defaults include explicit serial and frame parameters in [scripts/pi_bringup.sh](scripts/pi_bringup.sh).

## Common Pitfalls
- Source order matters for commands run outside Docker:
  1) `source /opt/ros/jazzy/setup.bash`
  2) `source install/setup.bash`
- Many runtime issues are discovery/network mismatches (ROS domain/peers), not code bugs.
- Camera stream port/resolution changes are not equivalent to a light reload; see [config/camera_params.yaml](config/camera_params.yaml) comments.
- Rebuilding the Docker image is often unnecessary for source-only changes; workspace rebuild is usually enough.

## Agent Change Policy
- Do not introduce broad refactors by default.
- Keep hardware and simulation behavior explicitly separated.
- Avoid adding fallback/autodiscovery layers unless required by a demonstrated failure mode.
- Keep comments short and intent-focused.
- After edits, run the smallest relevant verification command and report exact result.

## Where To Read More
- Primary workflow and operator commands: [README.md](README.md)
- LiDAR and RViz workflow: [docs/lidar_rviz.md](docs/lidar_rviz.md)
- Docker runtime and device mapping: [compose.yml](compose.yml)
- Build caveats and shebang patching: [scripts/ws_build.sh](scripts/ws_build.sh)
