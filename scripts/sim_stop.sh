#!/usr/bin/env bash
set -euo pipefail

# Stop active sim/ROS processes that can publish into the same topics as minimal_all.
patterns=(
  "ros2 launch robot_bringup minimal_all.launch.py"
  "ros2 launch robot_bringup pi_robot.launch.py"
  "ros2 launch robot_bringup nav2_stack.launch.py"
  "ros2 run ros_gz_bridge parameter_bridge"
  "lib/ros_gz_bridge/parameter_bridge"
  "gz sim "
  "robot_state_publisher"
  "ekf_node"
  "controller_server"
  "planner_server"
  "behavior_server"
  "bt_navigator"
  "velocity_smoother"
  "collision_monitor"
  "lifecycle_manager_navigation"
  "sim_cmd_vel_calibrator"
  "cmd_vel_mux_node"
  "nav_cmd_vel_flip_node"
  "ros_keyboard_teleop"
  "sim_keyboard_teleop"
  "lidar_static_tf_sim"
  "rviz2.*rviz.rviz"
)

found=0
for pattern in "${patterns[@]}"; do
  if pgrep -f "${pattern}" >/dev/null 2>&1; then
    found=1
    pkill -f "${pattern}" 2>/dev/null || true
  fi
done

# Give nodes a moment to exit cleanly, then force-kill leftovers.
sleep 0.8
for pattern in "${patterns[@]}"; do
  if pgrep -f "${pattern}" >/dev/null 2>&1; then
    pkill -9 -f "${pattern}" 2>/dev/null || true
  fi
done

if [[ "${found}" == "1" ]]; then
  echo "[sim-stop] Stopped existing sim/ROS processes." >&2
else
  echo "[sim-stop] No existing sim/ROS processes found." >&2
fi
