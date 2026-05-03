# Simulering

## Bunnlinje

Simuleringen kjøres på host/PC, ikke i Docker. Dagens `make sim` starter den
fulle simstacken: Gazebo, ROS-GZ bridge, sim sensors, tracked-drive adapter, EKF,
Nav2, teddy-deteksjon, teddy-approach, RViz og keyboard teleop.

Ikke kjør `make sim-nav2` etter vanlig `make sim`; da starter du en ekstra Nav2
stack på de samme topicene.

## Kommandoer

```bash
cd ~/Mekatronikk-4-MEPA2002
make sim-build
make sim
```

| Kommando | Effekt |
|---|---|
| `make sim-build` | Bygger workspace lokalt |
| `make sim` | Full simstack med Gazebo GUI, bridge, EKF, Nav2, teddy-approach, RViz og keyboard teleop |
| `make sim-headless` | Full simstack med Gazebo server-only, EKF, Nav2, teddy-approach og RViz |
| `make sim-stop` | Stopper gamle simprosesser |
| `make sim-topics` | Lister sentrale topics |
| `make sim-nav2` | Starter bare Nav2. Brukes kun hvis sim allerede kjører uten Nav2 |

## Launch

Sim startes fra
[`minimal_all.launch.py`](../../src/robot_bringup/launch/minimal_all.launch.py).

Default launch-argumenter som betyr noe:

| Argument | Default | Effekt |
|---|---:|---|
| `headless` | `false` | Gazebo GUI på |
| `autostart` | `true` | Unpauser Gazebo automatisk |
| `rviz` | `true` | Starter RViz |
| `use_nav2` | `true` | Starter Nav2 via delt core stack |
| `use_ekf` | `true` | Starter EKF via delt core stack |
| `use_teddy` | `true` | Starter sim-kamera UDP adapter og teddy-detektor |
| `use_teddy_approach` | `true` | Starter teddy-approach controller og RViz-markører |
| `keyboard_teleop` | `true` | Starter GUI teleop når ikke headless |

Den starter:

- Gazebo world fra [`tracked_robot_world.sdf`](../../src/robot_gz/worlds/tracked_robot_world.sdf).
- ROS-GZ bridge for sensors og clock.
- Separat bridge for `/wheel/odom`.
- `tracked_cmd_vel_adapter`.
- Delt `pi_robot.launch.py` core stack med `use_sim_time:=true`, `use_nav2:=true` og `use_ekf:=true` som default.
- Sim-kamera via UDP til `teddy_detector`, og `teddy_approach_node` som publiserer `/cmd_vel_teddy`.
- RViz.
- Keyboard teleop hvis ikke headless.

## Bridge Topics

| Gazebo/ROS topic | ROS type |
|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` |
| `/joint_states` | `sensor_msgs/msg/JointState` |
| `/lidar` | `sensor_msgs/msg/LaserScan` |
| `/lidar/points` | `sensor_msgs/msg/PointCloud2` |
| `/imu/data` | `sensor_msgs/msg/Imu` |
| `/camera` | `sensor_msgs/msg/Image` |
| `/wheel/odom` | `nav_msgs/msg/Odometry` |

## Sim Sensors

Fra [`model.sdf`](../../src/robot_gz/models/tracked_robot/model.sdf):

| Sensor | Topic | Frame | Data |
|---|---|---|---|
| `camera` | `/camera` | `camera_link` | 640x480, 15 Hz |
| `imu` | `/imu/data` | `imu_link` | 100 Hz |
| `lidar` | `/lidar` | `base_laser` | 360 samples, 10 Hz, 0.08-10.0 m |

## Sim Cmd Vel

Gazebo tracked plugin bruker `/model/tracked_robot/cmd_vel`.
Resten av ROS-systemet bruker `/cmd_vel`.

[`tracked_cmd_vel_adapter.py`](../../src/robot_sim_control/robot_sim_control/tracked_cmd_vel_adapter.py)
oversetter:

```text
/cmd_vel
    -> tracked_cmd_vel_adapter
    -> /model/tracked_robot/cmd_vel
```

Default sim-parametre:

| Parameter | Verdi |
|---|---:|
| `sim_track_width_eff_m` | `0.184` |
| `sim_max_track_speed_mps` | `0.5555555555555556` |

## Nav2 I Sim

Normal bruk:

```bash
make sim
```

Dette inkluderer Nav2 allerede.

Hvis du vil starte sim uten Nav2 og så starte Nav2 separat, gjør det eksplisitt:

Terminal A:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_bringup minimal_all.launch.py use_nav2:=false
```

Terminal B:

```bash
make sim-nav2
```

`make sim-nav2` tilsvarer:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_bringup nav2_stack.launch.py \
  use_sim_time:=true \
  params_file:=$PWD/config/nav2_params.yaml
```

## Verifikasjon

```bash
ros2 topic list | grep -E "/clock|/odom|/lidar|/cmd_vel"
ros2 topic hz /lidar
ros2 topic hz /wheel/odom
ros2 run tf2_ros tf2_echo odom base_link
```

## Stoppe Gammel Sim

`make sim` og `make sim-headless` kjører `make sim-stop` først. Scriptet
[`scripts/sim_stop.sh`](../../scripts/sim_stop.sh) stopper gamle Gazebo-, bridge-,
Nav2-, EKF-, RViz- og teleop-prosesser som ellers kan publisere på samme topics.
