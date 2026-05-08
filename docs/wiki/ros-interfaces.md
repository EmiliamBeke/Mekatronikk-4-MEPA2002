# ROS Interfaces

## Bunnlinje

Før du endrer launch, Nav2, EKF eller hardware-noder: sjekk topics og frames her.
Dette er kontraktene resten av systemet forventer.

## Frames

| Frame | Rolle | Kilde |
|---|---|---|
| `odom` | Lokal odometriframe | Mega-driver eller EKF |
| `base_link` | Nav2 robotbase | URDF og Nav2 config |
| `chassis` | Fysisk chassislink | URDF |
| `lidar_link` | LiDAR mount | URDF |
| `base_laser` | LaserScan frame | LiDAR launch og Gazebo |
| `imu_link` | IMU frame | URDF og BNO085 node |
| `camera_link` | Kamera frame | URDF og camera bridge |
| `map` | Kartframe | SLAM/EKF config, ikke hovedframe i Nav2 default |

Viktig frame-kontrakt:

- [`config/nav2_params.yaml`](../../config/nav2_params.yaml) bruker `robot_base_frame: base_link`.
- [`scripts/pi_bringup.sh`](../../scripts/pi_bringup.sh) setter `BASE_FRAME=base_link` som default til Mega-driver.
- [`config/ekf.yaml`](../../config/ekf.yaml) bruker `base_link_frame: base_link`.

Verifiser TF før fysisk Nav2-kjøring:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link chassis
ros2 run tf2_ros tf2_echo base_link base_laser
```

## Topics

| Topic | Type | Produsent | Bruk |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Collision monitor eller manuell test | Sluttkommando til Mega/sim |
| `/cmd_vel_manual` | `geometry_msgs/msg/Twist` | `ros_keyboard_teleop` | Manuell override |
| `/cmd_vel_nav_auto` | `geometry_msgs/msg/Twist` | Nav2 controller/behavior | Rå Nav2 output |
| `/cmd_vel_teddy` | `geometry_msgs/msg/Twist` | `teddy_approach` og `teddy_grab` | Assist-input til cmd_vel-mux |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | `cmd_vel_mux_node` | Valgt nav/manual input til smoother |
| `/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | `velocity_smoother` | Smoothed cmd_vel |
| `/cmd_vel_nav_flipped` | `geometry_msgs/msg/Twist` | `nav_cmd_vel_flip_node` | Input til collision monitor |
| `/cmd_vel_mux_active` | `std_msgs/msg/String` | `cmd_vel_mux_node` | `manual`, `nav` eller `idle` |
| `/odom` | `nav_msgs/msg/Odometry` | Mega-driver eller EKF | Nav2 og RViz |
| `/wheel/odom` | `nav_msgs/msg/Odometry` | Mega-driver når EKF er på | EKF input |
| `/imu/data` | `sensor_msgs/msg/Imu` | BNO085 eller Gazebo bridge | EKF input |
| `/lidar` | `sensor_msgs/msg/LaserScan` | LDLiDAR eller Gazebo bridge | Nav2 costmaps og RViz |
| `/lidar/points` | `sensor_msgs/msg/PointCloud2` | Gazebo bridge | Sim debug |
| `/camera` | `sensor_msgs/msg/Image` | UDP camera bridge eller Gazebo bridge | RViz image |
| `/teddy_detector/status` | `std_msgs/msg/String` | `teddy_detector` | Teddy count/offset/FPS |
| `/mega/distance_mm` | `std_msgs/msg/Int32` | `mega_driver` | VL53-avstand fra Mega `STATE D=<mm>` |
| `/robotarm/request/x_position` | `std_msgs/msg/Float64` | `teddy_grab` eller operatør | X-request til `robotarm_safety` |
| `/robotarm/request/z_position` | `std_msgs/msg/Float64` | `teddy_grab` eller operatør | Z-request til `robotarm_safety` |

## Nodes

| Node | Pakke | Rolle |
|---|---|---|
| `robot_state_publisher` | `robot_state_publisher` | Publiserer URDF TF |
| `zero_joint_state_publisher` | `mekk4_bringup` | Publiserer null joint states |
| `ldlidar` | `ldlidar_stl_ros2` | LiDAR driver |
| `bno085` | `mekk4_bringup` | IMU driver |
| `mega_driver` | `mekk4_bringup` | Serial motor/odom driver |
| `ekf_filter_node` | `robot_localization` | EKF |
| `teddy_detector` | `mekk4_perception` | YOLO teddy detector |
| `teddy_approach` | `mekk4_bringup` | Sentrerer roboten mot bamse og publiserer `/cmd_vel_teddy` |
| `teddy_grab` | `mekk4_bringup` | Stopper basen, bruker arm/gripper og Mega-avstand for grep |
| `udp_camera_bridge` | `mekk4_perception` | UDP H264 til ROS image |
| `cmd_vel_mux` | `mekk4_bringup` | Manual override over Nav2 |
| `nav_cmd_vel_flip` | `mekk4_bringup` | Valgfri angular flip |
| `tracked_cmd_vel_adapter` | `robot_sim_control` | Sim cmd_vel til tracked plugin |

## Nav2 Command Chain

Definert i [`nav2_stack.launch.py`](../../src/robot_bringup/launch/nav2_stack.launch.py):

```text
controller_server / behavior_server
  -> cmd_vel_nav_auto
  -> cmd_vel_mux_node
  -> cmd_vel_nav
  -> velocity_smoother
  -> cmd_vel_smoothed
  -> nav_cmd_vel_flip_node
  -> cmd_vel_nav_flipped
  -> collision_monitor
  -> cmd_vel
```

Manual override:

```text
ros_keyboard_teleop
  -> /cmd_vel_manual
  -> cmd_vel_mux_node
  -> cmd_vel_nav
```

Teddy assist:

```text
teddy_approach eller teddy_grab
  -> /cmd_vel_teddy
  -> cmd_vel_mux_node
  -> cmd_vel_nav
```

`cmd_vel_mux_node` prioriterer `manual` over `assist` over `nav`. `teddy_grab`
publiserer null-Twist på `/cmd_vel_teddy` mens gripe-sekvensen kjører, så basen
skal stå stille gjennom muxen.

## Teddy Grab

Konfig ligger i [`config/teddy_grab.yaml`](../../config/teddy_grab.yaml).

Implementert nå:

- Starter på `/teddy_approach/mode == close_enough_lidar`.
- Holder basen stoppet ved å publisere null `Twist` på `/cmd_vel_teddy`.
- Leser Mega VL53-avstand på `/mega/distance_mm`.
- Flytter X framover til `approach_x_max` i `approach_x_step_m` steg.
- Lukker gripper først når avstanden er `contact_distance_mm` i minst `contact_hold_s`.

Høyde fra `teddy_detector` er bare empirisk:

- `teddy_detector` publiserer `dy`, ikke kalibrert 3D-posisjon.
- `use_detector_dy_for_grab_z: true` bruker `lower_z + dy * dy_to_z_gain_m`.
- Gain-fortegn må testes fysisk; feil fortegn flytter gripperen motsatt vei.

Foretrukket høydeestimat før kameraet blokkeres:

- `use_lidar_geometry_for_grab_z: true` sampler siste `dy` og front-LiDAR når grab starter.
- Formelen i [`config/teddy_grab.yaml`](../../config/teddy_grab.yaml) er:
  `z = origin + sign * lidar_m * tan(camera_pitch_down + dy * vertical_fov/2) + offset`.
- Tune først `camera_pitch_down_rad`, `lidar_geometry_z_origin_m` og `lidar_geometry_z_offset_m`.
- Hvis høyden går feil vei når bamsen er lavt/høyt i bildet, bytt fortegn på `lidar_geometry_z_sign`.

## Verifikasjonskommandoer

```bash
ros2 node list
ros2 topic list
ros2 topic hz /lidar
ros2 topic hz /odom
ros2 topic hz /imu/data
ros2 topic echo --once /cmd_vel_mux_active
ros2 topic echo --once /teddy_detector/status
ros2 topic echo /mega/distance_mm
```
