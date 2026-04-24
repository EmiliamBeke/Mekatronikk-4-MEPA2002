# Feilsøking

## Bunnlinje

Start med å skille mellom fire feiltyper:

1. Workspace/source-problem.
2. ROS discovery/nettverk.
3. Hardware device/port.
4. TF/topic mismatch.

## Første Sjekk

```bash
cd ~/Mekatronikk-4-MEPA2002
git status --short
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 node list
ros2 topic list
```

På Pi i Docker-workflow:

```bash
make ws
make pi-bringup
```

## ROS Nettverk

På Pi:

```bash
env | grep ROS_
```

På PC:

```bash
eval "$(bash scripts/ros_discovery_env.sh pc gruppe5pi5)"
env | grep ROS_
ros2 topic list
```

Hvis hostname feiler:

```bash
PC_HOST=<pc-ip> make pi-bringup
make pc-teddy-rviz PI_HOST=<pi-ip>
```

## LiDAR

```bash
make lidar-setup
make lidar-test
ros2 topic hz /lidar
ros2 run tf2_ros tf2_echo chassis base_laser
```

Hvis device er feil:

```bash
PORT_NAME=/dev/serial0 make pi-bringup
```

Hvis RViz viser LaserScan-feil, sett LaserScan reliability til `Best Effort`.

Mer detaljert guide: [LiDAR og RViz](lidar-rviz.md).

## Kamera / YOLO

På Pi:

```bash
make camera-stop
make camera-reload
```

På PC:

```bash
make pc-teddy-rviz PI_HOST=<pi-ip>
ros2 topic hz /camera
```

Teddy status:

```bash
ros2 topic echo --once /teddy_detector/status
```

Hvis annotert bilde mangler:

- Sjekk `teddy_detector.stream_debug_video: true` i [`camera_params.yaml`](../../config/camera_params.yaml).
- Sjekk UDP-port `5602`.
- Sjekk PC GStreamer plugins.

## Mega

Verifiser firmware og serial:

```bash
MEGA_SKETCH=mega_keyboard_drive make mega-upload
make mega-calibrate ARGS="snapshot"
```

Test ROS-driver uten EKF:

```bash
WITH_MEGA_DRIVER=1 WITH_EKF=0 make pi-bringup
ros2 topic echo --once /odom
```

Hvis `/odom` ikke kommer:

- Sjekk at `left_m_per_tick` og `right_m_per_tick` ikke er `0.0`.
- Sjekk `MEGA_PORT`.
- Sjekk at ingen andre prosesser bruker serial-porten.

## EKF

```bash
ros2 topic hz /wheel/odom
ros2 topic hz /imu/data
ros2 topic echo --once /odom
ros2 run tf2_ros tf2_echo odom base_link
```

Hvis EKF ikke publiserer:

- Sjekk at `WITH_EKF=1`.
- Sjekk at `/wheel/odom` finnes.
- Sjekk at `/imu/data` finnes.
- Sjekk frame mismatch mellom `base_link` og `chassis`.

## Nav2

```bash
ros2 lifecycle nodes
ros2 topic echo --once /cmd_vel_mux_active
ros2 topic hz /cmd_vel
ros2 topic hz /lidar
ros2 run tf2_ros tf2_echo odom base_link
```

Vanlige feil:

- Ingen `odom -> base_link`.
- LiDAR frame kan ikke transformeres til robotbase.
- Manual teleop holder muxen i `manual`.
- Collision monitor stopper eller skalerer kommandoer.

## Sim

```bash
make sim-stop
make sim-build
make sim
make sim-topics
```

`make sim` starter allerede Nav2. Ikke kjør `make sim-nav2` i tillegg til vanlig
`make sim`, med mindre du startet `minimal_all.launch.py use_nav2:=false`.

Hvis sim ikke starter rent, stopp gamle prosesser først:

```bash
make sim-stop
```

## Pi Diskplass, Ytelse og Throttling

Se [Pi drift og vedlikehold](pi-maintenance.md) for forklaring av hver kommando:

- hva kommandoen gjør
- når den bør brukes
- hva som slettes
- risiko ved Docker prune og `rm -rf`
- CPU governor, temperatur og throttling
