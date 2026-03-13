# Mekatronikk-4-MEPA2002

Kort bruk av repoet.

## Formål

1. Simulering av robot (Gazebo + ROS 2).
2. Kjøring på fysisk robot (Pi5 + Docker + ROS 2).
3. Teddy-deteksjon (`/teddy_detector/status`).

## 1) Bygg workspace (host)

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Mekatronikk-4-MEPA2002
colcon build --symlink-install
source install/setup.bash
```

## 2) Simulering (PC)

```bash
make sim
```

Headless:

```bash
make sim-headless
```

Sjekk topics:

```bash
make sim-topics
```

## 3) Nav2 i simulering

Terminal A:

```bash
make sim
```

Terminal B:

```bash
make sim-nav2
```

Direkte kommando (samme som `make sim-nav2`):

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Mekatronikk-4-MEPA2002
source install/setup.bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  map:=$PWD/maps/my_map.yaml \
  params_file:=$PWD/config/nav2_params.yaml
```

I RViz:

1. Sett `Fixed Frame` til `map`.
2. Klikk `2D Pose Estimate`.
3. Klikk `2D Goal Pose`.

Merk:

1. `Timed out waiting for transform ... chassis to map` er normalt til `2D Pose Estimate` er satt.
2. Alle terminaler må ha samme `ROS_DOMAIN_ID`.

## Feilsøking (kort)

Hvis Nav2 kjører men roboten ikke beveger seg:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Mekatronikk-4-MEPA2002
source install/setup.bash
ros2 topic list | grep -E '/clock|/odom|/lidar|/cmd_vel|/tf'
ros2 node list | grep -E 'parameter_bridge|amcl|planner_server|controller_server|bt_navigator|map_server'
```

## 4) Robot (Pi5 + Docker)

```bash
ssh gruppe5@gruppe5pi5
cd ~/Mekatronikk-4-MEPA2002
make build   # ved behov
make ws
make up
make shell
make down
```

## 5) Vision og LiDAR

```bash
make vision
make lidar-setup
make lidar-test
```
