# LiDAR + RViz (Pi + PC)

Kort oppskrift for å se LiDAR live i RViz.

Kamera- og YOLO-innstillinger ligger i [config/camera_params.yaml](/home/emiliam/Mekatronikk-4-MEPA2002/config/camera_params.yaml).
`camera_stream` gjelder felles stream til baade teddy-detektor og RViz, mens `teddy_detector` bare gjelder YOLO-parametre.
Hvis fargene ser feil ut, er `camera_stream.awb`, `awb_gains`, `brightness`, `contrast`, `saturation`, `sharpness`, `denoise` og `ev` de foerste feltene aa justere.

## 1) Start robotstack på Pi

```bash
cd ~/Mekatronikk-4-MEPA2002
make pi-bringup
```

Dette scriptet:

1. finner PC-IP fra SSH-sesjonen
2. setter `ROS_DOMAIN_ID`, `ROS_AUTOMATIC_DISCOVERY_RANGE` og `ROS_STATIC_PEERS`
3. starter `robot_state_publisher`, LiDAR og Nav2 i samme Docker-kjoering

Hvis du bare vil se LiDAR + TF og ikke starte Nav2:

```bash
WITH_NAV2=0 make pi-bringup
```

## 2) Sett ROS discovery på PC

Fra PC-en:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Mekatronikk-4-MEPA2002
eval "$(bash scripts/ros_discovery_env.sh pc gruppe5pi5)"
```

Hvis hostnavnet ikke virker, bruk IP direkte:

```bash
eval "$(bash scripts/ros_discovery_env.sh pc 192.168.10.55)"
```

## 3) Verifiser fra PC

```bash
ros2 topic list | grep lidar
ros2 topic hz /lidar
ros2 topic echo --once /tf_static
```

## 4) Start RViz på PC

```bash
rviz2 -d src/robot_bringup/rviz/pre_odom_lidar.rviz
```

Denne konfigen er laget for situasjonen deres akkurat naa:

1. `Fixed Frame = chassis`
2. `LaserScan` bruker `/lidar` med `Reliability = Best Effort`
3. `RobotModel` bruker `/robot_description`
4. `TF`-display er av som standard, siden det fort bare blir visuelt stoy uten `odom`

Hvis du vil sette opp manuelt i stedet:

1. `Global Options -> Fixed Frame = chassis`
2. `Add -> LaserScan`
3. `Topic = /lidar`
4. Under `Topic`:
   `Reliability Policy = Best Effort`
5. `Add -> RobotModel` og bruk `/robot_description`
6. Ikke bruk `odom` eller `map` som fixed frame foer dere faktisk har odometri

## 5) Valgfritt: kamera til RViz over UDP

Pi:

```bash
WITH_NAV2=0 WITH_TEDDY=1 WITH_CAMERA_RVIZ=1 make pi-bringup
```

Dette beholder teddy-detektor paa Pi, men sender i tillegg H264 over UDP til PC-en paa port `5601`.

PC:

```bash
make pc-camera-rviz PI_HOST=gruppe5pi5
```

Det starter en lokal ROS-node paa PC-en som dekoder UDP-stroemmen og publiserer `/camera`, slik at RViz-konfigen viser baade LiDAR og bilde.

## Feilsøking (kort)

- `empty topicname`: `LaserScan`-display har tom `Topic`. Sett `/lidar`.
- Ingen topic på PC: kjør `eval "$(bash scripts/ros_discovery_env.sh pc <pi-host-eller-ip>)"` på nytt og sjekk at `make pi-bringup` fortsatt kjører på Pi.
- Hvis `RobotModel` ser rar ut, men `tf2_echo chassis base_laser` ser riktig ut: stol heller på LiDAR + TF foreløpig. Dere publiserer ikke `/joint_states` ennå.
- Hvis kamera ikke vises på PC: sjekk at PC-workspace er bygget, at `gst-launch-1.0` finnes på PC, og at UDP-port `5601` ikke blokkeres.
- Hvis `/dev/ttyAMA0` ikke virker: test `port_name:=/dev/serial0`.
- Hvis auto-detection på Pi bommer: bruk `PC_HOST=<pc-ip> make pi-bringup`.
