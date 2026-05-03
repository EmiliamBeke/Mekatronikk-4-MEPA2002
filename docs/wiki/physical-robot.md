# Fysisk Robot Bringup

## Bunnlinje

Standard fysisk workflow er:

```bash
ssh gruppe5@gruppe5pi5
cd ~/Mekatronikk-4-MEPA2002
make pi-bringup
```

Dette starter robotstacken i Docker på Pi og kamera-stream på Pi-host.

## Standard Defaults

Fra [`scripts/pi_bringup.sh`](../../scripts/pi_bringup.sh):

| Variabel | Default | Effekt |
|---|---:|---|
| `WITH_NAV2` | `1` | Starter Nav2 |
| `WITH_TEDDY` | `1` | Starter teddy-detektor |
| `WITH_IMU` | `1` | Starter BNO085 node |
| `WITH_MEGA_DRIVER` | `1` | Starter ROS Mega-driver |
| `WITH_EKF` | `1` | Starter EKF |
| `PORT_NAME` | `/dev/ttyAMA0` | LiDAR serial |
| `PORT_BAUDRATE` | `230400` | LiDAR baudrate |
| `PRODUCT_NAME` | `LDLiDAR_LD06` | LiDAR driver type |
| `LIDAR_FRAME` | `base_laser` | LaserScan frame |
| `BASE_FRAME` | `base_link` | Mega odom child frame |
| `IMU_FRAME` | `imu_link` | IMU frame |
| `MEGA_PORT` | `/dev/ttyACM0` | Arduino Mega serial |
| `MEGA_BAUDRATE` | `115200` | Mega baudrate |

## Hva Scriptet Gjør

1. Leser kamera-parametre fra [`config/camera_params.yaml`](../../config/camera_params.yaml).
2. Leser robotkalibrering fra [`config/robot_calibration.yaml`](../../config/robot_calibration.yaml).
3. Setter ROS discovery mot PC.
4. Bygger workspace i container hvis installasjonen mangler eller er stale.
5. Starter lokal kamera UDP stream hvis `WITH_TEDDY=1`.
6. Mapper Mega device inn i container hvis `WITH_MEGA_DRIVER=1`.
7. Kjører [`pi_robot.launch.py`](../../src/robot_bringup/launch/pi_robot.launch.py).

## Vanlige Varianter

```bash
# Sensor/vision uten Nav2
WITH_NAV2=0 make pi-bringup

# Uten Mega-driver hvis serial-porten brukes av annet verktøy
WITH_MEGA_DRIVER=0 make pi-bringup

# Uten EKF: Mega-driver publiserer /odom og TF selv
WITH_EKF=0 make pi-bringup

# Bare bringup uten teddy
WITH_TEDDY=0 make pi-bringup

# Overstyr PC-IP
PC_HOST=192.168.10.42 make pi-bringup

# Overstyr Mega-port
MEGA_PORT=/dev/ttyACM1 make pi-bringup
```

## Launch Innhold

[`pi_robot.launch.py`](../../src/robot_bringup/launch/pi_robot.launch.py) kan starte:

- `robot_state_publisher`
- `zero_joint_state_publisher`
- `ldlidar`
- `teddy_detector`
- `bno085`
- `mega_driver`
- `ekf_filter_node`
- Nav2 stack
- RViz hvis `rviz:=true`

## EKF-Modus

Når `WITH_EKF=1`:

| Mega-driver setting | Verdi |
|---|---|
| `mega_odom_topic` | `wheel/odom` |
| `mega_publish_tf` | `false` |

EKF publiserer da `/odom` og TF.

Når `WITH_EKF=0`:

| Mega-driver setting | Verdi |
|---|---|
| `mega_odom_topic` | `odom` |
| `mega_publish_tf` | `true` |

Mega-driver publiserer da rå `/odom`.

## ROS Discovery

Pi prøver å finne PC-IP fra SSH-sesjonen. Hvis det bommer:

```bash
PC_HOST=<pc-ip> make pi-bringup
```

Verifiser på Pi:

```bash
env | grep ROS_
```

Verifiser på PC:

```bash
make pc-teddy-rviz PI_HOST=<pi-ip>
```
