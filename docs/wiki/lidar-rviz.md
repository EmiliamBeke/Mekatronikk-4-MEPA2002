# LiDAR og RViz

## Bunnlinje

Denne siden er den praktiske oppskriften for å få LiDAR, TF og kamera/YOLO-bilde
opp i RViz fra fysisk robot. Standardvisningen på PC er `make pc-teddy-rviz`,
som viser `/lidar` og et annotert YOLO-bilde på `/camera`.

## Relevant Konfig

| Del | Fil |
|---|---|
| LiDAR launch | [`lidar_nav2_compat.launch.py`](../../src/robot_bringup/launch/lidar_nav2_compat.launch.py) |
| Pi bringup | [`pi_bringup.sh`](../../scripts/pi_bringup.sh) |
| RViz config | [`rviz.rviz`](../../src/robot_bringup/rviz/rviz.rviz) |
| Kamera/YOLO | [`camera_params.yaml`](../../config/camera_params.yaml) |

LiDAR default:

| Parameter | Verdi |
|---|---|
| `product_name` | `LDLiDAR_LD06` |
| `port_name` | `/dev/ttyAMA0` |
| `port_baudrate` | `230400` |
| `topic_name` | `/lidar` |
| `frame_id` | `base_laser` |
| `mount_frame` | `lidar_link` |

## 1. Start Robotstack På Pi

Fra SSH på Pi:

```bash
cd ~/Mekatronikk-4-MEPA2002
make pi-bringup
```

Hvis du bare vil se LiDAR, TF og teddy uten Nav2:

```bash
WITH_NAV2=0 make pi-bringup
```

Hvis du vil stoppe motor/Mega-driver mens du tester sensorer:

```bash
WITH_NAV2=0 WITH_MEGA_DRIVER=0 make pi-bringup
```

Hva Pi-bringup gjør relevant for RViz:

1. Setter ROS discovery mot PC.
2. Starter `robot_state_publisher`.
3. Starter LiDAR-driver via `pi_robot.launch.py` sin default `use_lidar:=true`.
4. Starter teddy-detektor hvis `WITH_TEDDY=1`.
5. Sender annotert YOLO-video til PC hvis `stream_debug_video: true`.

## 2. Start RViz På PC

Standard:

```bash
cd ~/Mekatronikk-4-MEPA2002
make pc-teddy-rviz
```

Hvis hostname ikke virker:

```bash
make pc-teddy-rviz PI_HOST=192.168.10.55
```

Dette starter lokal UDP bridge for annotert YOLO-stream og åpner RViz med
[`rviz.rviz`](../../src/robot_bringup/rviz/rviz.rviz).

## 3. Verifiser Fra PC

```bash
ros2 topic list | grep lidar
ros2 topic hz /lidar
ros2 topic echo --once /tf_static
ros2 topic list | grep teddy_detector
ros2 topic echo --once /teddy_detector/status
```

Hvis `/lidar` finnes, men RViz ikke viser scan, sjekk QoS i RViz.

## 4. RViz-oppsett

Standard RViz config er laget for nåværende robotoppsett:

| Display | Verdi |
|---|---|
| Fixed Frame | `odom` |
| LaserScan topic | `/lidar` |
| LaserScan reliability | `Best Effort` |
| RobotModel | `/robot_description` |
| Image topic | `/camera` |

Manuelt oppsett:

1. `Global Options -> Fixed Frame = odom`.
2. `Add -> LaserScan`.
3. Sett topic til `/lidar`.
4. Sett `Reliability Policy = Best Effort`.
5. `Add -> RobotModel`, bruk `/robot_description`.
6. `Add -> Image`, bruk `/camera`.
7. Slå på `TF` hvis du vil debugge frames.

## 5. YOLO-video

Annotert YOLO-video bruker UDP-port `5602` og vises med:

```bash
make pc-teddy-rviz
```

## 6. Vanlige Feil

| Symptom | Sjekk |
|---|---|
| `empty topicname` i RViz | LaserScan-displayet har tom topic. Sett `/lidar`. |
| Ingen `/lidar` på PC | Sjekk ROS discovery, Pi bringup og `PI_HOST`/`PC_HOST`. |
| `/lidar` finnes, men vises ikke | Sett LaserScan QoS til `Best Effort`. |
| TF-feil for scan | Sjekk `ros2 run tf2_ros tf2_echo base_link base_laser`. |
| RobotModel ser rar ut | Stol på LiDAR + TF først; joint states kan være statiske/null. |
| YOLO-bilde mangler | Sjekk `make pc-teddy-rviz`, port `5602` og `stream_debug_video`. |
| `/dev/ttyAMA0` feiler | Test `PORT_NAME=/dev/serial0 make pi-bringup`. |

## 7. Minimal LiDAR-test

```bash
make lidar-test
```

Dette kjøres i Docker via [`scripts/lidar_smoketest.sh`](../../scripts/lidar_smoketest.sh).
Bruk det når du vil isolere LiDAR fra resten av robotstacken.

## 8. Kamera-Tuning Som Påvirker RViz-bildet

Kamera- og YOLO-innstillinger ligger i
[`camera_params.yaml`](../../config/camera_params.yaml).

`camera_stream` påvirker input til både teddy-detektor og rå RViz-stream.
`teddy_detector` påvirker YOLO, status og annotert debugvideo.

Hvis fargene ser feil ut, start med:

- `camera_stream.awb`
- `camera_stream.awb_gains`
- `camera_stream.brightness`
- `camera_stream.contrast`
- `camera_stream.saturation`
- `camera_stream.sharpness`
- `camera_stream.denoise`
- `camera_stream.ev`

Reload etter tuning:

```bash
make camera-reload
```
