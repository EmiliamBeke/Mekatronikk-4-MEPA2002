# PC-verktøy

## Bunnlinje

PC-en brukes til RViz, UDP camera bridge og ROS keyboard teleop. Den skal ha
ROS 2 Jazzy workspace bygget lokalt.

## Første Setup På PC

```bash
cd ~/Mekatronikk-4-MEPA2002
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## RViz Med Annotert YOLO-bilde

```bash
cd ~/Mekatronikk-4-MEPA2002
make pc-teddy-rviz
```

Hvis Pi hostname ikke løses:

```bash
make pc-teddy-rviz PI_HOST=192.168.10.55
```

Dette kjører:

- [`scripts/pc_teddy_rviz.sh`](../../scripts/pc_teddy_rviz.sh)
- [`scripts/pc_udp_camera_rviz.sh`](../../scripts/pc_udp_camera_rviz.sh)
- [`pc_camera_view.launch.py`](../../src/mekk4_bringup/launch/pc_camera_view.launch.py)
- `rviz2 -d src/robot_bringup/rviz/rviz.rviz`

Default topic for bildet er `/camera`, selv om bildet er annotert YOLO-output.

Annotert teddy-stream bruker UDP-port `5602`.

## ROS Keyboard Teleop

```bash
make pc-ros-keyboard
```

Dette publiserer til `/cmd_vel_manual`.

Nyttige varianter:

```bash
make pc-ros-keyboard ARGS="--speed 0.15 --turn-speed 0.6"
make pc-ros-keyboard PI_HOST=192.168.10.55
```

Taster:

| Tast | Effekt |
|---|---|
| Hold `W` | Frem |
| Hold `S` | Bak |
| Hold `A` | Sving venstre |
| Hold `D` | Sving høyre |
| `E` / `Q` | Øk/senk lineær hastighet |
| `P` / `O` | Øk/senk svinghastighet |
| `SPACE` | Stopp |
| `-` | Avslutt |

## Direkte Mega Keyboard

```bash
make pc-mega-keyboard
```

Dette er en SSH/serial bridge til Mega. Bruk denne når du vil styre Mega direkte
uten ROS Mega-driver.

Viktig: `pc-mega-keyboard` og `mega_driver_node` kan ikke bruke samme serial-port
samtidig.

## PC GStreamer Krav

[`pc_udp_camera_rviz.sh`](../../scripts/pc_udp_camera_rviz.sh) krever:

- `gst-launch-1.0`
- `gst-inspect-1.0`
- `h264parse`
- `rtph264depay`
- `decodebin`
- `videoconvert`

Hvis en plugin mangler:

```bash
sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-bad gstreamer1.0-libav
```
