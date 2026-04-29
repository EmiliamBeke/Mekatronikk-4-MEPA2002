# Maskinvare og Sensorer

## Bunnlinje

Dette er bare det repoet faktisk dokumenterer gjennom kode, launch, config,
Docker og modellfiler. Ting som ikke finnes i repoet er markert som usikkert.

## Oversikt

| Del | Navn i repo | Interface | Default |
|---|---|---|---|
| Pi | Raspberry Pi 5 antatt | Host + Docker | `BLINKA_FORCEBOARD=RASPBERRY_PI_5` |
| LiDAR | `LDLiDAR_LD06` | Serial | `/dev/ttyAMA0`, `230400` baud |
| IMU | `BNO085` / BNO08x | I2C | `/dev/i2c-1`, frame `imu_link` |
| Kamera | Pi camera via `rpicam-vid` | H264/RTP UDP | `5600`, `5601`, `5602` |
| Motorcontroller | Arduino Mega | USB serial | `/dev/ttyACM0`, `115200` baud |
| Encodere | `ENC1`, `ENC2` | Arduino interrupt pins | M1 pins `3/2`, M2 pins `18/19` |
| Motor driver | DFR0601-testsketch indikerer DFR0601 | Arduino digital/PWM | M1/M2 pins under |

## Pi / Docker Devices

[`compose.yml`](../../compose.yml) mapper:

| Host device | Container | Bruk |
|---|---|---|
| `/dev/ttyAMA0` | `/dev/ttyAMA0` | LiDAR |
| `/dev/i2c-1` | `/dev/i2c-1` | BNO085 |
| `/dev/gpiochip0` | `/dev/gpiochip0` | Blinka/GPIO |
| `${COMPOSE_MEGA_DEVICE}` | samme path | Arduino Mega når aktiv |

Group IDs settes dynamisk i [`scripts/pi_bringup.sh`](../../scripts/pi_bringup.sh)
for LiDAR, I2C og GPIO.

## LiDAR

LiDAR launch er definert i
[`lidar_nav2_compat.launch.py`](../../src/robot_bringup/launch/lidar_nav2_compat.launch.py).

Default:

| Parameter | Verdi |
|---|---|
| `product_name` | `LDLiDAR_LD06` |
| `port_name` | `/dev/ttyAMA0` |
| `port_baudrate` | `230400` |
| `topic_name` | `/lidar` |
| `frame_id` | `base_laser` |
| `mount_frame` | `lidar_link` |

Hvis `/dev/ttyAMA0` ikke passer:

```bash
PORT_NAME=/dev/serial0 make pi-bringup
```

## IMU

IMU-node ligger i
[`bno085_node.py`](../../src/mekk4_bringup/mekk4_bringup/bno085_node.py).

Default:

| Parameter | Verdi |
|---|---|
| `frame_id` | `imu_link` |
| `i2c_bus` | `1` |
| `publish_rate_hz` | `50.0` |
| `report_interval_us` | `20000` |
| `use_game_rotation_vector` | `true` |
| Topic | `/imu/data` |

Node leser quaternion, gyro og linear acceleration fra Adafruit BNO08x-driveren.

## Kamera

Kamera styres fra [`config/camera_params.yaml`](../../config/camera_params.yaml).

Default i nåværende config:

| Felt | Verdi |
|---|---|
| `width` | `800` |
| `height` | `600` |
| `fps` | `6` |
| `bitrate_bps` | `2800000` |
| `local_udp_port` | `5600` |
| `remote_udp_port` | `5601` |
| `debug_stream_port` | `5602` |

Usikkert: eksakt fysisk kameramodell er ikke dokumentert i repoet.

## Arduino Mega Pins

Fra [`mega_keyboard_drive.ino`](../../arduino/mega_keyboard_drive/mega_keyboard_drive.ino):

Megaen har et Terminal Block Shield v1.1.0 montert. Tabellen under viser
firmware-pinnene. Skru-rekkefølge for shield-terminalene ligger i
[`arduino-mega.md`](arduino-mega.md).

| Signal | Pin |
|---|---:|
| `M1 INA` | `8` |
| `M1 INB` | `9` |
| `M1 PWM` | `10` |
| `M2 INA` | `30` |
| `M2 INB` | `31` |
| `M2 PWM` | `44` |
| `ENC1 Hall A` | `3` |
| `ENC1 Hall B` | `2` |
| `ENC2 Hall A` | `18` |
| `ENC2 Hall B` | `19` |

Firmware bruker quadrature decoding med interrupts på encoder-pinnene.

## Robotmodell

URDF: [`tracked_robot.urdf`](../../src/robot_description/urdf/tracked_robot.urdf)

Viktige linker:

- `base_link`
- `chassis`
- `lidar_link`
- `imu_link`
- `camera_link`
- `left_wheel`
- `right_wheel`
- `left_track_rviz`
- `right_track_rviz`

Simmodell: [`model.sdf`](../../src/robot_gz/models/tracked_robot/model.sdf)

Sim-sensorer:

| Sensor | Topic | Frame |
|---|---|---|
| `camera` | `/camera` | `camera_link` |
| `imu` | `/imu/data` | `imu_link` |
| `lidar` | `/lidar` | `base_laser` |
