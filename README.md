# Mekatronikk-4-MEPA2002

Wiki og operatørdokumentasjon for ROS 2-roboten i MEPA2002.

## Rask Start

### Simulering på PC

```bash
cd ~/Mekatronikk-4-MEPA2002
make sim-build
make sim
```

`make sim` starter dagens fulle simstack: Gazebo, bridge, EKF, Nav2, RViz og
keyboard teleop. Ikke start `make sim-nav2` i tillegg med mindre du bevisst har
startet sim uten Nav2.

### Fysisk robot på Pi

**Første gang (første gangs setup med sparse checkout):**

```bash
ssh gruppe5@gruppe5pi5
bash ~/pi_sparse_checkout_setup.sh
cd ~/Mekatronikk-4-MEPA2002
bash scripts/setup_ldlidar_driver.sh
```

**Senere oppstarting:**

```bash
ssh gruppe5@gruppe5pi5
cd ~/Mekatronikk-4-MEPA2002
make pi-bringup
```

### RViz på PC

```bash
cd ~/Mekatronikk-4-MEPA2002
make pc-teddy-rviz
```

Hvis hostname ikke virker:

```bash
make pc-teddy-rviz PI_HOST=<pi-ip>
```

## Wiki

Start her:

- [Wiki-forside](docs/wiki/index.md)
- [Systemoversikt](docs/wiki/system-overview.md)
- [Maskinvare og sensorer](docs/wiki/hardware.md)
- [LiDAR og RViz](docs/wiki/lidar-rviz.md)
- [ROS interfaces](docs/wiki/ros-interfaces.md)
- [Bygg og miljø](docs/wiki/build-and-environment.md)
- [Pi drift og vedlikehold](docs/wiki/pi-maintenance.md)
- [Fysisk robot bringup](docs/wiki/physical-robot.md)
- [PC-verktøy](docs/wiki/pc-tools.md)
- [Simulering](docs/wiki/simulation.md)
- [Nav2 og EKF](docs/wiki/nav2-ekf.md)
- [Kamera og teddy-deteksjon](docs/wiki/vision.md)
- [Arduino Mega](docs/wiki/arduino-mega.md)
- [Kalibrering](docs/wiki/calibration.md)
- [Feilsøking](docs/wiki/troubleshooting.md)

## Arduino Mega Motor Wiring

Robotens Mega har et terminal block shield montert. Dokumentasjonen som matcher
shieldet er DFRobot `DFR0921` / `Terminal Block Shield for Arduino Mega`:
[DFRobot wiki](https://wiki.dfrobot.com/Terminal_Block_Shield_for_Arduino_Mega_SKU_DFR0921).

Retningspinnene er lagt i ryddige terminalblokker, mens encoderne ligger på
Mega interrupt-pins og PWM ligger på ekte PWM-pins. Hvis roboten er koblet som
`M1 = venstre` og `M2 = høyre`, bruk denne mappingen:

| Side | Motor | INA | INB | PWM | Hall A | Hall B |
|---|---|---:|---:|---:|---:|---:|
| Venstre | `M1` | `4` | `5` | `6` | `3` | `2` |
| Høyre | `M2` | `11` | `12` | `10` | `18` | `19` |

Detaljert testprosedyre ligger i [Arduino Mega-wikien](docs/wiki/arduino-mega.md).

## Viktige Konfigfiler

| Fil | Bruk |
|---|---|
| [config/camera_params.yaml](config/camera_params.yaml) | Kamera, H264-stream, YOLO og debugvideo |
| [config/robot_calibration.yaml](config/robot_calibration.yaml) | Mega-driver, encoder og tracked-drive kalibrering |
| [config/ekf.yaml](config/ekf.yaml) | `robot_localization` EKF |
| [config/nav2_params.yaml](config/nav2_params.yaml) | Nav2 controller, planner, costmaps og collision monitor |
| [config/slam_params.yaml](config/slam_params.yaml) | SLAM Toolbox-parametre |

## Hovedkommandoer

| Kommando | Bruk |
|---|---|
| `make build` | Bygg Docker-image. Ikke vanlig drift; bruk ved Dockerfile/dependency-endringer |
| `make ws` | Bygg ROS workspace i Docker |
| `make shell` | Åpne shell i Docker-container |
| `make sim-build` | Bygg workspace lokalt på PC |
| `make sim` | Start full sim: Gazebo + bridge + EKF + Nav2 + RViz + keyboard teleop |
| `make sim-headless` | Start Gazebo uten GUI |
| `make sim-nav2` | Start bare Nav2 mot en sim som allerede kjører uten Nav2 |
| `make pi-bringup` | Start fysisk robotstack på Pi |
| `make pc-teddy-rviz` | Start PC RViz med annotert YOLO-stream |
| `make pc-camera-rviz` | Start PC RViz med rå kamerastream |
| `make pc-ros-keyboard` | Manuell ROS teleop via `/cmd_vel_manual` |
| `make mega-upload` | Last opp Arduino sketch |
| `make mega-motor-test` | Kjør direkte motor-/encoder-test på Mega |
| `make mega-calibrate ARGS="snapshot"` | Test Mega/encoder-kontakt |

## Prosjektstatus Kort

Implementert:

- ROS 2 Jazzy workspace.
- Dockerbasert Pi-runtime.
- Gazebo-simulering.
- LDLiDAR LD06 launch.
- BNO085 IMU-node.
- Arduino Mega ROS-driver.
- Kamera UDP pipeline.
- YOLO/NCNN teddy-detektor.
- Nav2 stack med cmd_vel mux, smoother og collision monitor.
- Enkel EKF for hjulodometri + IMU yaw.

Kjente usikkerheter:

- Eksakt fysisk kameramodell er ikke dokumentert i repoet.
- Eksakt motor-/belte-BOM er ikke dokumentert som egen hardwareliste.
- SLAM config finnes, men SLAM er ikke standard workflow.
- Nav2 bruker `odom` som global frame i nåværende oppsett.
