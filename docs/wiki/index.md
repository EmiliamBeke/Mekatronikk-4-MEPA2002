# Wiki Forside

Dette er den praktiske wikien for roboten i `Mekatronikk-4-MEPA2002`.

Målet er at denne dokumentasjonen skal være nyttig når du faktisk står med roboten,
Pi-en, PC-en, RViz, kamera, Mega og ROS-terminaler.

## Før Du Starter

Repoet er et ROS 2 Jazzy colcon workspace. Kildekode ligger i
[`src`](../../src). Fysisk robot kjøres normalt fra Raspberry Pi via Docker,
mens PC brukes til RViz, teleop og debugging.

Viktig kildegrunnlag:

- Launch: [`src/robot_bringup/launch`](../../src/robot_bringup/launch)
- Runtime scripts: [`scripts`](../../scripts)
- Konfig: [`config`](../../config)
- Arduino firmware: [`arduino`](../../arduino)
- Robotmodell: [`src/robot_description`](../../src/robot_description)
- Simmodell: [`src/robot_gz`](../../src/robot_gz)

## Navigasjon

| Side | Innhold |
|---|---|
| [Systemoversikt](system-overview.md) | Arkitektur, pakker, dataflyt og hva som faktisk er implementert |
| [Maskinvare og sensorer](hardware.md) | Pi, LiDAR, IMU, kamera, Mega, encoder- og motorpins |
| [LiDAR og RViz](lidar-rviz.md) | Praktisk LiDAR/RViz-oppsett på fysisk robot |
| [ROS interfaces](ros-interfaces.md) | Topics, frames, nodes og command chain |
| [Bygg og miljø](build-and-environment.md) | Docker, colcon, ROS setup og workspace-build |
| [Pi drift og vedlikehold](pi-maintenance.md) | Diskplass, Docker-opprydding, CPU governor og throttling |
| [Fysisk robot bringup](physical-robot.md) | `make pi-bringup`, env-vars og Pi-runtime |
| [PC-verktøy](pc-tools.md) | RViz, UDP camera bridge og ROS keyboard teleop |
| [Simulering](simulation.md) | Gazebo, bridge, sim sensors og sim commands |
| [Nav2 og EKF](nav2-ekf.md) | Nav2 stack, costmaps, cmd_vel flow og robot_localization |
| [Kamera og teddy-deteksjon](vision.md) | Pi camera, H264/UDP, YOLO/NCNN og debugstream |
| [Arduino Mega](arduino-mega.md) | Firmware, upload, serial commands og ROS Mega-driver |
| [Kalibrering](calibration.md) | Straight-trim, meter-per-tick og track width |
| [Feilsøking](troubleshooting.md) | Konkrete sjekker for ROS, LiDAR, kamera, Mega og Nav2 |

## Anbefalt Leseorden

1. Les [Systemoversikt](system-overview.md).
2. Sjekk [ROS interfaces](ros-interfaces.md) før du endrer launch eller topics.
3. Bruk [Fysisk robot bringup](physical-robot.md) eller [Simulering](simulation.md) etter hva du kjører.
4. Bruk [Feilsøking](troubleshooting.md) når noe ikke dukker opp i RViz eller ROS graph.

## Regler For Endringer

- Ikke gjett på hardware. Skriv hva repoet faktisk viser.
- Hold fysisk robot og simulering eksplisitt adskilt.
- Endre ikke topics, frames eller serial devices uten å oppdatere wiki og relevante launch/config-filer.
- Verifiser minste relevante kommando etter endring.
