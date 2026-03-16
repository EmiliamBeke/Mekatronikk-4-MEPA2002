# Mekatronikk-4-MEPA2002

Kort bruk av repoet.

## Formål

1. Simulering av robot (Gazebo + ROS 2).
2. Kjøring på fysisk robot (Pi5 + Docker + ROS 2).
3. Teddy-deteksjon (`/teddy_detector/status`).

## Bygg workspace (host)

| Kommando | Hva den gjør |
|---|---|
| `source /opt/ros/jazzy/setup.bash` | Laster ROS 2 Jazzy-miljø. |
| `cd ~/Mekatronikk-4-MEPA2002` | Går til repoet. |
| `colcon build --symlink-install` | Bygger pakkene i workspace. |
| `source install/setup.bash` | Laster de bygde pakkene i shellen. |

## Simulering (PC)

| Kommando | Hva den gjør |
|---|---|
| `make sim` | Starter simulering med Gazebo GUI + RViz. |
| `make sim-headless` | Starter simulering uten GUI. |
| `make sim-topics` | Viser sentrale sim-topics. |

## Nav2 i simulering

| Kommando | Hva den gjør |
|---|---|
| Terminal A: `make sim` | Starter selve simuleringen. |
| Terminal B: `make sim-nav2` | Starter Nav2 mot kart og params i repoet. |
| `ros2 launch robot_bringup nav2_stack.launch.py use_sim_time:=true map:=$PWD/maps/my_map.yaml params_file:=$PWD/config/nav2_params.yaml` | Direkte Nav2-launch (samme som `make sim-nav2`). |

I RViz:

1. Sett `Fixed Frame` til `map`.
2. Klikk `2D Pose Estimate`.
3. Klikk `2D Goal Pose`.

Merk:

1. `Timed out waiting for transform ... chassis to map` er normalt til `2D Pose Estimate` er satt.
2. Alle terminaler må ha samme `ROS_DOMAIN_ID`.

## Fysisk robot (Pi + PC)

Målet er at Pi-en kjører roboten autonomt, mens PC-en bare kobler seg på for RViz og debugging.

### Første gangs oppsett

Pi:

| Kommando | Hva den gjør |
|---|---|
| `ssh gruppe5@gruppe5pi5` | Logger inn på Pi. |
| `cd ~/Mekatronikk-4-MEPA2002` | Går til repoet på Pi. |
| `make build` | Bygger Docker-image (kun ved første gang eller etter Docker-endringer). |
| `make ws` | Bygger ROS-workspace i container. Kjør igjen hvis ROS-kode er endret. |

PC:

| Kommando | Hva den gjør |
|---|---|
| `source /opt/ros/jazzy/setup.bash` | Laster ROS 2 Jazzy-miljø. |
| `cd ~/Mekatronikk-4-MEPA2002` | Går til repoet på PC. |
| `colcon build --symlink-install` | Bygger PC-workspace. Kjør igjen hvis lokal ROS-kode er endret. |
| `source install/setup.bash` | Laster de bygde pakkene i shellen. |

### Standard workflow nå

Dette er den anbefalte oppskriften før dere har odometri og aktiv Nav2-bruk.

Pi, via SSH fra samme PC som skal bruke RViz:

```bash
cd ~/Mekatronikk-4-MEPA2002
WITH_NAV2=0 WITH_TEDDY=1 make pi-bringup
```

PC:

```bash
cd ~/Mekatronikk-4-MEPA2002
make pc-teddy-rviz
```

Dette skjer automatisk:

1. Pi finner PC-IP fra SSH-sesjonen.
2. Pi setter `ROS_DOMAIN_ID`, `ROS_AUTOMATIC_DISCOVERY_RANGE` og `ROS_STATIC_PEERS`.
3. Pi starter samlet bringup i Docker med `robot_state_publisher`, LiDAR og teddy-detektor.
4. Pi sender annotert YOLO-video over UDP til PC hvis `teddy_detector.stream_debug_video: true` i [config/camera_params.yaml](/home/emiliam/Mekatronikk-4-MEPA2002/config/camera_params.yaml).
5. PC setter ROS discovery mot Pi automatisk, starter lokal UDP->ROS bridge for YOLO-debugbildet og åpner RViz med [pre_odom_lidar.rviz](/home/emiliam/Mekatronikk-4-MEPA2002/src/robot_bringup/rviz/pre_odom_lidar.rviz).

I denne RViz-konfigen er standarden:

1. `Fixed Frame = chassis`
2. `/lidar` vises som LaserScan
3. `/teddy_detector/debug_image` vises som Image
4. `TF` er dempet for å unngå støy før dere har odom

### Nyttige varianter

Pi:

| Kommando | Hva den gjør |
|---|---|
| `make pi-bringup` | Standard bringup med default-verdier i scriptet. |
| `WITH_NAV2=0 make pi-bringup` | Starter uten Nav2. Dette er anbefalt før dere har odometri. |
| `WITH_TEDDY=1 make pi-bringup` | Starter teddy-detektor på Pi. |
| `WITH_TEDDY=1 WITH_CAMERA_RVIZ=1 make pi-bringup` | Starter teddy på Pi og sender også rå H264-kamerastrøm til PC på port `5601`. |
| `PC_HOST=192.168.10.42 make pi-bringup` | Overstyr automatisk valgt PC-IP for ROS discovery og debug-stream. |

PC:

| Kommando | Hva den gjør |
|---|---|
| `make pc-teddy-rviz` | Standard: viser LiDAR + annotert YOLO-bilde i RViz. |
| `make pc-camera-rviz` | Valgfritt: viser rå `/camera` i RViz via lokal UDP->ROS bridge på PC. |
| `make pc-teddy-rviz PI_HOST=192.168.10.55` | Bruk Pi-IP direkte hvis `gruppe5pi5` ikke løses på PC. |

### ROS discovery og IP

I normal bruk trenger du ikke å kjøre `scripts/ros_discovery_env.sh` manuelt.

Det er bare nyttig hvis du vil feilsøke:

```bash
bash scripts/ros_discovery_env.sh pi
bash scripts/ros_discovery_env.sh pc gruppe5pi5
```

Automatikken fungerer best når:

1. du SSH-er inn på Pi fra samme PC som skal bruke RViz
2. Pi og PC faktisk når hverandre på samme nett
3. PC kan løse `gruppe5pi5`, eller du overstyrer med IP

## Vision og LiDAR

| Kommando | Hva den gjør |
|---|---|
| `make lidar-setup` | Henter/bygger LiDAR-driver i workspace. |
| `make lidar-test` | Kjører enkel LiDAR-smoketest. |

Guide for LiDAR i RViz: [docs/lidar_rviz.md](/home/emiliam/Mekatronikk-4-MEPA2002/docs/lidar_rviz.md)

### Kamera- og YOLO-parametre

Kamera- og YOLO-parametre styres fra [config/camera_params.yaml](/home/emiliam/Mekatronikk-4-MEPA2002/config/camera_params.yaml).

Kort oppdeling:

1. `camera_stream.*` påvirker signalet som går inn til teddy-detektor på Pi.
2. `camera_stream.width/height/fps/bitrate_bps/intra/low_latency/denoise/...` er stedet å tune bildekvalitet og artifacts for YOLO-inputen.
3. `teddy_detector.*` påvirker YOLO-parametre og den annoterte debug-videoen som sendes til PC.
4. `teddy_detector.debug_stream_*` påvirker bare debug-visningen på PC, ikke hva YOLO faktisk ser.

### Kamera drift på Pi

| Kommando | Hva den gjør |
|---|---|
| `make camera-reload` | Restarter bare kamerastreamen med nye `camera_stream.*`-verdier fra [config/camera_params.yaml](/home/emiliam/Mekatronikk-4-MEPA2002/config/camera_params.yaml). Bruk denne etter tuning av farger, bitrate, intra, denoise osv. |
| `make camera-stop` | Stopper kamerastream/supervisor hvis noe henger igjen. Dette er en recovery-knapp, ikke vanlig workflow. |

### Arduino Mega smoke test

Hvis du vil verifisere at Pi-en faktisk kan snakke med en Arduino Mega over USB, finnes det nå en enkel smoke test.

1. Last opp [mega_smoketest.ino](/home/emiliam/Mekatronikk-4-MEPA2002/arduino/mega_smoketest/mega_smoketest.ino) til Mega.
2. Sørg for at Pi-host har `python3-serial` tilgjengelig.
3. Kjør testen på Pi:

```bash
make mega-test
```

Dette gjør:

1. finner `Mega`-porten automatisk (`/dev/serial/by-id`, `/dev/ttyACM*` eller `/dev/ttyUSB*`)
2. åpner USB-serial direkte på Pi-host
3. sender `ID`, `PING`, `LED ON`, `LED OFF`
4. forventer svar tilbake fra Mega og bekrefter at link fungerer

Hvis `python3-serial` mangler på Pi-host:

```bash
sudo apt install python3-serial
```

Hvis auto-detection bommer:

```bash
MEGA_PORT=/dev/ttyACM0 make mega-test
```

Praktisk:

1. Etter endring av farger, eksponering, bitrate, intra eller denoise i `camera_stream.*`, prøv `make camera-reload` på Pi.
2. Hvis du endrer `camera_stream.width/height`, porter eller `teddy_detector.*`, gjør full restart av `make pi-bringup`.
3. `make camera-stop` er bare en recovery-knapp hvis gamle kameraprosesser henger igjen.
4. Du kan fortsatt overstyre midlertidig med env vars, for eksempel `WIDTH=640 FPS=10 SATURATION=1.2 make pi-bringup`.

## Pi ytelse (host, ikke Docker)

| Kommando | Hva den gjør |
|---|---|
| `cpupower frequency-info` | Viser tilgjengelige governors og aktiv policy. |
| `sudo cpupower frequency-set -g performance` | Setter CPU i maks ytelse-modus. |
| `sudo cpupower frequency-set -g ondemand` | Setter CPU tilbake til dynamisk modus. |
| `cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor` | Verifiserer aktiv governor. |
| `watch -n1 'vcgencmd measure_temp; vcgencmd measure_clock arm; vcgencmd get_throttled'` | Overvåker temperatur, klokke og throttling live. |

Gjør `performance` permanent etter reboot:

| Kommando | Hva den gjør |
|---|---|
| `sudo nano /etc/systemd/system/cpu-governor-performance.service` | Oppretter systemd-service for governor ved boot. |
| `sudo systemctl daemon-reload` | Leser inn ny servicefil. |
| `sudo systemctl enable --now cpu-governor-performance.service` | Aktiverer service nå og ved neste reboot. |
| `sudo systemctl status cpu-governor-performance.service --no-pager` | Sjekker at service kjører uten feil. |
| `cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor` | Verifiserer at governor fortsatt er `performance`. |

Service-innhold:

```ini
[Unit]
Description=Set CPU governor to performance
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/bin/cpupower frequency-set -g performance
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

Tilbake til `ondemand` permanent:

| Kommando | Hva den gjør |
|---|---|
| `sudo systemctl disable --now cpu-governor-performance.service` | Skrur av permanent `performance`-service. |
| `sudo cpupower frequency-set -g ondemand` | Setter governor tilbake med en gang. |

Valgfritt (mer aktiv viftekurve):

| Kommando | Hva den gjør |
|---|---|
| `sudo nano /boot/firmware/config.txt` | Åpner Pi-bootconfig for vifteparametre. |
| `sudo reboot` | Rebooter Pi etter endringer i `config.txt`. |

## Rydd lagring på Pi

| Kommando | Hva den gjør |
|---|---|
| `df -h` | Viser total diskbruk på Pi. |
| `docker system df` | Viser hvor mye plass Docker bruker. |
| `du -h --max-depth=1 ~/Mekatronikk-4-MEPA2002` | Viser store mapper i repoet. |
| `docker system prune -af` | Fjerner ubrukte containere/nettverk/images. |
| `docker builder prune -af` | Fjerner docker build-cache. |
| `sudo apt clean` | Fjerner apt-pakke-cache. |
| `sudo rm -rf /var/lib/apt/lists/*` | Fjerner lokale apt-indekser. |
| `rm -rf ~/Mekatronikk-4-MEPA2002/build ~/Mekatronikk-4-MEPA2002/log` | Fjerner lokale build/log-mapper. |
| `rm -rf ~/Mekatronikk-4-MEPA2002/install` | Valgfritt: frigjør mer, men krever ny `make ws`. |
