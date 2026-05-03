# Kamera og Teddy-deteksjon

## Bunnlinje

Pi-host lager H264/RTP UDP stream fra `rpicam-vid`. Teddy-detektoren i Docker
dekoder lokal UDP stream, kjører YOLO/NCNN og kan sende annotert H264 debugvideo
til PC. PC gjør debugvideo om til ROS image på `/camera`.

## Konfig

Kilde: [`config/camera_params.yaml`](../../config/camera_params.yaml)

### `camera_stream`

| Felt | Nåværende verdi | Effekt |
|---|---:|---|
| `width` | `800` | Kamera/YOLO bredde |
| `height` | `600` | Kamera/YOLO høyde |
| `fps` | `6` | Capture rate |
| `bitrate_bps` | `2800000` | H264 bitrate |
| `intra` | `10` | I-frame interval |
| `low_latency` | `true` | Prøver low-latency encoder |
| `flush_output` | `true` | Flusher encoded output |
| `pc_jitter_ms` | `10` | PC jitterbuffer |
| `local_udp_port` | `5600` | Pi intern stream til detector |
| `denoise` | `cdn_hq` | Libcamera denoise |

### `teddy_detector`

| Felt | Nåværende verdi | Effekt |
|---|---:|---|
| `model_path` | `/ws/models/yolo26n_ncnn_model` | NCNN model |
| `conf` | `0.3` | Detection threshold |
| `imgsz` | `640` | YOLO input size |
| `center_tol` | `0.10` | Centered threshold |
| `stream_debug_video` | `true` | H264 debugvideo til PC |
| `debug_stream_port` | `5602` | Annotert stream port |
| `debug_stream_fps` | `auto` | Følger detector rate |
| `debug_stream_bitrate_bps` | `1400000` | Debugstream bitrate |

## Runtime

Pi bringup starter kamera hvis:

- `WITH_TEDDY=1`.

Script:

- [`scripts/pi_bringup.sh`](../../scripts/pi_bringup.sh)
- [`scripts/camera_stream_supervisor.sh`](../../scripts/camera_stream_supervisor.sh)
- [`scripts/camera_config_env.py`](../../scripts/camera_config_env.py)

## Teddy Detector

Kode: [`teddy_detector.py`](../../src/mekk4_perception/mekk4_perception/teddy_detector.py)

Viktige fakta:

- Bruker Ultralytics `YOLO(..., task="detect")`.
- Detekterer COCO class id `77`, kommentert som `teddy bear`.
- Publiserer status på `/teddy_detector/status`.
- Sender annotert H264/UDP til PC når `stream_debug_video=true`.

Statusformat:

```text
teddy_count=0 centered=false fps=...
teddy_count=1 dx=-0.123 dy=0.045 centered=false fps=...
```

`dx` og `dy` er normalisert offset fra bildesenter.

## PC Bridge

PC-side bridge:

```bash
make pc-teddy-rviz
```

Dette starter `udp_camera_bridge` og publiserer annotert stream som `/camera`.

## Reload

Etter color/exposure/FPS/encoder-tuning:

```bash
make camera-reload
```

For width/height/port-endringer: restart `make pi-bringup`.

Stoppe stream ved heng:

```bash
make camera-stop
```

## Verifikasjon

```bash
ros2 topic echo --once /teddy_detector/status
ros2 topic hz /camera
ros2 topic list | grep camera
```

Hvis PC-bildet mangler, sjekk UDP-port `5602`, GStreamer plugins på PC og at
`stream_debug_video: true` i config.
