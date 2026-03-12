# Mekatronikk-4-MEPA2002

## Komme i gang

### 1. Koble til Pi-en

```bash
ssh gruppe5@gruppe5pi5
passord: qwerty
cd ~/Mekatronikk-4-MEPA2002
```

### 2. Bygg Docker-imaget

Dette gjøres kun første gang, eller når `docker/Dockerfile` er endret.

```bash
docker compose build
```

### 3. Bygg ROS-workspace

Dette gjøres første gang og etter kodeendringer i `src/`.

```bash
docker compose run --rm ros bash -lc '/ws/scripts/ws_build.sh'
```

### 4. Start containeren

```bash
docker compose up -d
```

### 5. Åpne shell i containeren

```bash
docker compose run --rm ros
```

### 6. Stopp containeren

```bash
docker compose down
```

## Vision-stream

Startes fra host (krever `rpicam-apps` og `gstreamer1.0-tools` på Pi-en):

```bash
make vision
```

Dette starter kameraet og launcher `teddy_detector`-noden inni containeren.

## Når må jeg kjøre `docker compose build`?

| Endring | Hva du trenger |
|---|---|
| Kode i `src/` | Bare `ws_build.sh` |
| Ny ROS-pakke eller avhengighet i `package.xml` | Bare `ws_build.sh` |
| Endring i `docker/Dockerfile` | `docker compose build` |
| Ny `pip`/`apt`-avhengighet | `docker compose build` |

Koden mountes inn i containeren fra hosten, så endringer i `src/` krever aldri rebuild av Docker-imaget.

## YOLO-modell

Prosjektet bruker `yolo26n_ncnn_model` — nano-varianten, som er den minste og raskeste. For brukstilfellet (finne teddybjørn og beregne senter) er dette riktig valg på Pi.

Modellen legges i én av disse plasseringene (prioritert rekkefølge):

1. Satt via miljøvariabelen `MEKK4_NCNN_MODEL`
2. `/ws/models/yolo26n_ncnn_model`
3. `/ws/yolo26n_ncnn_model`

## Spare plass på Pi-en

Docker kan fort spise opp diskplass. Sjekk hva som brukes:

```bash
docker system df
```

**Fjern stoppede containere, ubrukte images og build-cache i én kommando:**
```bash
docker system prune
```

Legg til `-a` for å også fjerne images som ikke er i bruk (inkludert det dere har bygget):
```bash
docker system prune -a
```

**Fjern bare build-cache (vanligvis den største synderen):**
```bash
docker builder prune
```

**Fjern ROS build-output fra workspace:**
```bash
rm -rf build/ install/ log/
```
Disse regenereres av `ws_build.sh` og trenger ikke lagres.

**Sjekk hva som tar plass generelt:**
```bash
df -h          # diskbruk totalt
du -sh ~/*     # hva i hjemmemappa som er størst
```

> Kjør `docker compose down` før du sletter images, ellers er imaget i bruk.

## Feilsøking

**Docker-tjenesten kjører ikke:**
```bash
sudo systemctl start docker
sudo systemctl enable docker
```

**Permission denied på Docker:**
```bash
sudo usermod -aG docker $USER
newgrp docker
```
