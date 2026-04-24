# Pi Drift og Vedlikehold

## Bunnlinje

Denne siden er for driftsting på Raspberry Pi: diskplass, Docker-opprydding,
CPU governor, temperatur og throttling. Dette er ikke vanlig robot-bringup, men
nyttig når Pi-en blir treg, bygger feiler, eller Docker bruker for mye lagring.

Kjør kommandoene på **Pi-host**, ikke inne i Docker-containeren, med mindre noe
annet står eksplisitt.

## Docker Build-Policy

`make build` bygger hele Docker-imaget på nytt. Det er tregt, bruker mye
disk/cache, og skal ikke være en vanlig del av robotkjøring.

Kjør vanligvis dette etter kodeendringer:

```bash
make ws
```

Kjør `make build` bare når du faktisk har endret runtime-miljøet:

- [`docker/Dockerfile`](../../docker/Dockerfile)
- apt-avhengigheter
- pip-avhengigheter
- base image
- systembiblioteker som må finnes inne i containeren

Ikke kjør `make build` bare fordi du har endret:

- ROS Python-kode
- launch-filer
- YAML-konfig
- dokumentasjon
- Arduino-sketcher

Hvis du er usikker: prøv `make ws` først. Bruk `make build` først når feilen
handler om manglende pakker eller endret Docker-runtime.

## Diskplass: Første Diagnose

### `df -h`

```bash
df -h
```

Viser hvor fulle filsystemene er. Se spesielt på root-filsystemet `/`.

Bruk når:

- `make build` feiler med “no space left on device”.
- Pi-en oppfører seg tregt.
- Docker build stopper uventet.

Tolkning:

- Hvis `/` er over ca. 90 %, bør du rydde før ny Docker-build.
- Hvis `/boot` er full, er det et annet problem enn Docker/workspace.

### `docker system df`

```bash
docker system df
```

Viser hvor mye plass Docker bruker på images, containers, volumes og build cache.

Bruk når:

- `df -h` viser lite plass.
- Du mistenker at gamle Docker images eller build cache tar plass.

Tolkning:

- Stor `Build Cache` ryddes med `docker builder prune -af`.
- Mange gamle `Images` ryddes med `docker system prune -af`, men les advarselen under.

### `du -h --max-depth=1 ~/Mekatronikk-4-MEPA2002`

```bash
du -h --max-depth=1 ~/Mekatronikk-4-MEPA2002
```

Viser hvilke mapper i repoet som bruker mest plass.

Vanlige store mapper:

- `build/`
- `install/`
- `log/`
- `.ultralytics/`
- eventuelle modellmapper under `models/`

Dette endrer ingenting; det er bare inspeksjon.

## Diskplass: Anbefalt Ryddesekvens

Start alltid i repoet:

```bash
cd ~/Mekatronikk-4-MEPA2002
```

### 1. Stopp prosjektcontainere

```bash
docker compose down --remove-orphans
```

Stopper containere fra dette Compose-prosjektet og fjerner orphan-containere som
ikke lenger finnes i [`compose.yml`](../../compose.yml).

Bruk før:

- ny `make build`, hvis Docker-image faktisk må bygges på nytt
- Docker prune
- feilsøking av containere som henger

Risiko:

- Lav. Stopper kjørende prosjektcontainer, men sletter ikke image eller kildekode.

### 2. Fjern stoppede containere

```bash
docker container prune -f
```

Fjerner containere som allerede er stoppet.

Bruk når:

- `docker system df` viser mye container-bruk.
- Du har kjørt mange `docker compose run --rm`-lignende kommandoer uten `--rm`.

Risiko:

- Lav til moderat. Sletter stoppede containere, men ikke images eller volumes.
- Data som kun lå i en stoppet container forsvinner. Dette repoet mapper workspace
  inn som volume, så vanlig kildekode påvirkes ikke.

### 3. Fjern Docker build cache

```bash
docker builder prune -af
```

Fjerner cache Docker bruker for å bygge images raskere.

Bruk når:

- `docker system df` viser stor `Build Cache`.
- `make build` feiler på diskplass.

Konsekvens:

- Neste `make build` blir tregere fordi Docker må bygge lag på nytt.

Risiko:

- Lav. Sletter cache, ikke kildekode.

### 4. Fjern ubrukte Docker images og nettverk

```bash
docker system prune -af
```

Fjerner ubrukte Docker images, stoppede containere og ubrukte nettverk.

Bruk når:

- build cache-prune ikke frigjør nok.
- gamle Docker images tar mye plass.

Konsekvens:

- Images som ikke er i bruk må lastes/buildes på nytt senere.
- Kan fjerne andre ubrukte Docker images på Pi-en, ikke bare dette prosjektet.

Risiko:

- Moderat. Ikke destruktivt for repoet, men kan gjøre andre Docker-prosjekter på
  Pi-en tregere å starte neste gang fordi images må hentes/buildes på nytt.

### 5. Rydd apt cache

```bash
sudo apt clean
```

Fjerner nedlastede `.deb`-pakker fra apt cache.

Bruk når:

- Pi-en har lite diskplass.
- Du nylig har installert mange pakker.

Risiko:

- Lav. Pakker som allerede er installert blir ikke avinstallert.

## Workspace-Rydding

### Fjern `build/` og `log/`

```bash
rm -rf ~/Mekatronikk-4-MEPA2002/build ~/Mekatronikk-4-MEPA2002/log
```

Fjerner lokale colcon build- og logmapper.

Bruk når:

- workspace-build oppfører seg rart.
- du vil frigjøre plass uten å slette installasjonen.

Konsekvens:

- Neste build må kompilere/generere mer på nytt.
- `install/` beholdes, så eksisterende installasjon kan fortsatt fungere.

Risiko:

- Lav, så lenge pathen er riktig.
- Dette er en destruktiv kommando for genererte filer. Kontroller pathen før du
  kjører den.

### Fjern `install/`

```bash
rm -rf ~/Mekatronikk-4-MEPA2002/install
```

Fjerner installert ROS workspace.

Bruk når:

- du trenger mer plass enn `build/` og `log/` frigjør.
- installasjonen er stale eller inkonsistent.
- `source install/setup.bash` gir gamle pakker/launch-filer.

Konsekvens:

- ROS-pakkene i workspace er ikke tilgjengelige før du bygger på nytt.
- Kjør etterpå:

```bash
make ws
```

Risiko:

- Moderat. Ikke kildekode, men du må bygge workspace på nytt.

## Trygg Ryddeoppskrift

Kjør dette først ved diskplassproblemer:

```bash
cd ~/Mekatronikk-4-MEPA2002
df -h
docker system df
du -h --max-depth=1 ~/Mekatronikk-4-MEPA2002
docker compose down --remove-orphans
docker container prune -f
docker builder prune -af
sudo apt clean
rm -rf ~/Mekatronikk-4-MEPA2002/build ~/Mekatronikk-4-MEPA2002/log
df -h
docker system df
```

Hvis det fortsatt er for lite plass:

```bash
docker system prune -af
rm -rf ~/Mekatronikk-4-MEPA2002/install
make ws
```

## CPU Governor og Ytelse

### Se aktiv CPU-policy

```bash
cpupower frequency-info
```

Viser tilgjengelige CPU governors, frekvensområde og aktiv policy.

Bruk når:

- YOLO eller Nav2 virker tregt.
- du vil sjekke om Pi-en kjører i powersave/ondemand/performance.

Hvis `cpupower` mangler, installer på Pi-host:

```bash
sudo apt install linux-cpupower
```

### Sett maks ytelse

```bash
sudo cpupower frequency-set -g performance
```

Setter CPU governor til `performance`. Det gjør at CPU-en prøver å holde høyere
klokkefrekvens.

Bruk når:

- du tester YOLO/FPS.
- du kjører Nav2 og perception samtidig.
- du vil redusere latency under demo/test.

Konsekvens:

- Høyere strømforbruk.
- Mer varme.
- Større risiko for throttling hvis kjøling er dårlig.

### Verifiser aktiv governor

```bash
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```

Printer aktiv governor for CPU0, for eksempel:

```text
performance
```

Bruk etter `frequency-set` for å bekrefte at endringen faktisk slo inn.

### Sett tilbake dynamisk modus

```bash
sudo cpupower frequency-set -g ondemand
```

Setter CPU tilbake til dynamisk governor. Dette er bedre når roboten ikke trenger
maks ytelse hele tiden.

Bruk når:

- demo/test er ferdig.
- Pi-en blir varm.
- du kjører på batteri og vil spare strøm.

## Temperatur og Throttling

### Live overvåking

```bash
watch -n1 'vcgencmd measure_temp; vcgencmd measure_clock arm; vcgencmd get_throttled'
```

Kjører tre Pi-kommandoer hvert sekund:

| Del | Betydning |
|---|---|
| `vcgencmd measure_temp` | CPU/GPU-temperatur |
| `vcgencmd measure_clock arm` | aktuell ARM CPU-klokke |
| `vcgencmd get_throttled` | throttling/undervoltage-status |

Bruk når:

- YOLO FPS faller over tid.
- Pi-en blir fysisk varm.
- systemet virker raskt etter boot, men tregere etter noen minutter.

### Tolke `get_throttled`

Eksempel:

```text
throttled=0x0
```

`0x0` betyr ingen aktiv eller historisk throttling siden boot.

Hvis verdien ikke er `0x0`, har Pi-en hatt undervoltage, throttling eller
temperaturbegrensning. Da bør du sjekke:

- strømforsyning
- USB-belastning
- kjøling
- CPU governor
- om YOLO-oppløsning/FPS er for høy

## Praktisk Ytelsesrekkefølge

Hvis perception eller Nav2 er tregt:

1. Sjekk temperatur og throttling.
2. Sett `performance` governor midlertidig.
3. Reduser kamera `fps`, `width`, `height` eller YOLO `imgsz` i
   [`camera_params.yaml`](../../config/camera_params.yaml).
4. Kjør `make camera-reload` for kamera/encoder-tuning.
5. Full restart av `make pi-bringup` hvis du endrer width/height/porter.
