# Pi Sparse Checkout

## Oversikt

Pi bruker **git sparse checkout** for å only klone de filene som er nødvendige for robotdrift. Dette holder Pi-disken ren og forhindrer at simuleringskode (Gazebo, robot_sim_control, etc.) havner på Pi ved uhell.

Sparse checkout betyr:
- `main`-branch inneholder alt (sim + Pi-kode).
- Pi sjekker bare ut mappar relevant for robotkontroll.
- `git pull` på Pi henter bare de mappene som er i sparse-checkout-lista.

## Første gang: Setup

Kjør dette på Pi:

```bash
# Last ned setup-scriptet fra repoet
# (eller kopier det fra PC hvis du vil)
bash ~/pi_sparse_checkout_setup.sh
```

Alternativt, manuelt:

```bash
git clone --no-checkout https://github.com/EmiliamBeke/Mekatronikk-4-MEPA2002.git
cd Mekatronikk-4-MEPA2002

git sparse-checkout init --cone
git sparse-checkout set \
  src/mekk4_bringup \
  src/mekk4_perception \
  src/robot_bringup \
  src/robot_description \
  config \
  scripts \
  docker \
  compose.yml \
  Makefile \
  README.md \
  AGENTS.md \
  LICENSE \
  docs/wiki

git checkout main
```

## Daglig bruk

Når Pi skal oppdateres:

```bash
cd ~/Mekatronikk-4-MEPA2002
git pull origin main
make ws  # Rebuild workspace
```

Siden sparse checkout er satt opp, får Pi kun:
- Hardware driver nodes (`mekk4_bringup`)
- Perception (camera bridge, teddy detector)
- Launch-filer og config
- Dokumentasjon og scripts

## Hva som IKKE er på Pi

Med dette oppsettet inneholder Pi-disken **ikke**:

| Mappe | Hvorfor utelat |
|---|---|
| `src/robot_gz/` | Gazebo verden og SDF-modeller (bare sim) |
| `src/robot_sim_control/` | Sim-only tracked-drive adapter |
| `models/` | YOLO-modeller og andre sim-assets |
| `arduino/` | Arduino sketcher (ikke nødvendig kjøretid) |
| `maps/` | Simulatorbaserte kart |
| `yolo26n_ncnn_model/` | YOLO weights (hvis det ligger løst) |
| `build/`, `install/`, `log/` | Build artifacts (regenereres per `make ws`) |

## Verifikasjon

Sjekk hva som er sjekket ut:

```bash
git sparse-checkout list
```

Burde gi:

```
src/mekk4_bringup
src/mekk4_perception
src/robot_bringup
src/robot_description
config
scripts
docker
docs/wiki
```

Sjekk at Gazebo-filene IKKE finnes:

```bash
ls -la src/robot_gz/      # Burde si: No such file or directory
ls -la src/robot_sim_control/  # Burde si: No such file or directory
```

## Hvis du vil legge til mappe til sparse checkout

Hvis du senere trenger en ny mappe på Pi (f.eks. en ny hardware driver):

```bash
git sparse-checkout add path/to/new/folder
git pull
```

## Hvis sparse checkout blir ødelagt

Hvis sparse checkout blir rar (rare merge-konflikter eller ghost-filer):

```bash
# Resett sparse checkout
git sparse-checkout disable
git sparse-checkout init --cone
git sparse-checkout set <lista over mapper>
git checkout main
```

## Teknisk note

- Sparse checkout bruker `.git/info/sparse-checkout`-fila for å definere hva som sjekkes ut.
- `--cone` betyr at dere kan bruke hele mappepaths og git er smart nok til å håndtere strukturen.
- Hvis du gjør `git status`, vil Git bare være seg bevisst mappene som er sjekket ut.
- `git pull` henter alt fra remote, men skriver bare til de mappene som er i listen.

## Troubleshooting

**Spørsmål:** Jeg kjørte `git pull` og plutselig dukket `src/robot_gz/` opp på Pi!

**Svar:** Det kan skje hvis noen gjorde `git sparse-checkout disable` eller cloned uten sparse checkout. Resett:
```bash
git sparse-checkout init --cone
git sparse-checkout set src/mekk4_bringup src/mekk4_perception src/robot_bringup src/robot_description config scripts docker compose.yml Makefile README.md AGENTS.md LICENSE docs/wiki
git checkout main
```

**Spørsmål:** Jeg vil ha en ny mappe, men det virker ikke når jeg legger den til.

**Svar:** Sjekk at mappen eksisterer i repoet og at stien er riktig:
```bash
git sparse-checkout add path/to/folder
git pull origin main
```

Se også [AGENTS.md](../AGENTS.md) for info om arbeidsflyt og [README.md](../README.md) for kommandoer.
