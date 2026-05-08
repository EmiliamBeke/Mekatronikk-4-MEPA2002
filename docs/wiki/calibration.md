# Kalibrering

## Bunnlinje

Kalibreringen brukes av ROS Mega-driveren for å oversette encoder ticks til meter
og cmd_vel til PWM. Verdiene lagres i
[`config/robot_calibration.yaml`](../../config/robot_calibration.yaml).

## Nåværende Kalibrering

```yaml
mega_driver:
  swap_sides: false
  left_cmd_sign: 1
  right_cmd_sign: 1
  angular_cmd_sign: -1
  left_cmd_scale: 1.0
  right_cmd_scale: 0.95
  left_tick_sign: 1
  right_tick_sign: 1
  left_m_per_tick: 5.4302082484863296e-05
  right_m_per_tick: 5.5104008816641413e-05
  track_width_eff_m: 0.24644042850855596
```

[`scripts/robot_calibration_env.py`](../../scripts/robot_calibration_env.py)
eksporterer disse som env-vars for `make pi-bringup`.

## Før Du Kalibrerer

1. Stopp alt annet som bruker Mega-porten.
2. Sørg for at Mega kjører runtime firmware:

```bash
MEGA_SKETCH=mega_keyboard_drive make mega-upload
```

3. Verifiser kontakt:

```bash
make mega-calibrate ARGS="snapshot"
```

## Clean Slate

Nullstill bare verdiene som skal måles på nytt. Behold mapping/sign hvis wiring
ikke er endret.

```yaml
mega_driver:
  left_cmd_scale: 1.0
  right_cmd_scale: 1.0
  left_m_per_tick: 0.0
  right_m_per_tick: 0.0
  track_width_eff_m: 0.35
```

Behold `angular_cmd_sign: -1` for dagens fysiske robot hvis ren pivot med
ROS/Nav2 `angular.z > 0` ellers spinner høyre. ROS-konvensjonen er at positiv
`angular.z` betyr venstresving. Ved buesving med `linear.x != 0` bruker
Mega-driveren vanlig ROS-differensialmiks, fordi roboten svinger riktig vei da.

## Steg 1: Straight-trim

Brukes for å få roboten til å kjøre rettere før meter-per-tick måles.

```bash
make mega-calibrate ARGS="straight-trim --pwm 160 --duration 3.0 --left-cmd-scale 1.0 --right-cmd-scale 1.0 --drift-direction right"
```

Sett `--drift-direction right` hvis roboten driver mot høyre; da reduseres
`left_cmd_scale`. Sett `--drift-direction left` hvis roboten driver mot venstre;
da reduseres `right_cmd_scale`. Sett `--drift-direction none` hvis roboten går
rett nok. Driftretning er påkrevd fordi encoder-metrikk alene kan gi feil side
når fysisk side/fortegn/meter-per-tick ikke stemmer med faktisk drift.

Resultat:

- `left_cmd_scale`
- `right_cmd_scale`

## Steg 2: Straight Forward/Reverse

Mål faktisk kjørt distanse per retning. Denne kjører først framover og deretter
bakover, skriver ut forskjellen mellom retningene, og lagrer snittet som
`left_m_per_tick` og `right_m_per_tick`.

Hvis begge retninger kjøres samme målte distanse:

```bash
make mega-calibrate ARGS="straight-bidir --pwm 160 --duration 3.0 --left-cmd-scale 1.0 --right-cmd-scale 1.0 --distance-m 1.0"
```

Hvis framover og bakover faktisk ender på litt ulik distanse, mål begge og bruk:

```bash
make mega-calibrate ARGS="straight-bidir --pwm 160 --duration 3.0 --left-cmd-scale 1.0 --right-cmd-scale 1.0 --forward-distance-m 1.02 --reverse-distance-m 0.98"
```

Resultat:

- `left_m_per_tick`
- `right_m_per_tick`
- forward/reverse prosentforskjell for hver side

Hvis du bare vil kjøre én retning:

```bash
make mega-calibrate ARGS="straight --direction forward --pwm 160 --duration 3.0 --distance-m 1.0"
make mega-calibrate ARGS="straight --direction reverse --pwm 160 --duration 3.0 --distance-m 1.0"
```

## Steg 3: Spin / effektiv beltebredde

`track_width_eff_m` er ikke nødvendigvis den fysiske avstanden mellom beltene.
For skid-steer/belteplattform brukes en effektiv bredde som får odometriens yaw
til å stemme med faktisk rotasjon. Simmen har en egen `sim_track_width_eff_m`;
verdien her brukes av fysisk Mega-driver.

Ikke bytt denne til fysisk målt belteavstand bare fordi den er "ekte". Hvis
roboten spinner 360 grader fysisk, men wheel odom sier for lite/for mye yaw, er
det `track_width_eff_m` som skal justeres. Simverdien bør holdes separat fordi
simfriksjon og kontaktmodell ikke er den samme som fysisk beltegrep.

Mål faktisk rotasjon og sett `--angle-deg` hvis du trenger god yaw fra hjulodom.
Hvis EKF primært bruker IMU yaw kan denne kalibreringen prioriteres lavere enn
straight forward/reverse.

```bash
make mega-calibrate ARGS="spin --pwm 90 --duration 18.8 --left-cmd-scale 1.0 --right-cmd-scale 0.95 --left-m-per-tick 0.000054302 --right-m-per-tick 0.000055104 --angle-deg 1440"
```

Resultat:

- `track_width_eff_m`

## Nyttige Flagg

```bash
make mega-calibrate ARGS="snapshot"
make mega-calibrate ARGS="straight --help"
make mega-calibrate ARGS="spin --help"
make mega-calibrate ARGS="straight --direction reverse ..."
make mega-calibrate ARGS="spin --direction ccw ..."
make mega-calibrate ARGS="snapshot --no-swap-sides"
MEGA_PORT=/dev/ttyACM0 make mega-calibrate ARGS="snapshot"
```

Default i wrapperen er `--no-swap-sides`, fordi dagens wiring er
`M1/ENC1 = venstre` og `M2/ENC2 = høyre`.

## Etter Kalibrering

Test odometri uten EKF først:

```bash
WITH_EKF=0 WITH_MEGA_DRIVER=1 make pi-bringup
ros2 topic echo --once /odom
```

Deretter med EKF:

```bash
WITH_EKF=1 WITH_IMU=1 WITH_MEGA_DRIVER=1 make pi-bringup
ros2 topic echo --once /wheel/odom
ros2 topic echo --once /odom
```
