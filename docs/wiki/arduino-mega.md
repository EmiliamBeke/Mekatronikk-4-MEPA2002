# Arduino Mega

## Bunnlinje

Arduino Mega brukes som serial motor- og encoder-interface. ROS Mega-driveren
forventer runtime firmware `mega_keyboard_drive`.

## Sketcher

| Sketch | Formål | Firmware-ID |
|---|---|---|
| [`mega_smoketest`](../../arduino/mega_smoketest) | Serial/LED-test | `MEGA_SMOKETEST` |
| [`mega_dfr0601_test`](../../arduino/mega_dfr0601_test) | Motor/encoder-test | `MEGA_DFR0601_TEST` |
| [`mega_keyboard_drive`](../../arduino/mega_keyboard_drive) | Runtime firmware | `MEGA_KEYBOARD_DRIVE` |

## Upload

Kjør på Pi-host, ikke i Docker:

```bash
make mega-upload
```

Default sketch i [`Makefile`](../../Makefile) er `mega_keyboard_drive`.
Du kan også gi sketch-navnet direkte som ekstra argument:

```bash
make mega-upload mega_keyboard_drive
make mega-upload mega_dfr0601_test
make mega-upload mega_smoketest
```

Runtime firmware:

```bash
MEGA_SKETCH=mega_keyboard_drive make mega-upload
```

Overstyr port eller board:

```bash
MEGA_PORT=/dev/ttyACM0 make mega-upload
MEGA_FQBN=arduino:avr:mega make mega-upload
```

Oversikt over sketchene:

| Sketch | Bruk | Kommando |
|---|---|---|
| `mega_smoketest` | Serial/LED-test | `make mega-upload mega_smoketest` |
| `mega_dfr0601_test` | Motor-/encoder-test | `make mega-upload mega_dfr0601_test` |
| `mega_keyboard_drive` | Runtime firmware | `make mega-upload mega_keyboard_drive` |

Motor-/encoder-test bruker egen testfirmware. Last den opp først:

```bash
make mega-upload mega_dfr0601_test
```

Kjør deretter testen:

```bash
make mega-motor-test
```

Testen forventer firmware-ID `MEGA_DFR0601_TEST`. Hvis runtime-firmware
`mega_keyboard_drive` ligger på Megaen, stopper testen og ber deg laste opp
`mega_dfr0601_test`.

Denne testen sjekker elektrisk kanal-mapping i firmware:

- `M1` skal bevege motoren som hører til `ENC1`
- `M2` skal bevege motoren som hører til `ENC2`

Hvis roboten er koblet som `M1 = venstre` og `M2 = høyre`, skal Mega-pinnene
være:

| Side | Motor | INA | INB | PWM | Hall A | Hall B |
|---|---|---:|---:|---:|---:|---:|
| Venstre | `M1` | `4` | `5` | `6` | `3` | `2` |
| Høyre | `M2` | `12` | `11` | `10` | `18` | `19` |

### Terminal Block Shield v1.1.0 på Arduino Mega

Shieldet matcher DFRobot `DFR0921` / `Terminal Block Shield for Arduino Mega`:
[DFRobot wiki](https://wiki.dfrobot.com/Terminal_Block_Shield_for_Arduino_Mega_SKU_DFR0921).

På terminal block shieldet skal terminalnummeret følge pin-merkingen på Arduino
Mega/shield-silkscreen. DFRobot-skjemaet viser at terminalene ligger i fysiske
rekker, så skru inn ledningene innerst/fjernest fra skrutrekker-tilgangen først.
Da slipper du å løsne en tidligere ledning for å komme til neste terminal.

Retningspinnene er lagt i ryddige blokker, mens PWM og encoder Hall A/B bruker
Mega-pins med riktig hardware-støtte. `M1 PWM` pin `6` og `M2 PWM` pin `10`
er hardware PWM på Arduino Mega.

Koble `M2 = høyre` i denne skru-rekkefølgen. Denne ligger nær terminalraden
`10/11/12` for korte motor-driverledninger:

| Steg | Terminal | Signal | Går til |
|---:|---:|---|---|
| 1 | `10` | `M2 PWM` | Høyre motor-driver PWM |
| 2 | `12` | `M2 INA` | Høyre motor-driver INA |
| 3 | `11` | `M2 INB` | Høyre motor-driver INB |
| 4 | `19` | `ENC2 Hall B` | Høyre encoder Hall B |
| 5 | `18` | `ENC2 Hall A` | Høyre encoder Hall A |

Koble `M1 = venstre` i denne skru-rekkefølgen. Denne følger terminalraden
`2/3/4/5/6/7` nedover; `7` brukes ikke:

| Steg | Terminal | Signal | Går til |
|---:|---:|---|---|
| 1 | `7` | ubrukt | ikke koble |
| 2 | `6` | `M1 PWM` | Venstre motor-driver PWM |
| 3 | `5` | `M1 INB` | Venstre motor-driver INB |
| 4 | `4` | `M1 INA` | Venstre motor-driver INA |
| 5 | `3` | `ENC1 Hall A` | Venstre encoder Hall A |
| 6 | `2` | `ENC1 Hall B` | Venstre encoder Hall B |

Ikke bruk `7-9`, `13`, `30-31`, `32-35` eller `44` for motor/encoder i denne
firmwareversjonen.
Encoder 5V/GND skal tas fra shieldets `5V`/`GND` terminaler, ikke fra digitale
pins. Motor-driverens power skal følge motor-driverens egen power-wiring.

Dette er ikke det samme som robotens venstre/høyre-side. Dagens ROS-kalibrering
kan fortsatt bruke `swap_sides: true` hvis M1/ENC1 og M2/ENC2 er fysisk byttet
relativt til robotens sider.

### Motor-test prosedyre

Testen kjører denne rekkefølgen automatisk:

1. M1 forward, pause, M1 reverse
2. M2 forward, pause, M2 reverse
3. BOTH forward, pause, BOTH reverse

Mens hvert steg går leses `ENC1` og `ENC2` kontinuerlig. Etter hvert steg får du:

- `delta` (start til slutt)
- `span` (min/max under hele steget)

Diagnose i output:

- `M1 dominant encoder` og `M2 dominant encoder`: viser hvilken encoder som faktisk følger hver motor.
- `FAULT: wrong firmware`: last opp `mega_dfr0601_test`.
- `FAULT: M1 appears to move ENC2` / `M2 appears to move ENC1`: motor/encoder-kanaler er sannsynlig byttet.
- `FAULT: M1 and M2 appear to affect the same encoder`: én encoder er trolig frakoblet, eller begge motorer observeres gjennom samme encoderkanal.
- `FAULT: forward and reverse produced the same encoder sign`: motoren reverserer trolig ikke fysisk, eller én encoderfase mangler/støyer.
- `FAULT: BOTH command did not move encoders`: hvis enkeltmotorene fungerte, sjekk felles motor-power, driver enable/current limit og batteridrop.

Nyttige overstyringer:

```bash
PWM_VALUE=170 STEP_DURATION=2.0 make mega-motor-test
INTER_STEP_PAUSE=5.0 SAMPLE_PERIOD=0.10 make mega-motor-test
```

Default er `STEP_DURATION=2.5` og `INTER_STEP_PAUSE=3.0`, slik at det er tid
til å se hvilken side som faktisk roterer mellom hvert steg.

Etter motor-testen, last tilbake runtime-firmware før ROS-bringup:

```bash
make mega-upload mega_keyboard_drive
```

Krav:

```bash
arduino-cli core install arduino:avr
```

## Runtime Firmware Kommandoer

Fra [`mega_keyboard_drive.ino`](../../arduino/mega_keyboard_drive/mega_keyboard_drive.ino):

| Kommando | Svar | Effekt |
|---|---|---|
| `ID` | `MEGA_KEYBOARD_DRIVE` | Firmware-sjekk |
| `PING` | `PONG` | Kommunikasjonstest |
| `STOP` | `OK STOP` | Stopper motorene |
| `ENC1` | `ENC1 <ticks>` | Leser encoder 1 |
| `ENC2` | `ENC2 <ticks>` | Leser encoder 2 |
| `RESET ENC1` | `OK RESET ENC1` | Nullstiller encoder 1 |
| `RESET ENC2` | `OK RESET ENC2` | Nullstiller encoder 2 |
| `STATE` | State-linje | Debug |
| `M1 <pwm>` | ingen normal reply | Setter bare M1 PWM |
| `M2 <pwm>` | ingen normal reply | Setter bare M2 PWM |
| `BOTH <m1> <m2>` | ingen normal reply | Setter PWM på begge motorer |

Watchdog stopper motorene hvis drive-kommandoer ikke repeteres innen `700 ms`.

## ROS Mega-driver

Kode: [`mega_driver_node.py`](../../src/mekk4_bringup/mekk4_bringup/mega_driver_node.py)

Input:

```text
/cmd_vel
```

Output:

```text
/odom       når EKF er av
/wheel/odom når EKF er på
```

Driveren:

1. Åpner serial port.
2. Sender `ID`, forventer `MEGA_KEYBOARD_DRIVE`.
3. Sender `PING`.
4. Sender `STOP`.
5. Nullstiller encoderne hvis `reset_encoders_on_connect=true`.
6. Oversetter Twist til `BOTH <left_pwm> <right_pwm>`.
7. Leser `ENC1` og `ENC2` for odometri.

## Viktige Parametre

| Parameter | Default | Bruk |
|---|---:|---|
| `port` | `/dev/ttyACM0` | Serial device |
| `baudrate` | `115200` | Serial baud |
| `cmd_vel_timeout_s` | `0.5` | Stopper ved stale cmd_vel |
| `send_period_s` | `0.2` | Repetisjon av motor command |
| `odom_poll_period_s` | `0.05` | Encoder poll |
| `base_frame_id` | `chassis` via Pi launch | Odom child frame |
| `publish_tf` | av/på etter EKF | Om driver publiserer odom TF |
| `swap_sides` | `true` | Bytter venstre/høyre mapping |
| `left_m_per_tick` | fra config | Venstre encoderkalibrering |
| `right_m_per_tick` | fra config | Høyre encoderkalibrering |
| `track_width_eff_m` | fra config | Effektiv beltebredde |

## Direkte Verktøy

```bash
make mega-test
make mega-motor-test
make mega-terminal
make mega-keyboard
make pc-mega-keyboard
```

Viktig: direkte serial-verktøy og ROS Mega-driver kan ikke eie samme port
samtidig.

`make mega-terminal` åpner et manuelt serial-program. Bruk det for å teste én
kommando om gangen:

```text
ID
M1 120
STOP
M1 -120
STOP
M2 120
STOP
M2 -120
STOP
ENC1
ENC2
```
