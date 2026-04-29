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

Runtime firmware:

```bash
MEGA_SKETCH=mega_keyboard_drive make mega-upload
```

Overstyr port eller board:

```bash
MEGA_PORT=/dev/ttyACM0 make mega-upload
MEGA_FQBN=arduino:avr:mega make mega-upload
```

Motor-/encoder-test:

```bash
make mega-motor-test
```

Denne bruker [`mega_dfr0601_test`](../../arduino/mega_dfr0601_test) og er
ment for å sjekke at venstre og høyre motor faktisk beveger seg som forventet
før du justerer ROS-kalibrering.

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
| `BOTH <m1> <m2>` | `OK BOTH ...` | Setter PWM på begge motorer |

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
make mega-keyboard
make pc-mega-keyboard
```

Viktig: direkte serial-verktøy og ROS Mega-driver kan ikke eie samme port
samtidig.
