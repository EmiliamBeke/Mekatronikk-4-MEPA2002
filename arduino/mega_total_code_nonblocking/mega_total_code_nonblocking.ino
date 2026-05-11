#include <EEPROM.h>
#include <Servo.h>
#include <Wire.h>
#include <stdlib.h>
#include <string.h>
#include <vl53l4ed_class.h>

namespace {

constexpr long kBaudrate = 115200;
constexpr size_t kMaxCommandLength = 63;
constexpr unsigned long kDriveTimeoutMs = 700;
constexpr unsigned long kStatusPeriodMs = 50;
constexpr unsigned long kDistancePeriodMs = 20;
constexpr unsigned long kDistanceReinitIntervalMs = 5000;

constexpr int kHallA1Pin = 3;
constexpr int kHallB1Pin = 2;
constexpr int kIna1Pin = 4;
constexpr int kInb1Pin = 5;
constexpr int kPwm1Pin = 6;

constexpr int kIna2Pin = 11;
constexpr int kInb2Pin = 12;
constexpr int kPwm2Pin = 10;
constexpr int kHallA2Pin = 18;
constexpr int kHallB2Pin = 19;

constexpr int kXStepPin = 45;
constexpr int kXDirPin = 29;
constexpr int kXEnPin = 37;
constexpr int kXLimitPin = 27;
constexpr int kXLimitActiveState = HIGH;
constexpr long kXHomeDir = -1;

constexpr int kZStepPin = 36;
constexpr int kZDirPin = 28;
constexpr int kZEnPin = 52;
constexpr int kZLimitPin = 44;
constexpr int kZLimitActiveState = HIGH;
constexpr long kZHomeDir = -1;
constexpr int kArmStepperEnableActiveState = LOW;

constexpr int kServoPin = 46;
constexpr int kServoHomeUs = 500;
constexpr int kServoClosedUs = 1800;
constexpr int kServoMinUs = 500;
constexpr int kServoMaxUs = 2500;
constexpr unsigned long kHomeGripperOpenDelayMs = 500;
constexpr int kDistanceXshutPin = -1;
constexpr int kDistanceNoReadingMm = 9999;

constexpr float kXStepsPerMm = 18.65f;
constexpr float kZStepsPerMm = 2929.0f;
constexpr float kHomeBackoffMm = 5.0f;
constexpr float kStartupXHomeMm = 82.0f;
constexpr float kStartupXFinalMm = -70.0f;
constexpr float kStartupZClearanceMm = 130.0f;

constexpr unsigned int kPulseUs = 10;
constexpr unsigned int kXStepPeriodUs = 1000;
constexpr unsigned int kZStepPeriodUs = 70;
constexpr long kXHomeMaxSteps = 10000;
constexpr long kZHomeMaxSteps = 800000;
constexpr uint32_t kPersistMagic = 0x4D454741UL;
constexpr uint8_t kPersistVersion = 1;
constexpr int kPersistAddress = 0;
constexpr unsigned long kPersistIdleSaveMs = 1000;

struct Axis {
  int step_pin;
  int dir_pin;
  int en_pin;
  int limit_pin;
  bool invert_dir;
  long home_dir;
  long positive_dir;
  long home_position;
  unsigned int period_us;
  long position;
  long target;
  unsigned long last_step_us;
  bool moving;
};

struct PersistedArmState {
  uint32_t magic;
  uint8_t version;
  uint8_t homed;
  long x_position;
  long z_position;
  int servo_us;
  uint8_t checksum;
};

Axis x_axis = {kXStepPin, kXDirPin, kXEnPin, kXLimitPin, false, kXHomeDir, kXHomeDir, 0, kXStepPeriodUs, 0, 0, 0, false};
Axis z_axis = {kZStepPin, kZDirPin, kZEnPin, kZLimitPin, false, kZHomeDir, 1, 0, kZStepPeriodUs, 0, 0, 0, false};

volatile long encoder1_count = 0;
volatile uint8_t encoder1_state = 0;
volatile long encoder2_count = 0;
volatile uint8_t encoder2_state = 0;

char command_buffer[kMaxCommandLength + 1];
size_t command_length = 0;
int current_m1_speed = 0;
int current_m2_speed = 0;
unsigned long last_drive_command_ms = 0;
bool drive_watchdog_armed = false;
bool stream_status = false;
unsigned long last_status_ms = 0;
unsigned long last_distance_ms = 0;
unsigned long last_distance_fail_ms = 0;
int servo_us = kServoHomeUs;
int distance_mm = kDistanceNoReadingMm;
bool distance_ok = false;
int last_x_limit_state = -1;
int last_z_limit_state = -1;
bool arm_homed = false;
bool arm_steppers_enabled = false;
bool persist_dirty = false;
unsigned long last_motion_or_servo_ms = 0;

Servo gripper_servo;
VL53L4ED distance_sensor(&Wire, kDistanceXshutPin);

void step_blocking(Axis &axis, long physical_dir);

constexpr int8_t kQuadratureDelta[16] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};

long mm_to_steps(float mm, float steps_per_mm) {
  const float steps = mm * steps_per_mm;
  return steps >= 0.0f ? static_cast<long>(steps + 0.5f) : static_cast<long>(steps - 0.5f);
}

int clamp_pwm(int speed) {
  return max(-255, min(255, speed));
}

uint8_t checksum_state(const PersistedArmState &state) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&state);
  uint8_t checksum = 0;
  for (size_t i = 0; i < sizeof(PersistedArmState) - 1; i++) {
    checksum ^= bytes[i];
  }
  return checksum;
}

void save_arm_state() {
  PersistedArmState state;
  memset(&state, 0, sizeof(state));
  state.magic = kPersistMagic;
  state.version = kPersistVersion;
  state.homed = arm_homed ? 1 : 0;
  state.x_position = x_axis.position;
  state.z_position = z_axis.position;
  state.servo_us = servo_us;
  state.checksum = checksum_state(state);
  EEPROM.put(kPersistAddress, state);
  persist_dirty = false;
  Serial.println("EVENT ARM STATE SAVED");
}

bool load_arm_state() {
  PersistedArmState state;
  EEPROM.get(kPersistAddress, state);
  if (state.magic != kPersistMagic || state.version != kPersistVersion ||
      state.checksum != checksum_state(state)) {
    return false;
  }

  arm_homed = state.homed != 0;
  x_axis.position = state.x_position;
  x_axis.target = state.x_position;
  z_axis.position = state.z_position;
  z_axis.target = state.z_position;
  servo_us = constrain(state.servo_us, kServoMinUs, kServoMaxUs);
  persist_dirty = false;
  return true;
}

void clear_arm_state() {
  PersistedArmState state;
  memset(&state, 0, sizeof(state));
  EEPROM.put(kPersistAddress, state);
  arm_homed = false;
  persist_dirty = false;
  Serial.println("OK ARM STATE CLEARED");
}

void mark_persist_dirty() {
  if (!arm_homed) {
    return;
  }
  persist_dirty = true;
  last_motion_or_servo_ms = millis();
}

void maybe_save_arm_state() {
  if (!persist_dirty || x_axis.moving || z_axis.moving) {
    return;
  }
  if (millis() - last_motion_or_servo_ms < kPersistIdleSaveMs) {
    return;
  }
  save_arm_state();
}

bool should_flip_pure_spin(int m1, int m2) {
  if (m1 == 0 || m2 == 0 || ((m1 > 0) == (m2 > 0))) {
    return false;
  }
  const int a1 = abs(m1);
  const int a2 = abs(m2);
  return min(a1, a2) * 10 >= max(a1, a2) * 8;
}

uint8_t read_encoder_state(int a_pin, int b_pin) {
  return static_cast<uint8_t>((digitalRead(a_pin) << 1) | digitalRead(b_pin));
}

void on_encoder1_change() {
  const uint8_t state = read_encoder_state(kHallA1Pin, kHallB1Pin);
  encoder1_count += kQuadratureDelta[(encoder1_state << 2) | state];
  encoder1_state = state;
}

void on_encoder2_change() {
  const uint8_t state = read_encoder_state(kHallA2Pin, kHallB2Pin);
  encoder2_count += kQuadratureDelta[(encoder2_state << 2) | state];
  encoder2_state = state;
}

long read_encoder(volatile long &count) {
  noInterrupts();
  const long value = count;
  interrupts();
  return value;
}

void reset_encoder1() {
  noInterrupts();
  encoder1_count = 0;
  encoder1_state = read_encoder_state(kHallA1Pin, kHallB1Pin);
  interrupts();
}

void reset_encoder2() {
  noInterrupts();
  encoder2_count = 0;
  encoder2_state = read_encoder_state(kHallA2Pin, kHallB2Pin);
  interrupts();
}

void motor_output(int ina, int inb, int pwm_pin, int speed) {
  digitalWrite(ina, speed > 0 ? HIGH : LOW);
  digitalWrite(inb, speed < 0 ? HIGH : LOW);
  analogWrite(pwm_pin, abs(speed));
}

void set_arm_steppers_enabled(bool enabled) {
  arm_steppers_enabled = enabled;
  const int level = enabled ? kArmStepperEnableActiveState : !kArmStepperEnableActiveState;
  digitalWrite(kXEnPin, level);
  digitalWrite(kZEnPin, level);
}

void open_gripper_for_homing() {
  if (servo_us == kServoHomeUs) {
    return;
  }

  servo_us = kServoHomeUs;
  gripper_servo.writeMicroseconds(servo_us);
  mark_persist_dirty();
  Serial.print("EVENT SERVO OPEN ");
  Serial.println(servo_us);
  delay(kHomeGripperOpenDelayMs);
}

void apply_drive(int m1, int m2) {
  current_m1_speed = clamp_pwm(m1);
  current_m2_speed = clamp_pwm(m2);
  if (should_flip_pure_spin(current_m1_speed, current_m2_speed)) {
    current_m1_speed = -current_m1_speed;
    current_m2_speed = -current_m2_speed;
  }
  motor_output(kIna1Pin, kInb1Pin, kPwm1Pin, current_m1_speed);
  motor_output(kIna2Pin, kInb2Pin, kPwm2Pin, current_m2_speed);
  drive_watchdog_armed = current_m1_speed != 0 || current_m2_speed != 0;
  last_drive_command_ms = millis();
}

void stop_all() {
  x_axis.moving = false;
  z_axis.moving = false;
  x_axis.target = x_axis.position;
  z_axis.target = z_axis.position;
  drive_watchdog_armed = false;
  current_m1_speed = 0;
  current_m2_speed = 0;
  motor_output(kIna1Pin, kInb1Pin, kPwm1Pin, 0);
  motor_output(kIna2Pin, kInb2Pin, kPwm2Pin, 0);
}

bool limit_active(const Axis &axis) {
  if (axis.limit_pin == kXLimitPin) {
    return digitalRead(axis.limit_pin) == kXLimitActiveState;
  }
  return digitalRead(axis.limit_pin) == kZLimitActiveState;
}

void maybe_print_limit_switch_changes() {
  const int x_state = digitalRead(kXLimitPin);
  const int z_state = digitalRead(kZLimitPin);

  if (last_x_limit_state != x_state) {
    last_x_limit_state = x_state;
    Serial.print("EVENT LIMIT 27 ");
    Serial.println(x_state);
  }

  if (last_z_limit_state != z_state) {
    last_z_limit_state = z_state;
    Serial.print("EVENT LIMIT 44 ");
    Serial.println(z_state);
  }
}

void set_dir(const Axis &axis, long physical_dir) {
  const bool positive = physical_dir > 0;
  const bool level = axis.invert_dir ? !positive : positive;
  digitalWrite(axis.dir_pin, level ? HIGH : LOW);
}

void pulse_step(const Axis &axis) {
  digitalWrite(axis.step_pin, HIGH);
  delayMicroseconds(kPulseUs);
  digitalWrite(axis.step_pin, LOW);
}

void move_axis_relative(Axis &axis, long delta) {
  axis.target += delta;
  axis.moving = axis.target != axis.position;
  if (axis.moving) {
    mark_persist_dirty();
  }
}

bool move_axis_relative_blocking(Axis &axis, const char *name, long delta) {
  if (delta == 0) {
    axis.target = axis.position;
    axis.moving = false;
    return true;
  }

  set_arm_steppers_enabled(true);
  axis.moving = true;
  const long logical_dir = delta > 0 ? 1 : -1;
  const long physical_dir = logical_dir > 0 ? axis.positive_dir : -axis.positive_dir;
  const long count = labs(delta);

  for (long i = 0; i < count; i++) {
    if (physical_dir == axis.home_dir && limit_active(axis)) {
      axis.position = axis.home_position;
      axis.target = axis.position;
      axis.moving = false;
      mark_persist_dirty();
      Serial.print("EVENT ");
      Serial.print(name);
      Serial.println(" LIMIT");
      return false;
    }

    step_blocking(axis, physical_dir);
    axis.position += logical_dir;
    last_motion_or_servo_ms = millis();
  }

  axis.target = axis.position;
  axis.moving = false;
  mark_persist_dirty();
  return true;
}

void update_axis(Axis &axis, const char *name) {
  if (!axis.moving || axis.position == axis.target) {
    if (axis.moving) {
      axis.moving = false;
      mark_persist_dirty();
    }
    return;
  }

  const long logical_dir = axis.target > axis.position ? 1 : -1;
  const long physical_dir = logical_dir > 0 ? axis.positive_dir : -axis.positive_dir;
  if (physical_dir == axis.home_dir && limit_active(axis)) {
    axis.position = axis.home_position;
    axis.target = axis.position;
    axis.moving = false;
    mark_persist_dirty();
    Serial.print("EVENT ");
    Serial.print(name);
    Serial.println(" LIMIT");
    return;
  }

  const unsigned long now = micros();
  if (now - axis.last_step_us < axis.period_us) {
    return;
  }

  set_dir(axis, physical_dir);
  pulse_step(axis);
  axis.position += logical_dir;
  axis.last_step_us = now;
  last_motion_or_servo_ms = millis();
}

void step_blocking(Axis &axis, long physical_dir) {
  set_dir(axis, physical_dir);
  pulse_step(axis);
  delayMicroseconds(axis.period_us);
}

bool home_axis(Axis &axis, const char *name, long max_steps, bool print_ok = true) {
  set_arm_steppers_enabled(true);
  const long backoff = name[0] == 'X'
                         ? mm_to_steps(kHomeBackoffMm, kXStepsPerMm)
                         : mm_to_steps(kHomeBackoffMm, kZStepsPerMm);

  if (limit_active(axis)) {
    for (long i = 0; i < backoff; i++) {
      step_blocking(axis, -axis.home_dir);
    }
  }

  for (long i = 0; i < max_steps; i++) {
    if (limit_active(axis)) {
      for (long j = 0; j < backoff; j++) {
        step_blocking(axis, -axis.home_dir);
      }
      axis.position = axis.home_position;
      axis.target = axis.position;
      axis.moving = false;
      if (print_ok) {
        Serial.print("OK HOME ");
        Serial.print(name);
        Serial.print(" ");
        Serial.println(axis.position);
      }
      return true;
    }
    step_blocking(axis, axis.home_dir);
  }

  Serial.print("ERR HOME ");
  Serial.print(name);
  Serial.println(" TIMEOUT");
  return false;
}

bool startup_home_arm() {
  Serial.println("EVENT ARM STARTUP HOME BEGIN");
  open_gripper_for_homing();
  arm_homed = false;
  stop_all();
  x_axis.home_position = mm_to_steps(kStartupXHomeMm, kXStepsPerMm);
  z_axis.home_position = 0;
  if (!home_axis(x_axis, "X", kXHomeMaxSteps)) {
    Serial.println("ERR ARM STARTUP HOME X");
    return false;
  }
  if (!home_axis(z_axis, "Z", kZHomeMaxSteps)) {
    Serial.println("ERR ARM STARTUP HOME Z");
    return false;
  }

  move_axis_relative(z_axis, mm_to_steps(kStartupZClearanceMm, kZStepsPerMm));
  while (z_axis.moving) {
    update_axis(z_axis, "Z");
  }

  move_axis_relative(x_axis, mm_to_steps(kStartupXFinalMm - kStartupXHomeMm, kXStepsPerMm));
  while (x_axis.moving) {
    update_axis(x_axis, "X");
  }

  arm_homed = true;
  Serial.print("OK ARM STARTUP HOME X=");
  Serial.print(x_axis.position);
  Serial.print(" Z=");
  Serial.println(z_axis.position);
  save_arm_state();
  return true;
}

bool home_x_axis_only() {
  stop_all();
  open_gripper_for_homing();
  x_axis.home_position = mm_to_steps(kStartupXHomeMm, kXStepsPerMm);
  if (!home_axis(x_axis, "X", kXHomeMaxSteps, false)) {
    return false;
  }

  move_axis_relative(x_axis, mm_to_steps(kStartupXFinalMm - kStartupXHomeMm, kXStepsPerMm));
  while (x_axis.moving) {
    update_axis(x_axis, "X");
  }

  arm_homed = true;
  Serial.print("OK X HOME FINAL ");
  Serial.println(x_axis.position);
  save_arm_state();
  return true;
}

bool calibrate_x_axis_extended() {
  stop_all();
  open_gripper_for_homing();
  x_axis.home_position = mm_to_steps(kStartupXHomeMm, kXStepsPerMm);
  if (!home_axis(x_axis, "X", kXHomeMaxSteps, false)) {
    return false;
  }

  arm_homed = true;
  Serial.print("OK X CAL EXTENDED ");
  Serial.println(x_axis.position);
  save_arm_state();
  return true;
}

// silent=true brukes ved auto-reinit for å unngå at uanmodet output
// forstyrrer request/reply-protokollen til mega_driver_node.
void init_distance_sensor(bool silent = false) {
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);  // 25 ms I2C timeout; resets bus on hang
  delay(50);

  const int begin_status = distance_sensor.begin();
  if (begin_status != 0) {
    if (silent) { Serial.print("EVENT DIST REINIT ERR BEGIN "); } else { Serial.print("ERR DIST BEGIN "); }
    Serial.println(begin_status);
    distance_ok = false;
    return;
  }

  const int init_status = distance_sensor.InitSensor();
  if (init_status != 0) {
    if (silent) { Serial.print("EVENT DIST REINIT ERR INIT "); } else { Serial.print("ERR DIST INIT "); }
    Serial.println(init_status);
    distance_ok = false;
    return;
  }

  const int start_status = distance_sensor.VL53L4ED_StartRanging();
  if (start_status != 0) {
    if (silent) { Serial.print("EVENT DIST REINIT ERR START "); } else { Serial.print("ERR DIST START "); }
    Serial.println(start_status);
    distance_ok = false;
    return;
  }
  distance_ok = true;
  if (silent) { Serial.println("EVENT DIST REINIT OK"); } else { Serial.println("OK DIST INIT"); }
}

void update_distance() {
  if (!distance_ok) {
    if (millis() - last_distance_fail_ms >= kDistanceReinitIntervalMs) {
      last_distance_fail_ms = millis();
      init_distance_sensor(true);
    }
    return;
  }

  if (millis() - last_distance_ms < kDistancePeriodMs) {
    return;
  }
  last_distance_ms = millis();

  uint8_t ready = 0;
  if (distance_sensor.VL53L4ED_CheckForDataReady(&ready) != 0 || !ready) {
    if (Wire.getWireTimeoutFlag()) {
      Wire.clearWireTimeoutFlag();
      distance_ok = false;
      distance_mm = kDistanceNoReadingMm;
      last_distance_fail_ms = millis();
      Serial.println("ERR DIST I2C TIMEOUT");
    }
    return;
  }

  VL53L4ED_ResultsData_t result;
  distance_sensor.VL53L4ED_ClearInterrupt();
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    distance_ok = false;
    distance_mm = kDistanceNoReadingMm;
    last_distance_fail_ms = millis();
    Serial.println("ERR DIST I2C TIMEOUT");
    return;
  }

  if (distance_sensor.VL53L4ED_GetResult(&result) != 0) {
    if (Wire.getWireTimeoutFlag()) {
      Wire.clearWireTimeoutFlag();
      distance_ok = false;
      distance_mm = kDistanceNoReadingMm;
      last_distance_fail_ms = millis();
      Serial.println("ERR DIST I2C TIMEOUT");
      return;
    }
    distance_mm = kDistanceNoReadingMm;
    return;
  }

  if (result.distance_mm > 0) {
    distance_mm = static_cast<int>(result.distance_mm);
  } else {
    distance_mm = kDistanceNoReadingMm;
  }
}

void maybe_stop_on_watchdog() {
  if (drive_watchdog_armed && millis() - last_drive_command_ms > kDriveTimeoutMs) {
    stop_all();
    Serial.println("EVENT WATCHDOG STOP");
  }
}

void print_state(const char *prefix) {
  Serial.print(prefix);
  Serial.print(" M1=");
  Serial.print(current_m1_speed);
  Serial.print(" M2=");
  Serial.print(current_m2_speed);
  Serial.print(" ENC1=");
  Serial.print(read_encoder(encoder1_count));
  Serial.print(" ENC2=");
  Serial.print(read_encoder(encoder2_count));
  Serial.print(" X=");
  Serial.print(x_axis.position);
  Serial.print(" Z=");
  Serial.print(z_axis.position);
  Serial.print(" XM=");
  Serial.print(x_axis.moving ? 1 : 0);
  Serial.print(" ZM=");
  Serial.print(z_axis.moving ? 1 : 0);
  Serial.print(" XL=");
  Serial.print(limit_active(x_axis) ? 1 : 0);
  Serial.print(" ZL=");
  Serial.print(limit_active(z_axis) ? 1 : 0);
  Serial.print(" D=");
  Serial.print(distance_mm);
  Serial.print(" SERVO=");
  Serial.print(servo_us);
  Serial.print(" H=");
  Serial.print(arm_homed ? 1 : 0);
  Serial.print(" EH=");
  Serial.print(arm_steppers_enabled ? 1 : 0);
  Serial.print(" P=");
  Serial.println(persist_dirty ? 1 : 0);
}

void maybe_stream_status() {
  if (stream_status && millis() - last_status_ms >= kStatusPeriodMs) {
    last_status_ms = millis();
    print_state("ST");
  }
}

void reset_command_buffer() {
  command_length = 0;
  command_buffer[0] = '\0';
}

void handle_command(const char *cmd) {
  int a = 0;
  if (strcmp(cmd, "PING") == 0) {
    Serial.println("PONG");
  } else if (strcmp(cmd, "ID") == 0) {
    Serial.println("MEGA_KEYBOARD_DRIVE");
  } else if (strcmp(cmd, "STOP") == 0) {
    stop_all();
    Serial.println("OK STOP");
  } else if (strcmp(cmd, "ENC1") == 0) {
    Serial.print("ENC1 ");
    Serial.println(read_encoder(encoder1_count));
  } else if (strcmp(cmd, "ENC2") == 0) {
    Serial.print("ENC2 ");
    Serial.println(read_encoder(encoder2_count));
  } else if (strcmp(cmd, "RESET ENC1") == 0) {
    reset_encoder1();
    Serial.println("OK RESET ENC1");
  } else if (strcmp(cmd, "RESET ENC2") == 0) {
    reset_encoder2();
    Serial.println("OK RESET ENC2");
  } else if (strcmp(cmd, "STATE") == 0 || strcmp(cmd, "STATE?") == 0) {
    print_state("STATE");
  } else if (strcmp(cmd, "DIST") == 0) {
    Serial.print("DIST ");
    Serial.println(distance_mm);
  } else if (strcmp(cmd, "DIST?") == 0) {
    Serial.print("DIST D=");
    Serial.println(distance_mm);
  } else if (strcmp(cmd, "DIST INIT") == 0) {
    init_distance_sensor();
  } else if (strcmp(cmd, "LIMITS") == 0) {
    Serial.print("LIMITS RAW27=");
    Serial.print(digitalRead(kXLimitPin));
    Serial.print(" RAW44=");
    Serial.print(digitalRead(kZLimitPin));
    Serial.print(" PRESSED27=");
    Serial.print(limit_active(x_axis) ? 1 : 0);
    Serial.print(" PRESSED44=");
    Serial.println(limit_active(z_axis) ? 1 : 0);
  } else if (strcmp(cmd, "HOME ARM") == 0) {
    stop_all();
    startup_home_arm();
  } else if (strcmp(cmd, "CLEAR ARM STATE") == 0 || strcmp(cmd, "FORGET ARM") == 0) {
    stop_all();
    clear_arm_state();
  } else if (strcmp(cmd, "HOME X") == 0) {
    home_x_axis_only();
  } else if (strcmp(cmd, "CAL X") == 0) {
    calibrate_x_axis_extended();
  } else if (strcmp(cmd, "HOME Z") == 0) {
    stop_all();
    open_gripper_for_homing();
    home_axis(z_axis, "Z", kZHomeMaxSteps);
  } else if (sscanf(cmd, "ARM HOLD %d", &a) == 1 || sscanf(cmd, "HOLD %d", &a) == 1) {
    set_arm_steppers_enabled(a != 0);
    Serial.print("OK ARM HOLD ");
    Serial.println(arm_steppers_enabled ? 1 : 0);
  } else {
    int b = 0;
    long steps = 0;
    if (sscanf(cmd, "BOTH %d %d", &a, &b) == 2 || sscanf(cmd, "D %d %d", &a, &b) == 2) {
      apply_drive(a, b);
    } else if (sscanf(cmd, "M1 %d", &a) == 1) {
      apply_drive(a, current_m2_speed);
    } else if (sscanf(cmd, "M2 %d", &a) == 1) {
      apply_drive(current_m1_speed, a);
    } else if (sscanf(cmd, "ARM X %ld", &steps) == 1 || sscanf(cmd, "X %ld", &steps) == 1) {
      if (!arm_homed) {
        Serial.println("ERR ARM NOT HOMED");
        return;
      }
      move_axis_relative_blocking(x_axis, "X", steps);
      Serial.println("OK ARM X");
    } else if (sscanf(cmd, "ARM Z %ld", &steps) == 1 || sscanf(cmd, "Z %ld", &steps) == 1) {
      if (!arm_homed) {
        Serial.println("ERR ARM NOT HOMED");
        return;
      }
      move_axis_relative_blocking(z_axis, "Z", steps);
      Serial.println("OK ARM Z");
    } else if (sscanf(cmd, "SERVO %d", &a) == 1 || sscanf(cmd, "S %d", &a) == 1) {
      servo_us = constrain(a, kServoMinUs, kServoMaxUs);
      gripper_servo.writeMicroseconds(servo_us);
      mark_persist_dirty();
      Serial.print("OK SERVO ");
      Serial.println(servo_us);
    } else if (sscanf(cmd, "STREAM %d", &a) == 1) {
      stream_status = a != 0;
      Serial.println("OK STREAM");
    } else {
      Serial.println("ERR UNKNOWN");
    }
  }
}

void read_serial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      command_buffer[command_length] = '\0';
      if (command_length > 0) {
        handle_command(command_buffer);
      }
      reset_command_buffer();
    } else if (command_length < kMaxCommandLength) {
      command_buffer[command_length++] = c;
    }
  }
}

}  // namespace

void setup() {
  pinMode(kIna1Pin, OUTPUT);
  pinMode(kInb1Pin, OUTPUT);
  pinMode(kPwm1Pin, OUTPUT);
  pinMode(kIna2Pin, OUTPUT);
  pinMode(kInb2Pin, OUTPUT);
  pinMode(kPwm2Pin, OUTPUT);
  pinMode(kHallA1Pin, INPUT_PULLUP);
  pinMode(kHallB1Pin, INPUT_PULLUP);
  pinMode(kHallA2Pin, INPUT_PULLUP);
  pinMode(kHallB2Pin, INPUT_PULLUP);

  pinMode(kXStepPin, OUTPUT);
  pinMode(kXDirPin, OUTPUT);
  pinMode(kXEnPin, OUTPUT);
  pinMode(kXLimitPin, INPUT_PULLUP);
  pinMode(kZStepPin, OUTPUT);
  pinMode(kZDirPin, OUTPUT);
  pinMode(kZEnPin, OUTPUT);
  pinMode(kZLimitPin, INPUT_PULLUP);
  digitalWrite(kXStepPin, LOW);
  digitalWrite(kXDirPin, LOW);
  digitalWrite(kZStepPin, LOW);
  digitalWrite(kZDirPin, LOW);
  set_arm_steppers_enabled(true);

  Serial.begin(kBaudrate);
  while (!Serial && millis() < 3000) {
  }

  stop_all();
  reset_encoder1();
  reset_encoder2();
  attachInterrupt(digitalPinToInterrupt(kHallA1Pin), on_encoder1_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kHallB1Pin), on_encoder1_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kHallA2Pin), on_encoder2_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kHallB2Pin), on_encoder2_change, CHANGE);

  maybe_print_limit_switch_changes();
  reset_command_buffer();
  if (load_arm_state()) {
    Serial.print("EVENT ARM STATE LOADED X=");
    Serial.print(x_axis.position);
    Serial.print(" Z=");
    Serial.print(z_axis.position);
    Serial.print(" H=");
    Serial.println(arm_homed ? 1 : 0);
  } else {
    Serial.println("EVENT ARM STATE EMPTY");
  }
  gripper_servo.attach(kServoPin, kServoMinUs, kServoMaxUs);
  gripper_servo.writeMicroseconds(servo_us);
  Serial.println("MEGA_KEYBOARD_READY");
}

void loop() {
  maybe_print_limit_switch_changes();
  read_serial();
  maybe_stop_on_watchdog();
  update_axis(x_axis, "X");
  update_axis(z_axis, "Z");
  update_distance();
  maybe_save_arm_state();
  maybe_stream_status();
}
