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
constexpr long kXHomeDir = -1;

constexpr int kZStepPin = 36;
constexpr int kZDirPin = 28;
constexpr int kZEnPin = 52;
constexpr int kZLimitPin = 44;
constexpr long kZHomeDir = -1;

constexpr int kServoPin = 46;
constexpr int kDistanceXshutPin = 7;
constexpr int kDistanceOffsetMm = 15;

constexpr float kXStepsPerMm = 18.65f;
constexpr float kZStepsPerMm = 2929.0f;
constexpr float kHomeBackoffMm = 5.0f;
constexpr float kStartupXMaxMm = 150.0f;
constexpr float kStartupZClearanceMm = 120.0f;

constexpr unsigned int kPulseUs = 10;
constexpr unsigned int kXStepPeriodUs = 1000;
constexpr unsigned int kZStepPeriodUs = 100;
constexpr long kXHomeMaxSteps = 10000;
constexpr long kZHomeMaxSteps = 800000;

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
int servo_angle = 90;
int distance_mm = -1;
bool distance_ok = false;

Servo gripper_servo;
VL53L4ED distance_sensor(&Wire, kDistanceXshutPin);

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
  return digitalRead(axis.limit_pin) == HIGH;
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
}

void update_axis(Axis &axis, const char *name) {
  if (!axis.moving || axis.position == axis.target) {
    axis.moving = false;
    return;
  }

  const long logical_dir = axis.target > axis.position ? 1 : -1;
  const long physical_dir = logical_dir > 0 ? axis.positive_dir : -axis.positive_dir;
  if (physical_dir == axis.home_dir && limit_active(axis)) {
    axis.position = axis.home_position;
    axis.target = axis.position;
    axis.moving = false;
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
}

void step_blocking(Axis &axis, long physical_dir) {
  set_dir(axis, physical_dir);
  pulse_step(axis);
  delayMicroseconds(axis.period_us);
}

bool home_axis(Axis &axis, const char *name, long max_steps) {
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
      Serial.print("OK HOME ");
      Serial.print(name);
      Serial.print(" ");
      Serial.println(axis.position);
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
  stop_all();
  x_axis.home_position = mm_to_steps(kStartupXMaxMm, kXStepsPerMm);
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
  move_axis_relative(x_axis, -mm_to_steps(kStartupXMaxMm, kXStepsPerMm));
  while (x_axis.moving || z_axis.moving) {
    update_axis(x_axis, "X");
    update_axis(z_axis, "Z");
  }
  Serial.print("OK ARM STARTUP HOME X=");
  Serial.print(x_axis.position);
  Serial.print(" Z=");
  Serial.println(z_axis.position);
  return true;
}

void init_distance_sensor() {
  Wire.begin();
  Wire.setClock(100000);
  pinMode(kDistanceXshutPin, OUTPUT);
  digitalWrite(kDistanceXshutPin, LOW);
  delay(200);
  digitalWrite(kDistanceXshutPin, HIGH);
  delay(1000);
  if (distance_sensor.begin() != 0 || distance_sensor.InitSensor() != 0 ||
      distance_sensor.VL53L4ED_StartRanging() != 0) {
    Serial.println("ERR DIST INIT");
    distance_ok = false;
    return;
  }
  distance_ok = true;
  Serial.println("OK DIST INIT");
}

void update_distance() {
  if (!distance_ok || millis() - last_distance_ms < kDistancePeriodMs) {
    return;
  }
  last_distance_ms = millis();
  uint8_t ready = 0;
  if (distance_sensor.VL53L4ED_CheckForDataReady(&ready) != 0 || !ready) {
    return;
  }
  VL53L4ED_ResultsData_t result;
  distance_sensor.VL53L4ED_ClearInterrupt();
  distance_sensor.VL53L4ED_GetResult(&result);
  distance_mm = max(0, static_cast<int>(result.distance_mm) - kDistanceOffsetMm);
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
  Serial.print(" XL=");
  Serial.print(limit_active(x_axis) ? 1 : 0);
  Serial.print(" ZL=");
  Serial.print(limit_active(z_axis) ? 1 : 0);
  Serial.print(" D=");
  Serial.print(distance_mm);
  Serial.print(" S=");
  Serial.println(servo_angle);
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
  } else if (strcmp(cmd, "HOME ARM") == 0) {
    stop_all();
    startup_home_arm();
  } else if (strcmp(cmd, "HOME X") == 0) {
    stop_all();
    home_axis(x_axis, "X", kXHomeMaxSteps);
  } else if (strcmp(cmd, "HOME Z") == 0) {
    stop_all();
    home_axis(z_axis, "Z", kZHomeMaxSteps);
  } else {
    int a = 0;
    int b = 0;
    long steps = 0;
    if (sscanf(cmd, "BOTH %d %d", &a, &b) == 2 || sscanf(cmd, "D %d %d", &a, &b) == 2) {
      apply_drive(a, b);
    } else if (sscanf(cmd, "M1 %d", &a) == 1) {
      apply_drive(a, current_m2_speed);
    } else if (sscanf(cmd, "M2 %d", &a) == 1) {
      apply_drive(current_m1_speed, a);
    } else if (sscanf(cmd, "ARM X %ld", &steps) == 1 || sscanf(cmd, "X %ld", &steps) == 1) {
      move_axis_relative(x_axis, steps);
      Serial.println("OK ARM X");
    } else if (sscanf(cmd, "ARM Z %ld", &steps) == 1 || sscanf(cmd, "Z %ld", &steps) == 1) {
      move_axis_relative(z_axis, steps);
      Serial.println("OK ARM Z");
    } else if (sscanf(cmd, "SERVO %d", &a) == 1 || sscanf(cmd, "S %d", &a) == 1) {
      servo_angle = constrain(a, 0, 180);
      gripper_servo.write(servo_angle);
      Serial.println("OK SERVO");
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
  pinMode(kXLimitPin, INPUT);
  pinMode(kZStepPin, OUTPUT);
  pinMode(kZDirPin, OUTPUT);
  pinMode(kZEnPin, OUTPUT);
  pinMode(kZLimitPin, INPUT);
  digitalWrite(kXEnPin, LOW);
  digitalWrite(kZEnPin, LOW);

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

  gripper_servo.attach(kServoPin);
  gripper_servo.write(servo_angle);
  init_distance_sensor();
  reset_command_buffer();
  if (startup_home_arm()) {
    Serial.println("MEGA_KEYBOARD_READY");
  } else {
    Serial.println("ERR MEGA_KEYBOARD_NOT_READY");
  }
}

void loop() {
  read_serial();
  maybe_stop_on_watchdog();
  update_axis(x_axis, "X");
  update_axis(z_axis, "Z");
  update_distance();
  maybe_stream_status();
}
