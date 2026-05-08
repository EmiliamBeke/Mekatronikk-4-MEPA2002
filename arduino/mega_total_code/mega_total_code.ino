#include <stdlib.h>
#include <string.h>
#include <Servo.h>

namespace {

constexpr long kBaudrate = 115200;
constexpr size_t kMaxCommandLength = 63;
constexpr unsigned long kDriveTimeoutMs = 700;

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

constexpr int kGripperServoPin = 46;
constexpr int kGripperOpenUs = 500;
constexpr int kGripperClosedUs = 1800;

constexpr int kXStepPin = 45;
constexpr int kXDirPin = 29;
constexpr int kXEnPin = 37;
constexpr bool kInvertXDirection = false;
constexpr int kXLimitPin = 27;
constexpr int kXLimitActiveState = HIGH;
constexpr long kXHomeDirectionStep = -1;
constexpr long kXHomeMaxSteps = 10000;

constexpr int kZStepPin = 36;
constexpr int kZDirPin = 28;
constexpr int kZEnPin = 52;
constexpr bool kInvertZDirection = false;
constexpr int kZLimitPin = 44;
constexpr int kZLimitActiveState = HIGH;
constexpr long kZHomeDirectionStep = -1;
constexpr long kZHomeMaxSteps = 800000;

constexpr float kXStepsPerMm = 18.65f;
constexpr float kZStepsPerMm = 2929.0f;
constexpr float kHomeBackoffMm = 5.0f;
constexpr float kStartupXMaxMm = 150.0f;
constexpr float kStartupZClearanceMm = 120.0f;

constexpr unsigned int kStepperPulseUs = 10;
constexpr unsigned int kXStepDelayUs = 1000;
constexpr unsigned int kZStepDelayUs = 70;

volatile long encoder1_count = 0;
volatile uint8_t encoder1_state = 0;
volatile long encoder2_count = 0;
volatile uint8_t encoder2_state = 0;

char command_buffer[kMaxCommandLength + 1];
size_t command_length = 0;

int current_m1_speed = 0;
int current_m2_speed = 0;
long current_x_steps = 0;
long current_z_steps = 0;
int current_gripper_us = kGripperOpenUs;
unsigned long last_drive_command_ms = 0;
bool drive_watchdog_armed = false;
int last_x_limit_state = -1;
int last_z_limit_state = -1;

Servo gripper_servo;

void maybe_print_limit_switch_changes();

constexpr int8_t kQuadratureDelta[16] = {
  0, -1,  1,  0,
  1,  0,  0, -1,
 -1,  0,  0,  1,
  0,  1, -1,  0
};

void reset_command_buffer() {
  command_length = 0;
  command_buffer[0] = '\0';
}

int clamp_pwm(int speed) {
  if (speed > 255) {
    return 255;
  }
  if (speed < -255) {
    return -255;
  }
  return speed;
}

int clamp_servo_us(int pulse_us) {
  if (pulse_us < kGripperOpenUs) {
    return kGripperOpenUs;
  }
  if (pulse_us > kGripperClosedUs) {
    return kGripperClosedUs;
  }
  return pulse_us;
}

void set_gripper_servo_us(int pulse_us) {
  current_gripper_us = clamp_servo_us(pulse_us);
  gripper_servo.writeMicroseconds(current_gripper_us);
}

long mm_to_steps(float millimeters, float steps_per_mm) {
  const float steps = millimeters * steps_per_mm;
  if (steps >= 0.0f) {
    return static_cast<long>(steps + 0.5f);
  }
  return static_cast<long>(steps - 0.5f);
}

bool should_flip_pure_spin(int m1_speed, int m2_speed) {
  if (m1_speed == 0 || m2_speed == 0) {
    return false;
  }

  if ((m1_speed > 0) == (m2_speed > 0)) {
    return false;
  }

  const int abs_m1 = abs(m1_speed);
  const int abs_m2 = abs(m2_speed);
  const int smaller = abs_m1 < abs_m2 ? abs_m1 : abs_m2;
  const int larger = abs_m1 > abs_m2 ? abs_m1 : abs_m2;

  // Treat nearly equal-and-opposite commands as an in-place spin.
  return (smaller * 10) >= (larger * 8);
}

uint8_t read_encoder1_state() {
  const uint8_t a = static_cast<uint8_t>(digitalRead(kHallA1Pin));
  const uint8_t b = static_cast<uint8_t>(digitalRead(kHallB1Pin));
  return static_cast<uint8_t>((a << 1) | b);
}

uint8_t read_encoder2_state() {
  const uint8_t a = static_cast<uint8_t>(digitalRead(kHallA2Pin));
  const uint8_t b = static_cast<uint8_t>(digitalRead(kHallB2Pin));
  return static_cast<uint8_t>((a << 1) | b);
}

void update_encoder1() {
  const uint8_t new_state = read_encoder1_state();
  const uint8_t transition = static_cast<uint8_t>((encoder1_state << 2) | new_state);
  encoder1_count += kQuadratureDelta[transition];
  encoder1_state = new_state;
}

void on_encoder1_change() {
  update_encoder1();
}

void update_encoder2() {
  const uint8_t new_state = read_encoder2_state();
  const uint8_t transition = static_cast<uint8_t>((encoder2_state << 2) | new_state);
  encoder2_count += kQuadratureDelta[transition];
  encoder2_state = new_state;
}

void on_encoder2_change() {
  update_encoder2();
}

long read_encoder1_count() {
  noInterrupts();
  const long count = encoder1_count;
  interrupts();
  return count;
}

long read_encoder2_count() {
  noInterrupts();
  const long count = encoder2_count;
  interrupts();
  return count;
}

void reset_encoder1_count() {
  noInterrupts();
  encoder1_count = 0;
  encoder1_state = read_encoder1_state();
  interrupts();
}

void reset_encoder2_count() {
  noInterrupts();
  encoder2_count = 0;
  encoder2_state = read_encoder2_state();
  interrupts();
}

void apply_motor_output(int ina_pin, int inb_pin, int pwm_pin, int speed) {
  const int pwm = abs(speed);

  if (speed > 0) {
    digitalWrite(ina_pin, HIGH);
    digitalWrite(inb_pin, LOW);
  } else if (speed < 0) {
    digitalWrite(ina_pin, LOW);
    digitalWrite(inb_pin, HIGH);
  } else {
    digitalWrite(ina_pin, LOW);
    digitalWrite(inb_pin, LOW);
  }

  analogWrite(pwm_pin, pwm);
}

void apply_drive(int m1_speed, int m2_speed) {
  current_m1_speed = clamp_pwm(m1_speed);
  current_m2_speed = clamp_pwm(m2_speed);

  if (should_flip_pure_spin(current_m1_speed, current_m2_speed)) {
    current_m1_speed = -current_m1_speed;
    current_m2_speed = -current_m2_speed;
  }

  apply_motor_output(kIna1Pin, kInb1Pin, kPwm1Pin, current_m1_speed);
  apply_motor_output(kIna2Pin, kInb2Pin, kPwm2Pin, current_m2_speed);

  if (current_m1_speed == 0 && current_m2_speed == 0) {
    drive_watchdog_armed = false;
    return;
  }

  last_drive_command_ms = millis();
  drive_watchdog_armed = true;
}

void stop_all() {
  current_m1_speed = 0;
  current_m2_speed = 0;
  drive_watchdog_armed = false;
  apply_motor_output(kIna1Pin, kInb1Pin, kPwm1Pin, 0);
  apply_motor_output(kIna2Pin, kInb2Pin, kPwm2Pin, 0);
}

void move_stepper(
  int step_pin,
  int dir_pin,
  long steps,
  bool invert_direction,
  unsigned int step_delay_us
) {
  if (steps == 0) {
    return;
  }

  const bool positive_direction = steps > 0;
  const bool dir_level = invert_direction ? !positive_direction : positive_direction;
  const long count = labs(steps);

  digitalWrite(dir_pin, dir_level ? HIGH : LOW);

  for (long i = 0; i < count; i++) {
    maybe_print_limit_switch_changes();
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(kStepperPulseUs);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(step_delay_us);
    maybe_print_limit_switch_changes();
  }
}

bool x_limit_active() {
  return digitalRead(kXLimitPin) == kXLimitActiveState;
}

void step_x_physical_once(long direction_step) {
  move_stepper(kXStepPin, kXDirPin, direction_step, kInvertXDirection, kXStepDelayUs);
  if (direction_step == kXHomeDirectionStep) {
    current_x_steps += 1;
  } else {
    current_x_steps -= 1;
  }
}

void move_x_stepper(long steps) {
  if (steps == 0) {
    return;
  }

  const long direction_step = steps > 0 ? kXHomeDirectionStep : -kXHomeDirectionStep;
  const long count = labs(steps);

  for (long i = 0; i < count; i++) {
    if (direction_step == kXHomeDirectionStep && x_limit_active()) {
      move_stepper(
        kXStepPin,
        kXDirPin,
        -kXHomeDirectionStep * mm_to_steps(kHomeBackoffMm, kXStepsPerMm),
        kInvertXDirection,
        kXStepDelayUs
      );
      current_x_steps = mm_to_steps(kStartupXMaxMm, kXStepsPerMm);
      Serial.print("EVENT X LIMIT ");
      Serial.println(current_x_steps);
      return;
    }
    step_x_physical_once(direction_step);
  }
}

bool home_x_stepper() {
  const long x_backoff_steps = mm_to_steps(kHomeBackoffMm, kXStepsPerMm);

  if (x_limit_active()) {
    move_x_stepper(-x_backoff_steps);
  }

  for (long i = 0; i < kXHomeMaxSteps; i++) {
    if (x_limit_active()) {
      move_x_stepper(-x_backoff_steps);
      current_x_steps = mm_to_steps(kStartupXMaxMm, kXStepsPerMm);
      Serial.print("OK HOME X ");
      Serial.println(current_x_steps);
      return true;
    }
    step_x_physical_once(kXHomeDirectionStep);
  }

  Serial.println("ERR HOME X TIMEOUT");
  return false;
}

bool z_limit_active() {
  return digitalRead(kZLimitPin) == kZLimitActiveState;
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

void step_z_once(long direction_step) {
  move_stepper(kZStepPin, kZDirPin, direction_step, kInvertZDirection, kZStepDelayUs);
  current_z_steps += direction_step;
}

void move_z_stepper(long steps) {
  if (steps == 0) {
    return;
  }

  const long direction_step = steps > 0 ? 1 : -1;
  const long count = labs(steps);

  for (long i = 0; i < count; i++) {
    if (direction_step == kZHomeDirectionStep && z_limit_active()) {
      move_stepper(
        kZStepPin,
        kZDirPin,
        -kZHomeDirectionStep * mm_to_steps(kHomeBackoffMm, kZStepsPerMm),
        kInvertZDirection,
        kZStepDelayUs
      );
      current_z_steps = 0;
      Serial.print("EVENT Z LIMIT ");
      Serial.println(current_z_steps);
      return;
    }
    step_z_once(direction_step);
  }
}

bool home_z_stepper() {
  const long z_backoff_steps = mm_to_steps(kHomeBackoffMm, kZStepsPerMm);

  if (z_limit_active()) {
    move_z_stepper(-kZHomeDirectionStep * z_backoff_steps);
  }

  for (long i = 0; i < kZHomeMaxSteps; i++) {
    if (z_limit_active()) {
      move_z_stepper(-kZHomeDirectionStep * z_backoff_steps);
      current_z_steps = 0;
      Serial.print("OK HOME Z ");
      Serial.println(current_z_steps);
      return true;
    }
    step_z_once(kZHomeDirectionStep);
  }

  Serial.println("ERR HOME Z TIMEOUT");
  return false;
}

void startup_home_arm() {
  Serial.println("EVENT ARM STARTUP HOME BEGIN");

  if (!home_x_stepper()) {
    Serial.println("ERR ARM STARTUP HOME X");
    return;
  }

  if (!home_z_stepper()) {
    Serial.println("ERR ARM STARTUP HOME Z");
    return;
  }

  move_z_stepper(mm_to_steps(kStartupZClearanceMm, kZStepsPerMm));
  move_x_stepper(-mm_to_steps(kStartupXMaxMm, kXStepsPerMm));

  Serial.print("OK ARM STARTUP HOME X=");
  Serial.print(current_x_steps);
  Serial.print(" Z=");
  Serial.println(current_z_steps);
}

void maybe_stop_on_watchdog() {
  if (!drive_watchdog_armed) {
    return;
  }

  const unsigned long elapsed = millis() - last_drive_command_ms;
  if (elapsed <= kDriveTimeoutMs) {
    return;
  }

  stop_all();
  Serial.println("EVENT WATCHDOG STOP");
}

void handle_command(const char *command) {
  if (strcmp(command, "PING") == 0) {
    Serial.println("PONG");
    return;
  }

  if (strcmp(command, "ID") == 0) {
    Serial.println("MEGA_KEYBOARD_DRIVE");
    return;
  }

  if (strcmp(command, "STOP") == 0) {
    stop_all();
    Serial.println("OK STOP");
    return;
  }

  if (strcmp(command, "ENC1") == 0) {
    Serial.print("ENC1 ");
    Serial.println(read_encoder1_count());
    return;
  }

  if (strcmp(command, "RESET ENC1") == 0) {
    reset_encoder1_count();
    Serial.println("OK RESET ENC1");
    return;
  }

  if (strcmp(command, "ENC2") == 0) {
    Serial.print("ENC2 ");
    Serial.println(read_encoder2_count());
    return;
  }

  if (strcmp(command, "RESET ENC2") == 0) {
    reset_encoder2_count();
    Serial.println("OK RESET ENC2");
    return;
  }

  if (strcmp(command, "STATE") == 0) {
    Serial.print("STATE M1=");
    Serial.print(current_m1_speed);
    Serial.print(" M2=");
    Serial.print(current_m2_speed);
    Serial.print(" ENC1=");
    Serial.print(read_encoder1_count());
    Serial.print(" ENC2=");
    Serial.print(read_encoder2_count());
    Serial.print(" X=");
    Serial.print(current_x_steps);
    Serial.print(" Z=");
    Serial.print(current_z_steps);
    Serial.print(" L27=");
    Serial.print(digitalRead(kXLimitPin));
    Serial.print(" L44=");
    Serial.print(digitalRead(kZLimitPin));
    Serial.print(" P27=");
    Serial.print(x_limit_active() ? 1 : 0);
    Serial.print(" P44=");
    Serial.print(z_limit_active() ? 1 : 0);
    Serial.print(" SERVO=");
    Serial.println(current_gripper_us);
    return;
  }

  if (strcmp(command, "LIMITS") == 0) {
    Serial.print("LIMITS RAW27=");
    Serial.print(digitalRead(kXLimitPin));
    Serial.print(" RAW44=");
    Serial.print(digitalRead(kZLimitPin));
    Serial.print(" PRESSED27=");
    Serial.print(x_limit_active() ? 1 : 0);
    Serial.print(" PRESSED44=");
    Serial.println(z_limit_active() ? 1 : 0);
    return;
  }

  int speed = 0;
  if (sscanf(command, "M1 %d", &speed) == 1) {
    apply_drive(speed, current_m2_speed);
    return;
  }

  if (sscanf(command, "M2 %d", &speed) == 1) {
    apply_drive(current_m1_speed, speed);
    return;
  }

  int left = 0;
  int right = 0;
  if (sscanf(command, "BOTH %d %d", &left, &right) == 2) {
    apply_drive(left, right);
    return;
  }

  long arm_steps = 0;
  if (sscanf(command, "ARM X %ld", &arm_steps) == 1) {
    move_x_stepper(arm_steps);
    Serial.println("OK ARM X");
    return;
  }

  if (sscanf(command, "ARM Z %ld", &arm_steps) == 1) {
    move_z_stepper(arm_steps);
    Serial.println("OK ARM Z");
    return;
  }

  int servo_us = 0;
  if (sscanf(command, "SERVO %d", &servo_us) == 1) {
    set_gripper_servo_us(servo_us);
    Serial.print("OK SERVO ");
    Serial.println(current_gripper_us);
    return;
  }

  if (strcmp(command, "HOME Z") == 0) {
    home_z_stepper();
    return;
  }

  if (strcmp(command, "HOME X") == 0) {
    home_x_stepper();
    return;
  }

  if (strcmp(command, "HOME ARM") == 0) {
    startup_home_arm();
    return;
  }

  Serial.println("ERR UNKNOWN");
}

}  // namespace

void setup() {
  pinMode(kIna1Pin, OUTPUT);
  pinMode(kInb1Pin, OUTPUT);
  pinMode(kIna2Pin, OUTPUT);
  pinMode(kInb2Pin, OUTPUT);
  pinMode(kPwm1Pin, OUTPUT);
  pinMode(kPwm2Pin, OUTPUT);
  pinMode(kGripperServoPin, OUTPUT);
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
  digitalWrite(kXEnPin, LOW);
  digitalWrite(kZStepPin, LOW);
  digitalWrite(kZDirPin, LOW);
  digitalWrite(kZEnPin, LOW);
  gripper_servo.attach(kGripperServoPin);
  set_gripper_servo_us(kGripperOpenUs);

  stop_all();
  reset_encoder1_count();
  reset_encoder2_count();
  attachInterrupt(digitalPinToInterrupt(kHallA1Pin), on_encoder1_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kHallB1Pin), on_encoder1_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kHallA2Pin), on_encoder2_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kHallB2Pin), on_encoder2_change, CHANGE);

  Serial.begin(kBaudrate);
  while (!Serial && millis() < 3000) {
  }

  maybe_print_limit_switch_changes();
  reset_command_buffer();
  startup_home_arm();
  Serial.println("MEGA_KEYBOARD_READY");
}

void loop() {
  maybe_print_limit_switch_changes();
  maybe_stop_on_watchdog();

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
      continue;
    }

    if (command_length < kMaxCommandLength) {
      command_buffer[command_length++] = c;
    }
  }
}
