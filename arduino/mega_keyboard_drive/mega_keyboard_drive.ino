#include <stdlib.h>
#include <string.h>

namespace {

constexpr long kBaudrate = 115200;
constexpr size_t kMaxCommandLength = 63;
constexpr unsigned long kDriveTimeoutMs = 700;

constexpr int kHallA1Pin = 3;
constexpr int kHallB1Pin = 2;
constexpr int kIna1Pin = 4;
constexpr int kInb1Pin = 5;
constexpr int kPwm1Pin = 6;

constexpr int kIna2Pin = 12;
constexpr int kInb2Pin = 11;
constexpr int kPwm2Pin = 10;
constexpr int kHallA2Pin = 18;
constexpr int kHallB2Pin = 19;

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
    Serial.println(read_encoder2_count());
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
  pinMode(kHallA1Pin, INPUT_PULLUP);
  pinMode(kHallB1Pin, INPUT_PULLUP);
  pinMode(kHallA2Pin, INPUT_PULLUP);
  pinMode(kHallB2Pin, INPUT_PULLUP);

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

  reset_command_buffer();
  Serial.println("MEGA_KEYBOARD_READY");
}

void loop() {
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
