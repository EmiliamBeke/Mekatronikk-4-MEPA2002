#include <stdlib.h>
#include <string.h>

namespace {

constexpr long kBaudrate = 115200;
constexpr size_t kMaxCommandLength = 63;

constexpr int kIna1Pin = 22;
constexpr int kInb1Pin = 23;
constexpr int kIna2Pin = 24;
constexpr int kInb2Pin = 25;
constexpr int kPwm1Pin = 5;
constexpr int kPwm2Pin = 4;
constexpr int kHallA1Pin = 3;
constexpr int kHallB1Pin = 2;

volatile long encoder1_count = 0;
volatile uint8_t encoder1_state = 0;

char command_buffer[kMaxCommandLength + 1];
size_t command_length = 0;

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

uint8_t read_encoder1_state() {
  const uint8_t a = static_cast<uint8_t>(digitalRead(kHallA1Pin));
  const uint8_t b = static_cast<uint8_t>(digitalRead(kHallB1Pin));
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

long read_encoder1_count() {
  noInterrupts();
  const long count = encoder1_count;
  interrupts();
  return count;
}

void reset_encoder1_count() {
  noInterrupts();
  encoder1_count = 0;
  encoder1_state = read_encoder1_state();
  interrupts();
}

void set_motor(int ina_pin, int inb_pin, int pwm_pin, int speed) {
  int pwm = abs(speed);
  if (pwm > 255) {
    pwm = 255;
  }

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

void stop_all() {
  set_motor(kIna1Pin, kInb1Pin, kPwm1Pin, 0);
  set_motor(kIna2Pin, kInb2Pin, kPwm2Pin, 0);
}

void handle_command(const char *command) {
  if (strcmp(command, "PING") == 0) {
    Serial.println("PONG");
    return;
  }

  if (strcmp(command, "ID") == 0) {
    Serial.println("MEGA_DFR0601_TEST");
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

  if (strcmp(command, "STATE") == 0) {
    Serial.print("STATE ENC1=");
    Serial.print(read_encoder1_count());
    Serial.print(" HALLA=");
    Serial.print(digitalRead(kHallA1Pin));
    Serial.print(" HALLB=");
    Serial.println(digitalRead(kHallB1Pin));
    return;
  }

  int speed = 0;
  if (sscanf(command, "M1 %d", &speed) == 1) {
    set_motor(kIna1Pin, kInb1Pin, kPwm1Pin, speed);
    Serial.print("OK M1 ");
    Serial.println(speed);
    return;
  }

  if (sscanf(command, "M2 %d", &speed) == 1) {
    set_motor(kIna2Pin, kInb2Pin, kPwm2Pin, speed);
    Serial.print("OK M2 ");
    Serial.println(speed);
    return;
  }

  int left = 0;
  int right = 0;
  if (sscanf(command, "BOTH %d %d", &left, &right) == 2) {
    set_motor(kIna1Pin, kInb1Pin, kPwm1Pin, left);
    set_motor(kIna2Pin, kInb2Pin, kPwm2Pin, right);
    Serial.print("OK BOTH ");
    Serial.print(left);
    Serial.print(' ');
    Serial.println(right);
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

  stop_all();
  reset_encoder1_count();
  attachInterrupt(digitalPinToInterrupt(kHallA1Pin), on_encoder1_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kHallB1Pin), on_encoder1_change, CHANGE);

  Serial.begin(kBaudrate);
  while (!Serial && millis() < 3000) {
  }

  reset_command_buffer();
  Serial.println("MEGA_DFR0601_READY");
}

void loop() {
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
