constexpr int kLimitSwitchPin = 44;
constexpr long kBaudrate = 115200;

void setup() {
  pinMode(kLimitSwitchPin, INPUT_PULLUP);
  Serial.begin(kBaudrate);
  while (!Serial && millis() < 3000) {
  }
}

void loop() {
  const int raw = digitalRead(kLimitSwitchPin);
  const bool pressed = raw == LOW;

  Serial.print("RAW ");
  Serial.print(raw == HIGH ? 1 : 0);
  Serial.print(" PRESSED ");
  Serial.println(pressed ? 1 : 0);
  delay(100);
}
