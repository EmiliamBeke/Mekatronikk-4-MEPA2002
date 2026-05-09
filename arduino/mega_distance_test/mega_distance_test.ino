#include <Wire.h>
#include <vl53l4ed_class.h>

// -1 betyr at vi ikke bruker XSHUT-pin
VL53L4ED sensor(&Wire, -1);

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Starter VL53L4ED uten XSHUT...");

  Wire.begin();           // Arduino Mega: SDA = 20, SCL = 21
  Wire.setClock(100000);  // Stabil I2C-hastighet

  int status = sensor.begin();
  Serial.print("begin status: ");
  Serial.println(status);

  if (status != 0) {
    Serial.println("sensor.begin() feilet");
    while (1);
  }

  status = sensor.InitSensor();
  Serial.print("InitSensor status: ");
  Serial.println(status);

  if (status != 0) {
    Serial.println("InitSensor() feilet");
    while (1);
  }

  status = sensor.VL53L4ED_StartRanging();
  Serial.print("StartRanging status: ");
  Serial.println(status);

  if (status != 0) {
    Serial.println("StartRanging() feilet");
    while (1);
  }

  Serial.println("Sensor klar!");
}

void loop() {
  uint8_t dataReady = 0;
  VL53L4ED_ResultsData_t result;

  int status = sensor.VL53L4ED_CheckForDataReady(&dataReady);

  if (status != 0) {
    Serial.print("CheckForDataReady feil: ");
    Serial.println(status);
    delay(500);
    return;
  }

  if (dataReady) {
    sensor.VL53L4ED_ClearInterrupt();
    sensor.VL53L4ED_GetResult(&result);

    Serial.print("Avstand: ");
    Serial.print(result.distance_mm);
    Serial.println(" mm");
  }

  delay(50);
}