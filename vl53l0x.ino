#include <Adafruit_VL53L0X.h>

#define XSHUT_1 12
#define XSHUT_2 13
#define XSHUT_3 14

Adafruit_VL53L0X sensor1, sensor2, sensor3;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Hold all sensors in reset
  pinMode(XSHUT_1, OUTPUT); digitalWrite(XSHUT_1, LOW);
  pinMode(XSHUT_2, OUTPUT); digitalWrite(XSHUT_2, LOW);
  pinMode(XSHUT_3, OUTPUT); digitalWrite(XSHUT_3, LOW);
  delay(10);

  // Boot sensor 1, assign new address
  digitalWrite(XSHUT_1, HIGH);
  delay(10);
  sensor1.begin(0x30);

  // Boot sensor 2, assign new address
  digitalWrite(XSHUT_2, HIGH);
  delay(10);
  sensor2.begin(0x31);

  // Boot sensor 3, assign new address
  digitalWrite(XSHUT_3, HIGH);
  delay(10);
  sensor3.begin(0x32);
}

void loop() {
  VL53L0X_RangingMeasurementData_t m1, m2, m3;

  sensor1.rangingTest(&m1, false);
  sensor2.rangingTest(&m2, false);
  sensor3.rangingTest(&m3, false);

  // Send over serial as CSV
  Serial.print(m1.RangeMilliMeter);
  Serial.print(",");
  Serial.print(m2.RangeMilliMeter);
  Serial.print(",");
  Serial.println(m3.RangeMilliMeter);

  delay(50);
}