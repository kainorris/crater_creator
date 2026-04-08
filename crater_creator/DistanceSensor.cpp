#include "DistanceSensor.h"
#include "base.h"
#include <Wire.h>

void DistanceSensor::setup() {

  // Hold all sensors in reset
  pinMode(XSHUT_1, OUTPUT);
  digitalWrite(XSHUT_1, LOW);
  pinMode(XSHUT_2, OUTPUT);
  digitalWrite(XSHUT_2, LOW);
  // pinMode(XSHUT_3, OUTPUT);
  // digitalWrite(XSHUT_3, LOW);
  // delay(10)

  // Boot sensor 1, assign new address
  digitalWrite(XSHUT_1, HIGH);
  delay(10);
  // if (!sensor1.setAddress(0x30)){
  //         Serial.println("Failed to boot VL53L0X"));
  // }
  //
  sensor1.begin(0x30);

  // Boot sensor 2, assign new address
  digitalWrite(XSHUT_2, HIGH);
  delay(10);
  if (!sensor2.begin(0x32)) {
        Serial.println("Failed to boot VL53L0X"));
  }
  // sensor1.startRange();
  // sensor2.startRange();
  // Boot sensor 3, assign new address
  // digitalWrite(XSHUT_3, HIGH);
  // delay(10);
  // sensor3.begin(0x34);
}

Point3D DistanceSensor::loop() {
  VL53L0X_RangingMeasurementData_t m1, m2 /*, m3*/;

  sensor1.rangingTest(&m1, false);
  sensor2.rangingTest(&m2, false);
  // sensor3.rangingTest(&m3, false);

  // Send over serial as CSV
  Serial.print(m1.RangeMilliMeter);
  Serial.print(",");
  Serial.println(m2.RangeMilliMeter);
  // Serial.print(",");
  // Serial.println(m3.RangeMilliMeter);
  Serial.println("<3");
  return Point3D{(float)m1.RangeMilliMeter, (float)m2.RangeMilliMeter, 0.0f};

  delay(50);
}
