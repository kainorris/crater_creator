#include "Gyro.h"
#include "base.h"
#include <Wire.h>

void Gyro::setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin();
  delay(10);

  if (myIMU.begin())
    Serial.println("Ready.");
  else {
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if (myIMU.initialize(BASIC_SETTINGS))
    Serial.println("Loaded Settings.");
}

Point3D Gyro::loop() {
  Point3D current = {
      .x = myIMU.readFloatAccelX(),
      .y = myIMU.readFloatAccelY(),
      .z = myIMU.readFloatAccelZ(),
  };
  Point3D delta = current - prev;

  return delta;
  delay(100);
}
