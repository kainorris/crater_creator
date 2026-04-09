#include "Gyro.h"
#include "base.h"
#include <Wire.h>

void Gyro::setup() {
  delay(500);

  delay(1000);

  int res = myIMU.begin();
  Serial.print("res: ");
  Serial.println(res);
  if (res)
    Serial.println("Ready.");
  else {
    Serial.println("Could not connect to IMU.");
    for (;;)
      setup();
    ;
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
  if (delta.x > 0 || delta.y > 0 || delta.z > 0) {
    // Serial.print("X: ");
    // Serial.println(delta.x);
    // Serial.print("Y: ");
    // Serial.println(delta.y);
    // Serial.print("Z: ");
    // Serial.println(delta.z);
    // Serial.println("<3");
  }
  prev = current;
  return delta;
  delay(100);
}
