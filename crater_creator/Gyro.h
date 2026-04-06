#pragma once
#include "SparkFunLSM6DSO.h"
#include "base.h"
#include <Arduino.h>

class Gyro {
public:
  void setup();
  Point3D loop();

private:
  LSM6DSO myIMU;
  Point3D prev = {0.0f, 0.0f, 0.0f};
};
