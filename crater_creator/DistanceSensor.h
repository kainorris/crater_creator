#pragma once
#include "base.h"
#include <Adafruit_VL53L0X.h>
#include <Arduino.h>

#define XSHUT_1 12
#define XSHUT_2 13
#define XSHUT_3 14

class DistanceSensor {
public:
  void setup();
  Point3D loop();

private:
  Adafruit_VL53L0X sensor1, sensor2, sensor3;
};
