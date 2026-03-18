#ifndef HC_SR04_H

#define HC_SR04_H
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "hal/gpio_types.h"
#include <assert.h>
#include <driver/gpio.h>
#include <stdio.h>

typedef struct {
  int trig_pin;
  int echo_pin;
} HCSR04;

int hcsr04_init(HCSR04 *sensor);
float hcsr04_read_cm(HCSR04 *sensor);

#endif // !HC_SR04_H
