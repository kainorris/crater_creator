
#include "hc_sr04.h"
#include "esp_dsp.h"
#include "esp_log.h"

// Anything over 400 cm (23200 us pulse) is "out of range"
#define HCSR04_MAX_DIST_US 23200

int hcsr04_init(HCSR04 *sensor) {
  assert(GPIO_IS_VALID_OUTPUT_GPIO(sensor->trig_pin));
  assert(GPIO_IS_VALID_GPIO(sensor->echo_pin));

  // Trigger output conf
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = (1ULL << sensor->trig_pin),
      .pull_down_en = 0,
      .pull_up_en = 0,
  };
  int cfg_result = gpio_config(&io_conf);

  // Echo input conf
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.intr_type = GPIO_INTR_DISABLE; // Polling mode, so disable interrupts
  io_conf.pin_bit_mask = (1ULL << sensor->echo_pin);
  cfg_result |= gpio_config(&io_conf);
  return cfg_result;
}

// Blocks and measures distance in centimeters. Returns -1.0 if out of range.
float hcsr04_read_cm(HCSR04 *sensor) {
  int32_t t1;
  int32_t t2;
  int32_t pulse_width;

  // Hold the trigger pin high for at least 10 us
  gpio_set_level(sensor->trig_pin, 1);
  esp_rom_delay_us(10);
  gpio_set_level(sensor->trig_pin, 0);

  // Wait for pulse on echo pin
  int64_t wait_start = esp_timer_get_time();
  while (gpio_get_level(sensor->echo_pin) == 0) {
    if (esp_timer_get_time() - wait_start > 10000) {
      // Timeout waiting for echo to go high
      return -1.0f;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  t1 = esp_timer_get_time();
  while (gpio_get_level(sensor->echo_pin) == 1) {
    if (esp_timer_get_time() - t1 > HCSR04_MAX_DIST_US) {
      // Timeout, exceeded max distance
      return -2.0f;
    }
  }
  t2 = esp_timer_get_time();

  pulse_width = t2 - t1;

  // Calculate distance in centimeters. The constants
  // are found in the datasheet, and calculated from the assumed speed
  // of sound in air at sea level (~340 m/s).
  float cm = pulse_width / 58.0f;

  return cm;
}
