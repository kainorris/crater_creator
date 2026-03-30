#include "esp_dsp.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "hal/gpio_types.h"
#include "vl53l0x.h"
#include <stdio.h>

#define I2C_PORT 0
#define I2C_SDA 21
#define I2C_SCL 22
#define XSHUT_PIN 23

void app_main(void) {
  // Initialize the sensor

  vl53l0x_t *sensor =
      vl53l0x_config(I2C_PORT, I2C_SCL, I2C_SDA, XSHUT_PIN, 0x29, 1);
  if (!sensor) {
    ESP_LOGE("VL53L0X", "Failed to configure sensor");
    return;
  }

  const char *err = vl53l0x_init(sensor);
  if (err) {
    ESP_LOGE("VL53L0X", "Init failed: %s", err);
    vl53l0x_end(sensor);
    return;
  }

  // Start continuous ranging
  vl53l0x_startContinuous(sensor, 0);

  while (1) {
    uint16_t range = vl53l0x_readRangeContinuousMillimeters(sensor);
    if (range == 65535) {
      ESP_LOGW("VL53L0X", "Out of range or timeout");
    } else {
      ESP_LOGI("VL53L0X", "Range: %d mm", range);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
