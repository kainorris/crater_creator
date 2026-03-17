#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_adc/adc_oneshot.h>
#include <stdio.h>

static const char *TAG = "crater_fw";

void app_main(void) {
  ESP_LOGI(TAG, "Crater Creator firmware starting...");
  // TODO: Initialize I2C master (GPIO 21/22, 400 kHz)
  // TODO: Initialize VL53L5CX sensor (8x8, 15 Hz)
  // TODO: Run background calibration (CALIBRATION_FRAMES frames)
  // TODO: Send R@E#B:1\n ready signal
  // TODO: Main loop — IDLE / TRACKING / REPORT state machine
}
