#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hc_sr04.h"
#include "portmacro.h"
#include <esp_adc/adc_oneshot.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define DISTANCE_BETWEEN_SENSORS 10.0f // cm

static const char *TAG = "crater_fw";

typedef struct {
  HCSR04 *sensor;
  SemaphoreHandle_t xSemaphore;
  float distance_cm;
  int64_t last_updated_us;
} SensorData;

float get_delta_p(float y0, float y1) {
  //|dP| = sqrt( (y1 - y0)^2 + (x1 - x0)^2 )
  return sqrtf(powf((y1 - y0), 2) + powf(DISTANCE_BETWEEN_SENSORS, 2));
}

void vCalcTask(void *vParams) {

  enum CalcState {
    CALIBRATION,
    NO_PROJECTILE,
    SINGLE_SENSOR_LOCK,
  } calc_state = CALIBRATION;

  float first_sensor_dist = -1.0f;
  int first_sensor_id = -1;
  int64_t first_sensor_time = 0;

  typedef struct {
    float distance_cm;
    int64_t num_frames;
  } SensorCalibration;
  SensorCalibration calibration_data[2] = {0};

  float distances[2] = {0.0f, 0.0f};
  int64_t times[2] = {0, 0};

  // vParams (void*) -> sesnsor data array (SensorData*) -> indiviual data
  for (;;) {
    switch (calc_state) {
    case CALIBRATION:
      if (esp_timer_get_time() >
          1000000 * 2) // 2 second delay before calibrating
      {
        if (esp_timer_get_time() <
            1000000 * (3 + 2)) // 3 second calibration phase
        {
          // rolling average:
          //(old_average * (n-1) + new_value) / n
          for (size_t i = 0; i < 2; i++) {
            if (((SensorData **)vParams)[i]->xSemaphore != NULL &&
                xSemaphoreTake(((SensorData **)vParams)[i]->xSemaphore,
                               (TickType_t)10) == pdTRUE) {

              calibration_data[i].num_frames++;
              calibration_data[i].distance_cm =
                  (calibration_data[i].distance_cm *
                       (calibration_data[i].num_frames - 1) +
                   ((SensorData **)vParams)[i]->distance_cm) /
                  calibration_data[i].num_frames;

              xSemaphoreGive(((SensorData **)vParams)[i]->xSemaphore);
            }
          }
        } else {

          calc_state = NO_PROJECTILE;
        }
      }
      break;
    case NO_PROJECTILE:
      for (size_t i = 0; i < 2; i++) {
        if (((SensorData **)vParams)[i]->xSemaphore != NULL &&
            xSemaphoreTake(((SensorData **)vParams)[i]->xSemaphore,
                           (TickType_t)10) == pdTRUE) {
          float dist = ((SensorData **)vParams)[i]->distance_cm;
          xSemaphoreGive(((SensorData **)vParams)[i]->xSemaphore);
          if (dist < (calibration_data[i].distance_cm * 0.95)) {
            calc_state = SINGLE_SENSOR_LOCK;
            first_sensor_dist = dist;
            first_sensor_id = i;
            first_sensor_time = esp_timer_get_time();
            break;
          }
        }
      }
      break;
    case SINGLE_SENSOR_LOCK:
      // TODO: wait till target accuired and then record data. Then calc
      // velocity
      int i = first_sensor_id == 1 ? 0 : 1;
      if (((SensorData **)vParams)[i]->xSemaphore != NULL &&
          xSemaphoreTake(((SensorData **)vParams)[i]->xSemaphore,
                         (TickType_t)10) == pdTRUE) {
        float dist = ((SensorData **)vParams)[i]->distance_cm;
        xSemaphoreGive(((SensorData **)vParams)[i]->xSemaphore);
        if (dist < (calibration_data[i].distance_cm * 0.95)) {
          calc_state = NO_PROJECTILE;

          float delta_p = get_delta_p(first_sensor_dist, dist);
          float dt_s = (esp_timer_get_time() - first_sensor_time) / 1000000.0f;
          float vel = delta_p / dt_s; // cm/s
          ESP_LOGI(TAG, "VEL: %f", vel);
        }
      }
      break;
    }
    // OLD CODE:
    //  if (((SensorData **)vParams)[0]->xSemaphore != NULL &&
    //      xSemaphoreTake(((SensorData **)vParams)[0]->xSemaphore,
    //                     (TickType_t)10) == pdTRUE) {
    //    distances[0] = ((SensorData **)vParams)[0]->distance_cm;
    //    times[0] = ((SensorData **)vParams)[0]->last_updated_us;
    //    xSemaphoreGive(((SensorData **)vParams)[0]->xSemaphore);
    //  }
    //  if (((SensorData **)vParams)[1]->xSemaphore != NULL &&
    //      xSemaphoreTake(((SensorData **)vParams)[1]->xSemaphore,
    //                     (TickType_t)10) == pdTRUE) {
    //    distances[1] = ((SensorData **)vParams)[1]->distance_cm;
    //    times[1] = ((SensorData **)vParams)[1]->last_updated_us;
    //    xSemaphoreGive(((SensorData **)vParams)[1]->xSemaphore);
    //  }
    //  float delta_p = get_delta_p(distances[0], distances[1]);
    //  float velocity = (times[1] + times[0]) != 0
    //                       ? delta_p / ((times[1] - times[0]) / 1000000.0f)
    //                       : 0.0f; // cm/s
    //  ESP_LOGI(TAG, "Delta P: %.2f cm, Velocity: %.2f cm/s", delta_p,
    //  velocity); vTaskDelay(pdMS_TO_TICKS(100)); // Update every 100 ms
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void vSensorTask(void *vParams) {

  int init_result = hcsr04_init(((SensorData *)vParams)->sensor);
  if (init_result != 0) {
    ESP_LOGE(TAG, "Failed to initialize HC-SR04 sensor: %d", init_result);
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    float distance_cm = hcsr04_read_cm(((SensorData *)vParams)->sensor);
    if (distance_cm < 0) {
      ESP_LOGW(TAG, "HC-SR04 out of range");
    } else {
      ESP_LOGI(TAG, "HC-SR04 distance: %.2f cm", distance_cm);
      while ((((SensorData *)vParams)->xSemaphore == NULL) ||
             (xSemaphoreTake(((SensorData *)vParams)->xSemaphore,
                             (TickType_t)10) != pdTRUE)) {
        // Wait for the semaphore to be available
        ;
      }
      ((SensorData *)vParams)->distance_cm = distance_cm;
      ((SensorData *)vParams)->last_updated_us = esp_timer_get_time();
      xSemaphoreGive(
          ((SensorData *)vParams)->xSemaphore); // Release the semaphore

      vTaskDelay(pdMS_TO_TICKS(1000 / 35)); // Read 35 times every second
    }
  }
}

void app_main(void) {
  static HCSR04 sensor1 = {.trig_pin = 23, .echo_pin = 18};
  static HCSR04 sensor2 = {.trig_pin = 25, .echo_pin = 19};
  static SensorData sensor_data_1 = {.sensor = &sensor1,
                                     .xSemaphore = NULL,
                                     .distance_cm = 0.0f,
                                     .last_updated_us = 0};
  static SensorData sensor_data_2 = {.sensor = &sensor2,
                                     .xSemaphore = NULL,
                                     .distance_cm = 0.0f,
                                     .last_updated_us = 0};
  ESP_LOGI(TAG, "Crater Creator firmware starting...");
  sensor_data_1.xSemaphore = xSemaphoreCreateMutex();
  sensor_data_2.xSemaphore = xSemaphoreCreateMutex();
  xTaskCreate(vSensorTask, "SensorTask1", 2048, (void *)&sensor_data_1, 5,
              NULL);
  xTaskCreate(vSensorTask, "SensorTask2", 2048, (void *)&sensor_data_2, 5,
              NULL);
  static SensorData *sensor_data_array[2] = {&sensor_data_1, &sensor_data_2};
  xTaskCreate(vCalcTask, "CalcTask", 2048, (void *)&sensor_data_array, 5, NULL);
  // TODO: Initialize I2C master (GPIO 21/22, 400 kHz)
  // TODO: Initialize VL53L5CX sensor (8x8, 15 Hz)
  // TODO: Run background calibration (CALIBRATION_FRAMES frames)
  // TODO: Send R@E#B:1\n ready signal
  // TODO: Main loop — IDLE / TRACKING / REPORT state machine
}
