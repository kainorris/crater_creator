#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"
#include "hc_sr04.h"
#include "portmacro.h"
#include "vl53l5cx_api.h"
#include <esp_adc/adc_oneshot.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#define DISTANCE_BETWEEN_SENSORS 10.0f // cm

static const char *TAG = "crater_fw";

typedef struct {
  VL53L5CX_Configuration *sensor;
  SemaphoreHandle_t xSemaphore;
  VL53L5CX_ResultsData *result;
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
  int sleep_time0 = esp_timer_get_time();

  // vParams (void*) -> sesnsor data array (SensorData*) -> indiviual data
  for (;;) {
  /*switch (calc_state) {
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
        if (dist < (calibration_data[i].distance_cm * 0.4)) {
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
      if (dist < (calibration_data[i].distance_cm * 0.4)) {
        calc_state = NO_PROJECTILE;

        float delta_p = get_delta_p(first_sensor_dist, dist);
        float dt_s = (esp_timer_get_time() - first_sensor_time) / 1000000.0f;
        float vel = delta_p / dt_s; // cm/s
                                    // ESP_LOGI(TAG, "VEL: %f", vel);
        printf("V@E#F:%f\n", vel);
        sleep(2);
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
*/ }
}

void vHeartbeatTask(void *vParams) {
  int num_cycles = 0;
  for (;;) {
    num_cycles++;
    if (num_cycles % 5 == 0) {
      printf("<3\n");
    } else if (num_cycles % 10 == 0) {
      printf(".\n");
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void vSensorTask(void *vParams) {
  // Note: expects sensor already set up before initalizing task

  while (1) {
    uint8_t new_data_ready = 0;
    vl53l5cx_check_data_ready(((SensorData *)vParams)->sensor, &new_data_ready);
    if (new_data_ready) {
      while ((((SensorData *)vParams)->xSemaphore == NULL) ||
             (xSemaphoreTake(((SensorData *)vParams)->xSemaphore,
                             (TickType_t)10) != pdTRUE)) {
        // spinlock for the semaphore to be available
        ;
      }

      int status = vl53l5cx_get_ranging_data(((SensorData *)vParams)->sensor,
                                             ((SensorData *)vParams)->result);
      if (status) {
        ESP_LOGE(TAG, "COULD NOT GET RANGING DATA: %d", status);
      } else {
        ESP_LOGI(TAG, "result: %d",
                 ((SensorData *)vParams)->result->motion_indicator.motion[0]);
      }
      ((SensorData *)vParams)->last_updated_us = esp_timer_get_time();
      xSemaphoreGive(((SensorData *)vParams)->xSemaphore);
    }
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "here");
  i2c_master_bus_config_t i2c_mst_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = -1,
      .scl_io_num = 22,
      .sda_io_num = 21,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  ESP_LOGI(TAG, "here2");
  i2c_master_bus_handle_t bus_handle;
  ESP_LOGI(TAG, "here3");
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
  ESP_LOGI(TAG, "here4");
  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = 0x58,
      .scl_speed_hz = 100000,
  };

  ESP_LOGI(TAG, "here5");
  i2c_master_dev_handle_t dev_handle;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
  ESP_LOGI(TAG, "here6");
  static VL53L5CX_Configuration sensor_1 = {};
  ESP_LOGI(TAG, "here7");
  vl53l5cx_init(&sensor_1);
  ESP_LOGI(TAG, "here8");
  static VL53L5CX_ResultsData results_1;
  ESP_LOGI(TAG, "here9");
  static SensorData sensor_data_1 = {.sensor = &sensor_1,
                                     .xSemaphore = NULL,
                                     .result = &results_1,
                                     .last_updated_us = 0};
  // ESP_LOGI(TAG, "Crater Creator firmware starting...");
  sensor_data_1.xSemaphore = xSemaphoreCreateMutex();
  ESP_LOGI(TAG, "here10");
  xTaskCreate(vSensorTask, "SensorTask1", 2048, (void *)&sensor_data_1, 5,
              NULL);

  ESP_LOGI(TAG, "here11");
  xTaskCreate(vHeartbeatTask, "Heartbeat", 2048, NULL, 5, NULL);
  ESP_LOGI(TAG, "here12");
  // TODO: Initialize I2C master (GPIO 21/22, 400 kHz)
  // TODO: Initialize VL53L5CX sensor (8x8, 15 Hz)
  // TODO: Run background calibration (CALIBRATION_FRAMES frames)
  // TODO: Send R@E#B:1\n ready signal
  // TODO: Main loop — IDLE / TRACKING / REPORT state machine
}
