#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hc_sr04.h"
#include "portmacro.h"
#include <esp_adc/adc_oneshot.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#define X_TRIG_PIN 23
#define Y_TRIG_PIN 22
#define Z_TRIG_PIN 21
#define Y_ECHO_PIN 33
#define X_ECHO_PIN 32
#define Z_ECHO_PIN 35
#define DISTANCE_BETWEEN_SENSORS 10.0f // cm

static const char *TAG = "crater_fw";

typedef struct {
  float x;
  float y;
  float z;
} Point;

typedef struct {
  HCSR04 *sensor;
  SemaphoreHandle_t xSemaphore;
  float distance_cm;
  int64_t last_updated_us;
} SensorData;

typedef struct {
  SensorData *sensor_data_x;
  SensorData *sensor_data_y;
  SensorData *sensor_data_z;
} SensorTaskArgs;

float get_delta_p(float y0, float y1) {
  //|dP| = sqrt( (y1 - y0)^2 + (x1 - x0)^2 )
  return sqrtf(powf((y1 - y0), 2) + powf(DISTANCE_BETWEEN_SENSORS, 2));
}

float get_velocity(Point p_i, Point p_f, uint32_t t_i, uint32_t t_f) {
  return sqrtf((powf(p_f.x - p_i.x, 2.0) + powf(p_f.y - p_i.y, 2.0) +
                powf(p_f.z - p_i.z, 2.0)) /
               ((float)(t_f - t_i)));
}

Point get_velocity_vector(Point p_i, Point p_f, uint32_t t_i, uint32_t t_f) {
  return (Point){.x = (p_f.x - p_i.x) / (t_f - t_i),
                 .y = (p_f.y - p_i.y) / (t_f - t_i),
                 .z = (p_f.z - p_i.z) / (t_f - t_i)};
}

Point euler_method(Point prev, float step, Point deriv) {
  Point guard = prev;
  while (prev.z > 0.1 && prev.z < guard.z) {
    prev = (Point){
        .x = prev.x + deriv.x * step,
        .y = prev.y + deriv.y * step,
        .z = prev.z + deriv.z * step,
    };
  }
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
                                      // ESP_LOGI(TAG, "VEL: %f", vel);
          printf("V@E#F:%f\n", vel);
          sleep(5);
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

// Note that the sensor task should be monolithic such that it can run
// sequentaly
void vSensorTask(void *vParams) {

  ESP_LOGW(TAG, "x sensor: %d",
           hcsr04_init(((SensorTaskArgs *)vParams)->sensor_data_x->sensor));
  ESP_LOGW(TAG, "y sensor: %d",
           hcsr04_init(((SensorTaskArgs *)vParams)->sensor_data_y->sensor));
  ESP_LOGW(TAG, "z sensor: %d",
           hcsr04_init(((SensorTaskArgs *)vParams)->sensor_data_z->sensor));

  while (1) {
    float x =
        hcsr04_read_cm(((SensorTaskArgs *)vParams)->sensor_data_x->sensor);
    vTaskDelay(pdMS_TO_TICKS(200));
    float y =
        hcsr04_read_cm(((SensorTaskArgs *)vParams)->sensor_data_y->sensor);
    vTaskDelay(pdMS_TO_TICKS(200));
    float z =
        hcsr04_read_cm(((SensorTaskArgs *)vParams)->sensor_data_z->sensor);
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "(%f, %f, %f)", x, y, z);
    while (
        (((SensorTaskArgs *)vParams)->sensor_data_z->xSemaphore == NULL) ||
        (xSemaphoreTake(((SensorTaskArgs *)vParams)->sensor_data_z->xSemaphore,
                        (TickType_t)10) != pdTRUE)) {
      // Spinlock for the semaphore to be available
      ;
    }
    ((SensorTaskArgs *)vParams)->sensor_data_z->distance_cm = z;

    while (
        (((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore == NULL) ||
        (xSemaphoreTake(((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore,
                        (TickType_t)10) != pdTRUE)) {
      // Spinlock for the semaphore to be available
      ;
    }
    ((SensorTaskArgs *)vParams)->sensor_data_y->distance_cm = y;

    while (
        (((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore == NULL) ||
        (xSemaphoreTake(((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore,
                        (TickType_t)10) != pdTRUE)) {
      // Spinlock for the semaphore to be available
      ;
    }
    ((SensorTaskArgs *)vParams)->sensor_data_x->distance_cm = x;

    xSemaphoreGive(((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore);
    xSemaphoreGive(((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore);
    xSemaphoreGive(((SensorTaskArgs *)vParams)->sensor_data_z->xSemaphore);

    vTaskDelay(pdMS_TO_TICKS(1000 / 35)); // Read 35 times every second
  }
}

void vHeartbeatTask(void *vParams) {
  int num_cycles = 0;
  for (;;) {
    num_cycles++;
    if (num_cycles % 2 == 0) {
      printf("<3\n");
    } else {
      printf(".\n");
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void app_main(void) {
  xTaskCreate(vHeartbeatTask, "heartbeat", 2048, NULL, 5, NULL);
  static HCSR04 sensor_x = {.trig_pin = X_TRIG_PIN, .echo_pin = X_ECHO_PIN};
  static HCSR04 sensor_y = {.trig_pin = Y_TRIG_PIN, .echo_pin = Y_ECHO_PIN};
  static HCSR04 sensor_z = {.trig_pin = Z_TRIG_PIN, .echo_pin = Z_ECHO_PIN};

  static SensorData sensor_data_x = {.sensor = &sensor_x,
                                     .xSemaphore = NULL,
                                     .distance_cm = 0.0f,
                                     .last_updated_us = 0};

  static SensorData sensor_data_y = {.sensor = &sensor_y,
                                     .xSemaphore = NULL,
                                     .distance_cm = 0.0f,
                                     .last_updated_us = 0};

  static SensorData sensor_data_z = {.sensor = &sensor_z,
                                     .xSemaphore = NULL,
                                     .distance_cm = 0.0f,
                                     .last_updated_us = 0};

  static SensorTaskArgs sensor_task_args = {.sensor_data_z = &sensor_data_z,
                                            .sensor_data_y = &sensor_data_y,
                                            .sensor_data_x = &sensor_data_x};

  sensor_data_x.xSemaphore = xSemaphoreCreateMutex();
  sensor_data_y.xSemaphore = xSemaphoreCreateMutex();
  sensor_data_z.xSemaphore = xSemaphoreCreateMutex();

  xTaskCreate(vSensorTask, "SensorTask", 2048, (void *)&sensor_task_args, 5,
              NULL);

  static SensorData *sensor_data_array[3] = {&sensor_data_x, &sensor_data_y,
                                             &sensor_data_z};

  // xTaskCreate(vCalcTask, "CalcTask", 2048, (void *)&sensor_data_array, 5,
  // NULL);
  // TODO: Initialize I2C master (GPIO 21/22, 400 kHz)
  // TODO: Initialize VL53L5CX sensor (8x8, 15 Hz)
  // TODO: Run background calibration (CALIBRATION_FRAMES frames)
  // TODO: Send R@E#B:1\n ready signal
  // TODO: Main loop — IDLE / TRACKING / REPORT state machine
}
