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
#define NONOP(...)

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
  NONOP(TAG, "get_delta_p: y0=%.4f, y1=%.4f, sensor_dist=%.4f", y0, y1,
        DISTANCE_BETWEEN_SENSORS);
  float result = sqrtf(powf((y1 - y0), 2) + powf(DISTANCE_BETWEEN_SENSORS, 2));
  NONOP(TAG, "get_delta_p: result=%.4f", result);
  return result;
}

float get_velocity(Point p_i, Point p_f, uint32_t t_i, uint32_t t_f) {
  NONOP(TAG, "get_velocity: p_i=(%.4f, %.4f, %.4f) p_f=(%.4f, %.4f, %.4f)",
        p_i.x, p_i.y, p_i.z, p_f.x, p_f.y, p_f.z);
  NONOP(TAG, "get_velocity: t_i=%lu, t_f=%lu, dt=%lu", (unsigned long)t_i,
        (unsigned long)t_f, (unsigned long)(t_f - t_i));
  float result = sqrtf((powf(p_f.x - p_i.x, 2.0) + powf(p_f.y - p_i.y, 2.0) +
                        powf(p_f.z - p_i.z, 2.0)) /
                       ((float)(t_f - t_i)));
  NONOP(TAG, "get_velocity: result=%.4f", result);
  return result;
}

Point get_velocity_vector(Point p_i, Point p_f, uint32_t t_i, uint32_t t_f) {
  NONOP(TAG,
        "get_velocity_vector: p_i=(%.4f, %.4f, %.4f) p_f=(%.4f, %.4f, %.4f)",
        p_i.x, p_i.y, p_i.z, p_f.x, p_f.y, p_f.z);
  NONOP(TAG, "get_velocity_vector: t_i=%lu, t_f=%lu", (unsigned long)t_i,
        (unsigned long)t_f);
  Point result = (Point){.x = (p_f.x - p_i.x) / (t_f - t_i),
                         .y = (p_f.y - p_i.y) / (t_f - t_i),
                         .z = (p_f.z - p_i.z) / (t_f - t_i)};
  NONOP(TAG, "get_velocity_vector: result=(%.4f, %.4f, %.4f)", result.x,
        result.y, result.z);
  return result;
}

Point euler_method(Point prev, float step, Point deriv) {
  NONOP(TAG, "euler_method: initial prev=(%.4f, %.4f, %.4f), step=%.6f", prev.x,
        prev.y, prev.z, step);
  NONOP(TAG, "euler_method: deriv=(%.4f, %.4f, %.4f)", deriv.x, deriv.y,
        deriv.z);
  Point guard = prev;
  int iteration = 0;
  while (prev.z > 0.1 && prev.z < guard.z) {
    prev = (Point){
        .x = prev.x + deriv.x * step,
        .y = prev.y + deriv.y * step,
        .z = prev.z + deriv.z * step,
    };
    iteration++;
    if (iteration % 100 == 0) {
      NONOP(TAG, "euler_method: iteration %d, pos=(%.4f, %.4f, %.4f)",
            iteration, prev.x, prev.y, prev.z);
    }
  }
  NONOP(
      TAG,
      "euler_method: converged after %d iterations, result=(%.4f, %.4f, %.4f)",
      iteration, prev.x, prev.y, prev.z);
  return prev;
}

void vCalcTask(void *vParams) {
  NONOP(TAG, "vCalcTask: starting calc task");

  enum CalcState {
    CALIBRATION,
    NO_PROJECTILE,
    OBJECT_IN_FRAME,
  } calc_state = CALIBRATION;

  int64_t first_sensor_time = 0;
  Point first_sensor_pos;

  typedef struct {
    float distance_cm;
    int64_t num_frames;
  } SensorCalibration;

  typedef struct {
    SensorCalibration x;
    SensorCalibration y;
    SensorCalibration z;
  } SensorCalibrationEnvelope;

  SensorCalibrationEnvelope envelope = {
      .x = {.distance_cm = 0, .num_frames = 0},
      .y = {.distance_cm = 0, .num_frames = 0},
      .z = {.distance_cm = 0, .num_frames = 0}};
  Point prev;

  int sleep_time0 = esp_timer_get_time();
  NONOP(TAG, "vCalcTask: initial time=%d", sleep_time0);

  // vParams (void*) -> sesnsor data array (SensorData*) -> indiviual data
  for (;;) {
    NONOP(TAG, "vCalcTask: loop iteration, state=%d", calc_state);
    switch (calc_state) {
    case CALIBRATION:
      NONOP(TAG, "vCalcTask: CALIBRATION state, time=%lld",
            esp_timer_get_time());
      if (esp_timer_get_time() >
          1000000 * 2) // 2 second delay before calibrating
      {
        NONOP(TAG, "vCalcTask: past 2s delay, starting calibration");
        if (esp_timer_get_time() <
            1000000 * (3 + 2)) // 3 second calibration phase
        {
          // rolling average:
          //(old_average * (n-1) + new_value) / n
          if (((SensorTaskArgs *)vParams)->sensor_data_z->xSemaphore != NULL &&
              xSemaphoreTake(
                  ((SensorTaskArgs *)vParams)->sensor_data_z->xSemaphore,
                  (TickType_t)10) == pdTRUE) {
            envelope.z.num_frames++;
            envelope.z.distance_cm =
                (envelope.z.distance_cm * (envelope.z.num_frames - 1) +
                 ((SensorTaskArgs *)vParams)->sensor_data_z->distance_cm) /
                envelope.z.num_frames;
            NONOP(TAG, "vCalcTask: calibration Z: avg=%.4f, frames=%lld",
                  envelope.z.distance_cm, envelope.z.num_frames);
            xSemaphoreGive(
                ((SensorTaskArgs *)vParams)->sensor_data_z->xSemaphore);
          } else {
            NONOP(TAG, "vCalcTask: calibration Z: failed to take semaphore");
          }
          if (((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore != NULL &&
              xSemaphoreTake(
                  ((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore,
                  (TickType_t)10) == pdTRUE) {
            envelope.y.num_frames++;
            envelope.y.distance_cm =
                (envelope.y.distance_cm * (envelope.y.num_frames - 1) +
                 ((SensorTaskArgs *)vParams)->sensor_data_y->distance_cm) /
                envelope.y.num_frames;
            NONOP(TAG, "vCalcTask: calibration Y: avg=%.4f, frames=%lld",
                  envelope.y.distance_cm, envelope.y.num_frames);
            xSemaphoreGive(
                ((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore);
          } else {
            NONOP(TAG, "vCalcTask: calibration Y: failed to take semaphore");
          }
          if (((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore != NULL &&
              xSemaphoreTake(
                  ((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore,
                  (TickType_t)10) == pdTRUE) {
            envelope.x.num_frames++;
            envelope.x.distance_cm =
                (envelope.x.distance_cm * (envelope.x.num_frames - 1) +
                 ((SensorTaskArgs *)vParams)->sensor_data_x->distance_cm) /
                envelope.x.num_frames;
            NONOP(TAG, "vCalcTask: calibration X: avg=%.4f, frames=%lld",
                  envelope.x.distance_cm, envelope.x.num_frames);
            xSemaphoreGive(
                ((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore);
          } else {
            NONOP(TAG, "vCalcTask: calibration X: failed to take semaphore");
          }

        } else {
          NONOP(
              TAG,
              "vCalcTask: calibration complete! envelope X=%.4f Y=%.4f Z=%.4f",
              envelope.x.distance_cm, envelope.y.distance_cm,
              envelope.z.distance_cm);
          calc_state = NO_PROJECTILE;
        }
      } else {
        NONOP(TAG, "vCalcTask: waiting for 2s delay before calibration");
      }
      break;
    case NO_PROJECTILE:
      NONOP(TAG, "vCalcTask: NO_PROJECTILE state");
      float dist_x = 0.0;
      float dist_y = 0.0;
      float dist_z = 0.0;
      if (((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore != NULL &&
          xSemaphoreTake(((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore,
                         (TickType_t)10) == pdTRUE) {
        float dist = ((SensorTaskArgs *)vParams)->sensor_data_y->distance_cm;
        NONOP(TAG, "vCalcTask: NO_PROJ Y dist=%.4f, threshold=%.4f", dist,
              envelope.y.distance_cm * 0.95);
        xSemaphoreGive(((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore);
        if (dist < (envelope.y.distance_cm * 0.95)) {
          dist_y = dist;
          NONOP(TAG, "vCalcTask: NO_PROJ Y triggered! dist_y=%.4f", dist_y);
        }
      } else {
        NONOP(TAG, "vCalcTask: NO_PROJ Y: failed to take semaphore");
      }

      if (((SensorTaskArgs *)vParams)->sensor_data_z->xSemaphore != NULL &&
          xSemaphoreTake(((SensorTaskArgs *)vParams)->sensor_data_z->xSemaphore,
                         (TickType_t)10) == pdTRUE) {
        float dist = ((SensorTaskArgs *)vParams)->sensor_data_z->distance_cm;
        NONOP(TAG, "vCalcTask: NO_PROJ Z dist=%.4f, threshold=%.4f", dist,
              envelope.z.distance_cm * 0.95);
        xSemaphoreGive(((SensorTaskArgs *)vParams)->sensor_data_z->xSemaphore);
        if (dist < (envelope.z.distance_cm * 0.95)) {
          dist_z = dist;
          NONOP(TAG, "vCalcTask: NO_PROJ Z triggered! dist_z=%.4f", dist_z);
        }
      } else {
        NONOP(TAG, "vCalcTask: NO_PROJ Z: failed to take semaphore");
      }

      if (((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore != NULL &&
          xSemaphoreTake(((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore,
                         (TickType_t)10) == pdTRUE) {
        float dist = ((SensorTaskArgs *)vParams)->sensor_data_x->distance_cm;
        NONOP(TAG, "vCalcTask: NO_PROJ X dist=%.4f, threshold=%.4f", dist,
              envelope.x.distance_cm * 0.95);
        xSemaphoreGive(((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore);
        if (dist < (envelope.x.distance_cm * 0.95)) {
          dist_x = dist;
          NONOP(TAG, "vCalcTask: NO_PROJ X triggered! dist_x=%.4f", dist_x);
        }
      } else {
        NONOP(TAG, "vCalcTask: NO_PROJ X: failed to take semaphore");
      }

      if (dist_x > 0.0 && dist_y > 0.0 && dist_z > 0.0) {
        first_sensor_pos = (Point){.x = dist_x, .y = dist_y, .z = dist_z};
        first_sensor_time = esp_timer_get_time();
        ESP_LOGI(
            TAG,
            "vCalcTask: PROJECTILE DETECTED! pos=(%.4f, %.4f, %.4f), time=%lld",
            dist_x, dist_y, dist_z, first_sensor_time);
        calc_state = OBJECT_IN_FRAME;
      } else {
        ESP_LOGI(TAG,
                 "vCalcTask: NO_PROJ: not all axes triggered (x=%.4f, y=%.4f, "
                 "z=%.4f)",
                 dist_x, dist_y, dist_z);
      }

      break;
    case OBJECT_IN_FRAME:
      ESP_LOGI(TAG, "vCalcTask: OBJECT_IN_FRAME state");
      Point current = (Point){0, 0, 0};
      if (((SensorTaskArgs *)vParams)->sensor_data_z->xSemaphore != NULL &&
          xSemaphoreTake(((SensorTaskArgs *)vParams)->sensor_data_z->xSemaphore,
                         (TickType_t)10) == pdTRUE) {
        current.z = ((SensorTaskArgs *)vParams)->sensor_data_z->distance_cm;
        NONOP(TAG, "vCalcTask: OIF read Z=%.4f", current.z);
        xSemaphoreGive(((SensorTaskArgs *)vParams)->sensor_data_z->xSemaphore);
      } else {
        NONOP(TAG, "vCalcTask: OIF Z: failed to take semaphore");
      }
      if (((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore != NULL &&
          xSemaphoreTake(((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore,
                         (TickType_t)10) == pdTRUE) {
        current.y = ((SensorTaskArgs *)vParams)->sensor_data_y->distance_cm;
        NONOP(TAG, "vCalcTask: OIF read Y=%.4f", current.y);
        xSemaphoreGive(((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore);
      } else {
        NONOP(TAG, "vCalcTask: OIF Y: failed to take semaphore");
      }
      if (((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore != NULL &&
          xSemaphoreTake(((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore,
                         (TickType_t)10) == pdTRUE) {
        current.x = ((SensorTaskArgs *)vParams)->sensor_data_x->distance_cm;
        NONOP(TAG, "vCalcTask: OIF read X=%.4f", current.x);
        xSemaphoreGive(((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore);
      } else {
        NONOP(TAG, "vCalcTask: OIF X: failed to take semaphore");
      }

      NONOP(TAG, "vCalcTask: OIF current pos=(%.4f, %.4f, %.4f)", current.x,
            current.y, current.z);

      // If this case is met, there is no longer an object in frame
      if (current.x > envelope.x.distance_cm * 0.95 &&
          current.y > envelope.y.distance_cm * 0.95 &&
          current.z > envelope.z.distance_cm * 0.95) {
        NONOP(TAG, "vCalcTask: object LEFT frame, computing velocity and "
                   "trajectory");
        float vel = get_velocity(first_sensor_pos, current, first_sensor_time,
                                 esp_timer_get_time());
        NONOP(TAG, "vCalcTask: computed velocity=%.4f", vel);
        printf("V@E#F:%f\n", vel);
        Point position = euler_method(
            current, 0.01,
            get_velocity_vector(first_sensor_pos, current, first_sensor_time,
                                esp_timer_get_time()));
        NONOP(TAG, "vCalcTask: euler result pos=(%.4f, %.4f)", position.x,
              position.y);
        printf("X@E#F:%f\n", position.x);
        printf("Y@E#F:%f\n", position.y);
        vTaskDelay(pdMS_TO_TICKS(100));
        calc_state = NO_PROJECTILE;

      } else {
        NONOP(TAG,
              "vCalcTask: object still in frame, updating prev=(%.4f, %.4f, "
              "%.4f)",
              current.x, current.y, current.z);
        prev = current;
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
    //  NONOP(TAG, "Delta P: %.2f cm, Velocity: %.2f cm/s", delta_p,
    //  velocity); vTaskDelay(pdMS_TO_TICKS(100)); // Update every 100 ms
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Note that the sensor task should be monolithic such that it can run
// sequentaly
void vSensorTask(void *vParams) {
  NONOP(TAG, "vSensorTask: starting sensor task");

  NONOP(TAG, "vSensorTask: initializing X sensor (trig=%d, echo=%d)",
        X_TRIG_PIN, X_ECHO_PIN);
  ESP_LOGW(TAG, "x sensor: %d",
           hcsr04_init(((SensorTaskArgs *)vParams)->sensor_data_x->sensor));
  NONOP(TAG, "vSensorTask: initializing Y sensor (trig=%d, echo=%d)",
        Y_TRIG_PIN, Y_ECHO_PIN);
  ESP_LOGW(TAG, "y sensor: %d",
           hcsr04_init(((SensorTaskArgs *)vParams)->sensor_data_y->sensor));
  NONOP(TAG, "vSensorTask: initializing Z sensor (trig=%d, echo=%d)",
        Z_TRIG_PIN, Z_ECHO_PIN);
  ESP_LOGW(TAG, "z sensor: %d",
           hcsr04_init(((SensorTaskArgs *)vParams)->sensor_data_z->sensor));
  NONOP(TAG, "vSensorTask: all sensors initialized, entering read loop");

  int read_cycle = 0;
  while (1) {
    read_cycle++;
    NONOP(TAG, "vSensorTask: === read cycle %d ===", read_cycle);

    NONOP(TAG, "vSensorTask: reading X sensor...");
    float x =
        hcsr04_read_cm(((SensorTaskArgs *)vParams)->sensor_data_x->sensor);
    NONOP(TAG, "vSensorTask: X raw=%.4f cm", x);
    vTaskDelay(pdMS_TO_TICKS(20));

    NONOP(TAG, "vSensorTask: reading Y sensor...");
    float y =
        hcsr04_read_cm(((SensorTaskArgs *)vParams)->sensor_data_y->sensor);
    NONOP(TAG, "vSensorTask: Y raw=%.4f cm", y);
    vTaskDelay(pdMS_TO_TICKS(20));

    NONOP(TAG, "vSensorTask: reading Z sensor...");
    float z =
        hcsr04_read_cm(((SensorTaskArgs *)vParams)->sensor_data_z->sensor);
    NONOP(TAG, "vSensorTask: Z raw=%.4f cm", z);
    vTaskDelay(pdMS_TO_TICKS(20));

    NONOP(TAG, "vSensorTask: readings: (%f, %f, %f)", x, y, z);

    NONOP(TAG, "vSensorTask: acquiring Z semaphore...");
    while (
        (((SensorTaskArgs *)vParams)->sensor_data_z->xSemaphore == NULL) ||
        (xSemaphoreTake(((SensorTaskArgs *)vParams)->sensor_data_z->xSemaphore,
                        (TickType_t)10) != pdTRUE)) {
      NONOP(TAG, "vSensorTask: waiting for Z semaphore...");
      ;
    }
    ((SensorTaskArgs *)vParams)->sensor_data_z->distance_cm = z;
    NONOP(TAG, "vSensorTask: Z semaphore acquired, wrote z=%.4f", z);

    NONOP(TAG, "vSensorTask: acquiring Y semaphore...");
    while (
        (((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore == NULL) ||
        (xSemaphoreTake(((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore,
                        (TickType_t)10) != pdTRUE)) {
      NONOP(TAG, "vSensorTask: waiting for Y semaphore...");
      ;
    }
    ((SensorTaskArgs *)vParams)->sensor_data_y->distance_cm = y;
    NONOP(TAG, "vSensorTask: Y semaphore acquired, wrote y=%.4f", y);

    NONOP(TAG, "vSensorTask: acquiring X semaphore...");
    while (
        (((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore == NULL) ||
        (xSemaphoreTake(((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore,
                        (TickType_t)10) != pdTRUE)) {
      NONOP(TAG, "vSensorTask: waiting for X semaphore...");
      ;
    }
    ((SensorTaskArgs *)vParams)->sensor_data_x->distance_cm = x;
    NONOP(TAG, "vSensorTask: X semaphore acquired, wrote x=%.4f", x);

    xSemaphoreGive(((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore);
    xSemaphoreGive(((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore);
    xSemaphoreGive(((SensorTaskArgs *)vParams)->sensor_data_z->xSemaphore);
    NONOP(TAG, "vSensorTask: all semaphores released");

    vTaskDelay(pdMS_TO_TICKS(1000 / 35)); // Read 35 times every second
    NONOP(TAG, "vSensorTask: cycle %d complete, sleeping ~%dms", read_cycle,
          1000 / 35);
  }
}

void vHeartbeatTask(void *vParams) {
  NONOP(TAG, "vHeartbeatTask: starting heartbeat task");
  int num_cycles = 0;
  for (;;) {
    num_cycles++;
    NONOP(TAG, "vHeartbeatTask: cycle %d", num_cycles);
    if (num_cycles % 2 == 0) {
      printf("<3\n");
    } else {
      printf(".\n");
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void app_main(void) {
  NONOP(TAG, "app_main: ====== CRATER CREATOR FIRMWARE STARTING ======");
  NONOP(TAG, "app_main: compiled %s %s", __DATE__, __TIME__);

  NONOP(TAG, "app_main: creating heartbeat task");
  xTaskCreate(vHeartbeatTask, "heartbeat", 2048, NULL, 5, NULL);

  NONOP(TAG, "app_main: configuring sensors");
  NONOP(TAG, "app_main: sensor X: trig=%d, echo=%d", X_TRIG_PIN, X_ECHO_PIN);
  NONOP(TAG, "app_main: sensor Y: trig=%d, echo=%d", Y_TRIG_PIN, Y_ECHO_PIN);
  NONOP(TAG, "app_main: sensor Z: trig=%d, echo=%d", Z_TRIG_PIN, Z_ECHO_PIN);
  static HCSR04 sensor_x = {.trig_pin = X_TRIG_PIN, .echo_pin = X_ECHO_PIN};
  static HCSR04 sensor_y = {.trig_pin = Y_TRIG_PIN, .echo_pin = Y_ECHO_PIN};
  static HCSR04 sensor_z = {.trig_pin = Z_TRIG_PIN, .echo_pin = Z_ECHO_PIN};

  NONOP(TAG, "app_main: initializing sensor data structs");
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

  NONOP(TAG, "app_main: creating mutexes");
  sensor_data_x.xSemaphore = xSemaphoreCreateMutex();
  NONOP(TAG, "app_main: X mutex created: %p", (void *)sensor_data_x.xSemaphore);
  sensor_data_y.xSemaphore = xSemaphoreCreateMutex();
  NONOP(TAG, "app_main: Y mutex created: %p", (void *)sensor_data_y.xSemaphore);
  sensor_data_z.xSemaphore = xSemaphoreCreateMutex();
  NONOP(TAG, "app_main: Z mutex created: %p", (void *)sensor_data_z.xSemaphore);

  if (!sensor_data_x.xSemaphore || !sensor_data_y.xSemaphore ||
      !sensor_data_z.xSemaphore) {
    ESP_LOGE(TAG, "app_main: FATAL - failed to create one or more mutexes!");
  }

  NONOP(TAG, "app_main: creating SensorTask (stack=2048, priority=5)");
  xTaskCreate(vSensorTask, "SensorTask", 2048, (void *)&sensor_task_args, 5,
              NULL);

  static SensorData *sensor_data_array[3] = {&sensor_data_x, &sensor_data_y,
                                             &sensor_data_z};

  NONOP(TAG, "app_main: creating CalcTask (stack=2048, priority=5)");
  xTaskCreate(vCalcTask, "CalcTask", 2048 * 5, (void *)&sensor_task_args, 5,
              NULL);

  NONOP(TAG, "app_main: all tasks created, free heap=%lu",
        (unsigned long)esp_get_free_heap_size());
  NONOP(TAG, "app_main: sensor_data_array[0]=%p [1]=%p [2]=%p",
        (void *)sensor_data_array[0], (void *)sensor_data_array[1],
        (void *)sensor_data_array[2]);
  NONOP(TAG, "app_main: ====== INIT COMPLETE ======");
  // TODO: Initialize I2C master (GPIO 21/22, 400 kHz)
  // TODO: Initialize VL53L5CX sensor (8x8, 15 Hz)
  // TODO: Run background calibration (CALIBRATION_FRAMES frames)
  // TODO: Send R@E#B:1\n ready signal
  // TODO: Main loop — IDLE / TRACKING / REPORT state machine
}
