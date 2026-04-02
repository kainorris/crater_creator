#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
// #include "gyro.hpp"
#include "hc_sr04.hpp"
#include "portmacro.h"
#include "soc/gpio_num.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#define X_TRIG_PIN 19                  //
#define Y_TRIG_PIN 22                  //
#define Y_ECHO_PIN 23                  //
#define X_ECHO_PIN 21                  //
#define DISTANCE_BETWEEN_SENSORS 10.0f // cm
// #define ESP_LOGI(...)
gpio_num_t q = gpio_num_t(1);
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
  ESP_LOGI(TAG, "get_delta_p: y0=%.4f, y1=%.4f, sensor_dist=%.4f", y0, y1,
           DISTANCE_BETWEEN_SENSORS);
  float result = sqrtf(powf((y1 - y0), 2) + powf(DISTANCE_BETWEEN_SENSORS, 2));
  ESP_LOGI(TAG, "get_delta_p: result=%.4f", result);
  return result;
}

float get_velocity(Point p_i, Point p_f, uint32_t t_i, uint32_t t_f) {
  ESP_LOGI(TAG, "get_velocity: p_i=(%.4f, %.4f, %.4f) p_f=(%.4f, %.4f, %.4f)",
           p_i.x, p_i.y, p_i.z, p_f.x, p_f.y, p_f.z);
  ESP_LOGI(TAG, "get_velocity: t_i=%lu, t_f=%lu, dt=%lu", (unsigned long)t_i,
           (unsigned long)t_f, (unsigned long)(t_f - t_i));
  float result = sqrtf((powf(p_f.x - p_i.x, 2.0) + powf(p_f.y - p_i.y, 2.0) +
                        powf(p_f.z - p_i.z, 2.0)) /
                       ((float)(t_f - t_i)));
  ESP_LOGI(TAG, "get_velocity: result=%.4f", result);
  return result;
}

Point get_velocity_vector(Point p_i, Point p_f, uint32_t t_i, uint32_t t_f) {
  ESP_LOGI(TAG,
           "get_velocity_vector: p_i=(%.4f, %.4f, %.4f) p_f=(%.4f, %.4f, %.4f)",
           p_i.x, p_i.y, p_i.z, p_f.x, p_f.y, p_f.z);
  ESP_LOGI(TAG, "get_velocity_vector: t_i=%lu, t_f=%lu", (unsigned long)t_i,
           (unsigned long)t_f);
  Point result = (Point){.x = (p_f.x - p_i.x) / (t_f - t_i),
                         .y = (p_f.y - p_i.y) / (t_f - t_i),
                         .z = (p_f.z - p_i.z) / (t_f - t_i)};
  ESP_LOGI(TAG, "get_velocity_vector: result=(%.4f, %.4f, %.4f)", result.x,
           result.y, result.z);
  return result;
}

Point euler_method(Point prev, float step, Point deriv) {
  ESP_LOGI(TAG, "euler_method: initial prev=(%.4f, %.4f, %.4f), step=%.6f",
           prev.x, prev.y, prev.z, step);
  ESP_LOGI(TAG, "euler_method: deriv=(%.4f, %.4f, %.4f)", deriv.x, deriv.y,
           deriv.z);
  Point guard = prev;
  int iteration = 0;
  while (prev.y > 0.1 && prev.y < guard.y) {
    prev = (Point){
        .x = prev.x + deriv.x * step,
        .y = prev.y + deriv.y * step,
        .z = prev.z + deriv.z * step,
    };
    iteration++;
    if (iteration % 100 == 0) {
      ESP_LOGI(TAG, "euler_method: iteration %d, pos=(%.4f, %.4f, %.4f)",
               iteration, prev.x, prev.y, prev.z);
    }
  }
  ESP_LOGI(
      TAG,
      "euler_method: converged after %d iterations, result=(%.4f, %.4f, %.4f)",
      iteration, prev.x, prev.y, prev.z);
  return prev;
}

void vCalcTask(void *vParams) {
  ESP_LOGI(TAG, "vCalcTask: starting calc task");

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
  ESP_LOGI(TAG, "vCalcTask: initial time=%d", sleep_time0);

  // vParams (void*) -> sesnsor data array (SensorData*) -> indiviual data
  for (;;) {
    ESP_LOGI(TAG, "vCalcTask: loop iteration, state=%d", calc_state);
    float dist_x = 0.0;
    float dist_y = 0.0;
    float dist_z = 0.0;
    switch (calc_state) {
    case CALIBRATION:
      ESP_LOGI(TAG, "vCalcTask: CALIBRATION state, time=%lld",
               esp_timer_get_time());
      if (esp_timer_get_time() >
          1000000 * 2) // 2 second delay before calibrating
      {
        ESP_LOGI(TAG, "vCalcTask: past 2s delay, starting calibration");
        if (esp_timer_get_time() <
            1000000 * (3 + 2)) // 3 second calibration phase
        {
          // rolling average:
          //(old_average * (n-1) + new_value) / n
          if (((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore != NULL &&
              xSemaphoreTake(
                  ((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore,
                  (TickType_t)10) == pdTRUE) {
            envelope.y.num_frames++;
            envelope.y.distance_cm =
                (envelope.y.distance_cm * (envelope.y.num_frames - 1) +
                 ((SensorTaskArgs *)vParams)->sensor_data_y->distance_cm) /
                envelope.y.num_frames;
            ESP_LOGI(TAG, "vCalcTask: calibration Y: avg=%.4f, frames=%lld",
                     envelope.y.distance_cm, envelope.y.num_frames);
            xSemaphoreGive(
                ((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore);
          } else {
            ESP_LOGI(TAG, "vCalcTask: calibration Y: failed to take semaphore");
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
            ESP_LOGI(TAG, "vCalcTask: calibration X: avg=%.4f, frames=%lld",
                     envelope.x.distance_cm, envelope.x.num_frames);
            xSemaphoreGive(
                ((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore);
          } else {
            ESP_LOGI(TAG, "vCalcTask: calibration X: failed to take semaphore");
          }

        } else {
          ESP_LOGI(
              TAG,
              "vCalcTask: calibration complete! envelope X=%.4f Y=%.4f Z=%.4f",
              envelope.x.distance_cm, envelope.y.distance_cm,
              envelope.z.distance_cm);
          calc_state = NO_PROJECTILE;
        }
      } else {
        ESP_LOGI(TAG, "vCalcTask: waiting for 2s delay before calibration");
      }
      break;
    case NO_PROJECTILE:
      ESP_LOGI(TAG, "vCalcTask: NO_PROJECTILE state");

      if (((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore != NULL &&
          xSemaphoreTake(((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore,
                         (TickType_t)10) == pdTRUE) {
        float dist = ((SensorTaskArgs *)vParams)->sensor_data_y->distance_cm;
        ESP_LOGI(TAG, "vCalcTask: NO_PROJ Y dist=%.4f, threshold=%.4f", dist,
                 envelope.y.distance_cm * 0.95);
        xSemaphoreGive(((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore);
        if (dist < (envelope.y.distance_cm * 0.95)) {
          dist_y = dist;
          ESP_LOGI(TAG, "vCalcTask: NO_PROJ Y triggered! dist_y=%.4f", dist_y);
        }
      } else {
        ESP_LOGI(TAG, "vCalcTask: NO_PROJ Y: failed to take semaphore");
      }

      if (((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore != NULL &&
          xSemaphoreTake(((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore,
                         (TickType_t)10) == pdTRUE) {
        float dist = ((SensorTaskArgs *)vParams)->sensor_data_x->distance_cm;
        ESP_LOGI(TAG, "vCalcTask: NO_PROJ X dist=%.4f, threshold=%.4f", dist,
                 envelope.x.distance_cm * 0.95);
        xSemaphoreGive(((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore);
        if (dist < (envelope.x.distance_cm * 0.95)) {
          dist_x = dist;
          ESP_LOGI(TAG, "vCalcTask: NO_PROJ X triggered! dist_x=%.4f", dist_x);
        }
      } else {
        ESP_LOGI(TAG, "vCalcTask: NO_PROJ X: failed to take semaphore");
      }

      if (dist_x > 0.0 && dist_y > 0.0) {
        first_sensor_pos = (Point){.x = dist_x, .y = dist_y, .z = 0};
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
      if (((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore != NULL &&
          xSemaphoreTake(((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore,
                         (TickType_t)10) == pdTRUE) {
        current.y = ((SensorTaskArgs *)vParams)->sensor_data_y->distance_cm;
        ESP_LOGI(TAG, "vCalcTask: OIF read Y=%.4f", current.y);
        xSemaphoreGive(((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore);
      } else {
        ESP_LOGI(TAG, "vCalcTask: OIF Y: failed to take semaphore");
      }
      if (((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore != NULL &&
          xSemaphoreTake(((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore,
                         (TickType_t)10) == pdTRUE) {
        current.x = ((SensorTaskArgs *)vParams)->sensor_data_x->distance_cm;
        ESP_LOGI(TAG, "vCalcTask: OIF read X=%.4f", current.x);
        xSemaphoreGive(((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore);
      } else {
        ESP_LOGI(TAG, "vCalcTask: OIF X: failed to take semaphore");
      }

      ESP_LOGI(TAG, "vCalcTask: OIF current pos=(%.4f, %.4f, %.4f)", current.x,
               current.y, current.z);

      // If this case is met, there is no longer an object in frame
      if (current.x > envelope.x.distance_cm * 0.95 &&
          current.y > envelope.y.distance_cm * 0.95) {
        ESP_LOGI(TAG, "vCalcTask: object LEFT frame, computing velocity and "
                      "trajectory");
        float vel = get_velocity(first_sensor_pos, current, first_sensor_time,
                                 esp_timer_get_time());
        ESP_LOGI(TAG, "vCalcTask: computed velocity=%.4f", vel);
        printf("V@E#F:%f\n", vel);
        Point position = euler_method(
            current, 0.01,
            get_velocity_vector(first_sensor_pos, current, first_sensor_time,
                                esp_timer_get_time()));
        ESP_LOGI(TAG, "vCalcTask: euler result pos=(%.4f, %.4f)", position.x,
                 position.y);
        printf("X@E#F:%f\n", position.x);
        printf("Y@E#F:%f\n", position.y);
        vTaskDelay(pdMS_TO_TICKS(100));
        calc_state = NO_PROJECTILE;

      } else {
        ESP_LOGI(TAG,
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
    //  ESP_LOGI(TAG, "Delta P: %.2f cm, Velocity: %.2f cm/s", delta_p,
    //  velocity); vTaskDelay(pdMS_TO_TICKS(100)); // Update every 100 ms
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Note that the sensor task should be monolithic such that it can run
// sequentaly
void vSensorTask(void *vParams) {
  ESP_LOGI(TAG, "vSensorTask: starting sensor task");

  ESP_LOGI(TAG, "vSensorTask: initializing X sensor (trig=%d, echo=%d)",
           X_TRIG_PIN, X_ECHO_PIN);
  ESP_LOGW(TAG, "x sensor: %d",
           hcsr04_init(((SensorTaskArgs *)vParams)->sensor_data_x->sensor));
  ESP_LOGI(TAG, "vSensorTask: initializing Y sensor (trig=%d, echo=%d)",
           Y_TRIG_PIN, Y_ECHO_PIN);
  ESP_LOGW(TAG, "y sensor: %d",
           hcsr04_init(((SensorTaskArgs *)vParams)->sensor_data_y->sensor));
  ESP_LOGI(TAG, "vSensorTask: all sensors initialized, entering read loop");

  int read_cycle = 0;
  while (1) {
    read_cycle++;
    ESP_LOGI(TAG, "vSensorTask: === read cycle %d ===", read_cycle);

    ESP_LOGI(TAG, "vSensorTask: reading X sensor...");
    float x =
        hcsr04_read_cm(((SensorTaskArgs *)vParams)->sensor_data_x->sensor);
    ESP_LOGI(TAG, "vSensorTask: X raw=%.4f cm", x);
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "vSensorTask: reading Y sensor...");
    float y =
        hcsr04_read_cm(((SensorTaskArgs *)vParams)->sensor_data_y->sensor);
    ESP_LOGI(TAG, "vSensorTask: Y raw=%.4f cm", y);
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "vSensorTask: readings: (%f, %f)", x, y);

    ESP_LOGI(TAG, "vSensorTask: acquiring Y semaphore...");
    while (
        (((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore == NULL) ||
        (xSemaphoreTake(((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore,
                        (TickType_t)10) != pdTRUE)) {
      ESP_LOGI(TAG, "vSensorTask: waiting for Y semaphore...");
      ;
    }
    ((SensorTaskArgs *)vParams)->sensor_data_y->distance_cm = y;
    ESP_LOGI(TAG, "vSensorTask: Y semaphore acquired, wrote y=%.4f", y);

    ESP_LOGI(TAG, "vSensorTask: acquiring X semaphore...");
    while (
        (((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore == NULL) ||
        (xSemaphoreTake(((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore,
                        (TickType_t)10) != pdTRUE)) {
      ESP_LOGI(TAG, "vSensorTask: waiting for X semaphore...");
      ;
    }
    ((SensorTaskArgs *)vParams)->sensor_data_x->distance_cm = x;
    ESP_LOGI(TAG, "vSensorTask: X semaphore acquired, wrote x=%.4f", x);

    xSemaphoreGive(((SensorTaskArgs *)vParams)->sensor_data_x->xSemaphore);
    xSemaphoreGive(((SensorTaskArgs *)vParams)->sensor_data_y->xSemaphore);
    ESP_LOGI(TAG, "vSensorTask: all semaphores released");

    vTaskDelay(pdMS_TO_TICKS(1000 / 35)); // Read 35 times every second
    ESP_LOGI(TAG, "vSensorTask: cycle %d complete, sleeping ~%dms", read_cycle,
             1000 / 35);
  }
}

void vHeartbeatTask(void *vParams) {
  ESP_LOGI(TAG, "vHeartbeatTask: starting heartbeat task");
  int num_cycles = 0;
  for (;;) {
    num_cycles++;
    ESP_LOGI(TAG, "vHeartbeatTask: cycle %d", num_cycles);
    if (num_cycles % 2 == 0) {
      printf("<3\n");
    } else {
      printf(".\n");
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void run_gyro() {
  using Imu = espp::Lsm6dso<espp::lsm6dso::Interface::I2C>;

  // I2C config (customize as needed)
  static constexpr auto i2c_port = I2C_NUM_0;
  static constexpr auto i2c_clock_speed =
      CONFIG_EXAMPLE_I2C_CLOCK_SPEED_HZ; // Set in sdkconfig
  static constexpr gpio_num_t i2c_sda =
      (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO; // Set in sdkconfig
  static constexpr gpio_num_t i2c_scl =
      (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO; // Set in sdkconfig
  espp::I2c i2c({.port = i2c_port,
                 .sda_io_num = i2c_sda,
                 .scl_io_num = i2c_scl,
                 .sda_pullup_en = GPIO_PULLUP_ENABLE,
                 .scl_pullup_en = GPIO_PULLUP_ENABLE,
                 .clk_speed = i2c_clock_speed});

  // make the orientation filter to compute orientation from accel + gyro
  static constexpr float angle_noise = 0.001f;
  static constexpr float rate_noise = 0.1f;
  static espp::KalmanFilter<2> kf;
  kf.set_process_noise(rate_noise);
  kf.set_measurement_noise(angle_noise);

  auto kalman_filter_fn = [](float dt, const Imu::Value &accel,
                             const Imu::Value &gyro) -> Imu::Value {
    // Apply Kalman filter
    float accelRoll = atan2(accel.y, accel.z);
    float accelPitch =
        atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z));
    kf.predict({espp::deg_to_rad(gyro.x), espp::deg_to_rad(gyro.y)}, dt);
    kf.update({accelRoll, accelPitch});
    float roll, pitch;
    std::tie(roll, pitch) = kf.get_state();
    // return the computed orientation
    Imu::Value orientation{};
    orientation.roll = roll;
    orientation.pitch = pitch;
    orientation.yaw = 0.0f;
    return orientation;
  };

  // Madgwick filter for orientation
  static constexpr float beta = 0.1f;
  static espp::MadgwickFilter madgwick(beta);
  auto madgwick_filter_fn = [](float dt, const Imu::Value &accel,
                               const Imu::Value &gyro) -> Imu::Value {
    madgwick.update(dt, accel.x, accel.y, accel.z, espp::deg_to_rad(gyro.x),
                    espp::deg_to_rad(gyro.y), espp::deg_to_rad(gyro.z));
    float roll, pitch, yaw;
    madgwick.get_euler(roll, pitch, yaw);
    Imu::Value orientation{};
    orientation.roll = espp::deg_to_rad(roll);
    orientation.pitch = espp::deg_to_rad(pitch);
    orientation.yaw = espp::deg_to_rad(yaw);
    return orientation;
  };

  // IMU config
  Imu::Config config{
      .device_address = Imu::DEFAULT_I2C_ADDRESS,
      .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1,
                         std::placeholders::_2, std::placeholders::_3),
      .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3),
      .imu_config =
          {
              .accel_range = Imu::AccelRange::RANGE_2G,
              .accel_odr = Imu::AccelODR::ODR_416_HZ,
              .gyro_range = Imu::GyroRange::DPS_2000,
              .gyro_odr = Imu::GyroODR::ODR_416_HZ,
          },
      .orientation_filter = kalman_filter_fn,
      .auto_init = true,
      .log_level = espp::Logger::Verbosity::INFO,
  };

  logger.info("Creating LSM6DSO IMU");
  Imu imu(config);

  std::error_code ec;

  // set the accel / gyro on-chip filters
  static constexpr uint8_t accel_filter_bandwidth = 0b001; // ODR / 10
  static constexpr uint8_t gyro_lpf_bandwidth = 0b001;     // ODR / 3
  static constexpr bool gyro_hpf_enabled =
      false; // disable high-pass filter on gyro
  static constexpr auto gyro_hpf_bandwidth =
      Imu::GyroHPF::HPF_0_26_HZ; // 0.26Hz
  if (!imu.set_accelerometer_filter(accel_filter_bandwidth,
                                    Imu::AccelFilter::LOWPASS, ec)) {
    logger.error("Failed to set accelerometer filter: {}", ec.message());
  }
  // set the gyroscope filter to have lowpass
  if (!imu.set_gyroscope_filter(gyro_lpf_bandwidth, gyro_hpf_enabled,
                                gyro_hpf_bandwidth, ec)) {
    logger.error("Failed to set gyroscope filter: {}", ec.message());
  }

  // make a task to read out the IMU data and print it to console
  espp::Task imu_task(
      {.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
         static auto start = std::chrono::steady_clock::now();

         auto now = esp_timer_get_time(); // time in microseconds
         static auto t0 = now;
         auto t1 = now;
         float dt = (t1 - t0) / 1'000'000.0f; // convert us to s
         t0 = t1;

         std::error_code ec;
         // update the imu data
         if (!imu.update(dt, ec)) {
           return false;
         }

         // get accel
         auto accel = imu.get_accelerometer();
         auto gyro = imu.get_gyroscope();
         auto temp = imu.get_temperature();
         auto orientation = imu.get_orientation();
         auto gravity_vector = imu.get_gravity_vector();

         [[maybe_unused]] auto t2 =
             esp_timer_get_time(); // time in microseconds

         auto madgwick_orientation = madgwick_filter_fn(dt, accel, gyro);
         float roll = madgwick_orientation.roll;
         float pitch = madgwick_orientation.pitch;
         float yaw = madgwick_orientation.yaw;
         float vx = sin(pitch);
         float vy = -cos(pitch) * sin(roll);
         float vz = -cos(pitch) * cos(roll);

         // print time and raw IMU data
         std::string text = "";
         text += fmt::format("{:.3f},", now / 1'000'000.0f);
         text += fmt::format("{:02.3f},{:02.3f},{:02.3f},", (float)accel.x,
                             (float)accel.y, (float)accel.z);
         text += fmt::format("{:03.3f},{:03.3f},{:03.3f},", (float)gyro.x,
                             (float)gyro.y, (float)gyro.z);
         text += fmt::format("{:02.1f},", temp);
         // print kalman filter outputs
         text +=
             fmt::format("{:03.3f},{:03.3f},{:03.3f},", (float)orientation.x,
                         (float)orientation.y, (float)orientation.z);
         text +=
             fmt::format("{:03.3f},{:03.3f},{:03.3f},", (float)gravity_vector.x,
                         (float)gravity_vector.y, (float)gravity_vector.z);
         // print madgwick filter outputs
         text += fmt::format("{:03.3f},{:03.3f},{:03.3f},", roll, pitch, yaw);
         text += fmt::format("{:03.3f},{:03.3f},{:03.3f}", vx, vy, vz);

         fmt::print("{}\n", text);

         // fmt::print("IMU update took {:.3f} ms\n", (t2 - t0) / 1000.0f);

         // sleep first in case we don't get IMU data and need to exit early
         {
           std::unique_lock<std::mutex> lock(m);
           cv.wait_until(lock, start + 10ms);
         }

         return false;
       },
       .task_config = {
           .name = "IMU",
           .stack_size_bytes = 6 * 1024,
           .priority = 10,
           .core_id = 0,
       }});

  // print the header for the IMU data (for plotting)
  fmt::print("% Time (s), "
             // raw IMU data (accel, gyro, temp)
             "Accel X (m/s^2), Accel Y (m/s^2), Accel Z (m/s^2), "
             "Gyro X (rad/s), Gyro Y (rad/s), Gyro Z (rad/s), "
             "Temp (C), "
             // kalman filter outputs
             "Kalman Roll (rad), Kalman Pitch (rad), Kalman Yaw (rad), "
             "Kalman Gravity X, Kalman Gravity Y, Kalman Gravity Z, "
             // madgwick filter outputs
             "Madgwick Roll (rad), Madgwick Pitch (rad), Madgwick Yaw (rad), "
             "Madgwick Gravity X, Madgwick Gravity Y, Madgwick Gravity Z\n");

  logger.info("Starting IMU task");
  imu_task.start();

  // loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

// C++ is an ungodly language and main must be in C
extern "C" {
void app_main(void) {
  ESP_LOGI(TAG, "app_main: ====== CRATER CREATOR FIRMWARE STARTING ======");
  ESP_LOGI(TAG, "app_main: compiled %s %s", __DATE__, __TIME__);

  ESP_LOGI(TAG, "app_main: creating heartbeat task");
  xTaskCreate(vHeartbeatTask, "heartbeat", 2048, NULL, 5, NULL);

  ESP_LOGI(TAG, "app_main: configuring sensors");
  ESP_LOGI(TAG, "app_main: sensor X: trig=%d, echo=%d", X_TRIG_PIN, X_ECHO_PIN);
  ESP_LOGI(TAG, "app_main: sensor Y: trig=%d, echo=%d", Y_TRIG_PIN, Y_ECHO_PIN);
  static HCSR04 sensor_x = {.trig_pin = X_TRIG_PIN, .echo_pin = X_ECHO_PIN};
  static HCSR04 sensor_y = {.trig_pin = Y_TRIG_PIN, .echo_pin = Y_ECHO_PIN};

  ESP_LOGI(TAG, "app_main: initializing sensor data structs");
  static SensorData sensor_data_x = {.sensor = &sensor_x,
                                     .xSemaphore = NULL,
                                     .distance_cm = 0.0f,
                                     .last_updated_us = 0};

  static SensorData sensor_data_y = {.sensor = &sensor_y,
                                     .xSemaphore = NULL,
                                     .distance_cm = 0.0f,
                                     .last_updated_us = 0};

  static SensorTaskArgs sensor_task_args = {
      .sensor_data_x = &sensor_data_x,
      .sensor_data_y = &sensor_data_y,
      .sensor_data_z = NULL,
  };

  ESP_LOGI(TAG, "app_main: creating mutexes");
  sensor_data_x.xSemaphore = xSemaphoreCreateMutex();
  ESP_LOGI(TAG, "app_main: X mutex created: %p",
           (void *)sensor_data_x.xSemaphore);
  sensor_data_y.xSemaphore = xSemaphoreCreateMutex();
  ESP_LOGI(TAG, "app_main: Y mutex created: %p",
           (void *)sensor_data_y.xSemaphore);

  if (!sensor_data_x.xSemaphore || !sensor_data_y.xSemaphore) {
    ESP_LOGE(TAG, "app_main: FATAL - failed to create one or more mutexes!");
  }

  ESP_LOGI(TAG, "app_main: creating SensorTask (stack=2048, priority=5)");
  xTaskCreate(vSensorTask, "SensorTask", 2048, (void *)&sensor_task_args, 5,
              NULL);

  static SensorData *sensor_data_array[2] = {&sensor_data_x, &sensor_data_y};

  ESP_LOGI(TAG, "app_main: creating CalcTask (stack=2048, priority=5)");
  xTaskCreate(vCalcTask, "CalcTask", 2048 * 5, (void *)&sensor_task_args, 5,
              NULL);

  ESP_LOGI(TAG, "app_main: all tasks created, free heap=%lu",
           (unsigned long)esp_get_free_heap_size());
  ESP_LOGI(TAG, "app_main: sensor_data_array[0]=%p [1]=%p [2]=%p",
           (void *)sensor_data_array[0], (void *)sensor_data_array[1],
           (void *)sensor_data_array[2]);
  ESP_LOGI(TAG, "app_main: ====== INIT COMPLETE ======");
}
}
