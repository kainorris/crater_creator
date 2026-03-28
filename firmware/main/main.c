#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "hal/gpio_types.h"
#include "soc/gpio_num.h"
#include "vl53l1_api.h"
#include "vl53l1_api_core.h"
#include "vl53l1_def.h"
#include "vl53l1_ll_device.h"
#include "vl53l1_platform.h"
#include <stdint.h>
#include <stdlib.h>
#define TAG "main or something"

int app_main() {
  // drive 23 high
  gpio_num_t pin_23 = GPIO_NUM_23;
  gpio_set_level(pin_23, 1);
  ESP_LOGI(TAG, "mark 1\n");
  i2c_master_bus_config_t i2c_mst_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = 0,
      .scl_io_num = 22,
      .sda_io_num = 21,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  ESP_LOGI(TAG, "mark 2\n");
  i2c_master_bus_handle_t bus_handle;
  ESP_LOGI(TAG, "mark 3\n");
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
  ESP_LOGI(TAG, "mark 4\n");
  VL53L1_Dev_t *dev =
      malloc(sizeof(VL53L1_Dev_t)); // Heap allocated to prevent stack smashing

  dev->bus_handle = &bus_handle;
  dev->i2c_slave_address = 0x52;
  dev->comms_speed_khz = 400;
  // this might crash I guess
  ESP_LOGI(TAG, "mark 5\n");
  VL53L1_WaitDeviceBooted(dev);
  ESP_LOGI(TAG, "mark 6\n");
  int status;
  if ((status = VL53L1_CommsInitialise(dev, VL53L1_I2C, 400))) {
    ESP_LOGE(TAG, "Lidar Sensor VL53L1X [FAIL]: %d\n", status);
  } else {
    ESP_LOGI(TAG, "Lidar Sensor VL53L1X [OK]: %d\n", status);
  }
  ESP_LOGI(TAG, "mark 7\n");

  VL53L1_StopMeasurement(dev);
  ESP_LOGI(TAG, "mark 8\n");
  VL53L1_SetDistanceMode(dev, VL53L1_DISTANCEMODE_SHORT);
  ESP_LOGI(TAG, "mark 9\n");
  VL53L1_SetMeasurementTimingBudgetMicroSeconds(dev, 25000);

  while (1) {

    VL53L1_StartMeasurement(dev);
    uint8_t data_ready;
    VL53L1_GetMeasurementDataReady(dev, &data_ready);

    while (data_ready == 0) {
      VL53L1_GetMeasurementDataReady(dev, &data_ready);
      // yay spinlock
    }
    VL53L1_RangingMeasurementData_t data;

    status = VL53L1_GetRangingMeasurementData(dev, &data);
    int range = data.RangeStatus;

    VL53L1_StopMeasurement(dev);

    ESP_LOGI(TAG, "Distance %d mm", range);
  }
  return 0;
}
