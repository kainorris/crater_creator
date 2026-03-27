#include "driver/i2c_master.h"
#include "esp_log.h"
#include "vl53l1_api.h"
#include "vl53l1_api_core.h"
#include "vl53l1_ll_device.h"
#include "vl53l1_platform.h"
#include <stdint.h>

#define TAG "main or something"

int app_main() {

  i2c_master_bus_config_t i2c_mst_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = 0,
      .scl_io_num = 22,
      .sda_io_num = 21,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  i2c_master_bus_handle_t bus_handle;
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

  VL53L1_Dev_t dev = {
      .bus_handle = &bus_handle,
      .i2c_slave_address = 0x52,
      .comms_speed_khz = 400
      // this might crash I guess
  };
  VL53L1_WaitDeviceBooted(&dev);
  if (VL53L1_CommsInitialise(&dev, VL53L1_I2C, 400)) {
    ESP_LOGE(TAG, "Lidar Sensor VL53L1X [OK]");
  } else {
    ESP_LOGE(TAG, "Lidar Sensor VL53L1X [FAIL]");
    return 1;
  }

  VL53L1_StopMeasurement(&dev);
  VL53L1_SetDistanceMode(&dev, VL53L1_DISTANCEMODE_MEDIUM);
  VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev, 25000);

  // while (1) {

  VL53L1_StartMeasurement(&dev);

  //   while (dataReady == 0) {
  //     status = VL53L1_GetMeasurementDataReady(&dev, &dataReady);
  //     vTaskDelay(pdMS_TO_TICKS(1));
  //   }
  //
  //   status = VL53L1_GetRangingMeasurementData(&dev, &rangingData);
  //   range = rangingData.RangeMilliMeter;
  //
  //   VL53L1_StopMeasurement(&dev);
  //
  //   VL53L1_StartMeasurement(&dev);
  //
  //   ESP_LOGI(TAG, "Distance %d mm", range);
  //
  //   vTaskDelay(pdMS_TO_TICKS(1000));
  // }
  return 0;
}
