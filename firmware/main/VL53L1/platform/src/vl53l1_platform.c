
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

#include <driver/i2c_master.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2c_types.h"
#include "esp_log_level.h"
#include "vl53l1_platform.h"
#include "vl53l1_platform_log.h"

#define NYI(...)                                                               \
  printf("FATAL: User called nonexistent function. Bailing out.");             \
  exit(EXIT_FAILURE);

#define trace_print(level, ...)                                                \
  _LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_PLATFORM, level,                        \
                   VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

#define trace_i2c(...)                                                         \
  _LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_NONE, VL53L1_TRACE_LEVEL_NONE,          \
                   VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)

VL53L1_Error VL53L1_CommsInitialise(VL53L1_Dev_t *pdev, uint8_t comms_type,
                                    uint16_t comms_speed_khz) {
  VL53L1_Error status = 255;

  i2c_device_config_t dev_cfg = {
      .dev_addr_length = 7,
      .device_address = pdev->i2c_slave_address,
      .scl_speed_hz = pdev->comms_speed_khz,
  };
  pdev->comms_type = comms_type;

  i2c_master_dev_handle_t dev_handle;
  ESP_ERROR_CHECK(status = i2c_master_bus_add_device(*pdev->bus_handle,
                                                     &dev_cfg, &dev_handle));
  pdev->dev_handle = dev_handle;
  return status;
}

VL53L1_Error VL53L1_CommsClose(VL53L1_Dev_t *pdev) {
  printf("Closing comms is for cowards. We keep going till death or loss of "
         "power.");
  return 0;
}

VL53L1_Error VL53L1_WriteMulti(VL53L1_Dev_t *pdev, uint16_t index,
                               uint8_t *pdata, uint32_t count) {
  // I think index means buffer
  VL53L1_Error status = 255;
  /* To be filled by customer according to the platform request. Return 0 if OK
   */
  // if (count != (sizeof(pdata) / sizeof(pdata[0]))) {
  //   printf("WARN: length of pdata not count");
  // }
  i2c_master_transmit(i2c_master_dev_handle_t i2c_dev,
                      const uint8_t *write_buffer, size_t write_size,
                      int xfer_timeout_ms)
      i2c_master_transmit_multi_buffer_info_t i2c_tx_buff_arr[count];
  for (int i = 0; i < count; i++) {
    i2c_tx_buff_arr[i] = (i2c_master_transmit_multi_buffer_info_t){
        .buffer_size = sizeof(uint8_t), // idek vibes
        .write_buffer = &pdata[i]};
  }

  return i2c_master_multi_buffer_transmit(
      pdev->dev_handle, i2c_tx_buff_arr,
      sizeof(i2c_tx_buff_arr) / sizeof(i2c_master_transmit_multi_buffer_info_t),
      -1);
}

VL53L1_Error VL53L1_ReadMulti(VL53L1_Dev_t *pdev, uint16_t index,
                              uint8_t *pdata, uint32_t count) {
  VL53L1_Error status = 255;
  /* To be filled by customer according to the platform request. Return 0 if OK
   */
  return i2c_master_receive(pdev->dev_handle, pdata, count, -1);
}

VL53L1_Error VL53L1_WrByte(VL53L1_Dev_t *pdev, uint16_t index,
                           uint8_t VL53L1_p_002) {
  VL53L1_Error status = VL53L1_ERROR_NONE;
  uint8_t buffer[1];

  buffer[0] = (uint8_t)(VL53L1_p_002);

  status = VL53L1_WriteMulti(pdev, index, buffer, 1);

  return status;
}

VL53L1_Error VL53L1_WrWord(VL53L1_Dev_t *pdev, uint16_t index,
                           uint16_t VL53L1_p_002) {
  VL53L1_Error status = VL53L1_ERROR_NONE;
  uint8_t buffer[2];

  buffer[0] = (uint8_t)(VL53L1_p_002 >> 8);
  buffer[1] = (uint8_t)(VL53L1_p_002 & 0x00FF);

  status = VL53L1_WriteMulti(pdev, index, buffer, VL53L1_BYTES_PER_WORD);

  return status;
}

VL53L1_Error VL53L1_WrDWord(VL53L1_Dev_t *pdev, uint16_t index,
                            uint32_t VL53L1_p_002) {
  VL53L1_Error status = VL53L1_ERROR_NONE;
  uint8_t buffer[4];

  buffer[0] = (uint8_t)(VL53L1_p_002 >> 24);
  buffer[1] = (uint8_t)((VL53L1_p_002 & 0x00FF0000) >> 16);
  buffer[2] = (uint8_t)((VL53L1_p_002 & 0x0000FF00) >> 8);
  buffer[3] = (uint8_t)(VL53L1_p_002 & 0x000000FF);

  status = VL53L1_WriteMulti(pdev, index, buffer, VL53L1_BYTES_PER_DWORD);

  return status;
}

VL53L1_Error VL53L1_RdByte(VL53L1_Dev_t *pdev, uint16_t index, uint8_t *pdata) {
  VL53L1_Error status = VL53L1_ERROR_NONE;
  uint8_t buffer[1];

  status = VL53L1_ReadMulti(pdev, index, buffer, 1);

  *pdata = buffer[0];

  return status;
}

VL53L1_Error VL53L1_RdWord(VL53L1_Dev_t *pdev, uint16_t index,
                           uint16_t *pdata) {
  VL53L1_Error status = VL53L1_ERROR_NONE;
  uint8_t buffer[2];

  status = VL53L1_ReadMulti(pdev, index, buffer, VL53L1_BYTES_PER_WORD);

  *pdata = (uint16_t)(((uint16_t)(buffer[0]) << 8) + (uint16_t)buffer[1]);

  return status;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_Dev_t *pdev, uint16_t index,
                            uint32_t *pdata) {
  VL53L1_Error status = VL53L1_ERROR_NONE;
  uint8_t buffer[4];

  status = VL53L1_ReadMulti(pdev, index, buffer, VL53L1_BYTES_PER_DWORD);

  *pdata = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) +
           ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];

  return status;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us) {
  VL53L1_Error status = 255;
  /* To be filled by customer according to the platform request. Return 0 if OK
   */
  return status;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms) {
  VL53L1_Error status = 255;
  /* To be filled by customer according to the platform request. Return 0 if OK
   */
  return status;
}

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz) {
  VL53L1_Error status = 255;
  /* To be filled by customer according to the platform request. Return 0 if OK
   */
  return status;
}

VL53L1_Error VL53L1_GetTimerValue(int32_t *ptimer_count) {
  VL53L1_Error status = 255;
  /* To be filled by customer according to the platform request. Return 0 if OK
   */
  return status;
}

VL53L1_Error VL53L1_GpioSetMode(uint8_t pin, uint8_t mode) {
  VL53L1_Error status = 255;
  /* To be filled by customer according to the platform request. Return 0 if OK
   */
  return status;
}

VL53L1_Error VL53L1_GpioSetValue(uint8_t pin, uint8_t value) {
  VL53L1_Error status = 255;
  /* To be filled by customer according to the platform request. Return 0 if OK
   */
  return status;
}

VL53L1_Error VL53L1_GpioGetValue(uint8_t pin, uint8_t *pvalue) {
  VL53L1_Error status = 255;
  /* To be filled by customer according to the platform request. Return 0 if OK
   */
  return status;
}

VL53L1_Error VL53L1_GpioXshutdown(uint8_t value) {
  VL53L1_Error status = 255;
  /* To be filled by customer according to the platform request. Return 0 if OK
   */
  return status;
}

VL53L1_Error VL53L1_GpioCommsSelect(uint8_t value) {
  VL53L1_Error status = 255;
  /* To be filled by customer according to the platform request. Return 0 if OK
   */
  return status;
}

VL53L1_Error VL53L1_GpioPowerEnable(uint8_t value) {
  VL53L1_Error status = 255;
  /* To be filled by customer according to the platform request. Return 0 if OK
   */
  return status;
}

VL53L1_Error VL53L1_GpioInterruptEnable(void (*function)(void),
                                        uint8_t edge_type) {
  VL53L1_Error status = 255;
  /* To be filled by customer according to the platform request. Return 0 if OK
   */
  return status;
}

VL53L1_Error VL53L1_GpioInterruptDisable(void) {
  VL53L1_Error status = 255;
  /* To be filled by customer according to the platform request. Return 0 if OK
   */
  return status;
}

VL53L1_Error VL53L1_GetTickCount(VL53L1_Dev_t *pdev, uint32_t *ptick_count_ms) {

  VL53L1_Error status = 255;
  (void)pdev;

  /* To be filled by customer according to the platform request. Return 0 if OK
   */
  /* example: *ptick_count_ms = timeGetTime();   */
  trace_print(VL53L1_TRACE_LEVEL_DEBUG, "VL53L1_GetTickCount() = %5u ms;\n",
              *ptick_count_ms);

  return status;
}

VL53L1_Error VL53L1_WaitValueMaskEx(VL53L1_Dev_t *pdev, uint32_t timeout_ms,
                                    uint16_t index, uint8_t value, uint8_t mask,
                                    uint32_t poll_delay_ms) {

  VL53L1_Error status = VL53L1_ERROR_NONE;
  uint32_t start_time_ms = 0;
  uint32_t current_time_ms = 0;
  uint8_t byte_value = 0;
  uint8_t found = 0;
#ifdef VL53L1_LOG_ENABLE
  uint32_t trace_functions = 0;
#endif

#ifdef VL53L1_LOG_ENABLE
  trace_i2c("WaitValueMaskEx(%5d, %s, 0x%02X, 0x%02X, %5d);\n", timeout_ms,
            index, value, mask, poll_delay_ms);
#endif

  VL53L1_GetTickCount(pdev, &start_time_ms);
  pdev->new_data_ready_poll_duration_ms = 0;

#ifdef VL53L1_LOG_ENABLE
  trace_functions = _LOG_GET_TRACE_FUNCTIONS();
#endif
  _LOG_SET_TRACE_FUNCTIONS(VL53L1_TRACE_FUNCTION_NONE);

  while ((status == VL53L1_ERROR_NONE) &&
         (pdev->new_data_ready_poll_duration_ms < timeout_ms) && (found == 0)) {
    status = VL53L1_WaitMs(pdev, poll_delay_ms);
    status = VL53L1_RdByte(pdev, index, &byte_value);

    if ((byte_value & mask) == value) {
      found = 1;
    }

    VL53L1_GetTickCount(pdev, &current_time_ms);
    pdev->new_data_ready_poll_duration_ms = current_time_ms - start_time_ms;
  }

  _LOG_SET_TRACE_FUNCTIONS(trace_functions);

  if (found == 0 && status == VL53L1_ERROR_NONE)
    status = VL53L1_ERROR_TIME_OUT;

  return status;
}
