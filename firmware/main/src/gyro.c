#include "gyro.hpp"

Gyro::Gyro() {
  i2c({.port = i2c_port,
       .sda_io_num = i2c_sda,
       .scl_io_num = i2c_scl,
       .sda_pullup_en = GPIO_PULLUP_ENABLE,
       .scl_pullup_en = GPIO_PULLUP_ENABLE,
       .clk_speed = i2c_clock_speed});
}
