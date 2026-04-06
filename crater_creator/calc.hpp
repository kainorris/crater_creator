#include <cmath>
#include <iostream>

namespace Calculator {

static const Point2D calculate_landing_position(float distance, int i, int j) {
  float fov_deg = 90;
  float sensor_height = 3.0f; // In inches from bottom of box (sand)

  float fov_rad = fov_deg * (M_PI / 180.0);

  // Calculate the angle for this specific sub-sensor
  // Center the indices so 3.5, 3.5 is the center of 0-7
  float angle_x = ((static_cast<float>(i) - 3.5f) / 7.0f) * fov_rad;
  float angle_y = ((static_cast<float>(j) - 3.5f) / 7.0f) * fov_rad;

  // Calculate coordinates relative to the sensor
  float x_rel = distance * std::sin(angle_x);
  float z_rel = distance * std::cos(angle_x) * std::cos(angle_y);

  // Project to Box Coordinates
  // Assuming sensor is EXACT middle of 9" side
  Point2D landing_pos;
  landing_pos.x = 4.75f + x_rel;
  landing_pos.y = z_rel;

  return landing_pos;
}
}; // namespace Calculator
