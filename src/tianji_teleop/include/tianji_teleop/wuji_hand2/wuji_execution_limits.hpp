#pragma once

#include <cmath>

namespace tianji_teleop {

inline bool within_wuji_execution_limits(float value, double lower, double upper) noexcept {
  const float lower_float = static_cast<float>(lower);
  const float upper_float = static_cast<float>(upper);
  return std::isfinite(value) && value >= lower_float && value <= upper_float;
}

}  // namespace tianji_teleop
