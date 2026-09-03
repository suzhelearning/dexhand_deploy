#include "tianji_teleop/wuji_hand2/direct_target_interpolator.hpp"
#include "tianji_teleop/wuji_hand2/wuji_execution_limits.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <limits>
#include <iostream>

namespace {

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

void require_close(float actual, float expected, const char *message) {
  require(std::abs(actual - expected) <= 1.0e-6F, message);
}

}  // namespace

int main() {
  using Interpolator = tianji_teleop::DirectTargetInterpolator<2>;
  Interpolator interpolator;
  const std::array<float, 2> first{0.0F, 2.0F};
  const std::array<float, 2> second{2.0F, 4.0F};
  const std::array<float, 2> irregular{4.0F, 8.0F};

  interpolator.accept(first, 1'000'000'000);
  auto sample = interpolator.sample(1'010'000'000);
  require_close(sample[0], 0.0F, "first target must be held before a successor arrives");
  require_close(sample[1], 2.0F, "first target must be held before a successor arrives");

  interpolator.accept(second, 1'020'000'000);
  sample = interpolator.sample(1'020'000'000);
  require_close(sample[0], 0.0F, "new target must start from the previous target");
  sample = interpolator.sample(1'020'000'000 + 10'000'000);
  require_close(sample[0], 1.0F, "10 ms sample must be the exact midpoint");
  require_close(sample[1], 3.0F, "10 ms sample must be the exact midpoint");

  sample = interpolator.sample(1'020'000'000 + 20'000'000);
  require_close(sample[0], 2.0F, "sample at the window end must reach current target");
  sample = interpolator.sample(1'020'000'000 + 30'000'000);
  require_close(sample[0], 2.0F, "sample after the window must not extrapolate");

  interpolator.accept(irregular, 1'060'000'000);
  sample = interpolator.sample(1'060'000'000 + 10'000'000);
  require_close(sample[0], 3.0F, "irregular arrival must still use the fixed 20 ms window");
  interpolator.accept(first, 2'000'000'000);
  interpolator.reset();  // teleop -> idle
  interpolator.accept(second, 2'020'000'000);  // idle -> teleop
  sample = interpolator.sample(2'030'000'000);
  require_close(sample[0], 2.0F, "first target after session reset must not interpolate from the prior session");
  require_close(sample[1], 4.0F, "first target after session reset must not interpolate from the prior session");
  interpolator.accept(first, 3'000'000'000);
  interpolator.reset();  // teleop -> idle
  const bool have_direct = false;  // idle -> teleop, no new command
  const auto input_received_ns = std::int64_t{0};
  const bool tracking = have_direct && input_received_ns > 0 &&
    interpolator.has_current();
  require(!tracking, "teleop without a new direct command must not track");
  const double upper_limit = 1.5707963267948966;
  const double lower_limit = -upper_limit;
  const float encoded_upper = static_cast<float>(upper_limit);
  const float encoded_lower = static_cast<float>(lower_limit);
  require(tianji_teleop::within_wuji_execution_limits(
    encoded_upper, lower_limit, upper_limit), "exact encoded upper limit must pass");
  require(tianji_teleop::within_wuji_execution_limits(
    encoded_lower, lower_limit, upper_limit), "exact encoded lower limit must pass");
  require(!tianji_teleop::within_wuji_execution_limits(
    std::nextafter(encoded_upper, std::numeric_limits<float>::infinity()),
    lower_limit, upper_limit), "float above encoded upper limit must fail");
  require(!tianji_teleop::within_wuji_execution_limits(
    std::nextafter(encoded_lower, -std::numeric_limits<float>::infinity()),
    lower_limit, upper_limit), "float below encoded lower limit must fail");
  std::cout << "wuji hand2 interpolation probe passed\n";
  return 0;
}
