#pragma once

#include <array>
#include <cstdint>

namespace tianji_teleop {

constexpr std::int64_t kDirectInterpolationWindowNs = 20'000'000;

// A successor target starts a causal 20 ms segment at its local receive time.
template<std::size_t JointCount>
class DirectTargetInterpolator {
public:
  void accept(const std::array<float, JointCount> &target, std::int64_t received_ns) noexcept {
    if (have_current_) {
      previous_target_ = current_target_;
      previous_received_ns_ = current_received_ns_;
      have_previous_ = true;
    }
    current_target_ = target;
    current_received_ns_ = received_ns;
    have_current_ = true;
  }
  bool has_current() const noexcept { return have_current_; }

  std::array<float, JointCount> sample(std::int64_t sample_ns) const noexcept {
    if (!have_current_) return {};
    if (!have_previous_) return current_target_;
    if (current_received_ns_ <= previous_received_ns_) return current_target_;
    if (sample_ns <= current_received_ns_) return previous_target_;
    const auto elapsed_ns = sample_ns - current_received_ns_;
    if (elapsed_ns >= kDirectInterpolationWindowNs) return current_target_;
    const float alpha = static_cast<float>(elapsed_ns) /
      static_cast<float>(kDirectInterpolationWindowNs);
    std::array<float, JointCount> result{};
    for (std::size_t index = 0; index < JointCount; ++index) {
      result[index] = previous_target_[index] +
        (current_target_[index] - previous_target_[index]) * alpha;
    }
    return result;
  }

  void reset() noexcept {
    previous_target_.fill(0.0F);
    current_target_.fill(0.0F);
    previous_received_ns_ = 0;
    current_received_ns_ = 0;
    have_previous_ = false;
    have_current_ = false;
  }

private:
  std::array<float, JointCount> previous_target_{};
  std::array<float, JointCount> current_target_{};
  std::int64_t previous_received_ns_{0};
  std::int64_t current_received_ns_{0};
  bool have_previous_{false};
  bool have_current_{false};
};

}  // namespace tianji_teleop
