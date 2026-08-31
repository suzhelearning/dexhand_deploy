#pragma once

#include "pico_body_tianji/ik/arm_ik_solver.hpp"

#include <ruckig/ruckig.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

namespace pico_body_tianji
{

struct ArmMotionState
{
  ArmJointVector position{ArmJointVector::Zero()};
  ArmJointVector velocity{ArmJointVector::Zero()};
  ArmJointVector acceleration{ArmJointVector::Zero()};
};

struct JointTrajectoryLimits
{
  ArmJointVector lower_position{ArmJointVector::Zero()};
  ArmJointVector upper_position{ArmJointVector::Zero()};
  ArmJointVector maximum_velocity{ArmJointVector::Zero()};
  ArmJointVector maximum_acceleration{ArmJointVector::Zero()};
  ArmJointVector maximum_jerk{ArmJointVector::Zero()};
  double validation_tolerance{1.0e-8};
};

struct JointTrajectoryResult
{
  bool accepted{false};
  bool hard_failure{false};
  ArmMotionState state;
  double velocity_ratio{0.0};
  double acceleration_ratio{0.0};
  double jerk_ratio{0.0};
  std::string_view detail{"not_updated"};
};

// TJ_arm_control 的七关节 Ruckig 限幅器；速度级 QP 作为目标，失败时不更新状态。
class JointTrajectoryLimiter7
{
public:
  JointTrajectoryLimiter7(JointTrajectoryLimits limits, double control_period_s);

  bool reset(const ArmMotionState & state) noexcept;
  JointTrajectoryResult update(const ArmJointVector & target_velocity);
  const ArmMotionState & state() const noexcept {return state_;}
  bool initialized() const noexcept {return initialized_;}

private:
  bool valid_state(const ArmMotionState & state) const noexcept;

  JointTrajectoryLimits limits_;
  double control_period_s_;
  ruckig::Ruckig<7> otg_;
  ruckig::InputParameter<7> input_;
  ruckig::OutputParameter<7> output_;
  ArmMotionState state_;
  bool initialized_{false};
};

// 连续数值失败在 grace 内降级保持；一次成功即清零窗口。
class ConsecutiveFailureWindow
{
public:
  explicit ConsecutiveFailureWindow(std::int64_t grace_ns) : grace_ns_(grace_ns) {}

  bool failed(std::int64_t now_ns) noexcept
  {
    if (!first_failure_ns_) first_failure_ns_ = now_ns;
    ++count_;
    return now_ns - *first_failure_ns_ >= grace_ns_;
  }

  void recovered() noexcept
  {
    first_failure_ns_.reset();
    count_ = 0;
  }

  std::int64_t age_ns(std::int64_t now_ns) const noexcept
  {
    return first_failure_ns_ ? now_ns - *first_failure_ns_ : 0;
  }

  std::uint64_t count() const noexcept {return count_;}

private:
  std::int64_t grace_ns_;
  std::optional<std::int64_t> first_failure_ns_;
  std::uint64_t count_{0};
};

}  // namespace pico_body_tianji
