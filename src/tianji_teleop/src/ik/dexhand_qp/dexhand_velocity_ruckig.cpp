// Ported from TJ_arm_control_pico_ee_ik; see PORTING.md.
#include "tianji_qp_ik/dexhand_velocity_ruckig.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace tianji_qp_ik {
namespace {

template <typename Array>
Vec7 toEigen(const Array& value) {
  Vec7 result;
  for (int index = 0; index < kArmDof; ++index) {
    result[index] = value[static_cast<std::size_t>(index)];
  }
  return result;
}

template <typename Array>
Array toArray(const Vec7& value) {
  Array result{};
  for (int index = 0; index < kArmDof; ++index) {
    result[static_cast<std::size_t>(index)] = value[index];
  }
  return result;
}

bool finiteState(const ArmMotionState& state) noexcept {
  return state.q.allFinite() && state.qdot.allFinite() &&
         state.qddot.allFinite();
}

double maximumRatio(const Vec7& value, const Vec7& limit) noexcept {
  return (value.cwiseAbs().array() / limit.array()).maxCoeff();
}

}  // namespace

DexhandVelocityRuckig7::DexhandVelocityRuckig7(
    DexhandVelocityQpConfig config, ArmLimits limits, double initial_dt)
    : config_(std::move(config)),
      limits_(std::move(limits)),
      otg_(initial_dt) {
  if (!std::isfinite(initial_dt) || initial_dt <= 0.0 ||
      !limits_.lower_position.allFinite() ||
      !limits_.upper_position.allFinite() || !limits_.velocity.allFinite() ||
      (limits_.lower_position.array() > limits_.upper_position.array()).any() ||
      (limits_.velocity.array() <= 0.0).any() ||
      !config_.ruckig_max_velocity_rad_s.allFinite() ||
      (config_.ruckig_max_velocity_rad_s.array() <= 0.0).any() ||
      !config_.ruckig_max_acceleration_rad_s2.allFinite() ||
      (config_.ruckig_max_acceleration_rad_s2.array() <= 0.0).any() ||
      !config_.ruckig_max_jerk_rad_s3.allFinite() ||
      (config_.ruckig_max_jerk_rad_s3.array() <= 0.0).any() ||
      !std::isfinite(config_.ruckig_validation_tolerance) ||
      config_.ruckig_validation_tolerance <= 0.0) {
    throw std::invalid_argument("invalid dexhand velocity Ruckig configuration");
  }
  input_.control_interface = ruckig::ControlInterface::Velocity;
  input_.synchronization = ruckig::Synchronization::Time;
}

Vec7 DexhandVelocityRuckig7::velocityLimit() const noexcept {
  return config_.ruckig_max_velocity_rad_s.cwiseMin(limits_.velocity);
}

bool DexhandVelocityRuckig7::reset(const ArmMotionState& state) noexcept {
  if (!finiteState(state) ||
      (state.q.array() < limits_.lower_position.array()).any() ||
      (state.q.array() > limits_.upper_position.array()).any()) {
    initialized_ = false;
    state_ = ArmMotionState{};
    return false;
  }
  state_ = state;
  initialized_ = true;
  return true;
}

DexhandVelocityRuckigResult DexhandVelocityRuckig7::update(
    const Vec7& target_velocity, double dt) {
  DexhandVelocityRuckigResult result;
  result.state = state_;
  const Vec7 velocity_limit = velocityLimit();
  if (!initialized_ || !finiteState(state_) || !std::isfinite(dt) || dt <= 0.0 ||
      !velocity_limit.allFinite() ||
      (velocity_limit.array() <= 0.0).any()) {
    result.rejected = true;
    result.detail = "dexhand_velocity_ruckig_invalid_input";
    return result;
  }

  const auto holdRejected = [&]() {
    // Ruckig can return a non-negative result while the post-update contract
    // check still detects a numerical limit overrun.  Treat that exactly like
    // a transient negative result: preserve the last safe velocity, advance
    // one sample, and reset the limiter's acceleration history before retry.
    const Vec7 held_velocity =
        state_.qdot.cwiseMax(-velocity_limit).cwiseMin(velocity_limit);
    ArmMotionState held = state_;
    // A previous safe velocity may point out of a joint limit after the
    // model has advanced to that limit.  Clamp the held position first, then
    // derive the actually applied velocity from the clamped displacement so
    // re-anchoring itself cannot fail at a boundary.
    held.q = (state_.q + dt * held_velocity)
                 .cwiseMax(limits_.lower_position)
                 .cwiseMin(limits_.upper_position);
    held.qdot = (held.q - state_.q) / dt;
    held.qddot.setZero();
    if (!reset(held)) {
      result.rejected = true;
      result.detail = "dexhand_velocity_ruckig_rejected";
      return result;
    }
    result.accepted = true;
    result.held = true;
    result.rejected = true;
    result.state = state_;
    result.jerk.setZero();
    result.velocity_ratio = maximumRatio(state_.qdot, velocity_limit);
    result.detail = "dexhand_velocity_ruckig_held";
    return result;
  };

  if (!target_velocity.allFinite()) {
    return holdRejected();
  }

  otg_.delta_time = dt;
  input_.current_position = toArray<decltype(input_.current_position)>(state_.q);
  input_.current_velocity = toArray<decltype(input_.current_velocity)>(state_.qdot);
  input_.current_acceleration =
      toArray<decltype(input_.current_acceleration)>(state_.qddot);
  input_.target_velocity =
      toArray<decltype(input_.target_velocity)>(target_velocity);
  input_.target_acceleration =
      toArray<decltype(input_.target_acceleration)>(Vec7::Zero());
  input_.max_velocity = toArray<decltype(input_.max_velocity)>(velocity_limit);
  input_.max_acceleration = toArray<decltype(input_.max_acceleration)>(
      config_.ruckig_max_acceleration_rad_s2);
  input_.max_jerk =
      toArray<decltype(input_.max_jerk)>(config_.ruckig_max_jerk_rad_s3);
  input_.control_interface = ruckig::ControlInterface::Velocity;
  input_.synchronization = ruckig::Synchronization::Time;

  const ruckig::Result update_result = otg_.update(input_, output_);
  if (static_cast<int>(update_result) < 0) {
    return holdRejected();
  }

  ArmMotionState candidate;
  candidate.q = toEigen(output_.new_position);
  candidate.qdot = toEigen(output_.new_velocity);
  candidate.qddot = toEigen(output_.new_acceleration);
  if (!finiteState(candidate) ||
      (candidate.q.array() < limits_.lower_position.array() -
                              config_.ruckig_validation_tolerance)
           .any() ||
      (candidate.q.array() > limits_.upper_position.array() +
                              config_.ruckig_validation_tolerance)
           .any()) {
    return holdRejected();
  }
  result.jerk = (candidate.qddot - state_.qddot) / dt;
  result.velocity_ratio = maximumRatio(candidate.qdot, velocity_limit);
  result.acceleration_ratio = maximumRatio(
      candidate.qddot, config_.ruckig_max_acceleration_rad_s2);
  result.jerk_ratio = maximumRatio(result.jerk, config_.ruckig_max_jerk_rad_s3);
  const double tolerance = config_.ruckig_validation_tolerance;
  if (result.velocity_ratio > 1.0 + tolerance ||
      result.acceleration_ratio > 1.0 + tolerance ||
      result.jerk_ratio > 1.0 + tolerance) {
    return holdRejected();
  }
  state_ = candidate;
  result.state = state_;
  result.accepted = true;
  result.detail = "dexhand_velocity_ruckig_accepted";
  return result;
}

}  // namespace tianji_qp_ik
