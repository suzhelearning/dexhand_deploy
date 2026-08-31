#include "tianji_teleop/control/joint_trajectory_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace tianji_teleop
{
namespace
{

std::array<double, 7> to_array(const ArmJointVector & value)
{
  std::array<double, 7> result{};
  for (Eigen::Index index = 0; index < value.size(); ++index) {
    result[static_cast<std::size_t>(index)] = value[index];
  }
  return result;
}

ArmJointVector to_eigen(const std::array<double, 7> & value)
{
  ArmJointVector result;
  for (Eigen::Index index = 0; index < result.size(); ++index) {
    result[index] = value[static_cast<std::size_t>(index)];
  }
  return result;
}

double maximum_ratio(const ArmJointVector & value, const ArmJointVector & limit)
{
  return (value.cwiseAbs().array() / limit.array()).maxCoeff();
}

}  // namespace

JointTrajectoryLimiter7::JointTrajectoryLimiter7(
  JointTrajectoryLimits limits,
  double control_period_s)
: limits_(std::move(limits)), control_period_s_(control_period_s),
  otg_(control_period_s)
{
  if (
    !std::isfinite(control_period_s_) || control_period_s_ <= 0.0 ||
    !limits_.lower_position.allFinite() || !limits_.upper_position.allFinite() ||
    !limits_.maximum_velocity.allFinite() ||
    !limits_.maximum_acceleration.allFinite() ||
    !limits_.maximum_jerk.allFinite() ||
    (limits_.lower_position.array() >= limits_.upper_position.array()).any() ||
    (limits_.maximum_velocity.array() <= 0.0).any() ||
    (limits_.maximum_acceleration.array() <= 0.0).any() ||
    (limits_.maximum_jerk.array() <= 0.0).any() ||
    !std::isfinite(limits_.validation_tolerance) ||
    limits_.validation_tolerance < 0.0)
  {
    throw std::invalid_argument("Ruckig trajectory limits are invalid");
  }
  input_.control_interface = ruckig::ControlInterface::Velocity;
  input_.synchronization = ruckig::Synchronization::Time;
  input_.min_position = to_array(limits_.lower_position);
  input_.max_position = to_array(limits_.upper_position);
}

bool JointTrajectoryLimiter7::valid_state(const ArmMotionState & state) const noexcept
{
  const double tolerance = limits_.validation_tolerance;
  return state.position.allFinite() && state.velocity.allFinite() &&
         state.acceleration.allFinite() &&
         (state.position.array() >= limits_.lower_position.array() - tolerance).all() &&
         (state.position.array() <= limits_.upper_position.array() + tolerance).all() &&
         (state.velocity.cwiseAbs().array() <=
          limits_.maximum_velocity.array() + tolerance).all() &&
         (state.acceleration.cwiseAbs().array() <=
          limits_.maximum_acceleration.array() + tolerance).all();
}

bool JointTrajectoryLimiter7::reset(const ArmMotionState & state) noexcept
{
  if (!valid_state(state)) return false;
  state_ = state;
  initialized_ = true;
  otg_.reset();
  return true;
}

JointTrajectoryResult JointTrajectoryLimiter7::update(
  const ArmJointVector & target_velocity)
{
  JointTrajectoryResult result;
  result.state = state_;
  if (!initialized_ || !target_velocity.allFinite()) {
    result.hard_failure = true;
    result.detail = "invalid_joint_trajectory_input";
    return result;
  }
  const ArmJointVector bounded_target_velocity = target_velocity.cwiseMax(
    -limits_.maximum_velocity).cwiseMin(limits_.maximum_velocity);

  input_.current_position = to_array(state_.position);
  input_.current_velocity = to_array(state_.velocity);
  input_.current_acceleration = to_array(state_.acceleration);
  input_.target_position = to_array(state_.position);
  input_.target_velocity = to_array(bounded_target_velocity);
  input_.target_acceleration = to_array(ArmJointVector::Zero());
  input_.max_velocity = to_array(limits_.maximum_velocity);
  input_.max_acceleration = to_array(limits_.maximum_acceleration);
  input_.max_jerk = to_array(limits_.maximum_jerk);

  const ruckig::Result update_result = otg_.update(input_, output_);
  if (static_cast<int>(update_result) < 0) {
    result.detail = "joint_trajectory_ruckig_rejected";
    return result;
  }

  ArmMotionState candidate;
  candidate.position = to_eigen(output_.new_position);
  candidate.velocity = to_eigen(output_.new_velocity);
  candidate.acceleration = to_eigen(output_.new_acceleration);
  const ArmJointVector jerk =
    (candidate.acceleration - state_.acceleration) / control_period_s_;
  result.velocity_ratio = maximum_ratio(
    candidate.velocity, limits_.maximum_velocity);
  result.acceleration_ratio = maximum_ratio(
    candidate.acceleration, limits_.maximum_acceleration);
  result.jerk_ratio = maximum_ratio(jerk, limits_.maximum_jerk);
  const double maximum = std::max(
    {result.velocity_ratio, result.acceleration_ratio, result.jerk_ratio});
  if (
    !valid_state(candidate) || !jerk.allFinite() ||
    maximum > 1.0 + limits_.validation_tolerance)
  {
    result.hard_failure = true;
    result.detail = "joint_trajectory_output_rejected";
    return result;
  }

  state_ = candidate;
  result.state = state_;
  result.accepted = true;
  result.detail = update_result == ruckig::Result::Finished ?
    "joint_velocity_finished" : "joint_velocity_working";
  return result;
}

}  // namespace tianji_teleop
