// Ported from TJ_arm_control_pico_ee_ik; see PORTING.md.
#include "tianji_qp_ik/dexhand_velocity_qp_ik.hpp"

#include "tianji_qp_ik/so3.hpp"

#include <Eigen/Cholesky>
#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace tianji_qp_ik {
namespace {

using Clock = std::chrono::steady_clock;
using ArmHessian = Mat77;

struct BoxQpResult {
  Vec7 solution{Vec7::Zero()};
  bool converged{false};
  int iterations{0};
  int active_bounds{0};
};

bool finiteLimits(const ArmLimits& limits) noexcept {
  return limits.lower_position.allFinite() &&
         limits.upper_position.allFinite() && limits.velocity.allFinite() &&
         (limits.lower_position.array() <= limits.upper_position.array()).all() &&
         (limits.velocity.array() > 0.0).all();
}

bool finitePose(const Pose& pose) noexcept {
  return pose.position.allFinite() && pose.rotation.allFinite() &&
         isProperRotation(pose.rotation);
}

Eigen::Vector3d clampVectorNorm(const Eigen::Vector3d& value, double maximum) {
  const double norm = value.norm();
  if (norm <= maximum || norm <= std::numeric_limits<double>::epsilon()) {
    return value;
  }
  return value * (maximum / norm);
}

void addCartesianLeastSquares(ArmHessian& hessian, Vec7& linear,
                              const Eigen::Matrix<double, 3, 7>& jacobian,
                              const Eigen::Vector3d& target, double weight,
                              double normalization) {
  if (weight <= 0.0) {
    return;
  }
  const Eigen::Matrix<double, 3, 7> scaled_jacobian = jacobian / normalization;
  const Eigen::Vector3d scaled_target = target / normalization;
  hessian.noalias() +=
      2.0 * weight * scaled_jacobian.transpose() * scaled_jacobian;
  linear.noalias() -=
      2.0 * weight * scaled_jacobian.transpose() * scaled_target;
}

void addDiagonalTrackingCost(ArmHessian& hessian, Vec7& linear,
                             const Vec7& target, const Vec7& scale,
                             double weight) {
  if (weight <= 0.0) {
    return;
  }
  for (int index = 0; index < kArmDof; ++index) {
    const double coefficient = weight / (scale[index] * scale[index]);
    hessian(index, index) += 2.0 * coefficient;
    linear[index] -= 2.0 * coefficient * target[index];
  }
}

BoxQpResult solveBoxQp(const ArmHessian& hessian, const Vec7& linear,
                       const Vec7& lower, const Vec7& upper,
                       const Vec7& warm_start, int maximum_iterations,
                       double tolerance) {
  BoxQpResult result;
  if (!hessian.allFinite() || !linear.allFinite() || !lower.allFinite() ||
      !upper.allFinite() || !warm_start.allFinite() ||
      (lower.array() > upper.array()).any() || maximum_iterations <= 0 ||
      !std::isfinite(tolerance) || tolerance <= 0.0) {
    return result;
  }

  result.solution = warm_start.cwiseMax(lower).cwiseMin(upper);
  std::array<int, kArmDof> active{};
  for (int index = 0; index < kArmDof; ++index) {
    if (upper[index] - lower[index] <= tolerance) {
      result.solution[index] = 0.5 * (lower[index] + upper[index]);
      active[static_cast<std::size_t>(index)] = 2;
    } else if (result.solution[index] <= lower[index] + tolerance) {
      result.solution[index] = lower[index];
      active[static_cast<std::size_t>(index)] = -1;
    } else if (result.solution[index] >= upper[index] - tolerance) {
      result.solution[index] = upper[index];
      active[static_cast<std::size_t>(index)] = 1;
    }
  }

  for (int iteration = 0; iteration < maximum_iterations; ++iteration) {
    result.iterations = iteration + 1;
    const Vec7 gradient = hessian * result.solution + linear;
    std::vector<int> free_indices;
    free_indices.reserve(kArmDof);
    for (int index = 0; index < kArmDof; ++index) {
      if (active[static_cast<std::size_t>(index)] == 0) {
        free_indices.push_back(index);
      }
    }

    Vec7 direction = Vec7::Zero();
    if (!free_indices.empty()) {
      const Eigen::Index free_count =
          static_cast<Eigen::Index>(free_indices.size());
      Eigen::MatrixXd reduced_hessian(free_count, free_count);
      Eigen::VectorXd reduced_gradient(free_count);
      for (Eigen::Index row = 0; row < free_count; ++row) {
        const int row_index = free_indices[static_cast<std::size_t>(row)];
        reduced_gradient[row] = gradient[row_index];
        for (Eigen::Index column = 0; column < free_count; ++column) {
          const int column_index =
              free_indices[static_cast<std::size_t>(column)];
          reduced_hessian(row, column) = hessian(row_index, column_index);
        }
      }
      const Eigen::LDLT<Eigen::MatrixXd> factorization(reduced_hessian);
      if (factorization.info() != Eigen::Success) {
        return result;
      }
      const Eigen::VectorXd reduced_direction =
          factorization.solve(-reduced_gradient);
      if (factorization.info() != Eigen::Success ||
          !reduced_direction.allFinite()) {
        return result;
      }
      for (Eigen::Index index = 0; index < free_count; ++index) {
        direction[free_indices[static_cast<std::size_t>(index)]] =
            reduced_direction[index];
      }
    }

    if (direction.cwiseAbs().maxCoeff() <= tolerance) {
      int release_index = -1;
      double largest_violation = tolerance;
      for (int index = 0; index < kArmDof; ++index) {
        const int state = active[static_cast<std::size_t>(index)];
        double violation = 0.0;
        if (state == -1) {
          violation = -gradient[index];
        } else if (state == 1) {
          violation = gradient[index];
        }
        if (violation > largest_violation) {
          largest_violation = violation;
          release_index = index;
        }
      }
      if (release_index < 0) {
        result.converged = true;
        break;
      }
      active[static_cast<std::size_t>(release_index)] = 0;
      continue;
    }

    double step = 1.0;
    for (const int index : free_indices) {
      if (direction[index] > tolerance) {
        step = std::min(step, (upper[index] - result.solution[index]) /
                                  direction[index]);
      } else if (direction[index] < -tolerance) {
        step = std::min(step, (lower[index] - result.solution[index]) /
                                  direction[index]);
      }
    }
    step = std::clamp(step, 0.0, 1.0);
    result.solution += step * direction;
    result.solution = result.solution.cwiseMax(lower).cwiseMin(upper);
    if (step < 1.0 - tolerance) {
      for (const int index : free_indices) {
        if (result.solution[index] <= lower[index] + tolerance) {
          result.solution[index] = lower[index];
          active[static_cast<std::size_t>(index)] = -1;
        } else if (result.solution[index] >= upper[index] - tolerance) {
          result.solution[index] = upper[index];
          active[static_cast<std::size_t>(index)] = 1;
        }
      }
    }
  }

  result.active_bounds = static_cast<int>(std::count_if(
      active.begin(), active.end(), [](int value) { return value != 0; }));
  return result;
}

double minimumSingularValue(const Mat67& jacobian) {
  const Eigen::JacobiSVD<Mat67> svd(jacobian);
  return svd.info() == Eigen::Success && svd.singularValues().size() > 0
             ? svd.singularValues().minCoeff()
             : std::numeric_limits<double>::quiet_NaN();
}

bool validConfig(const DexhandVelocityQpConfig& config) noexcept {
  const auto positive = [](double value) {
    return std::isfinite(value) && value > 0.0;
  };
  const auto nonnegative = [](double value) {
    return std::isfinite(value) && value >= 0.0;
  };
  return config.nominal_left.allFinite() && config.nominal_right.allFinite() &&
         positive(config.position_time_constant_s) &&
         positive(config.orientation_time_constant_s) &&
         positive(config.max_linear_speed_m_s) &&
         positive(config.max_angular_speed_rad_s) &&
         config.qp_joint_velocity_limits_rad_s.allFinite() &&
         (config.qp_joint_velocity_limits_rad_s.array() > 0.0).all() &&
         positive(config.position_weight) &&
         nonnegative(config.orientation_weight) &&
         positive(config.velocity_regularization_weight) &&
         nonnegative(config.continuity_weight) &&
         nonnegative(config.posture_weight) &&
         positive(config.posture_time_constant_s) &&
         positive(config.maximum_joint_step_rad) &&
         positive(config.joint_limit_activation_margin_rad) &&
         positive(config.joint_limit_velocity_damper_gain) &&
         positive(config.singular_value_threshold) &&
         nonnegative(config.singularity_critical_threshold) &&
         config.singularity_critical_threshold < config.singular_value_threshold &&
         nonnegative(config.singularity_orientation_scale) &&
         config.singularity_orientation_scale <= 1.0 &&
         config.singularity_posture_multiplier >= 1.0 &&
         std::isfinite(config.singularity_posture_multiplier) &&
         config.singularity_velocity_multiplier >= 1.0 &&
         std::isfinite(config.singularity_velocity_multiplier) &&
         nonnegative(config.singularity_escape_weight) &&
         nonnegative(config.singularity_escape_speed_rad_s) &&
         config.max_active_set_iterations > 0 &&
         positive(config.active_set_tolerance);
}

}  // namespace

DexhandVelocityQpIk7::DexhandVelocityQpIk7(DexhandVelocityQpConfig config,
                                           ArmLimits limits, double initial_dt)
    : config_(std::move(config)), limits_(std::move(limits)), initial_dt_(initial_dt) {
  if (!validConfig(config_)) {
    throw std::invalid_argument("invalid dexhand velocity QP configuration");
  }
  if (!finiteLimits(limits_)) {
    throw std::invalid_argument("invalid dexhand velocity QP arm limits");
  }
  if (!std::isfinite(initial_dt_) || initial_dt_ <= 0.0) {
    throw std::invalid_argument("invalid dexhand velocity QP initial dt");
  }
  if (config_.ruckig_enabled) {
    ruckig_ = std::make_unique<DexhandVelocityRuckig7>(
        config_, limits_, initial_dt_);
  }
}

void DexhandVelocityQpIk7::reset(const ArmMotionState& state) noexcept {
  initialized_ = state.q.allFinite() && state.qdot.allFinite() &&
                 state.qddot.allFinite();
  warm_start_valid_ = initialized_;
  singularity_gradient_valid_ = false;
  if (initialized_) {
    last_q_ = state.q;
    last_qdot_ = state.qdot;
    if (ruckig_ != nullptr && !ruckig_->reset(state)) {
      initialized_ = false;
      warm_start_valid_ = false;
    }
  } else {
    last_q_.setZero();
    last_qdot_.setZero();
    if (ruckig_ != nullptr) {
      (void)ruckig_->reset(ArmMotionState{});
    }
  }
}

void DexhandVelocityQpIk7::clearWarmStart() noexcept {
  warm_start_valid_ = false;
  singularity_gradient_valid_ = false;
  if (ruckig_ != nullptr && initialized_ && last_q_.allFinite() &&
      last_qdot_.allFinite()) {
    // Keep the last accepted command as the only trajectory anchor after a
    // rejected target/kinematics/QP sample.  Clearing acceleration prevents a
    // failed sample from leaking a stale jerk history into the retry.
    (void)ruckig_->reset({last_q_, last_qdot_, Vec7::Zero()});
  }
}

DexhandVelocityQpResult DexhandVelocityQpIk7::solve(
    const DexhandVelocityQpIkInput& input) {
  DexhandVelocityQpResult result;
  result.nominal_mode = config_.dexhand_faithful_nominal
                            ? DexhandNominalMode::kFaithfulZero
                            : DexhandNominalMode::kConfigured;
  result.ruckig_enabled = config_.ruckig_enabled;
  result.q = initialized_ ? last_q_ : input.seed;
  result.q_qp = result.q;
  result.qdot = initialized_ ? last_qdot_ : Vec7::Zero();
  result.qdot_qp = result.qdot;

  const auto hold = [&](std::string_view detail) {
    result.accepted = result.q.allFinite() && result.qdot.allFinite();
    result.target_held = true;
    result.detail = detail;
    return result;
  };
  if (!input.seed.allFinite() || !input.qdot_previous.allFinite() ||
      !finiteLimits(input.limits) || !std::isfinite(input.dt) ||
      input.dt <= 0.0 || !input.evaluate) {
    result.qp_rejected = true;
    result.accepted = false;
    result.detail = "dexhand_velocity_qp_invalid_input";
    return result;
  }
  if (!initialized_) {
    last_q_ = input.seed;
    last_qdot_ = input.qdot_previous;
    initialized_ = true;
    warm_start_valid_ = false;
    if (ruckig_ != nullptr &&
        !ruckig_->reset({input.seed, input.qdot_previous, Vec7::Zero()})) {
      initialized_ = false;
      result.qp_rejected = true;
      result.accepted = false;
      result.detail = "dexhand_velocity_ruckig_init_rejected";
      return result;
    }
    result.q = last_q_;
    result.q_qp = last_q_;
    result.qdot = last_qdot_;
    result.qdot_qp = last_qdot_;
  }
  if (!input.target_valid || input.target_stale) {
    clearWarmStart();
    return hold("dexhand_velocity_qp_target_held");
  }
  if (!finitePose(input.target)) {
    clearWarmStart();
    result.qp_rejected = true;
    result.detail = "dexhand_velocity_qp_target_rejected";
    return result;
  }

  const auto start = Clock::now();
  ArmKinematicSample sample;
  try {
    sample = input.evaluate(input.seed);
  } catch (...) {
    clearWarmStart();
    result.qp_rejected = true;
    result.detail = "dexhand_velocity_qp_kinematics_rejected";
    return result;
  }
  if (!finitePose(sample.tcp_pose) || !sample.tcp_jacobian.allFinite()) {
    clearWarmStart();
    result.qp_rejected = true;
    result.detail = "dexhand_velocity_qp_kinematics_rejected";
    return result;
  }

  Vec6 pose_error;
  try {
    pose_error = poseErrorWorld(input.target, sample.tcp_pose);
  } catch (...) {
    clearWarmStart();
    result.qp_rejected = true;
    result.detail = "dexhand_velocity_qp_pose_rejected";
    return result;
  }
  result.position_error_m = pose_error.head<3>().norm();
  result.orientation_error_rad = pose_error.tail<3>().norm();
  if (config_.cartesian_reference_servo_enabled &&
      input.desired_twist_valid) {
    if (!input.desired_twist.allFinite()) {
      clearWarmStart();
      result.qp_rejected = true;
      result.detail = "dexhand_velocity_qp_twist_rejected";
      return result;
    }
    // The caller has already applied the Cartesian OTG limits and the main
    // profile's feedforward+feedback servo.  Do not reapply the Dexhand
    // pose-error/time-constant speed caps here.
    result.desired_linear_velocity = input.desired_twist.head<3>();
    result.desired_angular_velocity = input.desired_twist.tail<3>();
  } else {
    result.desired_linear_velocity = clampVectorNorm(
        pose_error.head<3>() / config_.position_time_constant_s,
        config_.max_linear_speed_m_s);
    result.desired_angular_velocity = clampVectorNorm(
        pose_error.tail<3>() / config_.orientation_time_constant_s,
        config_.max_angular_speed_rad_s);
  }
  result.desired_twist.head<3>() = result.desired_linear_velocity;
  result.desired_twist.tail<3>() = result.desired_angular_velocity;
  result.minimum_singular_value = minimumSingularValue(sample.tcp_jacobian);
  if (!std::isfinite(result.minimum_singular_value)) {
    clearWarmStart();
    result.qp_rejected = true;
    result.detail = "dexhand_velocity_qp_kinematics_rejected";
    return result;
  }

  const double denominator = config_.singular_value_threshold -
                             config_.singularity_critical_threshold;
  const double singularity_activation = std::clamp(
      (config_.singular_value_threshold - result.minimum_singular_value) /
          denominator,
      0.0, 1.0);
  const double orientation_scale =
      (1.0 - singularity_activation) +
      singularity_activation * config_.singularity_orientation_scale;
  const double posture_multiplier =
      (1.0 - singularity_activation) +
      singularity_activation * config_.singularity_posture_multiplier;
  const double velocity_multiplier =
      (1.0 - singularity_activation) +
      singularity_activation * config_.singularity_velocity_multiplier;

  const Vec7 velocity_limit =
      config_.qp_joint_velocity_limits_rad_s.cwiseMin(input.limits.velocity)
          .cwiseMin(Vec7::Constant(config_.maximum_joint_step_rad / input.dt));
  if (!velocity_limit.allFinite() || (velocity_limit.array() <= 0.0).any()) {
    clearWarmStart();
    result.qp_rejected = true;
    result.detail = "dexhand_velocity_qp_bounds_rejected";
    return result;
  }

  Vec7 lower = -velocity_limit;
  Vec7 upper = velocity_limit;
  for (int index = 0; index < kArmDof; ++index) {
    if (input.seed[index] < input.limits.lower_position[index] ||
        input.seed[index] > input.limits.upper_position[index]) {
      clearWarmStart();
      result.qp_rejected = true;
      result.detail = "dexhand_velocity_qp_bounds_rejected";
      return result;
    }
    lower[index] = std::max(
        lower[index],
        (input.limits.lower_position[index] - input.seed[index]) / input.dt);
    upper[index] = std::min(
        upper[index],
        (input.limits.upper_position[index] - input.seed[index]) / input.dt);
    const double lower_distance =
        input.seed[index] - input.limits.lower_position[index];
    const double upper_distance =
        input.limits.upper_position[index] - input.seed[index];
    if (lower_distance < config_.joint_limit_activation_margin_rad) {
      lower[index] = std::max(
          lower[index], -config_.joint_limit_velocity_damper_gain *
                            std::max(0.0, lower_distance));
    }
    if (upper_distance < config_.joint_limit_activation_margin_rad) {
      upper[index] = std::min(
          upper[index], config_.joint_limit_velocity_damper_gain *
                            std::max(0.0, upper_distance));
    }
  }
  if ((lower.array() > upper.array()).any()) {
    clearWarmStart();
    result.qp_rejected = true;
    result.detail = "dexhand_velocity_qp_bounds_rejected";
    return result;
  }

  ArmHessian hessian = ArmHessian::Zero();
  Vec7 linear = Vec7::Zero();
  addCartesianLeastSquares(
      hessian, linear, sample.tcp_jacobian.topRows<3>(),
      result.desired_linear_velocity, config_.position_weight,
      config_.max_linear_speed_m_s);
  addCartesianLeastSquares(
      hessian, linear, sample.tcp_jacobian.bottomRows<3>(),
      result.desired_angular_velocity,
      config_.orientation_weight * orientation_scale,
      config_.max_angular_speed_rad_s);
  addDiagonalTrackingCost(
      hessian, linear, Vec7::Zero(), velocity_limit,
      config_.velocity_regularization_weight * velocity_multiplier);

  Vec7 previous_velocity = input.qdot_previous;
  if (warm_start_valid_ && (last_q_ - input.seed).cwiseAbs().maxCoeff() <
                              3.0 * config_.maximum_joint_step_rad) {
    previous_velocity = last_qdot_;
  }
  previous_velocity = previous_velocity.cwiseMax(lower).cwiseMin(upper);
  addDiagonalTrackingCost(hessian, linear, previous_velocity, velocity_limit,
                          config_.continuity_weight);

  Vec7 posture_velocity =
      (dexhandNominal(config_, input.side) - input.seed) /
      config_.posture_time_constant_s;
  posture_velocity = posture_velocity.cwiseMax(-velocity_limit).cwiseMin(
      velocity_limit);
  addDiagonalTrackingCost(hessian, linear, posture_velocity, velocity_limit,
                          config_.posture_weight * posture_multiplier);

  // The source Dexhand implementation uses a finite-difference singularity
  // escape term. Keep it optional and fail soft if a probe is unavailable.
  // Reuse the gradient while the seed remains in a small local neighborhood:
  // this preserves the escape direction without paying for fourteen extra
  // Pinocchio samples on every 200 Hz cycle.
  if (singularity_activation > 0.0 &&
      config_.singularity_escape_weight > 0.0) {
    const double refresh_distance = std::max(
        0.02, 4.0 * config_.maximum_joint_step_rad);
    const bool refresh_gradient =
        !singularity_gradient_valid_ ||
        (input.seed - singularity_gradient_q_).cwiseAbs().maxCoeff() >
            refresh_distance;
    if (refresh_gradient) {
      constexpr double kFiniteDifference = 1.0e-4;
      Vec7 gradient = Vec7::Zero();
      bool gradient_valid = true;
      for (int joint = 0; joint < kArmDof; ++joint) {
        Vec7 positive = input.seed;
        Vec7 negative = input.seed;
        positive[joint] = std::min(
            input.limits.upper_position[joint], positive[joint] +
                                                     kFiniteDifference);
        negative[joint] = std::max(
            input.limits.lower_position[joint], negative[joint] -
                                                     kFiniteDifference);
        const double denominator_fd = positive[joint] - negative[joint];
        if (denominator_fd <= std::numeric_limits<double>::epsilon()) {
          continue;
        }
        try {
          const ArmKinematicSample positive_sample = input.evaluate(positive);
          const ArmKinematicSample negative_sample = input.evaluate(negative);
          if (!positive_sample.tcp_jacobian.allFinite() ||
              !negative_sample.tcp_jacobian.allFinite()) {
            gradient_valid = false;
            break;
          }
          gradient[joint] =
              (minimumSingularValue(positive_sample.tcp_jacobian) -
               minimumSingularValue(negative_sample.tcp_jacobian)) /
              denominator_fd;
        } catch (...) {
          gradient_valid = false;
          break;
        }
      }
      singularity_gradient_valid_ = gradient_valid && gradient.allFinite();
      if (singularity_gradient_valid_) {
        singularity_gradient_ = gradient;
        singularity_gradient_q_ = input.seed;
      }
    }
    if (singularity_gradient_valid_ && singularity_gradient_.norm() > 1.0e-10) {
      const Vec7 escape_velocity =
          singularity_activation * config_.singularity_escape_speed_rad_s *
          singularity_gradient_.normalized();
      addDiagonalTrackingCost(
          hessian, linear, escape_velocity, velocity_limit,
          config_.singularity_escape_weight * singularity_activation);
    }
  }

  hessian.diagonal().array() += 1.0e-12;
  hessian = 0.5 * (hessian + hessian.transpose());
  const BoxQpResult qp = solveBoxQp(
      hessian, linear, lower, upper, previous_velocity,
      config_.max_active_set_iterations, config_.active_set_tolerance);
  result.active_set_iterations = qp.iterations;
  result.active_bound_count = qp.active_bounds;
  if (!qp.converged || !qp.solution.allFinite()) {
    clearWarmStart();
    result.qp_rejected = true;
    result.detail = "dexhand_velocity_qp_solve_rejected";
    result.solve_time_us = std::chrono::duration<double, std::micro>(
                               Clock::now() - start)
                               .count();
    return result;
  }

  const Vec7 candidate = input.seed + input.dt * qp.solution;
  if (!candidate.allFinite() ||
      (candidate.array() < input.limits.lower_position.array()).any() ||
      (candidate.array() > input.limits.upper_position.array()).any()) {
    clearWarmStart();
    result.qp_rejected = true;
    result.detail = "dexhand_velocity_qp_candidate_rejected";
    result.solve_time_us = std::chrono::duration<double, std::micro>(
                               Clock::now() - start)
                               .count();
    return result;
  }

  result.qdot_qp = qp.solution;
  result.q_qp = candidate;
  result.linear_velocity_residual =
      sample.tcp_jacobian.topRows<3>() * qp.solution -
      result.desired_linear_velocity;
  result.angular_velocity_residual =
      sample.tcp_jacobian.bottomRows<3>() * qp.solution -
      result.desired_angular_velocity;
  result.cartesian_velocity_residual =
      std::sqrt(result.linear_velocity_residual.squaredNorm() +
                result.angular_velocity_residual.squaredNorm());
  result.qdot_max_ratio =
      (result.qdot_qp.cwiseAbs().array() / velocity_limit.array()).maxCoeff();
  result.solve_time_us = std::chrono::duration<double, std::micro>(
                             Clock::now() - start)
                             .count();

  if (ruckig_ != nullptr) {
    const DexhandVelocityRuckigResult trajectory =
        ruckig_->update(result.qdot_qp, input.dt);
    result.ruckig_enabled = true;
    result.ruckig_accepted = trajectory.accepted && !trajectory.held;
    result.ruckig_held = trajectory.held;
    result.ruckig_rejected = trajectory.rejected;
    result.velocity_ratio = trajectory.velocity_ratio;
    result.acceleration_ratio = trajectory.acceleration_ratio;
    result.jerk_ratio = trajectory.jerk_ratio;
    if (!trajectory.accepted) {
      clearWarmStart();
      result.ruckig_rejected = true;
      result.q = last_q_;
      result.qdot = last_qdot_;
      result.target_held = true;
      result.detail = "dexhand_velocity_ruckig_rejected";
      return result;
    }
    result.q = trajectory.state.q;
    result.qdot = trajectory.state.qdot;
    result.target_held = trajectory.held;
    result.accepted = true;
    result.detail = trajectory.held ? "dexhand_velocity_ruckig_held"
                                    : "dexhand_velocity_qp_solved_ruckig";
  } else {
    result.q = candidate;
    result.qdot = qp.solution;
    result.accepted = true;
    result.detail = "dexhand_velocity_qp_solved";
  }
  last_q_ = result.q;
  last_qdot_ = result.qdot;
  initialized_ = true;
  warm_start_valid_ = true;
  return result;
}

}  // namespace tianji_qp_ik
