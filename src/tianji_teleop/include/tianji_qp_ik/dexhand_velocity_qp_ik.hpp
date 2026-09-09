#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/dexhand_velocity_ruckig.hpp"
#include "tianji_qp_ik/kinematics.hpp"
#include "tianji_qp_ik/types.hpp"

#include <memory>
#include <string_view>

namespace tianji_qp_ik {

enum class DexhandNominalMode {
  kConfigured,
  kFaithfulZero,
};

struct DexhandVelocityQpIkInput {
  ArmSide side{ArmSide::kLeft};
  Pose target;
  bool target_valid{false};
  bool target_stale{false};
  // Optional desired Cartesian twist prepared by the caller's reference
  // generator.  When valid, DexhandVelocityQpIk7 uses it as the complete
  // task velocity request instead of deriving one from pose error/time
  // constants.  The controller fills this from Cartesian OTG + servo.
  Vec6 desired_twist{Vec6::Zero()};
  bool desired_twist_valid{false};
  Vec7 seed{Vec7::Zero()};
  Vec7 qdot_previous{Vec7::Zero()};
  ArmLimits limits;
  KinematicsEvaluator evaluate;
  double dt{0.005};
};

struct DexhandVelocityQpResult {
  bool accepted{false};
  bool target_held{false};
  bool qp_rejected{false};
  Vec7 qdot_qp{Vec7::Zero()};
  Vec7 q_qp{Vec7::Zero()};
  // Final command fields equal the raw fields in direct mode and contain the
  // post-limiter state when the Velocity-interface Ruckig stage is enabled.
  Vec7 q{Vec7::Zero()};
  Vec7 qdot{Vec7::Zero()};
  Vec6 desired_twist{Vec6::Zero()};
  Eigen::Vector3d desired_linear_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d desired_angular_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_velocity_residual{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_residual{Eigen::Vector3d::Zero()};
  double cartesian_velocity_residual{0.0};
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
  double minimum_singular_value{0.0};
  double qdot_max_ratio{0.0};
  int active_set_iterations{0};
  int active_bound_count{0};
  double solve_time_us{0.0};
  DexhandNominalMode nominal_mode{DexhandNominalMode::kConfigured};
  bool ruckig_enabled{false};
  bool ruckig_accepted{false};
  bool ruckig_held{false};
  bool ruckig_rejected{false};
  double velocity_ratio{0.0};
  double acceleration_ratio{0.0};
  double jerk_ratio{0.0};
  std::string_view detail{"not_solved"};
};

class DexhandVelocityQpIk7 {
 public:
  DexhandVelocityQpIk7(DexhandVelocityQpConfig config, ArmLimits limits,
                       double initial_dt);

  DexhandVelocityQpResult solve(const DexhandVelocityQpIkInput& input);
  void reset(const ArmMotionState& state) noexcept;
  IkAlgorithm algorithm() const noexcept { return IkAlgorithm::kPicoEeDexhandQp; }

 private:
  void clearWarmStart() noexcept;

  DexhandVelocityQpConfig config_;
  ArmLimits limits_;
  double initial_dt_{0.005};
  Vec7 last_q_{Vec7::Zero()};
  Vec7 last_qdot_{Vec7::Zero()};
  Vec7 singularity_gradient_{Vec7::Zero()};
  Vec7 singularity_gradient_q_{Vec7::Zero()};
  std::unique_ptr<DexhandVelocityRuckig7> ruckig_;
  bool initialized_{false};
  bool warm_start_valid_{false};
  bool singularity_gradient_valid_{false};
};

}  // namespace tianji_qp_ik
