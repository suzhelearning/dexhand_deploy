#pragma once
#include "tianji_qp_ik/types.hpp"
namespace tianji_qp_ik {
enum class IkAlgorithm { kPicoEeDexhandQp };
struct DexhandVelocityQpConfig {
  // When enabled, the controller may supply a Cartesian-reference servo
  // command (OTG twist plus pose feedback) directly to the QP.  False keeps
  // the original Dexhand pose-error/time-constant law unchanged.
  bool cartesian_reference_servo_enabled{false};
  bool dexhand_faithful_nominal{false};
  Vec7 nominal_left{
      (Vec7() << 1.10, -0.60, -1.52, -1.10, 0.0, 0.0, 0.0).finished()};
  Vec7 nominal_right{
      (Vec7() << -1.10, -0.60, 1.52, -1.10, 0.0, 0.0, 0.0).finished()};
  double position_time_constant_s{0.30};
  double orientation_time_constant_s{0.40};
  double max_linear_speed_m_s{0.25};
  double max_angular_speed_rad_s{1.0};
  Vec7 qp_joint_velocity_limits_rad_s{
      (Vec7() << 0.9599310886, 0.9599310886, 0.9599310886,
       0.9599310886, 0.9599310886, 0.9599310886, 0.9599310886)
          .finished()};
  double position_weight{1.0};
  double orientation_weight{0.45};
  double velocity_regularization_weight{0.02};
  double continuity_weight{0.06};
  double posture_weight{0.008};
  double posture_time_constant_s{2.5};
  double maximum_joint_step_rad{0.00596902599};
  double joint_limit_activation_margin_rad{0.2617993878};
  double joint_limit_velocity_damper_gain{4.0};
  double singular_value_threshold{0.05};
  double singularity_critical_threshold{0.015};
  double singularity_orientation_scale{0.15};
  double singularity_posture_multiplier{8.0};
  double singularity_velocity_multiplier{4.0};
  double singularity_escape_weight{0.03};
  double singularity_escape_speed_rad_s{0.15};
  int max_active_set_iterations{48};
  double active_set_tolerance{1.0e-9};
  bool ruckig_enabled{false};
  Vec7 ruckig_max_velocity_rad_s{
      (Vec7() << 0.8, 0.8, 1.0, 1.0, 1.2, 1.2, 1.2).finished()};
  Vec7 ruckig_max_acceleration_rad_s2{
      (Vec7() << 7.854, 7.854, 15.708, 15.708, 15.708, 15.708, 15.708)
          .finished()};
  Vec7 ruckig_max_jerk_rad_s3{
      (Vec7() << 600.0, 600.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0)
          .finished()};
  double ruckig_validation_tolerance{1.0e-8};
};


inline Vec7 dexhandNominal(const DexhandVelocityQpConfig& c, ArmSide s) noexcept {
 if(c.dexhand_faithful_nominal) return Vec7::Zero();
 return s == ArmSide::kLeft ? c.nominal_left : c.nominal_right;
}
}
