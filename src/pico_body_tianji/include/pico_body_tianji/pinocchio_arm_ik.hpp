#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>

namespace pico_body_tianji
{

enum class ArmSide
{
  kLeft,
  kRight,
};

using ArmJointVector = Eigen::Matrix<double, 7, 1>;

struct IkSettings
{
  int max_iterations{24};
  double position_tolerance_m{1.0e-3};
  double orientation_tolerance_rad{1.0e-2};
  double minimum_damping{1.0e-3};
  double maximum_damping{1.5e-1};
  double singular_value_threshold{5.0e-2};
  double maximum_iteration_step_rad{8.0e-2};
  double maximum_joint_step_rad{3.0 * 3.14159265358979323846 / 180.0};
  double joint_limit_margin_rad{5.0 * 3.14159265358979323846 / 180.0};
  double arm_angle_gain{0.0};
  double arm_angle_tolerance_rad{
    2.0 * 3.14159265358979323846 / 180.0};
  double arm_angle_finite_difference_rad{1.0e-4};
  double arm_angle_merit_weight{1.0e-3};
  double nullspace_damping{1.0e-3};
  double joint_center_gain{0.0};
  double joint_center_activation_margin_rad{
    15.0 * 3.14159265358979323846 / 180.0};
  double joint_center_merit_weight{1.0e-3};
  double singularity_avoidance_gain{0.0};
  double singularity_finite_difference_rad{1.0e-4};
  double singularity_merit_weight{1.0e-2};
};

struct IkResult
{
  ArmJointVector joints_rad{ArmJointVector::Zero()};
  Eigen::Isometry3d achieved_pose{Eigen::Isometry3d::Identity()};
  bool accepted{false};
  bool converged{false};
  bool saturated{false};
  bool joint_step_limited{false};
  bool singularity_active{false};
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
  double minimum_singular_value{0.0};
  double damping{0.0};
  double arm_angle_error_rad{0.0};
  double minimum_limit_margin_rad{0.0};
  double maximum_joint_step_rad{0.0};
  std::string status;
};

class PinocchioArmIk
{
public:
  explicit PinocchioArmIk(
    const std::string & urdf_path,
    const IkSettings & settings = IkSettings{});
  ~PinocchioArmIk();

  PinocchioArmIk(PinocchioArmIk &&) noexcept;
  PinocchioArmIk & operator=(PinocchioArmIk &&) noexcept;

  PinocchioArmIk(const PinocchioArmIk &) = delete;
  PinocchioArmIk & operator=(const PinocchioArmIk &) = delete;

  Eigen::Isometry3d forward(
    ArmSide side,
    const ArmJointVector & joints_rad) const;

  Eigen::Vector3d elbow_ik_direction(
    ArmSide side,
    const ArmJointVector & joints_rad) const;

  IkResult solve(
    ArmSide side,
    const Eigen::Isometry3d & target_pose,
    const ArmJointVector & current_joints_rad,
    const Eigen::Vector3d & smpl_ik_direction) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pico_body_tianji
