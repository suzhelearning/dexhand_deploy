#pragma once

#include "tianji_teleop/ik/arm_ik_solver.hpp"

#include <memory>
#include <string>

namespace tianji_teleop
{

class PinocchioArmIk final : public ArmIkSolver
{
public:
  explicit PinocchioArmIk(
    const std::string & urdf_path,
    const IkSettings & settings = IkSettings{});
  ~PinocchioArmIk() override;

  PinocchioArmIk(PinocchioArmIk &&) noexcept;
  PinocchioArmIk & operator=(PinocchioArmIk &&) noexcept;

  PinocchioArmIk(const PinocchioArmIk &) = delete;
  PinocchioArmIk & operator=(const PinocchioArmIk &) = delete;

  Eigen::Isometry3d forward(
    ArmSide side,
    const ArmJointVector & joints_rad) const override;

  Eigen::Vector3d elbow_reference_direction(
    ArmSide side,
    const ArmJointVector & joints_rad) const;

  IkResult solve(
    ArmSide side,
    const Eigen::Isometry3d & target_pose,
    const ArmJointVector & current_joints_rad,
    const Eigen::Vector3d & elbow_reference_direction) const override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tianji_teleop
