#pragma once

#include "pico_body_tianji/ik/arm_ik_solver.hpp"

#include <memory>
#include <string>

namespace pico_body_tianji
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

  Eigen::Vector3d elbow_ik_direction(
    ArmSide side,
    const ArmJointVector & joints_rad) const;

  IkResult solve(
    ArmSide side,
    const Eigen::Isometry3d & target_pose,
    const ArmJointVector & current_joints_rad,
    const Eigen::Vector3d & smpl_ik_direction) const override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pico_body_tianji
