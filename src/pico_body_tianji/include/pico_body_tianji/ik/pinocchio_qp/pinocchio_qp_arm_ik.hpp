#pragma once

#include "pico_body_tianji/ik/arm_ik_solver.hpp"

#include <memory>
#include <string>

namespace pico_body_tianji
{

// 基于 Pinocchio 运动学的一阶速度级约束 QP。末端任务、连续性、自然姿态
// 与奇异逃逸均作为软代价；关节位置和速度作为 box 硬约束。
class PinocchioQpArmIk final : public ArmIkSolver
{
public:
  explicit PinocchioQpArmIk(
    const std::string & urdf_path,
    const IkSettings & settings = IkSettings{});
  ~PinocchioQpArmIk() override;

  PinocchioQpArmIk(PinocchioQpArmIk &&) noexcept;
  PinocchioQpArmIk & operator=(PinocchioQpArmIk &&) noexcept;

  PinocchioQpArmIk(const PinocchioQpArmIk &) = delete;
  PinocchioQpArmIk & operator=(const PinocchioQpArmIk &) = delete;

  Eigen::Isometry3d forward(
    ArmSide side,
    const ArmJointVector & joints_rad) const override;

  IkResult solve(
    ArmSide side,
    const Eigen::Isometry3d & target_pose,
    const ArmJointVector & current_joints_rad,
    const Eigen::Vector3d & elbow_ik_direction) const override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pico_body_tianji
