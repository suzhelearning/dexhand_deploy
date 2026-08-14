#pragma once

#include "pico_body_tianji/ik/arm_ik_solver.hpp"

#include <memory>
#include <string>

namespace pico_body_tianji
{

// 天机官方 libKine 的可选运行时适配器。构造时才会 dlopen 厂商库，因此
// 选择 pinocchio_cpp 时进程不依赖也不会加载 libKine。
class TianjiOfficialArmIk final : public ArmIkSolver
{
public:
  TianjiOfficialArmIk(
    const std::string & library_path,
    const std::string & config_path,
    const IkSettings & settings = IkSettings{});
  ~TianjiOfficialArmIk() override;

  TianjiOfficialArmIk(TianjiOfficialArmIk &&) noexcept;
  TianjiOfficialArmIk & operator=(TianjiOfficialArmIk &&) noexcept;

  TianjiOfficialArmIk(const TianjiOfficialArmIk &) = delete;
  TianjiOfficialArmIk & operator=(const TianjiOfficialArmIk &) = delete;

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
