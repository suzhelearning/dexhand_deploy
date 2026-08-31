#include "tianji_teleop/ik/tianji_official/tianji_official_arm_ik.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kProbeMaximumJointStepRad = 0.2 * kPi / 180.0;

tianji_teleop::ArmJointVector radians(
  std::initializer_list<double> degrees)
{
  if (degrees.size() != 7) {
    throw std::invalid_argument("probe joint vector must contain seven values");
  }
  tianji_teleop::ArmJointVector result;
  Eigen::Index index = 0;
  for (double value : degrees) {
    result[index++] = value * kPi / 180.0;
  }
  return result;
}

void check_round_trip(
  tianji_teleop::TianjiOfficialArmIk & solver,
  tianji_teleop::ArmSide side,
  const tianji_teleop::ArmJointVector & reference,
  const Eigen::Vector3d & elbow_direction = Eigen::Vector3d::Zero())
{
  const Eigen::Isometry3d target = solver.forward(side, reference);
  if (!target.matrix().allFinite()) {
    throw std::runtime_error("official FK returned a non-finite pose");
  }
  const tianji_teleop::IkResult result = solver.solve(
    side, target, reference, elbow_direction);
  if (!result.accepted || !result.converged) {
    throw std::runtime_error(
            "official FK -> IK round trip failed: " + result.status);
  }
  if ((result.joints_rad - reference).cwiseAbs().maxCoeff() > 1.0e-6) {
    throw std::runtime_error("official FK -> IK changed the reference branch");
  }
}

void check_common_step_limit(
  tianji_teleop::TianjiOfficialArmIk & solver)
{
  const auto target_joints =
    radians({21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68});
  auto current_joints = target_joints;
  current_joints[0] += 10.0 * kPi / 180.0;
  const auto target = solver.forward(
    tianji_teleop::ArmSide::kLeft, target_joints);
  const auto result = solver.solve(
    tianji_teleop::ArmSide::kLeft,
    target,
    current_joints,
    Eigen::Vector3d::Zero());
  if (!result.accepted || !result.joint_step_limited) {
    throw std::runtime_error("official adapter did not apply common step limit");
  }
  if (result.maximum_joint_step_rad > kProbeMaximumJointStepRad + 1.0e-10) {
    throw std::runtime_error("official adapter exceeded common step limit");
  }
}

void check_unreachable_target_backs_off_safely(
  tianji_teleop::TianjiOfficialArmIk & solver)
{
  const auto reference =
    radians({21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68});
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = Eigen::Vector3d(5.0, 0.0, 0.0);
  const auto result = solver.solve(
    tianji_teleop::ArmSide::kLeft,
    target,
    reference,
    Eigen::Vector3d::Zero());
  if (
    !result.accepted || !result.saturated ||
    !result.workspace_backoff_active ||
    !(result.workspace_backoff_fraction > 0.0 &&
    result.workspace_backoff_fraction < 1.0))
  {
    throw std::runtime_error(
            "official adapter did not back off unreachable target");
  }
  if (
    (result.joints_rad - reference).cwiseAbs().maxCoeff() >
    kProbeMaximumJointStepRad + 1.0e-10)
  {
    throw std::runtime_error(
            "official adapter exceeded step limit during workspace backoff");
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "usage: tianji_official_ik_probe LIBKINE MVKDCFG\n";
    return 2;
  }
  try {
    tianji_teleop::IkSettings settings;
    settings.arm_angle_gain = 0.8;
    // 覆盖纯手柄 0.20° 边界：proxy 必须无损地把该弧度值传给 worker，
    // 否则 worker 限幅会略大于主进程契约并被外层安全检查连续拒绝。
    settings.maximum_joint_step_rad = kProbeMaximumJointStepRad;
    settings.official_use_zsp = true;
    settings.official_left_nominal_rad =
      radians({55.0, -65.0, -70.0, -60.0, 60.0, 0.0, 0.0});
    settings.official_right_nominal_rad =
      radians({-55.0, -65.0, 70.0, -60.0, -60.0, 0.0, 0.0});
    tianji_teleop::TianjiOfficialArmIk solver(argv[1], argv[2], settings);
    check_round_trip(
      solver,
      tianji_teleop::ArmSide::kLeft,
      radians({21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68}));
    check_round_trip(
      solver,
      tianji_teleop::ArmSide::kRight,
      radians({-21.8, -41.0, 4.75, -63.67, -10.15, 14.72, -7.68}));
    // 这两个方向来自厂商 FK_NSP 在项目安全初始位返回矩阵的第一列，
    // 用于覆盖 zsp_para 类型 1 的真实 ABI 路径。
    check_round_trip(
      solver,
      tianji_teleop::ArmSide::kLeft,
      radians({55.0, -65.0, -70.0, -60.0, 60.0, 0.0, 0.0}),
      Eigen::Vector3d(0.45638698, -0.74604902, -0.48489358));
    check_round_trip(
      solver,
      tianji_teleop::ArmSide::kRight,
      radians({-55.0, -65.0, 70.0, -60.0, -60.0, 0.0, 0.0}),
      Eigen::Vector3d(0.45638698, 0.74604902, -0.48489358));
    check_common_step_limit(solver);
    check_unreachable_target_backs_off_safely(solver);
    std::cout << "tianji official IK adapter round trip passed\n";
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
}
