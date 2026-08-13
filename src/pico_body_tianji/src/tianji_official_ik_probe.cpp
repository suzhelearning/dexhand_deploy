#include "pico_body_tianji/tianji_official_arm_ik.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

constexpr double kPi = 3.14159265358979323846;

pico_body_tianji::ArmJointVector radians(
  std::initializer_list<double> degrees)
{
  if (degrees.size() != 7) {
    throw std::invalid_argument("probe joint vector must contain seven values");
  }
  pico_body_tianji::ArmJointVector result;
  Eigen::Index index = 0;
  for (double value : degrees) {
    result[index++] = value * kPi / 180.0;
  }
  return result;
}

void check_round_trip(
  pico_body_tianji::TianjiOfficialArmIk & solver,
  pico_body_tianji::ArmSide side,
  const pico_body_tianji::ArmJointVector & reference,
  const Eigen::Vector3d & elbow_direction = Eigen::Vector3d::Zero())
{
  const Eigen::Isometry3d target = solver.forward(side, reference);
  if (!target.matrix().allFinite()) {
    throw std::runtime_error("official FK returned a non-finite pose");
  }
  const pico_body_tianji::IkResult result = solver.solve(
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
  pico_body_tianji::TianjiOfficialArmIk & solver)
{
  const auto target_joints =
    radians({21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68});
  auto current_joints = target_joints;
  current_joints[0] += 10.0 * kPi / 180.0;
  const auto target = solver.forward(
    pico_body_tianji::ArmSide::kLeft, target_joints);
  const auto result = solver.solve(
    pico_body_tianji::ArmSide::kLeft,
    target,
    current_joints,
    Eigen::Vector3d::Zero());
  if (!result.accepted || !result.joint_step_limited) {
    throw std::runtime_error("official adapter did not apply common step limit");
  }
  if (result.maximum_joint_step_rad > 3.0 * kPi / 180.0 + 1.0e-10) {
    throw std::runtime_error("official adapter exceeded common step limit");
  }
}

void check_unreachable_target_is_rejected(
  pico_body_tianji::TianjiOfficialArmIk & solver)
{
  const auto reference =
    radians({21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68});
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = Eigen::Vector3d(5.0, 0.0, 0.0);
  const auto result = solver.solve(
    pico_body_tianji::ArmSide::kLeft,
    target,
    reference,
    Eigen::Vector3d::Zero());
  if (result.accepted || !result.saturated) {
    throw std::runtime_error("official adapter accepted unreachable target");
  }
  if ((result.joints_rad - reference).cwiseAbs().maxCoeff() > 1.0e-12) {
    throw std::runtime_error("official adapter moved after rejected target");
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
    pico_body_tianji::IkSettings settings;
    settings.arm_angle_gain = 0.8;
    pico_body_tianji::TianjiOfficialArmIk solver(argv[1], argv[2], settings);
    check_round_trip(
      solver,
      pico_body_tianji::ArmSide::kLeft,
      radians({21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68}));
    check_round_trip(
      solver,
      pico_body_tianji::ArmSide::kRight,
      radians({-21.8, -41.0, 4.75, -63.67, -10.15, 14.72, -7.68}));
    // 这两个方向来自厂商 FK_NSP 在项目安全初始位返回矩阵的第一列，
    // 用于覆盖 zsp_para 类型 1 的真实 ABI 路径。
    check_round_trip(
      solver,
      pico_body_tianji::ArmSide::kLeft,
      radians({55.0, -65.0, -70.0, -60.0, 60.0, 0.0, 0.0}),
      Eigen::Vector3d(0.45638698, -0.74604902, -0.48489358));
    check_round_trip(
      solver,
      pico_body_tianji::ArmSide::kRight,
      radians({-55.0, -65.0, 70.0, -60.0, -60.0, 0.0, 0.0}),
      Eigen::Vector3d(0.45638698, 0.74604902, -0.48489358));
    check_common_step_limit(solver);
    check_unreachable_target_is_rejected(solver);
    std::cout << "tianji official IK adapter round trip passed\n";
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
}
