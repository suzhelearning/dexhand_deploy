#include "tianji_teleop/ik/pinocchio_qp/pinocchio_qp_arm_ik.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

constexpr double kPi = 3.14159265358979323846;

double radians(double degrees)
{
  return degrees * kPi / 180.0;
}

tianji_teleop::ArmJointVector joints_from_degrees(
  std::initializer_list<double> degrees)
{
  if (degrees.size() != 7) {
    throw std::invalid_argument("测试关节必须包含 7 个元素");
  }
  tianji_teleop::ArmJointVector result;
  Eigen::Index index = 0;
  for (const double value : degrees) {
    result[index++] = radians(value);
  }
  return result;
}

struct Scenario
{
  std::string name;
  Eigen::Vector3d translation;
  Eigen::Vector3d rotation_axis;
  double rotation_rad;
  bool require_convergence;
};

void run_side(
  tianji_teleop::PinocchioQpArmIk & solver,
  tianji_teleop::ArmSide side,
  const tianji_teleop::ArmJointVector & home,
  std::vector<double> & solve_times_us)
{
  const std::vector<Scenario> scenarios{
    {"hold", Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 0.0, true},
    {"x+3cm", {0.03, 0.0, 0.0}, Eigen::Vector3d::UnitX(), 0.0, true},
    {"y+3cm", {0.0, 0.03, 0.0}, Eigen::Vector3d::UnitY(), 0.0, true},
    {"z+3cm", {0.0, 0.0, 0.03}, Eigen::Vector3d::UnitZ(), 0.0, true},
    {"roll+10deg", Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), radians(10.0), true},
    {"combined", {0.02, -0.02, 0.02}, Eigen::Vector3d::UnitZ(), radians(8.0), true},
    {"unreachable", {0.80, 0.0, 0.0}, Eigen::Vector3d::UnitY(), radians(30.0), false},
  };

  for (const Scenario & scenario : scenarios) {
    tianji_teleop::ArmJointVector current = home;
    Eigen::Isometry3d target = solver.forward(side, home);
    target.translation() += scenario.translation;
    target.linear() *= Eigen::AngleAxisd(
      scenario.rotation_rad, scenario.rotation_axis).toRotationMatrix();
    tianji_teleop::IkResult last;
    bool converged = false;
    for (int tick = 0; tick < 500; ++tick) {
      const auto start = std::chrono::steady_clock::now();
      last = solver.solve(
        side, target, current, Eigen::Vector3d::Zero());
      const auto end = std::chrono::steady_clock::now();
      solve_times_us.push_back(
        std::chrono::duration<double, std::micro>(end - start).count());
      if (!last.accepted || !last.joints_rad.allFinite()) {
        throw std::runtime_error(
                scenario.name + " 返回了不可接受的 QP 解：" + last.status);
      }
      if ((last.joints_rad - current).cwiseAbs().maxCoeff() > radians(0.68) + 1.0e-9) {
        throw std::runtime_error(scenario.name + " 超过公共单步限制");
      }
      if (last.minimum_limit_margin_rad < -1.0e-8) {
        throw std::runtime_error(scenario.name + " 越过安全关节限位");
      }
      current = last.joints_rad;
      converged = last.converged;
      if (converged) {
        break;
      }
    }
    if (scenario.require_convergence && !converged) {
      throw std::runtime_error(
              scenario.name + " 未收敛：位置误差=" +
              std::to_string(1000.0 * last.position_error_m) +
              " mm，姿态误差=" +
              std::to_string(last.orientation_error_rad * 180.0 / kPi) +
              " deg，状态=" + last.status);
    }
    std::cout << std::left << std::setw(14) << scenario.name
              << " position_mm=" << std::setw(10)
              << 1000.0 * last.position_error_m
              << " orientation_deg=" << std::setw(10)
              << last.orientation_error_rad * 180.0 / kPi
              << " sigma_min=" << std::setw(10)
              << last.minimum_singular_value
              << " status=" << last.status << '\n';

    if (!scenario.require_convergence) {
      const Eigen::Isometry3d recovery_target = solver.forward(side, home);
      for (int tick = 0; tick < 800 && !last.converged; ++tick) {
        const auto start = std::chrono::steady_clock::now();
        last = solver.solve(
          side, recovery_target, current, Eigen::Vector3d::Zero());
        const auto end = std::chrono::steady_clock::now();
        solve_times_us.push_back(
          std::chrono::duration<double, std::micro>(end - start).count());
        if (!last.accepted || !last.joints_rad.allFinite()) {
          throw std::runtime_error(
                  scenario.name + " 后无法恢复：" + last.status);
        }
        current = last.joints_rad;
      }
      if (!last.converged) {
        throw std::runtime_error(scenario.name + " 后 800 tick 内未恢复");
      }
      std::cout << std::left << std::setw(14) << "recovery"
                << " position_mm=" << std::setw(10)
                << 1000.0 * last.position_error_m
                << " orientation_deg=" << std::setw(10)
                << last.orientation_error_rad * 180.0 / kPi
                << " sigma_min=" << std::setw(10)
                << last.minimum_singular_value
                << " status=" << last.status << '\n';
    }
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc != 2) {
      std::cerr << "用法：pinocchio_qp_ik_probe ROBOT.urdf\n";
      return 2;
    }
    tianji_teleop::IkSettings settings;
    settings.control_period_s = 1.0 / 90.0;
    settings.maximum_joint_step_rad = radians(0.68);
    settings.position_tolerance_m = 1.0e-3;
    settings.orientation_tolerance_rad = radians(0.6);
    settings.arm_angle_gain = 0.0;
    settings.qp_left_nominal_rad = joints_from_degrees(
      {55.0, -65.0, -70.0, -60.0, 60.0, 0.0, 0.0});
    settings.qp_right_nominal_rad = joints_from_degrees(
      {-55.0, -65.0, 70.0, -60.0, -60.0, 0.0, 0.0});
    tianji_teleop::PinocchioQpArmIk solver(argv[1], settings);
    std::vector<double> solve_times_us;
    std::cout << "left arm\n";
    run_side(
      solver,
      tianji_teleop::ArmSide::kLeft,
      settings.qp_left_nominal_rad,
      solve_times_us);
    std::cout << "right arm\n";
    run_side(
      solver,
      tianji_teleop::ArmSide::kRight,
      settings.qp_right_nominal_rad,
      solve_times_us);
    std::sort(solve_times_us.begin(), solve_times_us.end());
    const std::size_t p99_index = std::min(
      solve_times_us.size() - 1,
      static_cast<std::size_t>(0.99 * solve_times_us.size()));
    std::cout << "solve_us_p50="
              << solve_times_us[solve_times_us.size() / 2]
              << " solve_us_p99=" << solve_times_us[p99_index]
              << " solve_us_max=" << solve_times_us.back() << '\n';
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "QP IK probe 失败：" << exception.what() << '\n';
    return 1;
  }
}
