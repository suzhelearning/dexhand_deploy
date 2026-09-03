#include "tianji_teleop/control/joint_trajectory_limiter.hpp"
#include "tianji_teleop/ik/pinocchio_qp/pinocchio_qp_arm_ik.hpp"

#include <iostream>
#include <stdexcept>

namespace
{

void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

}  // namespace

int main(int argc, char ** argv)
{
  using namespace tianji_teleop;
  JointTrajectoryLimits limits;
  limits.lower_position = ArmJointVector::Constant(-1.0);
  limits.upper_position = ArmJointVector::Constant(1.0);
  limits.maximum_velocity = ArmJointVector::Constant(0.8);
  limits.maximum_acceleration = ArmJointVector::Constant(7.854);
  limits.maximum_jerk = ArmJointVector::Constant(600.0);
  JointTrajectoryLimiter7 limiter(limits, 0.005);
  require(limiter.reset(ArmMotionState{}), "initial Ruckig reset failed");

  const ArmJointVector target_velocity = ArmJointVector::Constant(0.2);
  JointTrajectoryResult result;
  for (int tick = 0; tick < 100; ++tick) {
    result = limiter.update(target_velocity);
    require(result.accepted, "Ruckig rejected a valid target");
    require(result.velocity_ratio <= 1.0 + limits.validation_tolerance, "velocity limit exceeded");
    require(result.acceleration_ratio <= 1.0 + limits.validation_tolerance, "acceleration limit exceeded");
    require(result.jerk_ratio <= 1.0 + limits.validation_tolerance, "jerk limit exceeded");
  }
  for (int tick = 0; tick < 100; ++tick) {
    result = limiter.update(ArmJointVector::Zero());
    require(result.accepted, "Ruckig rejected a valid stop target");
  }
  require(result.state.velocity.cwiseAbs().maxCoeff() < 1.0e-8, "Ruckig did not stop");

  ConsecutiveFailureWindow failures(150000000);
  require(!failures.failed(1000000000), "failure window expired immediately");
  require(!failures.failed(1149999999), "failure window expired early");
  require(failures.failed(1150000000), "failure window did not expire");
  failures.recovered();
  require(failures.count() == 0, "failure window did not recover");

  if (argc == 2) {
    IkSettings settings;
    settings.control_period_s = 0.005;
    settings.maximum_joint_step_rad = 0.00596902599;
    PinocchioQpArmIk solver(argv[1], settings);
    ArmJointVector home;
    home << 0.9599310886, -1.1344640138, -1.2217304764,
      -1.0471975512, 1.0471975512, 0.0, 0.0;
    limits.lower_position << -2.8797932658, -2.0071286398, -2.8797932658,
      -2.4434609528, -2.8797932658, -0.9599310886, -1.4835298642;
    limits.upper_position << 2.8797932658, 2.0071286398, 2.8797932658,
      0.9599310886, 2.8797932658, 0.9599310886, 1.4835298642;
    JointTrajectoryLimiter7 cascade(limits, settings.control_period_s);
    ArmMotionState initial;
    initial.position = home;
    require(cascade.reset(initial), "QP/Ruckig cascade reset failed");
    Eigen::Isometry3d target_pose = solver.forward(ArmSide::kLeft, home);
    target_pose.translation().x() += 0.03;
    for (int tick = 0; tick < 1000; ++tick) {
      const auto ik = solver.solve(
        ArmSide::kLeft, target_pose, cascade.state().position,
        Eigen::Vector3d::UnitZ());
      require(ik.accepted, "QP rejected reachable target");
      const ArmJointVector target_velocity =
        (ik.joints_rad - cascade.state().position) / settings.control_period_s;
      result = cascade.update(target_velocity);
      require(result.accepted, "Ruckig rejected QP output");
    }
    const Eigen::Isometry3d achieved = solver.forward(
      ArmSide::kLeft, cascade.state().position);
    const double position_error =
      (achieved.translation() - target_pose.translation()).norm();
    std::cerr << "qp_ruckig_position_error_m=" << position_error << '\n';
    require(position_error < 0.002,
      "200 Hz QP/Ruckig cascade did not track reachable target");

    PinocchioQpArmIk moving_solver(argv[1], settings);
    require(cascade.reset(initial), "moving-target cascade reset failed");
    target_pose = moving_solver.forward(ArmSide::kLeft, home);
    for (int tick = 0; tick < 400; ++tick) {
      target_pose.translation().x() += 0.036 * settings.control_period_s;
      const auto ik = moving_solver.solve(
        ArmSide::kLeft, target_pose, cascade.state().position,
        Eigen::Vector3d::UnitZ());
      require(ik.accepted, "QP rejected moving target");
      result = cascade.update(
        (ik.joints_rad - cascade.state().position) /
        settings.control_period_s);
      require(result.accepted, "Ruckig rejected moving-target QP output");
    }
    const Eigen::Isometry3d moving_achieved = moving_solver.forward(
      ArmSide::kLeft, cascade.state().position);
    const double moving_error =
      (moving_achieved.translation() - target_pose.translation()).norm();
    std::cerr << "qp_ruckig_moving_error_m=" << moving_error << '\n';
    require(moving_error < 0.005,
      "200 Hz QP/Ruckig cascade lags the Regrind moving target");

  }

  std::cout << "joint trajectory limiter probe passed\n";
  return 0;
}
