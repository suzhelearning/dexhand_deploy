#include "tianji_qp_ik/dexhand_velocity_qp_ik.hpp"
#include <iostream>
#include <stdexcept>
int main() {
  using namespace tianji_qp_ik;
  ArmLimits limits;
  limits.lower_position.setConstant(-1); limits.upper_position.setConstant(1);
  limits.velocity.setConstant(2);
  DexhandVelocityQpConfig config;
  config.posture_weight = 0; config.singularity_escape_weight = 0;
  DexhandVelocityQpIk7 solver(config, limits, .005);
  DexhandVelocityQpIkInput input;
  input.target_valid = true; input.limits = limits;
  input.target.position.x() = .03;
  input.evaluate = [](const Vec7& q) {
    ArmKinematicSample s; s.tcp_pose.position = q.head<3>();
    s.tcp_jacobian.leftCols<6>().setIdentity(); return s;
  };
  for (int i=0;i<500;++i) {
    auto result = solver.solve(input);
    if (!result.accepted || result.ruckig_enabled ||
        (result.q-result.q_qp).norm() != 0 ||
        (result.q-input.seed).cwiseAbs().maxCoeff() > config.maximum_joint_step_rad+1e-10)
      throw std::runtime_error("direct QP contract failed");
    input.seed = result.q; input.qdot_previous = result.qdot;
  }
  if ((input.seed.head<3>()-input.target.position).norm() > .001)
    throw std::runtime_error("convergence failed");
  input.target.rotation.setZero();
  if (solver.solve(input).accepted) throw std::runtime_error("invalid rotation accepted");
  std::cout << "DexhandVelocityQpIk7 direct core passed\n";
}
