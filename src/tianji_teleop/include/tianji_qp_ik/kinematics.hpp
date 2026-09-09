#pragma once
#include "tianji_qp_ik/types.hpp"
#include <functional>
namespace tianji_qp_ik {
struct ArmKinematicSample { Pose tcp_pose; Mat67 tcp_jacobian{Mat67::Zero()}; };
using KinematicsEvaluator = std::function<ArmKinematicSample(const Vec7&)>;
}
