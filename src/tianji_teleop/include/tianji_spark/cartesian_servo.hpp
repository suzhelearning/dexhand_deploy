#pragma once

#include "tianji_spark/config.hpp"
#include "tianji_spark/target_manager.hpp"
#include "tianji_spark/types.hpp"

namespace tianji_spark {

Vec6 cartesianServoTwist(const CartesianServoConfig& config, const Pose& desired,
                         const Pose& current,
                         const Vec6& target_twist = Vec6::Zero());

Vec6 cartesianReferenceServoTwist(const CartesianServoConfig& config,
                                  const CartesianReference& reference,
                                  const Pose& measured);

}  // namespace tianji_spark
