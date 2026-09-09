#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/types.hpp"

#include <ruckig/ruckig.hpp>

#include <string_view>

namespace tianji_qp_ik {

struct DexhandVelocityRuckigResult {
  bool accepted{false};
  bool held{false};
  // True when this update rejected the requested target/candidate.  A
  // transient rejection is still returned as an accepted hold after the
  // limiter is re-anchored, so callers can distinguish recovery from an
  // ordinary accepted sample.
  bool rejected{false};
  ArmMotionState state;
  Vec7 jerk{Vec7::Zero()};
  double velocity_ratio{0.0};
  double acceleration_ratio{0.0};
  double jerk_ratio{0.0};
  std::string_view detail{"not_updated"};
};

class DexhandVelocityRuckig7 {
 public:
  DexhandVelocityRuckig7(DexhandVelocityQpConfig config, ArmLimits limits,
                         double initial_dt);

  bool reset(const ArmMotionState& state) noexcept;
  DexhandVelocityRuckigResult update(const Vec7& target_velocity,
                                     double dt);
  const ArmMotionState& state() const noexcept { return state_; }
  ruckig::ControlInterface controlInterface() const noexcept {
    return input_.control_interface;
  }
  ruckig::Synchronization synchronization() const noexcept {
    return input_.synchronization;
  }

 private:
  Vec7 velocityLimit() const noexcept;
  DexhandVelocityQpConfig config_;
  ArmLimits limits_;
  ruckig::Ruckig<kArmDof> otg_;
  ruckig::InputParameter<kArmDof> input_;
  ruckig::OutputParameter<kArmDof> output_;
  ArmMotionState state_;
  bool initialized_{false};
};

}  // namespace tianji_qp_ik
