#pragma once
#include "session_commands.hpp"
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
namespace tianji_control {
// Canonical protocol order, left then right. Missing updates hold, never zero.
using HandJoints=std::array<double,20>;
using HandPair=std::array<HandJoints,2>;
using HandUpdates=std::array<std::optional<HandJoints>,2>;
// Native simulation only. Factory, calls and destruction share the scheduler
// thread. Inputs are final coordinator commands, never an independent authority.
class SimulationEndpoint {
 public:
  virtual ~SimulationEndpoint()=default;
  virtual void apply(const ArmPair&)=0;
  // This is an execution interface, not command admission. The session owner
  // must validate source, epoch, freshness, phase and limits before calling it.
  virtual void apply_frame(const ArmPair& arms,const HandUpdates& hands) {
    if(hands[0] || hands[1]) throw std::invalid_argument("simulation hands are disabled");
    apply(arms);
  }
  virtual std::optional<HandPair> hand_feedback() { return std::nullopt; }
  virtual ArmPair feedback()=0;
  virtual std::optional<std::array<double,2>> height_reference() { return std::nullopt; }
  virtual std::optional<std::array<std::array<double,2>,2>> forward_reference() { return std::nullopt; }
};
using SimulationFactory=std::function<std::unique_ptr<SimulationEndpoint>()>;
}
