#pragma once
#include "hand_reset_endpoint.hpp"
#include "../hand/scheduler.hpp"
namespace tianji_control {
// Concurrent reset is serialized by the concrete owner, not by the caller.
// phase/submit/pop are bounded non-IPC operations on the main session thread.
class HandPipelineEndpoint:public HandResetEndpoint {
 public:
  virtual bool phase(std::int64_t,tianji_hand::HandPhase)=0;
  virtual bool submit(const tianji_hand::HandInput&)=0;
  virtual bool pop(tianji_hand::HandCommand&)=0;
  virtual std::string failure() const=0;
};
}
