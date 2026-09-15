#pragma once
#include "hand_reset_endpoint.hpp"
#include "../hand/worker_client.hpp"
namespace tianji_control {
class NativeHandResetEndpoint final:public HandResetEndpoint {
 public:
  NativeHandResetEndpoint(const std::vector<std::string>& command,int timeout_ms)
    :client_(command,timeout_ms){}
  void reset(std::int64_t epoch) override {client_.reset_at_idle(epoch);}
  std::int64_t epoch() const override {return client_.epoch();}
  void request_stop() noexcept override {client_.request_stop();}
 private:
  tianji_hand::NativeHandWorkerClient client_;
};
}
