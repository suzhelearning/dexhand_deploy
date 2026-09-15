#pragma once
#include "ik_endpoint.hpp"
#include "worker_reset_client.hpp"
namespace tianji_control {
class WorkerIkEndpoint final:public IkEndpoint {
 public:
  WorkerIkEndpoint(const std::vector<std::string>& command,const std::string& prefix,
                   const std::string& algorithm,int timeout_ms,bool deterministic=false)
    :client_(command,prefix,algorithm,timeout_ms,true,deterministic) {}
  WorkerResult step(const WorkerTick& tick) override { return client_.step(tick); }
  ResetEndpoint* reset_service() noexcept override { return &client_; }
  void request_stop() noexcept override { client_.request_stop(); }
 private:
  WorkerResetClient client_;
};
}
