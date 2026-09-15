#pragma once
#include "worker_result.hpp"
#include "reset_endpoint.hpp"
#include <functional>
#include <memory>
namespace tianji_control {
// Native worker service, not command authority. Only request_stop may be concurrent.
class IkEndpoint {
 public:
  virtual ~IkEndpoint()=default;
  virtual WorkerResult step(const WorkerTick&)=0;
  // Optional borrowed service, stable until this endpoint is destroyed. No step
  // may run during reset; the owner must join reset before destroying the endpoint.
  virtual ResetEndpoint* reset_service() noexcept { return nullptr; }
  virtual void request_stop() noexcept=0;
};
using IkFactory=std::function<std::unique_ptr<IkEndpoint>()>;
}
