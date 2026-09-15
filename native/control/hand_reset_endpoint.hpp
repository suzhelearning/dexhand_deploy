#pragma once
#include <cstdint>
namespace tianji_control {
// Main-session reset task exclusively owns reset/epoch. Only cancellation may
// run concurrently. No readiness or motion authorization is granted here.
class HandResetEndpoint {
 public:
  virtual ~HandResetEndpoint()=default;
  virtual void reset(std::int64_t epoch)=0;
  virtual std::int64_t epoch() const=0;
  virtual void request_stop() noexcept=0;
};
}
