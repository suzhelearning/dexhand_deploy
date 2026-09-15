#pragma once
#include <array>
#include <cstdint>
#include <stdexcept>
namespace tianji_control {
// reset/epoch have a single owner; request_stop alone is callable concurrently.
// Implementations must bound cancellation and validate actual backend state.
class ResetEndpoint {
 public:
  virtual ~ResetEndpoint()=default;
  virtual void reset(const std::array<double,14>&,std::int64_t)=0;
  virtual std::int64_t epoch() const=0;
  virtual void request_stop() noexcept=0;
  virtual bool supports_height() const noexcept { return false; }
  virtual void configure_height(const std::array<double,2>&) { throw std::logic_error("height configuration unsupported"); }
  virtual void configure_xz(const std::array<double,2>&,const std::array<double,2>&) { throw std::logic_error("XZ configuration unsupported"); }
};
}
