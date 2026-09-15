#pragma once
#include "session_commands.hpp"
#include <nlohmann/json.hpp>
namespace tianji_control {
// Encoding belongs on an output consumer, never inside the control tick. This
// function neither publishes nor registers/authorizes a coordinator identity.
inline nlohmann::json encode_bilateral_receipt(const SessionCommandResult& result,
                                              const std::string& publisher_instance) {
  if(!result.receipt) throw std::invalid_argument("command has no receipt");
  const auto& r=*result.receipt;
  for(const auto* id:{&r.run,&r.router,&publisher_instance})
    if(id->empty() || id->size()>256 || id->find('\0')!=std::string::npos)
      throw std::invalid_argument("fixed receipt identity required");
  if(r.tick<=0 || r.epoch<=0 || r.timestamp<0) throw std::invalid_argument("invalid receipt epoch/tick/time");
  for(const auto& q:result.positions) for(double x:q)
    if(!std::isfinite(x)) throw std::invalid_argument("nonfinite receipt position");
  return {{"schema_version",1},{"kind","arm_bilateral_receipt"},{"run_id",r.run},
    {"execution_epoch",r.epoch},{"tick_id",r.tick},{"timestamp_ns",r.timestamp},
    {"publisher_instance_id",publisher_instance},{"router_zid",r.router},{"stage","coordinator_command"},
    {"accepted",r.accepted},{"reason",r.reason},
    {"command_position_rad",{{"left",result.positions[0]},{"right",result.positions[1]}}}};
}
}
