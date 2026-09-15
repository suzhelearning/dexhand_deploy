#pragma once
#include "session_publication.hpp"
#include <zenoh.hxx>

namespace tianji_control {
// Only used by the opt-in native gateway. Domain liveliness/duplicate-owner
// admission stays with the Python launcher. No hardware executor is declared.
class SessionZenohPublication {
 public:
  explicit SessionZenohPublication(const nlohmann::json& manifest)
    :encoder_(manifest),router_(manifest.at("coordinator_authority").at("router")),
     session_(open(manifest.at("publication_endpoint").get<std::string>())) {
    check_router();
  }
  void publish(const SessionCycleSnapshot& cycle) {
    check_router();
    for(const auto& row:encoder_.encode(cycle)) {
      auto options=zenoh::Session::PutOptions::create_default();
      // Same nonblocking push policy as the Python session.put default. Queue
      // acceptance is not a remote delivery acknowledgement or a disk flush.
      options.congestion_control=Z_CONGESTION_CONTROL_DROP;
      options.encoding=zenoh::Encoding("application/json");
      session_.put(zenoh::KeyExpr(row.at(0).get<std::string>()),
                   zenoh::Bytes(row.at(1).dump()),std::move(options));
    }
  }
 private:
  static zenoh::Session open(const std::string& endpoint) {
    if(endpoint.empty() || endpoint.size()>4096 || endpoint.find('\0')!=std::string::npos)
      throw std::invalid_argument("explicit publication endpoint required");
    auto config=zenoh::Config::create_default();
    config.insert_json5("mode","\"client\"");
    config.insert_json5("connect/endpoints",nlohmann::json::array({endpoint}).dump());
    config.insert_json5("scouting/multicast/enabled","false");
    return zenoh::Session::open(std::move(config));
  }
  void check_router() const {
    const auto routers=session_.get_routers_z_id();
    if(routers.size()!=1 || routers.front().to_string()!=router_)
      throw std::runtime_error("native publication router identity mismatch or disconnected");
  }
  SessionPublication encoder_;
  std::string router_;
  zenoh::Session session_;
};
} // namespace tianji_control
