#pragma once
#include "session_hand_domain.hpp"
#include <nlohmann/json.hpp>

namespace tianji_control {
struct HandLaunchConfig {
  SessionHandConfig domain;
  int fd=0,timeout_ms=0;
  std::uint64_t generation=0;
  std::vector<std::string> command;
  std::string right_glove,left_glove;
};
// Pure validation before constructing workers or adopting descriptors.
inline std::optional<HandLaunchConfig> parse_hand_launch(const nlohmann::json& m,int control_fd) {
  if(!m.contains("hand_runtime")) return std::nullopt;
  using Json=nlohmann::json;
  const auto invalid=[] {throw std::invalid_argument("invalid native hand launch configuration");};
  auto number=[&](const Json& v,std::int64_t low,std::int64_t high){
    if(!v.is_number_integer() || (v.is_number_unsigned() && v.get<std::uint64_t>()>static_cast<std::uint64_t>(high))) invalid();
    const auto n=v.get<std::int64_t>();if(n<low || n>high) invalid();return n;
  };
  auto text=[&](const Json& v,std::size_t max,bool empty=false){
    if(!v.is_string()) invalid();
    auto s=v.get<std::string>();
    if((!empty && s.empty()) || s.size()>max || s.find('\0')!=std::string::npos) invalid();
    (void)Json(s).dump();return s;
  };
  auto identity=[&](const Json& a){
    Authority out{text(a.at("logical"),256),text(a.at("instance"),256),text(a.at("router"),256)};
    if(!out.bounded() || out.router!=m.at("router_zid")) invalid();
    return out;
  };
  if(m.at("publication_backend")!="cpp" || m.at("viewer_backend")!="cpp" || m.at("diagnostic_transport")!="summary") invalid();
  HandLaunchConfig c;const auto& h=m.at("hand_runtime");
  c.fd=number(h.at("stdout_fd"),3,std::numeric_limits<int>::max());
  if(c.fd==control_fd || (m.contains("recording_fd") && m.at("recording_fd")==c.fd)) invalid();
  c.generation=number(h.at("generation"),1,std::numeric_limits<std::int64_t>::max());
  c.timeout_ms=number(h.at("timeout_ms"),1,60000);
  c.domain.capacity=number(h.at("capacity"),1,8192);
  c.domain.age=number(h.at("age_ns"),1,std::numeric_limits<std::int64_t>::max());
  c.domain.run=text(m.at("run_id"),256);c.domain.source=identity(m.at("manus_source_authority"));
  c.domain.producer=identity(m.at("hand_authorities").at("producer"));
  for(const auto* side:{"left","right"}) (void)identity(m.at("hand_authorities").at(side));
  c.right_glove=text(h.at("right_glove"),256,true);c.left_glove=text(h.at("left_glove"),256,true);
  if(!c.right_glove.empty() && c.right_glove==c.left_glove) invalid();
  const auto& argv=h.at("worker_command");
  if(!argv.is_array() || argv.empty() || argv.size()>64) invalid();
  for(const auto& arg:argv)c.command.push_back(text(arg,4096));
  if(c.command.front().front()!='/') invalid();
  auto joints=[&](const char* key){
    const auto& value=h.at(key);HandPair out;
    if(!value.is_array() || value.size()!=2) invalid();
    for(int s=0;s<2;++s) {
      if(!value[s].is_array() || value[s].size()!=20) invalid();
      for(int j=0;j<20;++j) {
        if(!value[s][j].is_number()) invalid();
        out[s][j]=value[s][j].get<double>();
        if(!std::isfinite(out[s][j])) invalid();
      }
    }
    return out;
  };
  c.domain.lower=joints("lower");c.domain.upper=joints("upper");
  c.domain.zero=joints("zero");c.domain.zero_tolerance=joints("zero_tolerance");
  (void)SessionHandDomain(c.domain); // Same limits and ownership validation as execution.
  return c;
}
}
