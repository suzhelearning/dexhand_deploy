#pragma once
#include "session_runtime.hpp"
#include "hand_joint_names.hpp"
#include <nlohmann/json.hpp>
namespace tianji_control {
// Encode only commands actually taken by the session and measured hand state.
// Absent updates are holds, not permission to manufacture fresh commands.
class SessionHandPublication {
  using Json=nlohmann::json;
 public:
  explicit SessionHandPublication(const Json& config) {
    const std::array<const char*,3> keys{{"producer","left","right"}};
    for(std::size_t i=0;i<keys.size();++i) {
      const auto& a=config.at(keys[i]);
      authorities_[i]={a.at("logical"),a.at("instance"),a.at("router")};
      if(!authorities_[i].bounded() || authorities_[i].router!=authorities_[0].router)
        throw std::invalid_argument("invalid hand publication authority");
    }
  }
  Json encode(const SessionCycleSnapshot& cycle) const {
    const auto& s=cycle.snapshot;
    if(cycle.timestamp_ns<=0 || !s.hand_feedback || s.ticks>=(std::uint64_t(1)<<63) || s.state.epoch<=0 ||
       (s.state.phase!="idle" && s.state.phase!="teleop" && s.state.phase!="returning" && s.state.phase!="fault"))
      throw std::invalid_argument("invalid hand publication snapshot");
    Json out=Json::array();
    for(int side=0;side<2;++side) {
      const std::string key=side?"right":"left";
      const auto names=joint_names(side);
      const auto& measured=(*s.hand_feedback)[side];
      for(double q:measured)if(!std::isfinite(q))throw std::invalid_argument("nonfinite hand feedback");
      auto feedback=base(side+1,cycle);
      feedback.update({{"executor",authorities_[side+1].logical},{"side",key},{"names",names},
        {"position_rad",measured},{"velocity_rad_s",nullptr}});
      out.push_back(Json::array({"tianji/state/hand/"+key,std::move(feedback)}));
      if(s.hand_command[side]) {
        if(s.state.phase!="teleop" && s.state.phase!="returning")
          throw std::invalid_argument("hand command outside authorized phase");
        for(double q:*s.hand_command[side])if(!std::isfinite(q))throw std::invalid_argument("nonfinite hand command");
        auto command=base(0,cycle);
        command.update({{"producer",authorities_[0].logical},{"side",key},{"names",names},
                        {"position_rad",*s.hand_command[side]}});
        out.push_back(Json::array({"tianji/command/hand/"+key,std::move(command)}));
      }
    }
    return out;
  }
  static std::vector<std::string> joint_names(int side) {
    return hand_joint_names(side);
  }
 private:
  Json base(int role,const SessionCycleSnapshot& c) const {
    return {{"schema_version",1},{"publisher_instance_id",authorities_[role].instance},
      {"router_zid",authorities_[role].router},{"sequence",c.snapshot.ticks},{"timestamp_ns",c.timestamp_ns}};
  }
  std::array<Authority,3> authorities_;
};
}
