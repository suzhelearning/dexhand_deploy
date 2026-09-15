#pragma once
#include "simulation_endpoint.hpp"
#include "session_health.hpp"
#include "owned_mujoco.hpp"
namespace tianji_control {
class MujocoEndpoint final:public SimulationEndpoint {
 public:
  explicit MujocoEndpoint(const std::string& xml,bool hands=false)
    :hands_(hands),sim_(xml,groups(hands),{},aliases(hands)) {}
  void apply(const ArmPair& q) override {
    if(hands_) sim_.apply_mixed(q,HandUpdates{});
    else sim_.apply(q);
  }
  void apply_frame(const ArmPair& arms,const HandUpdates& hands) override {
    if(hands_) sim_.apply_mixed(arms,hands);
    else SimulationEndpoint::apply_frame(arms,hands);
  }
  std::optional<HandPair> hand_feedback() override {
    if(!hands_) return std::nullopt;
    return HandPair{sim_.group_positions<20>(2),sim_.group_positions<20>(3)};
  }
  ArmPair feedback() override {
    return {sim_.group_positions<7>(0),sim_.group_positions<7>(1)};
  }
  std::optional<std::array<double,2>> height_reference() override {
    return sim_.zero_group_site_heights({"hand_tcp_frame_L","hand_tcp_frame_R"});
  }
  std::optional<std::array<std::array<double,2>,2>> forward_reference() override {
    return sim_.forward_site_xz({"hand_tcp_frame_L","hand_tcp_frame_R"});
  }
 private:
  static std::vector<std::vector<std::string>> groups(bool hands) {
    const auto names=arm_names();
    std::vector<std::vector<std::string>> out{{names.begin(),names.begin()+7},{names.begin()+7,names.end()}};
    if(hands) {
      constexpr std::array<const char*,20> suffixes{
        "thumb_cmc_flex","thumb_cmc_abd","thumb_mcp","thumb_ip",
        "index_mcp_flex","index_mcp_abd","index_pip","index_dip",
        "middle_mcp_flex","middle_mcp_abd","middle_pip","middle_dip",
        "ring_mcp_flex","ring_mcp_abd","ring_pip","ring_dip",
        "pinky_mcp_flex","pinky_mcp_abd","pinky_pip","pinky_dip"};
      for(const auto* prefix:{"l_","r_"}) {
        std::vector<std::string> group;
        for(const auto* suffix:suffixes) group.emplace_back(std::string(prefix)+suffix);
        out.push_back(std::move(group));
      }
    }
    return out;
  }
  static std::unordered_map<std::string,std::string> aliases(bool hands) {
    std::unordered_map<std::string,std::string> out;
    if(hands) {
      const auto names=groups(true);
      for(std::size_t s=2;s<4;++s) for(const auto& name:names[s]) {
        if(name.find("thumb_")==std::string::npos) {
          auto alias=name; alias.insert(alias.find('_',2),"_finger");
          out.emplace(name,std::move(alias));
        }
      }
    }
    return out;
  }
  const bool hands_;
  OwnedMujoco sim_;
};
}
