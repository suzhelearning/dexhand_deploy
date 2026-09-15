#pragma once
#include "session_runtime.hpp"
#include "session_hand_publication.hpp"
#include <nlohmann/json.hpp>

namespace tianji_control {
// Pure output-thread adapter. Never advances the state machine, issues a new
// command, or fabricates input/feedback. The launcher owns these identities.
class SessionPublication {
  using Json=nlohmann::json;
 public:
  explicit SessionPublication(const Json& manifest)
      :run_(manifest.at("run_id")),algorithm_(manifest.at("algorithm")),model_(manifest.at("model")) {
    const std::array<const char*,4> keys{{"source_authority","producer_authority",
                                         "coordinator_authority","executor_authority"}};
    for(std::size_t i=0;i<keys.size();++i) {
      const auto& a=manifest.at(keys[i]);
      authorities_[i]={a.at("logical"),a.at("instance"),a.at("router")};
      for(const auto* field:{&authorities_[i].logical,&authorities_[i].instance,&authorities_[i].router})
        if(field->empty() || field->size()>256 || field->find('\0')!=std::string::npos)
          throw std::invalid_argument("invalid publication authority");
      if(authorities_[i].router!=authorities_[0].router)
        throw std::invalid_argument("publication authority router mismatch");
    }
    if(run_.empty() || algorithm_.empty() || model_.empty())
      throw std::invalid_argument("publication metadata required");
    if(manifest.contains("hand_authorities")) {
      const auto& config=manifest.at("hand_authorities");
      hands_.emplace(config);
      if(config.at("producer").at("router")!=authorities_[0].router)
        throw std::invalid_argument("arm/hand publication router mismatch");
    }
  }
  Json encode(const SessionCycleSnapshot& c) const {
    const auto& s=c.snapshot; const auto& phase=s.state.phase;
    if(!s.command || !s.feedback || c.timestamp_ns<0 || s.state.epoch<=0 ||
       (phase!="idle" && phase!="teleop" && phase!="returning" && phase!="fault"))
      throw std::invalid_argument("invalid publication cycle");
    for(const auto* pair:{&s.command->positions,&*s.feedback})
      for(const auto& side:*pair) for(double q:side)
        if(!std::isfinite(q)) throw std::invalid_argument("nonfinite publication joint");
    const std::string reason=s.state.reason.empty()?"native session":s.state.reason;
    const std::string mode=phase=="idle"?"idle":phase=="teleop"?"teleop":"returning";
    const bool healthy=phase!="fault" && !s.cycle_capture_failed;
    const bool source_ready=c.source.accepted || c.source.skeleton_valid || healthy;
    const bool producer_ready=healthy && (c.result.has_value() || source_ready);
    Json proposal=nullptr,target=nullptr;
    if(phase=="teleop" && c.result && c.result->tick>0) {
      proposal=c.result->tick;target=c.result->applied_sequence;
    }
    auto base=[&](int role,std::uint64_t sequence) -> Json {
      return {{"schema_version",1},{"publisher_instance_id",authorities_[role].instance},
              {"router_zid",authorities_[role].router},{"sequence",sequence},{"timestamp_ns",c.timestamp_ns}};
    };
    auto status=[&](int role,const std::string& name,const std::string& id,
                    const std::string& state,bool ready,Json diagnostics,std::uint64_t sequence) {
      auto j=base(role,sequence);
      j.update({{"component_role",name},{"component_id",id},{"phase",state},{"ready",ready},
                {"healthy",healthy},{"capabilities",Json::array({"simulation"})},
                {"error",healthy?Json(nullptr):Json(reason)},{"diagnostics",std::move(diagnostics)}});
      return j;
    };
    Json rows=Json::array();
    auto put=[&](const std::string& key,Json value){rows.push_back(Json::array({key,std::move(value)}));};
    put("tianji/source/status",status(0,"source","tjvr",healthy?"ready":"fault",source_ready && healthy,
        {{"native_scheduler","cpp"},{"accepted",c.source.accepted},{"skeleton_valid",c.source.skeleton_valid},
         {"rotations_valid",c.source.rotations_valid},{"source_revision",c.source_revision}},c.source.sequence));
    auto producer=status(1,"producer_arm",authorities_[1].logical,producer_ready?"ready":"waiting_input",producer_ready,
        {{"algorithm",algorithm_},{"state_source","model_reference"},{"native_scheduler","cpp"},
         {"native_ticks",c.result?c.result->tick:0},{"input_live",c.result && c.result->input_live}},s.ticks);
    put("tianji/producer/status",producer);
    put("tianji/executor/status",status(3,"executor_arm","mujoco",phase,healthy,
        {{"native_scheduler","cpp"},{"model",model_}},s.ticks));
    auto feedback=base(3,s.ticks);std::vector<double> positions;
    for(const auto& side:*s.feedback) positions.insert(positions.end(),side.begin(),side.end());
    const auto names=arm_names();
    feedback.update({{"executor","mujoco"},{"names",names},{"position_rad",positions},{"velocity_rad_s",nullptr}});
    put("tianji/state/arm",feedback);
    auto session=base(2,s.ticks);
    session.update({{"state",phase},{"reason",reason},{"source","coordinator"},{"intent_sequence",nullptr}});
    put("tianji/session/state",session);
    producer.update({{"component_role","coordinator_arm"},{"component_id","arm"},
                     {"publisher_instance_id",authorities_[2].instance},
                     {"diagnostics",{{"native_scheduler","cpp"},{"authority","final_command_and_session_state"}}}});
    put("tianji/coordinator/status",producer);
    auto latched=base(2,s.ticks);latched["value"]=mode=="idle";
    put("tianji/coordinator/at_home",latched);
    // Preserve the current Python publication contract. The terminal completion
    // acknowledgement is a separate gateway frame, never inferred here.
    latched["value"]=false;put("tianji/coordinator/return_complete",latched);
    Json bilateral={{"schema_version",1},{"kind","arm_bilateral_command"},{"run_id",run_},
                    {"execution_epoch",s.state.epoch},{"reference_tick_id",proposal}};
    for(int side=0;side<2;++side) {
      const std::string key=side?"right":"left";
      auto command=base(2,s.ticks);
      command.update({{"producer","coordinator"},{"side",key},{"mode",mode},
                      {"proposal_sequence",proposal},{"target_sequence",target},
                      {"names",std::vector<std::string>(names.begin()+7*side,names.begin()+7*side+7)},
                      {"position_rad",s.command->positions[side]}});
      bilateral[key]=command;put("tianji/command/arm/"+key,command);
    }
    put("tianji/command/arm/bilateral",bilateral);
    if(c.result) put("tianji/coordinator/arm/bilateral_receipt",
        {{"schema_version",1},{"kind","arm_bilateral_receipt"},{"run_id",run_},
         {"execution_epoch",s.state.epoch},{"tick_id",c.result->tick},{"timestamp_ns",c.timestamp_ns},
         {"publisher_instance_id",authorities_[2].instance},{"router_zid",authorities_[2].router},
         {"stage","coordinator_command"},{"accepted",c.ik_adopted},{"reason",c.ik_adopted?"accepted":reason},
         {"command_position_rad",{{"left",s.command->positions[0]},{"right",s.command->positions[1]}}}});
    if(hands_)for(auto& row:hands_->encode(c))rows.push_back(std::move(row));
    return rows;
  }
 private:
  std::string run_,algorithm_,model_;
  std::array<Authority,4> authorities_;
  std::optional<SessionHandPublication> hands_;
};
} // namespace tianji_control
