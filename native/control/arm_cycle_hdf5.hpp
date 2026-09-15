#pragma once
#include "hdf5_stream_client.hpp"
#include "session_runtime.hpp"
#include <nlohmann/json.hpp>

namespace tianji_control {
// Metadata comes from the recording/publication owner, not inferred from an IK
// tick. Message sequence, proposal sequence and target sequence are distinct.
// time_ns uses the SAME timeline origin as every other stream in the file.
struct ArmCycleRecordMetadata {
  std::int64_t time_ns=-1,feedback_sequence=-1;
  std::array<std::int64_t,2> command_sequences{{-1,-1}};
  std::array<std::optional<std::int64_t>,2> proposal_sequences,target_sequences;
  std::optional<std::int64_t> intent_sequence;
  std::string command_instance,command_producer,command_mode;
  std::string executor_instance,executor,session_instance,session_source;
};

// Output-thread encoder for the existing arm command/state/session-event columns.
// Does NOT encode the full native_cycle audit, declare a publisher, invent absent
// feedback or mark the recording complete. Validation precedes all disk writes.
inline Hdf5Block encode_arm_cycle_columns(const SessionCycleSnapshot& cycle,
                                         const ArmCycleRecordMetadata& meta) {
  const auto invalid=[] {throw std::invalid_argument("invalid arm-cycle recording fields");};
  const auto utf8=[&](const std::string& value) {
    // JSON's strict encoder validates UTF-8 without replacement; HDF5's string
    // datatype alone does not prevent malformed byte sequences reaching disk.
    try {(void)nlohmann::json(value).dump();}catch(const nlohmann::json::type_error&){invalid();}
  };
  // File-relative time may precede zero when distinct output queues are drained
  // in a different order. Still require a representable, nonnegative origin.
  if(cycle.timestamp_ns<0 || cycle.timestamp_ns<meta.time_ns ||
     meta.time_ns<cycle.timestamp_ns-std::numeric_limits<std::int64_t>::max() || !cycle.snapshot.command ||
     !cycle.snapshot.feedback || meta.feedback_sequence<0) invalid();
  for(auto sequence:meta.command_sequences) if(sequence<0) invalid();
  for(const auto* sequences:{&meta.proposal_sequences,&meta.target_sequences})
    for(auto sequence:*sequences) if(sequence && *sequence<0) invalid();
  if(meta.intent_sequence && *meta.intent_sequence<0) invalid();
  for(const auto* id:{&meta.command_instance,&meta.command_producer,&meta.executor_instance,
                      &meta.executor,&meta.session_instance,&meta.session_source}) {
    if(id->empty() || id->size()>256 || id->find('\0')!=std::string::npos) invalid();
    utf8(*id);
  }
  if(meta.command_mode!="idle" && meta.command_mode!="teleop" && meta.command_mode!="returning") invalid();
  const auto& state=cycle.snapshot.state;
  if(state.phase!="idle" && state.phase!="teleop" && state.phase!="returning" && state.phase!="fault") invalid();
  if(state.reason.size()>4096 || state.reason.find('\0')!=std::string::npos) invalid();
  utf8(state.reason);
  for(const auto* pair:{&cycle.snapshot.command->positions,&*cycle.snapshot.feedback})
    for(const auto& side:*pair) for(double q:side) if(!std::isfinite(q)) invalid();

  Hdf5Block block;
  const auto names=arm_names();
  auto integer=[&](const std::string& path,std::int64_t value){block.integers(path,1,{value});};
  auto text=[&](const std::string& path,const std::string& value){block.strings(path,{value});};
  auto nullable=[&](const std::string& path,std::optional<std::int64_t> value){
    integer(path,value.value_or(-1));block.octets(path+"_valid",1,{static_cast<std::uint8_t>(value.has_value())});
  };
  auto attributes=[&](const std::string& path,const std::vector<std::string>& joints,const std::string& logical){
    const auto json=nlohmann::json(joints).dump();
    block.attribute(path,"names",json);block.attribute(path,"joint_names",json);block.attribute(path,"logical_id",logical);
  };
  for(int s=0;s<2;++s) {
    const std::string path=std::string("/joint/command/arm/")+(s?"right":"left");
    integer(path+"/time_ns",meta.time_ns);text(path+"/publisher_instance_id",meta.command_instance);
    integer(path+"/sequence",meta.command_sequences[s]);
    nullable(path+"/proposal_sequence",meta.proposal_sequences[s]);nullable(path+"/target_sequence",meta.target_sequences[s]);
    const auto& q=cycle.snapshot.command->positions[s];
    block.reals(path+"/position_rad",1,{q.begin(),q.end()});text(path+"/mode",meta.command_mode);
    attributes(path,{names.begin()+s*7,names.begin()+s*7+7},meta.command_producer);
  }
  const std::string feedback="/joint/state/arm";
  integer(feedback+"/time_ns",meta.time_ns);text(feedback+"/publisher_instance_id",meta.executor_instance);
  integer(feedback+"/sequence",meta.feedback_sequence);
  std::vector<double> q;q.reserve(14);
  for(const auto& side:*cycle.snapshot.feedback) q.insert(q.end(),side.begin(),side.end());
  block.reals(feedback+"/position_rad",1,q);
  // The native snapshot exposes position feedback only: do not fabricate zero velocity.
  block.octets(feedback+"/velocity_valid",1,{0});
  block.reals(feedback+"/velocity_rad_s",1,std::vector<double>(14,std::numeric_limits<double>::quiet_NaN()));
  attributes(feedback,{names.begin(),names.end()},meta.executor);
  const std::string event="/meta/session_events";
  integer(event+"/time_ns",meta.time_ns);text(event+"/publisher_instance_id",meta.session_instance);
  text(event+"/state",state.phase);text(event+"/reason",state.reason);text(event+"/source",meta.session_source);
  nullable(event+"/intent_sequence",meta.intent_sequence);
  return block;
}
} // namespace tianji_control
