#pragma once
#include "manus_ingress.hpp"
#include "hdf5_stream_client.hpp"
#include <nlohmann/json.hpp>

namespace tianji_control {
// Output-lane encoder. Preserve every complete driver line, including lines
// that produce no callback. The callback is normalized right21-left21, NOT raw25.
inline Hdf5Block encode_manus_ingress_columns(const ManusIngressRecord& raw,
    std::int64_t origin,const std::string& receiver,const std::string& run) {
  using Json=nlohmann::json;
  const auto invalid=[] {throw std::invalid_argument("invalid Manus recording fields");};
  if(origin<0 || raw.received_ns<=0 || !raw.line_sequence || raw.line_sequence>=(std::uint64_t(1)<<63) ||
     raw.raw_line.size()>65536 || raw.raw_line.find('\0')!=std::string::npos ||
     raw.raw_line.find('\n')!=std::string::npos) invalid();
  for(const auto* id:{&receiver,&run})
    if(id->empty() || id->size()>256 || id->find('\0')!=std::string::npos) invalid();
  Hdf5Block block;
  Json payloads=Json::array();
  payloads.push_back({{"line_sequence",raw.line_sequence},{"text",raw.raw_line},{"terminator","LF"},
    {"input_stage","rawviz_stdout_before_parser"},{"run_id",run}});
  std::vector<std::string> kinds{"manus_rawviz_line"};
  if(raw.sample) {
    const auto& envelope=*raw.sample;const auto& s=envelope.value;
    if(!envelope.source.bounded() || envelope.source.instance!=receiver || !s.sequence ||
       s.sequence>=(std::uint64_t(1)<<63) || !s.generation || s.generation>=(std::uint64_t(1)<<63) ||
       s.flags!=3 || s.timestamp_ns!=static_cast<std::uint64_t>(raw.received_ns)) invalid();
    for(double p:s.points) if(!std::isfinite(p)) invalid();
    Json sequences=Json::object(),timestamps=Json::object();
    for(int i=0;i<2;++i) {
      if(raw.source_sequences[i]<0 || raw.source_timestamps[i]<0) invalid();
      const auto key=i?"left":"right";
      sequences[key]=raw.source_sequences[i];timestamps[key]=raw.source_timestamps[i];
    }
    const std::string path="/raw/manus_callbacks";
    block.integers(path+"/time_ns",1,{raw.received_ns-origin});
    block.integers(path+"/received_timestamp_ns",1,{raw.received_ns});
    block.integers(path+"/callback_sequence",1,{static_cast<std::int64_t>(s.sequence)});
    block.integers(path+"/point_count",1,{126});
    block.strings(path+"/receiver_instance_id",{receiver});
    block.strings(path+"/single_hand_side",{"right"});
    block.reals(path+"/points",1,{s.points.begin(),s.points.end()});
    block.strings(path+"/source_sequences_json",{sequences.dump()});
    block.strings(path+"/source_timestamps_ns_json",{timestamps.dump()});
    kinds.push_back("manus_callback_metadata");
    payloads.push_back({{"callback_sequence",s.sequence},{"receiver_instance_id",receiver},
      {"source_sequences",sequences},{"source_timestamps_ns",timestamps},{"run_id",run}});
  }
  std::vector<std::string> encoded;
  for(const auto& p:payloads) encoded.push_back(p.dump()); // Strict UTF-8 before disk admission.
  const std::string audit="/meta/dual_audit";
  block.integers(audit+"/time_ns",kinds.size(),std::vector<std::int64_t>(kinds.size(),raw.received_ns-origin));
  block.integers(audit+"/received_timestamp_ns",kinds.size(),std::vector<std::int64_t>(kinds.size(),raw.received_ns));
  block.strings(audit+"/kind",kinds);block.strings(audit+"/payload_json",encoded);
  return block;
}
}
