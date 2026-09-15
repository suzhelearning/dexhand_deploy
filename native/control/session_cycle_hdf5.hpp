#pragma once
#include "arm_cycle_hdf5.hpp"
#include "raw_tjvr_hdf5.hpp"
#include "session_publication.hpp"
#include "worker_result_json.hpp"

namespace tianji_control {
// Cycle recording adapter, with optional executed hand commands and feedback.
// Uses the publication contract for
// command identities/receipts; recording cannot become a second command owner.
class SessionCycleRecording {
  using Json=nlohmann::json;
 public:
  explicit SessionCycleRecording(const Json& manifest)
      :publication_(manifest),run_(manifest.at("run_id")),
       receiver_(manifest.at("source_authority").at("instance")) {
    const auto prefix=manifest.at("worker_prefix").get<std::string>();
    if(prefix!="spark" && prefix!="mapped_palm") throw std::invalid_argument("unknown recording worker");
    mapped_=prefix=="mapped_palm";
    const std::string algorithm=mapped_?"pico_ee_mapped_corrected_palm_velocity_qp":
      "spark_upper_qpoases_headroom_feedforward_velocity_qp";
    if(manifest.at("algorithm")!=algorithm) throw std::invalid_argument("recording worker algorithm mismatch");
    hands_=manifest.contains("hand_authorities");
  }
  Hdf5Block encode(const SessionCycleSnapshot& cycle,std::int64_t origin_ns) const {
    if(origin_ns<0 || cycle.timestamp_ns<=0)
      throw std::invalid_argument("invalid recording timeline");
    Json messages=Json::object();
    for(const auto& row:publication_.encode(cycle)) messages[row.at(0).get<std::string>()]=row.at(1);
    const auto& left=messages.at("tianji/command/arm/left");
    const auto& right=messages.at("tianji/command/arm/right");
    const auto& feedback=messages.at("tianji/state/arm");
    const auto& session=messages.at("tianji/session/state");
    auto integer=[](const Json& value) -> std::int64_t {
      if((!value.is_number_unsigned() && !value.is_number_integer()) ||
         (value.is_number_unsigned() && value.get<std::uint64_t>()>
          static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) ||
         (!value.is_number_unsigned() && value.get<std::int64_t>()<0))
        throw std::invalid_argument("recording integer out of range");
      return value.get<std::int64_t>();
    };
    ArmCycleRecordMetadata meta;
    meta.time_ns=cycle.timestamp_ns-origin_ns;
    meta.feedback_sequence=integer(feedback.at("sequence"));
    meta.command_sequences={integer(left.at("sequence")),integer(right.at("sequence"))};
    for(int s=0;s<2;++s) {
      const auto& command=s?right:left;
      if(!command.at("proposal_sequence").is_null()) meta.proposal_sequences[s]=integer(command.at("proposal_sequence"));
      if(!command.at("target_sequence").is_null()) meta.target_sequences[s]=integer(command.at("target_sequence"));
    }
    meta.command_instance=left.at("publisher_instance_id");meta.command_producer=left.at("producer");
    meta.command_mode=left.at("mode");meta.executor_instance=feedback.at("publisher_instance_id");
    meta.executor=feedback.at("executor");meta.session_instance=session.at("publisher_instance_id");
    meta.session_source=session.at("source");
    // Apply the same empty-reason fallback as the established Python snapshot.
    auto normalized=cycle;normalized.snapshot.state.reason=session.at("reason");
    auto block=encode_arm_cycle_columns(normalized,meta);
    Json accepted_hands=Json::object();
    for(const std::string side:{"left","right"}) for(const std::string kind:{"state","command"}) {
      const auto key="tianji/"+kind+"/hand/"+side;
      if(!messages.contains(key)) continue;
      const auto& message=messages.at(key);
      const auto path="/joint/"+kind+"/hand/"+side;
      block.integers(path+"/time_ns",1,{meta.time_ns});
      block.integers(path+"/sequence",1,{integer(message.at("sequence"))});
      block.strings(path+"/publisher_instance_id",{message.at("publisher_instance_id").get<std::string>()});
      block.reals(path+"/position_rad",1,message.at("position_rad").get<std::vector<double>>());
      block.attribute(path,"names",message.at("names").dump());
      block.attribute(path,"joint_names",message.at("names").dump());
      block.attribute(path,"logical_id",message.at(kind=="state"?"executor":"producer").get<std::string>());
      if(kind=="state") {
        // Native feedback is position-only. Unknown velocity is not zero.
        block.octets(path+"/velocity_valid",1,{0});
        block.reals(path+"/velocity_rad_s",1,std::vector<double>(20,std::numeric_limits<double>::quiet_NaN()));
      } else accepted_hands[side]=message;
    }
    Json native=nullptr,attempt=nullptr,receipt=nullptr;
    if(cycle.result) {
      native=worker_result_json(cycle.result->wire,mapped_);
      if(native.at("tick_id")!=cycle.result->tick || native.at("applied_sequence")!=cycle.result->applied_sequence ||
         native.at("timestamp_ns")!=cycle.result->timestamp)
        throw std::invalid_argument("recording result/wire identity mismatch");
      receipt=messages.at("tianji/coordinator/arm/bilateral_receipt");
      // A control tick between input packets legitimately has no sample. Keep
      // the IK result but do not manufacture a raw association for that tick.
      if(cycle.request && cycle.request->received_ns>0 && !cycle.request->packet.empty()) {
        const auto& r=*cycle.request;
        // Validate using the same raw recording boundary; do not invent an
        // ingress ordinal or replace malformed raw with derived geometry.
        validate_raw_tjvr_recording(r.packet,r.source_sequence,r.received_ns,receiver_,run_);
        const Json sample={{"schema_version",1},{"kind","tjvr_upper_limb_observation"},
          {"raw_packet_base64",base64(r.packet)},{"received_timestamp_ns",r.received_ns},
          {"receiver_instance_id",receiver_},{"receiver_frame_sequence",r.source_sequence},
          {"stream_discontinuity",r.discontinuity},{"resynchronization_generation",r.generation}};
        attempt={{"tick_id",cycle.result->tick},{"timestamp_ns",cycle.result->timestamp},{"sample",sample}};
      }
    }
    Json payload={{"execution_epoch",cycle.snapshot.state.epoch},{"native",native},
      {"native_attempt",attempt},{"coordinator_receipt",receipt},{"receipt_accepted",cycle.ik_adopted},
      {"bilateral_command",messages.at("tianji/command/arm/bilateral")},
      {"accepted_hand_commands",accepted_hands},{"source_status",messages.at("tianji/source/status")},{"run_id",run_}};
    if(!hands_ && !cycle.hand_results.empty())
      throw std::invalid_argument("hand result audit without configured hand producer");
    if(hands_) {
      auto rows=Json::array();
      for(const auto& audit:cycle.hand_results) {
        const auto& envelope=audit.result;const auto& v=envelope.value;
        // Keep dispositions, including rejected/stale results. They are not
        // execution commands and must not be substituted for accepted_hands.
        if(audit.observed_ns<=0 || audit.observed_ns>cycle.timestamp_ns ||
           !envelope.producer.bounded() || envelope.run.empty() || envelope.run.size()>256 ||
           audit.outcome.reason.size()>4096)
          throw std::invalid_argument("invalid hand result audit metadata");
        for(double q:v.positions) if(!std::isfinite(q))
          throw std::invalid_argument("nonfinite hand audit result");
        rows.push_back({{"run_id",envelope.run},{"producer_authority",{
            {"logical",envelope.producer.logical},{"instance",envelope.producer.instance},{"router",envelope.producer.router}}},
          {"observed_timestamp_ns",audit.observed_ns},{"accepted",audit.outcome.accepted},{"reason",audit.outcome.reason},
          {"result",{{"output_sequence",v.output_sequence},{"input_sequence",v.input_sequence},
            {"input_timestamp_ns",v.input_timestamp_ns},{"epoch",v.epoch},
            {"scheduler_timestamp_ns",v.scheduler_timestamp_ns},{"valid_flags",v.valid_flags},
            {"phase",static_cast<unsigned>(v.phase)},{"status",static_cast<unsigned>(v.status)},
            {"positions_rad",v.positions}}}});
      }
      payload["native_hand_results"]=std::move(rows);
    }
    const auto encoded=payload.dump();
    if(encoded.size()>1048576) throw std::invalid_argument("recording audit exceeds 1 MiB");
    block.integers("/meta/dual_audit/time_ns",1,{meta.time_ns});
    block.integers("/meta/dual_audit/received_timestamp_ns",1,{cycle.timestamp_ns});
    block.strings("/meta/dual_audit/kind",{"native_cycle"});
    block.strings("/meta/dual_audit/payload_json",{encoded});
    return block;
  }
 private:
  static std::string base64(const std::vector<std::uint8_t>& bytes) {
    constexpr char alphabet[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;out.reserve((bytes.size()+2)/3*4);
    for(std::size_t i=0;i<bytes.size();i+=3) {
      const auto n=std::min<std::size_t>(3,bytes.size()-i);
      const std::uint32_t bits=(std::uint32_t(bytes[i])<<16) |
        (n>1?std::uint32_t(bytes[i+1])<<8:0) | (n>2?bytes[i+2]:0);
      out.push_back(alphabet[(bits>>18)&63]);out.push_back(alphabet[(bits>>12)&63]);
      out.push_back(n>1?alphabet[(bits>>6)&63]:'=');out.push_back(n>2?alphabet[bits&63]:'=');
    }
    return out;
  }
  SessionPublication publication_;
  std::string run_,receiver_;
  bool mapped_=false,hands_=false;
};
} // namespace tianji_control
