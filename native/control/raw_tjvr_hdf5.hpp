#pragma once
#include "hdf5_stream_client.hpp"
#include "session_runtime.hpp"
#include "tianji_mapped_palm/pico_teleop_protocol.hpp"
#include <nlohmann/json.hpp>

namespace tianji_control {
// Encode on the output consumer, never the control thread. Preserve datagram
// bytes even for duplicate/rejected gate input; no geometry reserialization.
// origin_ns is the file-wide origin, not a separate per-stream first timestamp.
inline void validate_raw_tjvr_recording(const std::vector<std::uint8_t>& bytes,
                                       std::uint64_t sequence,std::int64_t received_ns,
                                       const std::string& receiver,const std::string& run_id) {
  const auto invalid=[] {throw std::invalid_argument("invalid raw TJVR recording fields");};
  if(received_ns<=0 || sequence==0 ||
     sequence>static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) invalid();
  for(const auto* text:{&receiver,&run_id}) {
    if(text->empty() || text->size()>256 || text->find('\0')!=std::string::npos) invalid();
    (void)nlohmann::json(*text).dump(); // Strict UTF-8, no replacement.
  }
  if(bytes.empty() || bytes.size()>tianji_mapped_palm::kPicoTeleopMaximumPacketSize ||
     !tianji_mapped_palm::decodePicoTeleopPacket(bytes.data(),bytes.size()).frame) invalid();
}
inline Hdf5Block encode_raw_tjvr_columns(const SessionRawSnapshot& raw,
                                        std::int64_t origin_ns,
                                        const std::string& receiver,
                                        const std::string& run_id) {
  // Relative timestamps may be negative: output streams retain per-stream
  // FIFO, not global chronology. Both absolute values are nonnegative int64.
  if(origin_ns<0) throw std::invalid_argument("invalid recording origin");
  validate_raw_tjvr_recording(raw.bytes,raw.sequence,raw.received_ns,receiver,run_id);
  const auto ordinal=static_cast<std::int64_t>(raw.sequence);
  const nlohmann::json audit={{"version",1},{"receiver_instance_id",receiver},
    {"receiver_frame_sequence",ordinal},{"received_timestamp_ns",raw.received_ns},
    {"accepted",raw.accepted},{"parser_error",nullptr},{"run_id",run_id}};
  Hdf5Block block;
  const std::string path="/raw/tjvr_upper_limb";
  block.integers(path+"/time_ns",1,{raw.received_ns-origin_ns});
  block.integers(path+"/received_timestamp_ns",1,{raw.received_ns});
  block.integers(path+"/receiver_frame_sequence",1,{ordinal});
  block.strings(path+"/receiver_instance_id",{receiver});
  block.bytes(path+"/raw_packet",{raw.bytes});
  const std::string meta="/meta/dual_audit";
  block.integers(meta+"/time_ns",1,{raw.received_ns-origin_ns});
  block.integers(meta+"/received_timestamp_ns",1,{raw.received_ns});
  block.strings(meta+"/kind",{"native_tjvr_ingress"});
  block.strings(meta+"/payload_json",{audit.dump()});
  return block;
}
} // namespace tianji_control
