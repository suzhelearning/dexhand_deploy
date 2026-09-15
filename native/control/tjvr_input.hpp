#pragma once
#include "raw_input.hpp"
#include "tianji_mapped_palm/pico_teleop_protocol.hpp"
#include "tianji_mapped_palm/pico_mapped_corrected_palm.hpp"
#include <limits>
#include <stdexcept>
namespace tianji_control {
// Reuse imported decoder/gate/selection without changing the reference sources.
// This is receive-side acceptance, not an IK tick or a motion authorization.
class TjvrInput final:public RawInputEndpoint {
 public:
  TjvrInput(bool mapped,double position_jump,double rotation_jump)
    :mapped_(mapped),gate_(position_jump,rotation_jump) {}
  RawProgress ingest(const std::vector<std::uint8_t>& bytes) override {
    RawProgress result=latest_; result.accepted=false;
    if(ingress_sequence_==std::numeric_limits<std::uint64_t>::max())
      throw std::overflow_error("raw ingress sequence exhausted");
    result.decoded=false;
    result.ingress_sequence=++ingress_sequence_;
    if(bytes.empty() || bytes.size()>tianji_mapped_palm::kPicoTeleopMaximumPacketSize) return result;
    auto decoded=tianji_mapped_palm::decodePicoTeleopPacket(bytes.data(),bytes.size());
    if(!decoded.frame) return result;
    result.decoded=true;
    auto frame=*decoded.frame;
    if(mapped_) {
      const auto selected=tianji_mapped_palm::selectMappedCorrectedPalm(frame);
      if(!selected.valid) return result;
      frame.left=selected.left; frame.right=selected.right;
    }
    const auto decision=gate_.evaluate(frame);
    if(!decision.accepted) return result;
    if(decision.stream_discontinuity) {
      if(latest_.generation==std::numeric_limits<std::uint64_t>::max()) throw std::overflow_error("raw generation exhausted");
      ++latest_.generation;
    }
    // Accepted packets must carry the same decode/ingress metadata as rejected
    // decoded packets. Runtime uses it for raw capture and IK sample association.
    latest_.decoded=true; latest_.ingress_sequence=result.ingress_sequence;
    latest_.accepted=true; latest_.epoch=frame.tracking_epoch; latest_.sequence=frame.sequence;
    latest_.discontinuity=decision.stream_discontinuity;
    latest_.skeleton_valid=frame.upper_limb_skeleton.valid;
    latest_.rotations_valid=frame.upper_limb_skeleton.rotations_valid;
    for(int side=0;side<2;++side) for(int axis=0;axis<3;++axis)
      latest_.palms[side][axis]=frame.upper_limb_skeleton.points[side?7:3][axis];
    return latest_;
  }
 private:
  bool mapped_;
  tianji_mapped_palm::PicoTeleopStreamGate gate_;
  RawProgress latest_;
  std::uint64_t ingress_sequence_=0;
};
}
