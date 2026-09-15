#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>
namespace tianji_control {
struct WorkerTick {
  std::uint64_t id=0,generation=0;
  // Receiver ordinal of the decoded source packet.  It is distinct from the
  // accepted-source revision because rejected decoded packets still consume
  // an ingress ordinal in the reference recorder.
  std::uint64_t source_sequence=0;
  std::int64_t now_ns=0,received_ns=0;
  bool discontinuity=false;
  std::vector<std::uint8_t> packet;
};
struct WorkerGuidance {
  std::array<double,7> stage1_q{},ik_q{},feedforward_q{},feedforward_qdot{},feedforward_qddot{};
  bool ik_accepted=false,budget_exhausted=false,feedforward_target_accepted=false;
  bool settled_hold=false,stationary_hold=false;
  std::int32_t stage1_iterations=0,stage2_iterations=0,feedforward_state=0,headroom_state=0,settled_hold_reason=0;
};
struct WorkerArm {
  std::array<double,7> q{},qdot{},qddot{};
  bool accepted=false;
  std::int32_t qp_status=0,hold_reason=0;
  double headroom_scale=0,task_scale_position=0,task_scale_orientation=0;
  std::array<double,3> target_position{};
  std::array<double,4> target_quaternion{};
  WorkerGuidance guidance;
};
struct WorkerResult {
  bool deterministic=false,epoch_reset=false,input_live=false,control_executed=false;
  std::uint64_t tick=0,timestamp=0,applied_epoch=0,applied_sequence=0;
  std::int32_t button_action=0;
  bool joint_takeover_cycle=false,guidance_accepted=false,height_present=false;
  std::uint64_t guidance_updates=0,headroom_updates=0,step_ns=0,encode_ns=0;
  std::array<double,2> height_offsets{};
  bool x_present=false;
  std::array<double,2> x_offsets{};
  std::array<WorkerArm,2> arms;
  // Preserve the complete versioned diagnostic frame, not a reduced command schema.
  std::vector<std::uint8_t> wire;
};
inline std::size_t worker_result_size(bool mapped,bool xz=false) { return mapped?(xz?611:595):1206; }
inline void validate_worker_header(const std::vector<std::uint8_t>& bytes,bool mapped) {
  if(bytes.size()<8 || bytes[0]!='T' || bytes[1]!='J' || bytes[2]!='B' || bytes[3]!='R' ||
     bytes[4]!=1 || (mapped?(bytes[5]!=2 && bytes[5]!=3):bytes[5]!=1) ||
     (std::size_t(bytes[6]) | (std::size_t(bytes[7])<<8))!=worker_result_size(mapped,bytes[5]==3)-8)
    throw std::runtime_error("incompatible native binary result header");
}
class WorkerResultReader {
 public:
  explicit WorkerResultReader(const std::vector<std::uint8_t>& bytes):bytes_(bytes) {}
  std::uint64_t integer(unsigned size=8) {
    if(size>8 || at_>bytes_.size() || size>bytes_.size()-at_) throw std::runtime_error("truncated native result");
    std::uint64_t value=0;
    for(unsigned i=0;i<size;++i) value|=std::uint64_t(bytes_[at_++])<<(8*i);
    return value;
  }
  bool boolean() {
    auto v=integer(1); if(v>1) throw std::runtime_error("invalid native binary boolean"); return v!=0;
  }
  std::int32_t code() {
    const auto v=integer(4);
    return static_cast<std::int32_t>(v<=0x7fffffffULL?static_cast<std::int64_t>(v):static_cast<std::int64_t>(v)-0x100000000LL);
  }
  double real() {
    static_assert(sizeof(double)==8 && std::numeric_limits<double>::is_iec559);
    const auto bits=integer(); double value; std::memcpy(&value,&bits,8);
    if(!std::isfinite(value)) throw std::runtime_error("non-finite native binary result");
    return value;
  }
  template<std::size_t N> void array(std::array<double,N>& values) { for(auto& v:values) v=real(); }
  bool done() const { return at_==bytes_.size(); }
 private:
  const std::vector<std::uint8_t>& bytes_;
  std::size_t at_=8;
};
inline WorkerResult decode_worker_result(std::vector<std::uint8_t> bytes,bool mapped) {
  validate_worker_header(bytes,mapped);
  if(bytes.size()!=worker_result_size(mapped,bytes[5]==3)) throw std::runtime_error("incomplete or unsolicited native binary result");
  WorkerResultReader r(bytes); WorkerResult out;
  out.deterministic=r.boolean(); out.tick=r.integer(); out.timestamp=r.integer();
  out.applied_epoch=r.integer(); out.applied_sequence=r.integer(); out.epoch_reset=r.boolean();
  out.input_live=r.boolean(); out.button_action=r.code(); out.control_executed=r.boolean();
  if(mapped) { out.height_present=r.boolean(); r.array(out.height_offsets); }
  else {
    out.joint_takeover_cycle=r.boolean(); out.guidance_accepted=r.boolean();
    out.guidance_updates=r.integer(); out.headroom_updates=r.integer();
  }
  if(mapped && bytes[5]==3) {out.x_present=true;r.array(out.x_offsets);}
  for(auto& a:out.arms) {
    r.array(a.q); r.array(a.qdot); r.array(a.qddot); a.accepted=r.boolean();
    a.qp_status=r.code(); a.hold_reason=r.code(); a.headroom_scale=r.real();
    a.task_scale_position=r.real(); a.task_scale_orientation=r.real();
    r.array(a.target_position); r.array(a.target_quaternion);
    if(!mapped) {
      auto& g=a.guidance;
      r.array(g.stage1_q); r.array(g.ik_q); g.ik_accepted=r.boolean();
      g.stage1_iterations=r.code(); g.stage2_iterations=r.code(); g.budget_exhausted=r.boolean();
      r.array(g.feedforward_q); r.array(g.feedforward_qdot); r.array(g.feedforward_qddot);
      g.feedforward_state=r.code(); g.feedforward_target_accepted=r.boolean(); g.headroom_state=r.code();
      g.settled_hold=r.boolean(); g.settled_hold_reason=r.code(); g.stationary_hold=r.boolean();
    }
  }
  out.step_ns=r.integer(); out.encode_ns=r.integer();
  if(!r.done()) throw std::runtime_error("native result layout mismatch");
  out.wire=std::move(bytes); return out;
}
} // namespace tianji_control
