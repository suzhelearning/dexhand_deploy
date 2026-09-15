#pragma once
#include "session_health.hpp"
#include "simulation_endpoint.hpp"
#include "../hand/scheduler.hpp"

namespace tianji_control {
// Internal post-parser/post-retarget boundary, not a public wire protocol.
// The ingress owner supplies the unchanged source's local receive timestamp.
struct HandInputReceipt {
  Authority source;
  std::uint64_t sequence=0,timestamp_ns=0,generation=0;
  std::uint8_t flags=0; // left=1, right=2
};
struct HandResultEnvelope {
  std::string run;
  Authority producer;
  tianji_hand::HandCommand value;
};
struct HandSampleEnvelope {
  Authority source;
  tianji_hand::HandInput value;
};
struct SessionHandConfig {
  std::string run;
  Authority source,producer;
  HandPair lower{},upper{},zero{},zero_tolerance{};
  std::int64_t age=200000000;
  std::size_t capacity=1024;
};
// Single scheduler-thread owner. Validated inputs are associated with results;
// the last observed phase/epoch fences queued output before model mutation.
class SessionHandDomain {
 public:
  explicit SessionHandDomain(SessionHandConfig c):config_(std::move(c)) {
    if(!id(config_.run) || !authority(config_.source) || !authority(config_.producer) ||
       config_.source.router!=config_.producer.router || config_.age<=0 ||
       !config_.capacity || config_.capacity>8192) throw std::invalid_argument("invalid hand domain configuration");
    for(int s=0;s<2;++s) for(int j=0;j<20;++j) {
      const auto lo=config_.lower[s][j],hi=config_.upper[s][j],q=config_.zero[s][j],tol=config_.zero_tolerance[s][j];
      if(!std::isfinite(lo) || !std::isfinite(hi) || !std::isfinite(q) || !std::isfinite(tol) ||
         lo>q || q>hi || tol<0) throw std::invalid_argument("invalid hand limits/zero");
    }
    inputs_.resize(config_.capacity);
  }
  const std::string& error() const {return error_;}
  const HandPair& zero() const {return config_.zero;}
  void sync(const SessionState& state,std::int64_t now) {
    if(!clock(now)) return;
    if(state.epoch<epoch_ || state.epoch<=0 ||
       (state.phase!="idle" && state.phase!="teleop" && state.phase!="returning" && state.phase!="fault")) {
      fail("invalid hand session state");return;
    }
    if(state.epoch!=epoch_ || state.phase!=phase_) {
      pending_={};size_=head_=0;retired_input_=last_input_;
      if(state.epoch!=epoch_) {processed_={};epoch_cutoff_=now;}
    }
    epoch_=state.epoch;phase_=state.phase;
  }
  IntentOutcome input(std::int64_t now,const HandInputReceipt& v) {
    if(!clock(now)) return {false,error_};
    if(!authority(v.source) || !(v.source==config_.source) || !positive(v.sequence) ||
       !positive(v.timestamp_ns) || v.generation>=limit_ || !v.flags || v.flags>3)
      return fail("invalid hand input identity/metadata");
    if(v.sequence<=last_input_ || v.timestamp_ns<last_input_time_ ||
       (last_input_ && v.generation!=generation_)) return fail("hand input sequence/time/generation changed");
    last_input_=v.sequence;last_input_time_=v.timestamp_ns;generation_=v.generation;
    if(v.timestamp_ns>static_cast<std::uint64_t>(now)) return fail("future hand input timestamp");
    if(!fresh(now,v.timestamp_ns) || v.timestamp_ns<=epoch_cutoff_) return {false,"hand input expired or before epoch barrier"};
    // Old unprocessed inputs may expire without poisoning a recovered source.
    while(size_ && !fresh(now,inputs_[head_].timestamp_ns)) pop_input();
    if(size_==inputs_.size()) return fail("hand input association queue overflow");
    inputs_[(head_+size_)%inputs_.size()]=v;++size_;
    return {true,"accepted"};
  }
  IntentOutcome result(std::int64_t now,const HandResultEnvelope& envelope) {
    using namespace tianji_hand;
    if(!clock(now)) return {false,error_};
    const auto& v=envelope.value;
    if(envelope.run!=config_.run || !authority(envelope.producer) || !(envelope.producer==config_.producer) ||
       !positive(v.output_sequence) || !positive(v.input_sequence) || !positive(v.input_timestamp_ns) ||
       !positive(v.scheduler_timestamp_ns) || v.scheduler_timestamp_ns<v.input_timestamp_ns ||
       v.scheduler_timestamp_ns>static_cast<std::uint64_t>(now) || v.epoch<=0 || v.valid_flags>3 ||
       phase_name(v.phase)=="invalid" ||
       (v.status!=HandOutputStatus::processed && v.status!=HandOutputStatus::command && v.status!=HandOutputStatus::stale))
      return fail("invalid hand result authority/metadata");
    // The native wire contract requires every encoded value to be finite,
    // including inactive-side slots; only active sides receive limit checks.
    for(double q:v.positions) if(!std::isfinite(q)) return fail("nonfinite hand result payload");
    if(v.output_sequence<=last_output_) return fail("hand result sequence rollback");
    last_output_=v.output_sequence;
    if(v.epoch>epoch_) return fail("hand result from future epoch");
    if(v.epoch!=epoch_ || phase_name(v.phase)!=phase_ || v.input_sequence<=retired_input_ ||
       v.input_timestamp_ns<=epoch_cutoff_)
      return {false,"hand result crossed session barrier"};
    if(!fresh(now,v.input_timestamp_ns) || v.status==HandOutputStatus::stale) return {false,"hand result expired"};
    std::optional<HandInputReceipt> source;
    while(size_ && inputs_[head_].sequence<=v.input_sequence) {
      if(inputs_[head_].sequence==v.input_sequence) source=inputs_[head_];
      pop_input();
    }
    if(!source || source->timestamp_ns!=v.input_timestamp_ns || (v.valid_flags & ~source->flags))
      return fail("unassociated hand result");
    if((phase_=="teleop")!=(v.status==HandOutputStatus::command)) return fail("hand result status/phase mismatch");
    HandUpdates checked;
    for(int s=0;s<2;++s) if(v.valid_flags & (1<<s)) {
      HandJoints q;
      for(int j=0;j<20;++j) {
        q[j]=v.positions[s*20+j];
        if(!std::isfinite(q[j]) || q[j]<config_.lower[s][j] || q[j]>config_.upper[s][j])
          return fail("hand result outside finite joint limits");
      }
      checked[s]=q;
    }
    for(int s=0;s<2;++s) if(checked[s]) {
      processed_[s]=v.input_timestamp_ns;
      if(phase_=="teleop") {pending_[s]=checked[s];pending_time_[s]=v.input_timestamp_ns;}
    }
    return {true,"accepted"};
  }
  void feedback(std::int64_t now,const HandPair& q) {
    if(!clock(now)) return;
    for(int s=0;s<2;++s) for(int j=0;j<20;++j)
      if(!std::isfinite(q[s][j]) || q[s][j]<config_.lower[s][j] || q[s][j]>config_.upper[s][j]) {
        fail("invalid owned hand feedback");return;
      }
    positions_=q;feedback_time_=now;
  }
  SessionFacts facts(std::int64_t now) const {
    SessionFacts f;
    if(!error_.empty() || now<last_time_) return f;
    f.hand_producer_ready=fresh(now,processed_[0]) && fresh(now,processed_[1]);
    // Admission at Home must not rely on nearly expired startup backlog.
    // During teleop the established age gate still allows asynchronous solves.
    // Continuous input need not stop for start to become eligible. Reserve
    // headroom for the phase transition instead of demanding exact catch-up.
    if(phase_=="idle") for(int s=0;s<2;++s)
      f.hand_producer_ready=f.hand_producer_ready && fresh(now,processed_[s]) &&
        static_cast<std::uint64_t>(now)-processed_[s]<=static_cast<std::uint64_t>(std::max<std::int64_t>(1,std::min<std::int64_t>(config_.age/4,50000000)));
    f.hand_zero=fresh(now,feedback_time_);
    f.hand_tracking=f.hand_zero && phase_=="teleop";
    for(int s=0;s<2;++s) for(int j=0;j<20;++j)
      if(std::abs(positions_[s][j]-config_.zero[s][j])>config_.zero_tolerance[s][j]) f.hand_zero=false;
    return f;
  }
  HandUpdates take(std::int64_t now) {
    if(!clock(now)) return {};
    HandUpdates out;
    if(phase_=="returning") out={config_.zero[0],config_.zero[1]};
    else if(phase_=="teleop") for(int s=0;s<2;++s)
      if(pending_[s] && fresh(now,pending_time_[s])) out[s]=pending_[s];
    pending_={};return out;
  }
 private:
  static constexpr std::uint64_t limit_=std::uint64_t{1}<<63;
  static bool positive(std::uint64_t v) {return v>0 && v<limit_;}
  static bool id(const std::string& s) {return !s.empty() && s.size()<=256 && s.find('\0')==std::string::npos;}
  static bool authority(const Authority& a) {return id(a.logical) && id(a.instance) && id(a.router);}
  static std::string phase_name(tianji_hand::HandPhase phase) {
    switch(phase) {
      case tianji_hand::HandPhase::idle:return "idle";
      case tianji_hand::HandPhase::teleop:return "teleop";
      case tianji_hand::HandPhase::returning:return "returning";
      case tianji_hand::HandPhase::fault:return "fault";
    }
    return "invalid";
  }
  bool fresh(std::int64_t now,std::uint64_t time) const {
    return time && now>=0 && time<=static_cast<std::uint64_t>(now) &&
      static_cast<std::uint64_t>(now)-time<=static_cast<std::uint64_t>(config_.age);
  }
  bool clock(std::int64_t now) {
    if(!error_.empty()) return false;
    if(now<=0 || now<last_time_) {fail("hand domain clock rollback");return false;}
    last_time_=now;return true;
  }
  IntentOutcome fail(const std::string& why) {
    if(error_.empty()) error_=why;
    pending_={};return {false,error_};
  }
  void pop_input() {head_=(head_+1)%inputs_.size();--size_;}
  SessionHandConfig config_;
  std::vector<HandInputReceipt> inputs_;
  std::size_t head_=0,size_=0;
  std::uint64_t last_input_=0,last_input_time_=0,generation_=0,last_output_=0,epoch_cutoff_=0,feedback_time_=0,retired_input_=0;
  std::array<std::uint64_t,2> processed_{},pending_time_{};
  std::int64_t last_time_=0,epoch_=1;
  std::string phase_="idle",error_;
  HandUpdates pending_;
  HandPair positions_{};
};
} // namespace tianji_control
