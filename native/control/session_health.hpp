#pragma once
#include "session_commands.hpp"

namespace tianji_control {
struct Authority {
  std::string logical, instance, router;
  bool operator==(const Authority& other) const {
    return logical==other.logical && instance==other.instance && router==other.router;
  }
  bool bounded() const {
    for (const auto* field:{&logical,&instance,&router})
      if(field->empty() || field->size()>256) return false;
    return true;
  }
};
inline std::array<std::string,14> arm_names() {
  return {"Joint1_L","Joint2_L","Joint3_L","Joint4_L","Joint5_L","Joint6_L","Joint7_L",
          "Joint1_R","Joint2_R","Joint3_R","Joint4_R","Joint5_R","Joint6_R","Joint7_R"};
}
struct HealthStatus {
  int role=0; // source, producer_arm, executor_arm; fixed launcher authorities only
  Authority authority;
  std::int64_t sequence=0;
  bool ready=false, healthy=false, simulation=false, observation_only=false;
};
struct ArmFeedback {
  Authority authority;
  std::int64_t sequence=0;
  std::array<std::string,14> names{};
  ArmPair positions{};
};
struct SessionHealthConfig {
  std::array<Authority,3> authorities;
  ArmPair home{};
  std::int64_t age=1000000000;
  double home_tolerance=.0174532925199433;
};
// Typed, fixed-authority, arms-only input reducer. Caller stamps local receive
// time; source timestamps never rejuvenate state. Wire decoding is still external.
class SessionHealth {
 public:
  explicit SessionHealth(SessionHealthConfig config):config_(std::move(config)) {
    if (config_.age<=0 || !std::isfinite(config_.home_tolerance) || config_.home_tolerance<0)
      throw std::invalid_argument("invalid health freshness or Home tolerance");
    for(const auto& id:config_.authorities)
      if(!id.bounded() || id.router!=config_.authorities[0].router)
        throw std::invalid_argument("fixed same-router authorities required");
    for(const auto& side:config_.home) for(double q:side)
      if(!std::isfinite(q)) throw std::invalid_argument("finite Home required");
  }
  const std::string& error() const { return error_; }
  IntentOutcome status(std::int64_t received,const HealthStatus& value) {
    if(!observe(received)) return {false,error_};
    if(value.role<0 || value.role>=3 || !value.authority.bounded() || value.sequence<0)
      return fail("malformed component status");
    if(value.authority.router!=config_.authorities[0].router) return fail("component router_zid mismatch");
    if(value.role==0 && value.observation_only) return {true,"observation ignored"};
    const std::string key=std::string(roles_[value.role])+"/"+value.authority.logical;
    if(!(value.authority==config_.authorities[value.role])) return fail("component authority mismatch for "+key);
    auto& slot=slots_[value.role];
    if(slot.received>=0 && value.sequence<=slot.sequence) return fail("component sequence rollback for "+key);
    slot={received,value.sequence,value.ready && value.healthy && value.simulation};
    return {true,"accepted"};
  }
  IntentOutcome feedback(std::int64_t received,const ArmFeedback& value) {
    if(!observe(received)) return {false,error_};
    if(!value.authority.bounded() || value.sequence<0) return fail("malformed arm state");
    for(const auto& side:value.positions) for(double q:side)
      if(!std::isfinite(q)) return fail("malformed arm state");
    if(value.authority.router!=config_.authorities[0].router || value.names!=arm_names())
      return fail("arm state identity/order mismatch");
    if(!(value.authority==config_.authorities[2])) return fail("arm state executor authority mismatch");
    if(feedback_.received>=0 && value.sequence<=feedback_.sequence) return fail("arm state sequence rollback");
    feedback_={received,value.sequence,true}; positions_=value.positions;
    return {true,"accepted"};
  }
  SessionFacts facts(std::int64_t now) const {
    SessionFacts out;
    if(!error_.empty() || now<last_received_) return out;
    out.source_ready=fresh(slots_[0],now) && slots_[0].ready;
    out.producer_ready=fresh(slots_[1],now) && slots_[1].ready;
    out.executor_ready=fresh(slots_[2],now) && slots_[2].ready;
    out.arm_fresh=fresh(feedback_,now);
    out.arm_home=out.arm_fresh;
    out.arm_exact_home=out.arm_fresh && positions_==config_.home;
    for(int side=0;side<2;++side) for(int j=0;j<7;++j)
      if(std::abs(positions_[side][j]-config_.home[side][j])>config_.home_tolerance) out.arm_home=false;
    // No fabricated hand readiness or raw-input progress from status heartbeats.
    return out;
  }
 private:
  struct Slot { std::int64_t received=-1, sequence=0; bool ready=false; };
  bool fresh(const Slot& slot,std::int64_t now) const {
    return slot.received>=0 && now>=slot.received && now-slot.received<=config_.age;
  }
  bool observe(std::int64_t now) {
    if(!error_.empty()) return false;
    if(now<0 || now<last_received_) { fail("monotonic clock rollback"); return false; }
    last_received_=now; return true;
  }
  IntentOutcome fail(const std::string& reason) { if(error_.empty()) error_=reason; return {false,error_}; }
  SessionHealthConfig config_;
  std::array<Slot,3> slots_;
  Slot feedback_;
  ArmPair positions_{};
  std::string error_;
  std::int64_t last_received_=0;
  static constexpr const char* roles_[3]={"source","producer_arm","executor_arm"};
};
} // namespace tianji_control
