#pragma once
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace tianji_control {
// Trusted facts produced by validated ingress, NOT a wire authorization protocol.
// This reducer models the bilateral simulation route; it cannot publish commands.
struct SessionFacts {
  bool source_ready=false, producer_ready=false, executor_ready=false;
  bool arm_fresh=false, arm_home=false, hand_producer_ready=false;
  bool hand_zero=false, hand_tracking=false, command_home=false;
  bool proposal_stale=false, arm_exact_home=false;
  std::uint64_t input_revision=0;
};
struct IntentOutcome { bool accepted=false; std::string reason; };
struct SessionState {
  std::string phase="idle", reason="startup";
  std::int64_t epoch=1;
  bool at_home=true, return_complete=false, shutdown_complete=false;
};
class SessionMachine {
 public:
  SessionMachine(std::int64_t grace, std::int64_t hand_timeout, bool hands)
      : grace_(grace), hand_timeout_(hand_timeout), hands_(hands) {
    if (grace<=0 || hand_timeout<=0) throw std::invalid_argument("positive timeout required");
  }
  const SessionState& state() const { return state_; }
  void fault(std::int64_t now, const std::string& reason) {
    if (!clock(now)) return;
    latch(reason);
  }
  IntentOutcome intent(std::int64_t now, const std::string& action,
      const std::string& reason, bool authority, const SessionFacts& f) {
    if (!clock(now)) return {false, "monotonic clock rollback"};
    if (!authority) return reject("intent source authority mismatch");
    if (state_.phase=="fault") {
      state_.reason=fault_reason_;
      return {false, "fault latched; restart required"};
    }
    if (action=="start") {
      if (state_.phase!="idle") return reject("start requires idle");
      const auto why=readiness(f);
      if (!why.empty()) return reject(why);
      if (needs_input_ && f.input_revision<=rearm_revision_)
        return reject("fresh input after rearm required");
      needs_input_=false;
      state_.phase="teleop"; state_.reason="accepted"; started_=now;
      state_.at_home=false; state_.return_complete=false; state_.shutdown_complete=false;
      return {true, "accepted"};
    }
    if (action=="return" || action=="shutdown") {
      state_.phase="returning"; state_.reason=reason.empty()?action:reason;
      returned_=now; state_.return_complete=false; state_.shutdown_complete=false;
      if (action=="shutdown") shutdown_=true;
      return {true, "accepted"};
    }
    return {false, "unsupported intent"};
  }
  IntentOutcome rearm(std::int64_t now, std::int64_t epoch,
                      const SessionFacts& f, bool reset_ack) {
    if (!clock(now)) return {false, "monotonic clock rollback"};
    if (state_.phase!="idle" || !f.arm_fresh || !f.arm_home || !f.arm_exact_home ||
        !f.command_home || (hands_ && !f.hand_zero) || !reset_ack ||
        state_.epoch==std::numeric_limits<std::int64_t>::max() || epoch!=state_.epoch+1)
      return {false, "bilateral rearm requires idle, fresh exact Home feedback and next epoch"};
    state_.epoch=epoch; state_.reason="explicit Home rearm; fresh input and start required";
    state_.return_complete=false; state_.shutdown_complete=false;
    needs_input_=true; rearm_revision_=f.input_revision;
    return {true, "accepted"};
  }
  void tick(std::int64_t now, const SessionFacts& f) {
    if (!clock(now)) return;
    if (state_.phase=="teleop") {
      const bool grace=now-started_<=grace_;
      if (!f.source_ready) latch("source stale or unhealthy");
      else if (!f.producer_ready) latch("producer_arm stale or unhealthy");
      else if (!f.executor_ready || !f.arm_fresh) latch("executor arm/state stale or unhealthy");
      else if (hands_ && !f.hand_producer_ready) latch("producer_hand stale or unhealthy");
      else if (hands_ && !f.hand_tracking && !grace)
        latch("hand executor/status state stale, unhealthy, or identity mismatch");
      else if (f.proposal_stale && !grace) latch("arm proposal timeout");
    }
    if (state_.phase=="returning") {
      if (hands_ && !f.hand_zero && now-returned_>hand_timeout_)
        latch("hand return timeout");
      else if (f.arm_fresh && f.arm_home && f.command_home && (!hands_ || f.hand_zero)) {
        state_.phase="idle"; state_.reason="return complete";
        state_.return_complete=true; state_.shutdown_complete=shutdown_;
      }
    }
    state_.at_home=f.command_home;
  }
 private:
  bool clock(std::int64_t now) {
    if (now<0 || now<last_) { latch("monotonic clock rollback"); return false; }
    last_=now; return true;
  }
  void latch(const std::string& why) {
    fault_reason_=why;
    if (state_.phase!="fault") { state_.phase="fault"; state_.reason=why; }
    state_.return_complete=false; state_.shutdown_complete=false;
  }
  IntentOutcome reject(const std::string& why) { state_.reason=why; return {false, why}; }
  std::string readiness(const SessionFacts& f) const {
    if (!f.source_ready) return "source not exactly-one fresh healthy ready";
    if (!f.producer_ready) return "producer_arm not exactly-one fresh healthy ready";
    if (!f.executor_ready) return "executor_arm not exactly-one fresh healthy ready";
    if (hands_ && !f.hand_producer_ready) return "producer_hand not exactly-one fresh healthy ready";
    if (hands_ && !f.hand_zero) return "hand executor/state not fresh at zero";
    if (!f.arm_fresh || !f.arm_home) return "arm state is not fresh at Home";
    if (!f.command_home) return "final command is not at Home";
    return {};
  }
  SessionState state_;
  std::string fault_reason_;
  std::int64_t grace_, hand_timeout_, last_=0, started_=0, returned_=0;
  bool hands_, shutdown_=false, needs_input_=false;
  std::uint64_t rearm_revision_=0;
};
}  // namespace tianji_control
