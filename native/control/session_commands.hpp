#pragma once
#include "command_math.hpp"
#include "session_machine.hpp"
#include <utility>

namespace tianji_control {
using ArmPair = std::array<Joints7,2>;
struct SessionCommandConfig {
  std::string run, producer, instance, router;
  ArmPair home{}, lower{}, upper{};
  CommandConfig math{.02,.05,200.,.2};
  double home_duration=2., home_speed=.4363323129985824;
  std::int64_t ingress_age=1000000000;
  bool clipping=false;
};
// Typed paired proposal, not a replacement for the JSON/schema decoder.
// Shared metadata makes mismatched left/right epochs and timestamps impossible.
struct SessionProposal {
  std::string run, producer, instance, router;
  std::int64_t epoch=0, tick=0, timestamp=0;
  ArmPair positions{};
  std::array<bool,2> tracking_hold{}, failure_hold{};
};
struct CommandReceipt {
  std::int64_t tick=0;
  bool accepted=false;
  std::string reason;
  std::int64_t epoch=0,timestamp=0;
  std::string run{},router{};
};
struct SessionCommandResult {
  ArmPair positions{};
  std::optional<CommandReceipt> receipt;
};

// Single-owner paired command reducer. No actuator output or claimed feedback.
// SessionMachine must outlive this object and have the same thread owner.
class SessionCommands {
 public:
  SessionCommands(SessionMachine& machine, SessionCommandConfig config)
      : machine_(machine), config_(std::move(config)), positions_(config_.home) {
    for (const auto* id:{&config_.run,&config_.producer,&config_.instance,&config_.router})
      if (!valid_id(*id)) throw std::invalid_argument("command authority identity required");
    for (double value:{config_.math.maximum_step,config_.math.rate,config_.math.proposal_timeout,
                       config_.home_duration,config_.home_speed})
      if (!std::isfinite(value) || value<=0) throw std::invalid_argument("positive command settings required");
    if (!std::isfinite(config_.math.time_window) || config_.math.time_window<0 || config_.ingress_age<=0)
      throw std::invalid_argument("invalid command time bounds");
    for (int side=0;side<2;++side) for (int joint=0;joint<7;++joint) {
      const auto lo=config_.lower[side][joint], hi=config_.upper[side][joint], q=positions_[side][joint];
      if (!std::isfinite(lo) || !std::isfinite(hi) || !std::isfinite(q) || !(lo<=q && q<=hi))
        throw std::invalid_argument("invalid Home or joint limits");
    }
  }
  const ArmPair& positions() const { return positions_; }
  SessionFacts facts(SessionFacts input) const {
    input.command_home=positions_==config_.home;
    return input;
  }
  IntentOutcome intent(std::int64_t now,const std::string& action,const std::string& reason,
                       bool authority,SessionFacts input) {
    if (!clock(now)) return {false,"monotonic clock rollback"};
    const auto result=machine_.intent(now,action,reason,authority,facts(input));
    if (result.accepted && action=="start") {
      proposal_.reset(); anchors_={now,now};
    }
    if (result.accepted && (action=="return" || action=="shutdown")) {
      return_start_=positions_; return_time_=now;
    }
    return result;
  }
  IntentOutcome rearm(std::int64_t now,std::int64_t epoch,SessionFacts input,bool ack) {
    if (!clock(now)) return {false,"monotonic clock rollback"};
    auto result=machine_.rearm(now,epoch,facts(input),ack);
    if (result.accepted) { proposal_.reset(); pending_.reset(); last_tick_=0; anchors_={}; }
    return result;
  }
  bool accept(std::int64_t now,const SessionProposal& value) {
    if (!clock(now)) return false;
    if (value.tick<=0 || value.epoch<=0 || value.timestamp<0 || !valid_id(value.run) ||
        !valid_id(value.producer) || !valid_id(value.instance) || !valid_id(value.router))
      return reject(now,"malformed bilateral proposal");
    for (const auto& q:value.positions) for(double x:q)
      if (!std::isfinite(x)) return reject(now,"malformed bilateral proposal");
    if (value.run!=config_.run || value.epoch!=machine_.state().epoch || value.tick<=last_tick_)
      return false;
    if (machine_.state().phase!="teleop") return false;
    if (pending_) return reject(now,"bilateral proposal superseded before command tick");
    if (value.router!=config_.router) return reject(now,"arm proposal identity mismatch");
    if (value.producer!=config_.producer || value.instance!=config_.instance)
      return reject(now,"proposal producer authority mismatch");
    if (value.timestamp>now || now-value.timestamp>config_.ingress_age)
      return reject(now,"arm proposal timestamp stale");
    if (proposal_ && config_.math.time_window>0 && value.timestamp<proposal_->timestamp)
      return reject(now,"arm proposal timestamp rollback");
    proposal_=value; received_=now; pending_=value.tick; pending_epoch_=value.epoch; last_tick_=value.tick;
    return true;
  }
  SessionCommandResult tick(std::int64_t now,SessionFacts input) {
    clock(now);
    const bool fresh=proposal_ && now>=received_ && now-received_<=config_.ingress_age;
    // Numeric validation precedes health transitions, as in the coordinator.
    if (machine_.state().phase=="teleop" && fresh) {
      for (int side=0;side<2;++side) {
        const auto check=validate_command(proposal_->positions[side],positions_[side],
            config_.lower[side],config_.upper[side],config_.math,now,proposal_->timestamp,
            anchors_[side],proposal_->tracking_hold[side],true);
        if (check.error!=CommandError::ok) {
          machine_.fault(now,error_reason(check.error)); break;
        }
      }
    }
    input=facts(input); input.proposal_stale=proposal_.has_value() && !fresh;
    machine_.tick(now,input);
    const auto phase=machine_.state().phase;
    if (phase=="teleop") {
      for (int side=0;side<2;++side) {
        if (!fresh) { positions_[side]=config_.home[side]; continue; }
        const auto old=positions_[side];
        positions_[side]=track_command(proposal_->positions[side],old,config_.math.maximum_step,
                                     config_.clipping,proposal_->tracking_hold[side]);
        if (positions_[side]==proposal_->positions[side] && !proposal_->tracking_hold[side] &&
            !(proposal_->failure_hold[side] && positions_[side]==old)) anchors_[side]=proposal_->timestamp;
      }
    } else if (phase=="returning") {
      for (int side=0;side<2;++side)
        positions_[side]=home_command(return_start_[side],config_.home[side],
            static_cast<double>(now-return_time_)/1e9,config_.home_duration,config_.home_speed);
    } else if (phase=="idle") positions_=config_.home;
    // fault: preserve both last final commands, including a fault during Home.
    if (phase!="teleop") anchors_={};
    machine_.tick(now,facts(input)); // completion uses THIS tick's command and measured Home
    SessionCommandResult result{positions_,std::nullopt};
    if (pending_) {
      const bool accepted=machine_.state().phase=="teleop" && fresh;
      result.receipt=CommandReceipt{*pending_,accepted,accepted?"accepted":machine_.state().reason,
        pending_epoch_,now,config_.run,config_.router};
      pending_.reset();
    }
    return result;
  }
 private:
  static bool valid_id(const std::string& id) { return !id.empty() && id.size()<=256; }
  static const char* error_reason(CommandError error) {
    switch(error) {
      case CommandError::hard_limits:return "proposal exceeds hard joint limits or is nonfinite";
      case CommandError::timestamp_rollback:return "arm proposal timestamp precedes adopted command";
      case CommandError::source_stale:return "arm proposal source timestamp stale";
      case CommandError::maximum_step:return "proposal exceeds maximum command step";
      default:return "tracking hold is simulation-only";
    }
  }
  bool clock(std::int64_t now) {
    if (now<0 || now<last_now_) return reject(last_now_,"monotonic clock rollback");
    last_now_=now; return true;
  }
  bool reject(std::int64_t now,const std::string& reason) { machine_.fault(now,reason); return false; }
  SessionMachine& machine_;
  SessionCommandConfig config_;
  ArmPair positions_,return_start_{};
  std::optional<SessionProposal> proposal_;
  std::optional<std::int64_t> pending_;
  std::array<std::optional<std::int64_t>,2> anchors_{};
  std::int64_t last_tick_=0,received_=0,return_time_=0,last_now_=0,pending_epoch_=0;
};
}  // namespace tianji_control
