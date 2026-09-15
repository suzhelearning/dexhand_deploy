#pragma once
#include "session_machine.hpp"
#include "session_commands.hpp"
#include "session_health.hpp"
#include "reset_endpoint.hpp"
#include "raw_input.hpp"
#include "simulation_endpoint.hpp"
#include "ik_endpoint.hpp"
#include "execution_guard.hpp"
#include "height_calibration.hpp"
#include "session_hand_domain.hpp"
#include "hand_reset_endpoint.hpp"
#include "hand_pipeline_endpoint.hpp"
#include "manus_ingress.hpp"
#include <future>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <memory>
#include <thread>

namespace tianji_control {
class AbsoluteDeadline {
 public:
  AbsoluteDeadline(std::int64_t now, std::int64_t period): period_(period) {
    if (period<=0) throw std::invalid_argument("positive period required");
    reanchor(now);
  }
  std::int64_t next() const { return next_; }
  void advance() { reanchor(next_); }
  void reanchor(std::int64_t now) {
    if (now<0 || now>std::numeric_limits<std::int64_t>::max()-period_)
      throw std::overflow_error("deadline overflow");
    next_=now+period_;
  }
 private:
  std::int64_t period_, next_=0;
};
struct RawDatagram {
  Authority authority;
  std::vector<std::uint8_t> bytes;
  // Optional receive timestamp captured by the ingress owner.  -1 preserves
  // the legacy event-admission timestamp behavior for existing callers.
  std::int64_t received_ns=-1;
};
struct SessionEvent {
  std::uint64_t id;
  std::string action, reason;
  bool authority=false;
  std::int64_t next_epoch=0;
  bool reset_ack=false;
  std::optional<SessionProposal> proposal=std::nullopt;
  std::optional<HealthStatus> status=std::nullopt;
  std::optional<ArmFeedback> feedback=std::nullopt;
  std::int64_t received_ns=0; // Always overwritten at queue admission, never sender-controlled.
  std::optional<RawDatagram> raw=std::nullopt;
  // Telemetry/raw events normally do not need a reply.  Keep true by default
  // so the existing standalone runtime/tests retain their original contract.
  bool reply=true;
  std::optional<HandInputReceipt> hand_input=std::nullopt;
  std::optional<HandResultEnvelope> hand_result=std::nullopt;
  std::optional<HandSampleEnvelope> hand_sample=std::nullopt;
};
struct SessionReply {
  std::uint64_t id=0;
  IntentOutcome outcome;
  // Native audit metadata; existing gateway wire reply layout stays unchanged.
  std::string action{};
  std::int64_t timestamp_ns=0;
};
struct SessionSnapshot {
  SessionState state;
  std::uint64_t ticks=0, late_ticks=0;
  bool running=false, queue_overflow=false;
  std::optional<SessionCommandResult> command=std::nullopt;
  bool reset_pending=false;
  bool simulation_ready=false;
  std::optional<ArmPair> feedback;
  bool ik_ready=false;
  std::uint64_t ik_ticks=0;
  std::optional<ArmPair> ik_reference;
  std::optional<HeightCalibration::Status> height;
  bool cycle_capture_failed=false;
  std::optional<HandPair> hand_feedback;
  HandUpdates hand_command{};
};
// Value-owned record of a completed scheduler cycle, not an actuator command.
// Optional request/result belong to THIS cycle, never the previous IK tick.
struct SessionHandResultAudit {
  HandResultEnvelope result;
  IntentOutcome outcome;
  std::int64_t observed_ns=0;
};
struct SessionCycleSnapshot {
  std::int64_t timestamp_ns=0,source_received_ns=-1;
  std::uint64_t source_revision=0;
  RawProgress source;
  SessionSnapshot snapshot;
  std::optional<WorkerTick> request;
  std::optional<WorkerResult> result;
  bool ik_adopted=false;
  std::vector<SessionHandResultAudit> hand_results;
};
// A decoded source packet captured before the stream gate.  The Python
// reference recorder keeps this packet even when continuity/jump validation
// rejects it; ``accepted`` records that decision without changing the raw
// bytes or their host receive timestamp.
struct SessionRawSnapshot {
  std::uint64_t sequence=0;
  std::int64_t received_ns=0;
  bool accepted=false;
  std::vector<std::uint8_t> bytes;
};

// Offline supervisor. Ingress must validate identity, sequence and each domain's
// age before update(). Aggregate TTL is an additional bound, not a substitute.
// Optional native IK/command/simulation ownership. Still no network transport,
// Python callback or physical actuator output; complete live gates remain external.
class SessionRuntime {
 public:
  SessionRuntime(std::int64_t period, std::int64_t freshness,
                 std::size_t capacity, bool hands,
                 std::optional<SessionCommandConfig> command_config=std::nullopt,
                 std::optional<SessionHealthConfig> health_config=std::nullopt,
                 std::unique_ptr<ResetEndpoint> reset_endpoint=nullptr,
                 std::unique_ptr<RawInputEndpoint> raw_endpoint=nullptr,
                 std::int64_t raw_age=200000000,
                 SimulationFactory simulation_factory={},IkFactory ik_factory={},bool height_calibration=false,
                 std::optional<SessionHandConfig> hand_config=std::nullopt,
                 std::unique_ptr<HandResetEndpoint> hand_reset_endpoint=nullptr)
      : machine_(1000000000,2000000000,hands), period_(period),
        freshness_(freshness), capacity_(capacity) {
    if (period<=0 || freshness<=0 || capacity==0)
      throw std::invalid_argument("positive runtime bounds required");
    if (command_config) commands_=std::make_unique<SessionCommands>(machine_,*command_config);
    if (health_config) {
      if ((hands && !hand_config) || !command_config || health_config->home!=command_config->home ||
          !(health_config->authorities[1]==Authority{command_config->producer,
                                                   command_config->instance,command_config->router}))
        throw std::invalid_argument("health mode requires consistent command and optional hand configuration");
      health_=std::make_unique<SessionHealth>(*health_config);
    }
    if(hand_config) {
      if(!hands || !health_ || !simulation_factory || hand_config->run!=command_config->run ||
         hand_config->producer.router!=command_config->router)
        throw std::invalid_argument("hand domain requires matching owned simulation session");
      hand_domain_=std::make_unique<SessionHandDomain>(*hand_config);
    }
    if(reset_endpoint && (!health_ || reset_endpoint->epoch()!=machine_.state().epoch))
      throw std::invalid_argument("reset endpoint requires health mode and matching epoch");
    reset_endpoint_=std::move(reset_endpoint);
    if(hand_reset_endpoint && (!hand_domain_ || (!reset_endpoint_ && !ik_factory) ||
        hand_reset_endpoint->epoch()!=machine_.state().epoch))
      throw std::invalid_argument("hand reset requires hand domain, arm reset and matching epoch");
    hand_reset_endpoint_=std::move(hand_reset_endpoint);
    hand_pipeline_=dynamic_cast<HandPipelineEndpoint*>(hand_reset_endpoint_.get());
    if(hand_pipeline_) {hand_result_run_=hand_config->run;hand_result_producer_=hand_config->producer;}
    if(raw_endpoint && (!health_ || raw_age<=0)) throw std::invalid_argument("raw input requires healthy arms mode and positive freshness");
    if(raw_endpoint) raw_authority_=health_config->authorities[0];
    raw_endpoint_=std::move(raw_endpoint); raw_age_=raw_age;
    if(simulation_factory && !health_) throw std::invalid_argument("owned simulation requires validated health mode");
    simulation_factory_=std::move(simulation_factory);
    if(simulation_factory_) simulation_authority_=health_config->authorities[2];
    if(ik_factory && (!simulation_factory_ || !raw_endpoint_ || reset_endpoint_ || command_config->clipping))
      throw std::invalid_argument("owned IK requires raw input, owned simulation, direct commands and no separate reset owner");
    ik_factory_=std::move(ik_factory);
    if(height_calibration && !ik_factory_) throw std::invalid_argument("height calibration requires owned IK");
    height_requested_=height_calibration;
    if(ik_factory_) {
      ik_proposal_.run=command_config->run; ik_proposal_.router=command_config->router;
      ik_proposal_.producer=command_config->producer; ik_proposal_.instance=command_config->instance;
    }
  }
  ~SessionRuntime() { stop(); }
  SessionRuntime(const SessionRuntime&)=delete;
  SessionRuntime& operator=(const SessionRuntime&)=delete;
  void enable_auto_home_rearm() {
    std::lock_guard<std::mutex> lock(mutex_);
    if(started_ || stopped_ || !ik_factory_ || !health_ || (hand_domain_ && !hand_pipeline_))
      throw std::logic_error("automatic Home rearm requires unstarted owned arm/hand workers");
    auto_home_rearm_=true;
  }
  void enable_xz_calibration(bool common_x=false) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(started_ || stopped_ || !height_requested_) throw std::logic_error("XZ calibration requires height mode before start");
    xz_requested_=true;
    common_x_requested_=common_x;
  }
  // Opt-in before start. Sole recorder consumer drains pop_cycle; disabling by
  // default leaves legacy callers free of extra copies or queue obligations.
  void enable_cycle_capture(std::size_t capacity) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(started_ || stopped_ || cycle_capacity_) throw std::logic_error("cycle capture configured before start only");
    if(!capacity || capacity>8192) throw std::invalid_argument("cycle capacity must be 1..8192");
    cycle_capacity_=capacity;
  }
  // Opt-in before start.  This is a separate bounded queue because source
  // ingress can be faster than the fixed-rate scheduler and must not be
  // collapsed to the latest accepted frame.
  void enable_raw_capture(std::size_t capacity) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(started_ || stopped_ || raw_capacity_) throw std::logic_error("raw capture configured before start only");
    if(!capacity || capacity>8192) throw std::invalid_argument("raw capture capacity must be 1..8192");
    raw_capacity_=capacity;
  }
  bool pop_cycle(SessionCycleSnapshot& cycle) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(cycles_.empty()) return false;
    cycle=std::move(cycles_.front());cycles_.pop_front();return true;
  }
  bool pop_raw(SessionRawSnapshot& raw) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(raws_.empty()) return false;
    raw=std::move(raws_.front());raws_.pop_front();return true;
  }
  void enable_manus_capture(std::size_t capacity) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(started_ || stopped_ || manus_capacity_ || !hand_pipeline_)
      throw std::logic_error("Manus capture requires unstarted owned hand pipeline");
    if(!capacity || capacity>8192) throw std::invalid_argument("invalid Manus capture capacity");
    manus_capacity_=capacity;
  }
  // Receiver-only admission: raw record and optional control input enter under
  // one lock. Capacity failure cannot admit motion while dropping its raw line.
  bool submit_manus(ManusIngressRecord raw) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(stopped_ || overflow_ || !manus_capacity_) return false;
    if(raw.raw_line.size()>65536 || !raw.line_sequence || raw.line_sequence>=(std::uint64_t(1)<<63) ||
       raw.received_ns<=0 || raw.received_ns>now() || raw.raw_line.find('\0')!=std::string::npos ||
       raw.raw_line.find('\n')!=std::string::npos) return false;
    if(raw.sample && (!raw.sample->source.bounded() ||
       raw.sample->value.timestamp_ns!=static_cast<std::uint64_t>(raw.received_ns))) return false;
    if(manus_raws_.size()==manus_capacity_ || (raw.sample && events_.size()==capacity_)) {
      overflow_=true;wake_.notify_all();return false;
    }
    if(raw.sample) {
      SessionEvent event{0,"hand_sample","",false};event.reply=false;
      event.hand_sample=raw.sample;event.received_ns=now();events_.push_back(std::move(event));
    }
    manus_raws_.push_back(std::move(raw));return true;
  }
  bool pop_manus(ManusIngressRecord& raw) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(manus_raws_.empty()) return false;
    raw=std::move(manus_raws_.front());manus_raws_.pop_front();return true;
  }
  void start() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_);
    std::lock_guard<std::mutex> lock(mutex_);
    if (started_ || stopped_) throw std::logic_error("runtime is single-use");
    worker_=std::thread([this]{ run(); });
    started_=true;
  }
  void stop() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_);
    { std::lock_guard<std::mutex> lock(mutex_);
      stopped_=true; if(reset_endpoint_) reset_endpoint_->request_stop(); if(ik_) ik_->request_stop();
      if(hand_reset_endpoint_) hand_reset_endpoint_->request_stop(); }
    wake_.notify_all();
    if (worker_.joinable()) worker_.join();
    if(reset_future_.valid()) { try { reset_future_.get(); } catch(...) {} }
    reset_endpoint_.reset();
    hand_reset_endpoint_.reset();
    hand_pipeline_=nullptr;
    { std::lock_guard<std::mutex> lock(mutex_); snapshot_.reset_pending=false; }
  }
  bool update(const SessionFacts& facts) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_ || health_) return false;
    facts_=facts; received_=now();
    return true;
  }
  // Local owner-only failure lane, not a wire/operator intent. Independent of
  // queue capacity; the scheduler applies it before adopting any further result.
  // First failure wins. Reporting never grants authority or clears a fault.
  bool report_failure(const std::string& reason) {
    if(reason.empty() || reason.size()>4096 || reason.find('\0')!=std::string::npos) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if(stopped_ || !reported_failure_.empty()) return false;
    reported_failure_=reason; failure_pending_=true; wake_.notify_all(); return true;
  }
  bool submit(SessionEvent event) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_ || overflow_) return false;
    if (events_.size()==capacity_) { overflow_=true; wake_.notify_all(); return false; }
    // Bound payload storage as well as queue entry count.
    if (event.action.size()>32 || event.reason.size()>4096) return false;
    if(simulation_factory_ && (event.feedback || (event.status && event.status->role==2))) return false;
    if(ik_factory_ && (event.proposal || (event.status && event.status->role==1))) return false;
    if (int(event.proposal.has_value())+int(event.status.has_value())+int(event.feedback.has_value())+int(event.raw.has_value())+
        int(event.hand_input.has_value())+int(event.hand_result.has_value())+int(event.hand_sample.has_value())>1)
      return false;
    if(hand_pipeline_ && (event.hand_input || event.hand_result)) return false;
    if(event.hand_sample && (!hand_pipeline_ || !event.hand_sample->source.bounded())) return false;
    if(event.hand_input && (!hand_domain_ || !event.hand_input->source.bounded())) return false;
    if(event.hand_result && (!hand_domain_ || !event.hand_result->producer.bounded() || event.hand_result->run.size()>256)) return false;
    if(event.raw && (!raw_endpoint_ || !event.raw->authority.bounded() || event.raw->bytes.size()>656)) return false;
    if (event.status && (!health_ || !event.status->authority.bounded())) return false;
    if (event.feedback) {
      if (!health_ || !event.feedback->authority.bounded()) return false;
      for(const auto& name:event.feedback->names) if(name.size()>64) return false;
    }
    if (event.proposal && (!commands_ || event.proposal->run.size()>256 ||
        event.proposal->producer.size()>256 || event.proposal->instance.size()>256 ||
        event.proposal->router.size()>256)) return false;
    event.received_ns=now();
    events_.push_back(std::move(event));
    return true;
  }
  bool submit_proposal(std::uint64_t id,SessionProposal proposal) {
    SessionEvent event{id,"proposal","",false};
    event.proposal=std::move(proposal);
    return submit(std::move(event));
  }
  bool pop(SessionReply& reply) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (replies_.empty()) return false;
    reply=std::move(replies_.front()); replies_.pop_front(); return true;
  }
  bool pop_command_receipt(SessionCommandResult& result) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (command_receipts_.empty()) return false;
    result=std::move(command_receipts_.front()); command_receipts_.pop_front(); return true;
  }
  SessionSnapshot snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }
 private:
  void sync_hands(std::int64_t timestamp) {
    if(!hand_domain_) return;
    hand_domain_->sync(machine_.state(),timestamp);
    if(!hand_domain_->error().empty()) machine_.fault(timestamp,hand_domain_->error());
    if(hand_pipeline_) {
      if(machine_.state().phase=="fault") {hand_pipeline_->request_stop();return;}
      if(!snapshot_.reset_pending) {
        const auto phase=machine_.state().phase=="teleop"?tianji_hand::HandPhase::teleop:
          machine_.state().phase=="returning"?tianji_hand::HandPhase::returning:tianji_hand::HandPhase::idle;
        if(!hand_pipeline_->phase(machine_.state().epoch,phase))
          throw std::runtime_error("native hand phase synchronization failed: "+hand_pipeline_->failure());
      }
    }
  }
  void poll_hands(std::int64_t& timestamp,SessionCycleSnapshot* cycle) {
    if(!hand_pipeline_ || snapshot_.reset_pending || machine_.state().phase=="fault") return;
    if(!hand_pipeline_->failure().empty()) throw std::runtime_error(hand_pipeline_->failure());
    tianji_hand::HandCommand result;
    // Bound main-thread work even when a producer outpaces the session.
    for(std::size_t i=0;i<capacity_ && hand_pipeline_->pop(result);++i) {
      timestamp=now();
      HandResultEnvelope envelope{hand_result_run_,hand_result_producer_,result};
      const auto outcome=hand_domain_->result(timestamp,envelope);
      if(cycle) cycle->hand_results.push_back({std::move(envelope),outcome,timestamp});
      sync_hands(timestamp);
    }
  }
  void apply_reported_failure(std::int64_t timestamp) {
    if(!failure_pending_) return;
    machine_.fault(timestamp,reported_failure_); failure_pending_=false;
    if(snapshot_.reset_pending) cancel_reset();
  }
  SessionFacts collect_facts(std::int64_t timestamp) const {
    auto f=health_?health_->facts(timestamp):facts_;
    if(!health_ && (received_<0 || timestamp<received_ || timestamp-received_>freshness_)) f=SessionFacts{};
    if(raw_endpoint_) {
      f.source_ready=f.source_ready && raw_received_>=0 && timestamp>=raw_received_ && timestamp-raw_received_<=raw_age_;
      const bool eligible=!raw_barrier_ || (raw_received_>raw_cutoff_ && raw_progress_.epoch==barrier_epoch_ &&
                                          raw_progress_.generation==barrier_generation_);
      f.input_revision=eligible?raw_revision_:barrier_revision_;
    }
    if(ik_factory_) f.producer_ready=snapshot_.ik_ready && !ik_guard_.reason() &&
      (snapshot_.ik_ticks || (raw_progress_.skeleton_valid && f.source_ready));
    if(hand_domain_) {
      const auto hf=hand_domain_->facts(timestamp);
      f.hand_producer_ready=hf.hand_producer_ready;f.hand_zero=hf.hand_zero;f.hand_tracking=hf.hand_tracking;
    }
    return f;
  }
  bool reset_safe(SessionFacts f) const {
    f=commands_->facts(f);
    // Leaving teleop pauses the execution guard. That pause is not a dead worker:
    // the exact-Home reset transaction is the only route to clear it.
    if(ik_factory_) f.producer_ready=snapshot_.ik_ready && raw_progress_.skeleton_valid;
    return machine_.state().phase=="idle" && f.source_ready && f.producer_ready &&
      f.executor_ready && f.arm_fresh && f.arm_home && f.arm_exact_home && f.command_home &&
      (!hand_domain_ || f.hand_zero) &&
      (!snapshot_.reset_pending || !raw_endpoint_ || (raw_progress_.epoch==reset_raw_epoch_ &&
                                                    raw_progress_.generation==reset_raw_generation_));
  }
  ResetEndpoint* reset_service() const noexcept {
    return ik_?ik_->reset_service():reset_endpoint_.get();
  }
  void cancel_reset() noexcept {
    if(auto* endpoint=reset_service()) endpoint->request_stop();
    if(hand_reset_endpoint_) hand_reset_endpoint_->request_stop();
  }
  IntentOutcome begin_reset(const SessionEvent& event,SessionFacts facts,bool with_height=false) {
    if(!event.authority) return {false,"intent source authority mismatch"};
    auto* endpoint=reset_service();
    if(!endpoint) return {false,"native reset endpoint required"};
    if(hand_domain_ && !hand_reset_endpoint_) return {false,"native hand reset endpoint required"};
    const auto epoch=machine_.state().epoch;
    if(!reset_safe(facts) || epoch==std::numeric_limits<std::int64_t>::max() ||
       event.next_epoch!=epoch+1 || endpoint->epoch()!=epoch ||
       (hand_reset_endpoint_ && hand_reset_endpoint_->epoch()!=epoch))
      return {false,"native rearm requires healthy exact Home and next epoch"};
    std::array<double,14> q;
    for(int side=0;side<2;++side) for(int j=0;j<7;++j) q[side*7+j]=commands_->positions()[side][j];
    reset_event_=event;
    auto_home_pending_=false;
    reset_height_=with_height;
    const auto offsets=with_height?height_->candidate():std::nullopt;
    const auto x_offsets=with_height?height_->candidate_x():std::nullopt;
    if(with_height && (!offsets || !endpoint->supports_height())) throw std::logic_error("height reset requires supported candidate");
    reset_raw_epoch_=raw_progress_.epoch; reset_raw_generation_=raw_progress_.generation;
    auto* hand=hand_reset_endpoint_.get();
    reset_future_=std::async(std::launch::async,[endpoint,hand,q,epoch=event.next_epoch,offsets,x_offsets] {
      endpoint->reset(q,epoch);
      if(offsets) {
        if(x_offsets) endpoint->configure_xz(*x_offsets,*offsets);
        else endpoint->configure_height(*offsets);
      }
      if(endpoint->epoch()!=epoch) throw std::runtime_error("arm reset epoch mismatch");
      if(hand) {
        hand->reset(epoch);
        if(hand->epoch()!=epoch) throw std::runtime_error("hand reset epoch mismatch");
      }
      return endpoint->epoch();
    });
    snapshot_.reset_pending=true;
    return {true,"native reset pending"};
  }
  void finish_reset(std::int64_t& timestamp,SessionFacts& facts) {
    if(!snapshot_.reset_pending) return;
    timestamp=now(); facts=collect_facts(timestamp);
    if(!reset_safe(facts)) {
      machine_.fault(timestamp,"native reset lost healthy exact Home");
      cancel_reset();
    }
    if(reset_future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready) return;
    IntentOutcome outcome;
    try {
      const auto epoch=reset_future_.get();
      timestamp=now(); facts=collect_facts(timestamp);
      if(epoch!=reset_event_.next_epoch || !reset_safe(facts))
        throw std::runtime_error("native reset postcondition failed");
      if(raw_endpoint_) facts.input_revision=raw_revision_; // Snapshot all pre-commit input, even behind an older barrier.
      outcome=commands_->rearm(timestamp,epoch,facts,true);
      if(!outcome.accepted) throw std::runtime_error(outcome.reason);
      if(ik_) {
        ik_guard_=ExecutionGuard(100000000,1);
        snapshot_.ik_ticks=0; snapshot_.ik_reference.reset(); pending_ik_input_.reset();
      }
      if(reset_height_) height_->commit();
      if(raw_endpoint_) {
        raw_barrier_=true; raw_cutoff_=timestamp; barrier_revision_=raw_revision_;
        barrier_epoch_=raw_progress_.epoch; barrier_generation_=raw_progress_.generation;
      }
    } catch(const std::exception& error) {
      machine_.fault(timestamp,std::string("native reset failed: ")+error.what());
      cancel_reset(); outcome={false,machine_.state().reason};
      if(reset_height_) height_->fail("height reset/configuration failed");
    }
    replies_.push_back({reset_event_.id,std::move(outcome),reset_event_.action,timestamp});
    if(reset_height_) height_reply_pending_=false;
    snapshot_.reset_pending=false;
    reset_height_=false;
  }
  IntentOutcome begin_height(const SessionEvent& event,SessionFacts facts,std::int64_t timestamp) {
    if(!event.authority) return {false,"intent source authority mismatch"};
    if(height_->status().state=="collecting" || snapshot_.reset_pending) return {false,"height calibration already pending"};
    if(!reset_safe(facts)) return {false,"height calibration requires healthy exact Home"};
    report_height_failure(timestamp); // Finish the previous request before a retry replaces its ID.
    height_->begin(timestamp); height_event_=event; height_reply_pending_=true;
    return {true,"hold both arms horizontal and steady for 2 seconds"};
  }
  void advance_height(std::int64_t timestamp,SessionFacts facts) {
    report_height_failure(timestamp);
    if(!height_ || height_->status().state!="collecting") return;
    if(!reset_safe(facts)) {
      height_->fail("session/input unhealthy during calibration"); report_height_failure(timestamp); return;
    }
    // Match the existing control loop's latest-sample consumption, not every
    // datagram that happened to accumulate in the ingress queue this cycle.
    height_->add(raw_progress_,raw_received_,timestamp);
    if(!height_->tick(timestamp)) { report_height_failure(timestamp); return; }
    auto event=height_event_; event.action="height_commit";
    if(machine_.state().epoch==std::numeric_limits<std::int64_t>::max()) {
      height_->fail("execution epoch exhausted"); report_height_failure(timestamp); return;
    }
    event.next_epoch=machine_.state().epoch+1;
    auto outcome=begin_reset(event,facts,true);
    if(!outcome.accepted) { height_->fail(outcome.reason); report_height_failure(timestamp); }
  }
  void report_height_failure(std::int64_t timestamp) {
    if(!height_reply_pending_ || !height_ || height_->status().state!="failed") return;
    replies_.push_back({height_event_.id,{false,height_->status().error},"calibrate",timestamp});
    height_reply_pending_=false;
  }
  static std::int64_t now() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void publish_simulation(SimulationEndpoint& simulation,std::int64_t timestamp) {
    if(simulation_sequence_==std::numeric_limits<std::int64_t>::max()) throw std::overflow_error("simulation sequence exhausted");
    const auto q=simulation.feedback();
    ArmFeedback feedback; feedback.authority=simulation_authority_; feedback.sequence=++simulation_sequence_;
    feedback.names=arm_names(); feedback.positions=q;
    if(!health_->feedback(timestamp,feedback).accepted) throw std::runtime_error(health_->error());
    HealthStatus status; status.role=2; status.authority=simulation_authority_; status.sequence=simulation_sequence_;
    status.ready=status.healthy=status.simulation=true;
    if(!health_->status(timestamp,status).accepted) throw std::runtime_error(health_->error());
    snapshot_.feedback=q; snapshot_.simulation_ready=true;
    if(hand_domain_) {
      const auto hands=simulation.hand_feedback();
      if(!hands) throw std::runtime_error("owned simulation omitted required hand feedback");
      hand_domain_->feedback(timestamp,*hands);
      if(!hand_domain_->error().empty()) throw std::runtime_error(hand_domain_->error());
      snapshot_.hand_feedback=hands;
    }
  }
  void run() noexcept {
    std::unique_lock<std::mutex> lock(mutex_);
    std::unique_ptr<SimulationEndpoint> simulation;
    try {
      if(simulation_factory_) {
        // Model loading does not block event/snapshot admission behind this mutex.
        // The native loader itself is synchronous; stop joins after it returns.
        lock.unlock();
        try { simulation=simulation_factory_(); } catch(...) { lock.lock(); throw; }
        lock.lock();
        if(!simulation) throw std::runtime_error("simulation factory returned null");
        if(!stopped_) {
          if(hand_domain_) simulation->apply_frame(commands_->positions(),
            HandUpdates{hand_domain_->zero()[0],hand_domain_->zero()[1]});
          else simulation->apply(commands_->positions());
        }
        // First feedback follows queued ingress, whose admission timestamps can
        // precede loading. Never clamp/rejuvenate those timestamps to hide order.
      }
      if(ik_factory_ && !stopped_) {
        lock.unlock();
        std::unique_ptr<IkEndpoint> endpoint;
        try { endpoint=ik_factory_(); } catch(...) { lock.lock(); throw; }
        lock.lock(); ik_=std::move(endpoint);
        if(!ik_) throw std::runtime_error("IK factory returned null");
        snapshot_.ik_ready=!stopped_;
      }
      if(height_requested_ && !stopped_) {
        auto* service=reset_service();
        if(!service || !service->supports_height()) throw std::invalid_argument("height calibration requires mapped-palm reset service");
        if(xz_requested_) {
          const auto reference=simulation->forward_reference();
          if(!reference) throw std::invalid_argument("simulation has no forward TCP reference");
          height_=std::make_unique<HeightCalibration>(
            std::array<double,2>{(*reference)[0][1],(*reference)[1][1]},
            std::array<double,2>{(*reference)[0][0],(*reference)[1][0]},common_x_requested_);
        } else {
          const auto reference=simulation->height_reference();
          if(!reference) throw std::invalid_argument("simulation has no horizontal TCP reference");
          height_=std::make_unique<HeightCalibration>(*reference);
        }
        snapshot_.height=height_->status();
      }
      AbsoluteDeadline deadline(now(),period_);
      snapshot_.running=true;
      while (!stopped_) {
        std::optional<SessionCycleSnapshot> cycle;
        auto timestamp=now();
        apply_reported_failure(timestamp);
        sync_hands(timestamp);
        if(cycle_capacity_ && cycles_.size()==cycle_capacity_) {
          snapshot_.cycle_capture_failed=true;
          machine_.fault(timestamp,"native cycle capture queue overflow");
          if(snapshot_.reset_pending) cancel_reset();
        }
        if(cycle_capacity_ && !snapshot_.cycle_capture_failed) cycle.emplace();
        poll_hands(timestamp,cycle?&*cycle:nullptr);
        SessionFacts facts=collect_facts(timestamp);
        // Reserve room for this cycle's disposition before accepting work.
        if (commands_ && command_receipts_.size()==capacity_) overflow_=true;
        if (overflow_) machine_.fault(timestamp,"session event/reply queue overflow");
        // Finite batch: callers cannot append while this lock is held.
        auto process_events=[&] {
        while (!events_.empty() && !overflow_) {
          if (replies_.size()+(snapshot_.reset_pending?1:0)>=capacity_) {
            overflow_=true; machine_.fault(timestamp,"session event/reply queue overflow"); break;
          }
          auto event=std::move(events_.front()); events_.pop_front();
          IntentOutcome outcome;
          if(event.hand_sample) {
            if(snapshot_.reset_pending) outcome={false,"hand sample rejected during reset"};
            else {
              const auto& sample=*event.hand_sample;const auto& v=sample.value;
              if(v.timestamp_ns>static_cast<std::uint64_t>(event.received_ns))
                throw std::runtime_error("hand sample timestamp after local admission");
              outcome=hand_domain_->input(timestamp,HandInputReceipt{sample.source,v.sequence,v.timestamp_ns,v.generation,v.flags});
              sync_hands(timestamp);
              if(outcome.accepted && !hand_pipeline_->submit(v))
                throw std::runtime_error("native hand sample queue rejected: "+hand_pipeline_->failure());
              facts=collect_facts(timestamp);
            }
          } else if(event.hand_input || event.hand_result) {
            const auto source_time=event.hand_input?event.hand_input->timestamp_ns:event.hand_result->value.scheduler_timestamp_ns;
            if(source_time>static_cast<std::uint64_t>(event.received_ns))
              throw std::runtime_error("hand event timestamp after local admission");
            outcome=event.hand_input?hand_domain_->input(timestamp,*event.hand_input):hand_domain_->result(timestamp,*event.hand_result);
            sync_hands(timestamp);facts=collect_facts(timestamp);
          } else if(event.raw) {
            if(!(event.raw->authority==raw_authority_)) {
              machine_.fault(timestamp,"raw input source authority mismatch"); outcome={false,machine_.state().reason};
            } else {
              auto progress=raw_endpoint_->ingest(event.raw->bytes);
              const auto received=event.raw->received_ns>=0?event.raw->received_ns:event.received_ns;
              if(progress.decoded) {
                if(progress.ingress_sequence==0 || received<=0 || received>event.received_ns)
                  throw std::runtime_error("invalid native decoded raw receive metadata");
                if(raw_capacity_ && raws_.size()==raw_capacity_) {
                  snapshot_.cycle_capture_failed=true;
                  machine_.fault(timestamp,"native raw capture queue overflow");
                } else if(raw_capacity_) {
                  raws_.push_back(SessionRawSnapshot{progress.ingress_sequence,received,
                                                      progress.accepted,event.raw->bytes});
                }
              }
              if(progress.accepted) {
                if(raw_revision_==std::numeric_limits<std::uint64_t>::max()) throw std::overflow_error("raw input revision exhausted");
                ++raw_revision_; raw_progress_=progress;
                raw_received_=received;
                if(raw_received_<=0 || raw_received_>event.received_ns)
                  throw std::runtime_error("invalid native raw receive timestamp");
                if(ik_factory_) {
                  WorkerTick sample; sample.packet=event.raw->bytes; sample.received_ns=raw_received_;
                  sample.source_sequence=progress.ingress_sequence;
                  sample.generation=progress.generation; sample.discontinuity=progress.discontinuity;
                  pending_ik_input_=std::move(sample);
                }
              }
              outcome={progress.accepted,progress.accepted?"raw frame accepted":"raw frame rejected"};
            }
            facts=collect_facts(timestamp);
          } else if (event.status || event.feedback) {
            outcome=event.status?health_->status(event.received_ns,*event.status):
                                 health_->feedback(event.received_ns,*event.feedback);
            facts=collect_facts(timestamp);
            if (!health_->error().empty()) machine_.fault(timestamp,health_->error());
          } else if(snapshot_.reset_pending) {
            outcome={false,"native reset pending; no new command or operator transition"};
          } else if(height_ && event.action=="calibrate") {
            outcome=begin_height(event,facts,timestamp);
          } else if(height_ && event.action=="start" && !height_->ready()) {
            outcome={false,"complete height calibration before start"};
          } else if(height_ && event.action=="rearm" && height_->status().state=="collecting") {
            outcome={false,"height calibration is collecting"};
          } else if(ik_factory_ && event.action=="start" && snapshot_.ik_ticks) {
            outcome={false,"owned IK requires acknowledged Home rearm before another start"};
          } else if (event.proposal) {
            const bool accepted=commands_->accept(timestamp,*event.proposal);
            outcome={accepted,accepted?"proposal queued for command tick":"proposal rejected"};
          } else if (event.action=="rearm" && health_) {
            outcome=begin_reset(event,facts);
            if(outcome.accepted) continue; // One final reply only, after acknowledged commit.
          }
          else if (event.action=="rearm" && event.authority)
            outcome=commands_?commands_->rearm(timestamp,event.next_epoch,facts,event.reset_ack):
                              machine_.rearm(timestamp,event.next_epoch,facts,event.reset_ack);
          else {
            outcome=commands_?commands_->intent(timestamp,event.action,event.reason,event.authority,facts):
                              machine_.intent(timestamp,event.action,event.reason,event.authority,facts);
            if(outcome.accepted && event.action=="shutdown") auto_home_pending_=false;
            if(outcome.accepted && event.action=="return" && auto_home_rearm_ && snapshot_.ik_ticks)
              auto_home_pending_=true;
            if(height_ && outcome.accepted && (event.action=="return" || event.action=="shutdown") && height_->status().state=="collecting") {
              height_->fail("calibration cancelled by operator");
              report_height_failure(timestamp);
            }
          }
          if(!event.raw && !event.status && !event.feedback && !event.proposal && !event.hand_input && !event.hand_result && !event.hand_sample &&
             event.action=="start" && outcome.accepted) raw_barrier_=false;
          if(ik_factory_ && snapshot_.ik_ticks && machine_.state().phase!="teleop")
            ik_guard_.pause("session left teleop; joint reset and re-authorization required");
          if(hand_domain_) {sync_hands(timestamp);facts=collect_facts(timestamp);}
          if(event.reply) replies_.push_back({event.id,std::move(outcome),event.action,timestamp});
        }
        };
        process_events();
        if(auto_home_pending_ && !snapshot_.reset_pending && machine_.state().phase=="idle" &&
           machine_.state().return_complete && !machine_.state().shutdown_complete && reset_safe(facts)) {
          if(machine_.state().epoch==std::numeric_limits<std::int64_t>::max())
            throw std::overflow_error("automatic rearm epoch exhausted");
          if(replies_.size()>=capacity_)
            throw std::runtime_error("automatic rearm reply queue full");
          // ID 0 is reserved for internal completion reports; not a wire command.
          SessionEvent automatic{0,"auto_rearm","",true,machine_.state().epoch+1,false};
          auto_home_pending_=false;
          auto outcome=begin_reset(automatic,facts);
          if(!outcome.accepted) {
            machine_.fault(timestamp,"automatic Home rearm failed: "+outcome.reason);
            replies_.push_back({0,std::move(outcome),"auto_rearm",timestamp});
          }
        }
        advance_height(timestamp,facts);
        finish_reset(timestamp,facts);
        if(hand_domain_) {sync_hands(timestamp);facts=collect_facts(timestamp);}
        if(ik_ && !snapshot_.reset_pending && machine_.state().phase=="teleop") {
          machine_.tick(timestamp,commands_->facts(facts));
          if(machine_.state().phase=="teleop") {
            if(snapshot_.ik_ticks>=static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) throw std::overflow_error("IK tick exhausted");
            WorkerTick request=pending_ik_input_?std::move(*pending_ik_input_):WorkerTick{};
            pending_ik_input_.reset(); request.id=snapshot_.ik_ticks+1; request.now_ns=timestamp;
            if(cycle) {
              if(request.packet.size()>656) throw std::runtime_error("oversized captured IK request");
              cycle->request=request;
            }
            WorkerResult result;
            lock.unlock();
            try { result=ik_->step(request); } catch(...) { lock.lock(); throw; }
            lock.lock();
            if(stopped_) break; // Cancellation cannot authorize a late result.
            if(cycle) {
              if(result.wire.size()>1206) throw std::runtime_error("oversized captured IK result");
              cycle->result=result;
            }
            timestamp=now(); facts=collect_facts(timestamp);
            apply_reported_failure(timestamp);
            if(overflow_) machine_.fault(timestamp,"session event/reply queue overflow");
            if(machine_.state().phase=="teleop") {
              if(result.tick!=request.id || result.timestamp!=static_cast<std::uint64_t>(request.now_ns))
                throw std::runtime_error("unassociated scheduled IK result");
              if(height_ && height_->status().offsets && (!result.height_present || result.height_offsets!=*height_->status().offsets))
                throw std::runtime_error("scheduled IK height differs from committed calibration");
              if(height_ && height_->status().x_offsets && (!result.x_present || result.x_offsets!=*height_->status().x_offsets))
                throw std::runtime_error("scheduled IK X differs from committed calibration");
              snapshot_.ik_ticks=result.tick;
            }
            // Consume ingress that arrived during IPC before publishing later
            // simulation feedback. Operators can cancel adoption of this result.
            process_events(); facts=collect_facts(timestamp);
            machine_.tick(timestamp,commands_->facts(facts));
            if(machine_.state().phase=="teleop") {
              auto proposal=ik_proposal_; proposal.epoch=machine_.state().epoch;
              proposal.tick=result.tick; proposal.timestamp=request.now_ns;
              Positions expected{};
              for(int s=0;s<2;++s) for(int j=0;j<7;++j) expected[s*7+j]=proposal.positions[s][j]=result.arms[s].q[j];
              ik_guard_.register_tick(proposal.tick,request.now_ns,expected);
              if(!ik_guard_.check(timestamp)) throw std::runtime_error(*ik_guard_.reason());
              if(!commands_->accept(timestamp,proposal)) throw std::runtime_error("scheduled IK proposal rejected");
              snapshot_.ik_reference=proposal.positions;
              if(cycle) cycle->ik_adopted=true;
            }
          }
        }
        if (commands_) {
          snapshot_.command=commands_->tick(timestamp,facts);
          if(ik_ && snapshot_.command->receipt) {
            const auto& receipt=*snapshot_.command->receipt; Positions q{};
            for(int s=0;s<2;++s) for(int j=0;j<7;++j) q[s*7+j]=snapshot_.command->positions[s][j];
            if(!ik_guard_.check(now()) || !ik_guard_.observe(receipt.tick,receipt.accepted,receipt.reason,q))
              throw std::runtime_error("scheduled IK execution guard rejected command");
          }
          if(simulation) {
            if(hand_domain_) {
              sync_hands(timestamp);
              snapshot_.hand_command=hand_domain_->take(timestamp);
              simulation->apply_frame(snapshot_.command->positions,snapshot_.hand_command);
            } else simulation->apply(snapshot_.command->positions);
            publish_simulation(*simulation,now());
          }
          if (snapshot_.command->receipt) command_receipts_.push_back(*snapshot_.command);
        }
        else machine_.tick(timestamp,facts);
        snapshot_.state=machine_.state(); snapshot_.queue_overflow=overflow_;
        if(height_) snapshot_.height=height_->status();
        ++snapshot_.ticks;
        if (now()>deadline.next()) ++snapshot_.late_ticks;
        if(cycle) {
          cycle->timestamp_ns=now();cycle->snapshot=snapshot_;
          cycle->source=raw_progress_;cycle->source_received_ns=raw_received_;cycle->source_revision=raw_revision_;
          cycle->ik_adopted=cycle->ik_adopted && snapshot_.command && snapshot_.command->receipt && snapshot_.command->receipt->accepted;
          cycles_.push_back(std::move(*cycle));
        }
        if (snapshot_.state.shutdown_complete) { stopped_=true; break; }
        const bool interrupted=wake_.wait_until(lock,std::chrono::steady_clock::time_point(
            std::chrono::nanoseconds(deadline.next())),[this]{return stopped_ || failure_pending_;});
        if(!interrupted) deadline.advance(); // Failure wakeups don't consume a normal deadline.
      }
    } catch (const std::exception& error) {
      machine_.fault(now(),failure_pending_?reported_failure_:error.what()); snapshot_.state=machine_.state(); stopped_=true;
    } catch (...) {
      machine_.fault(now(),"unknown native supervisor failure");
      snapshot_.state=machine_.state(); stopped_=true;
    }
    simulation.reset(); // Including faults/interruption: destroy on the owning scheduler thread.
    if(height_ && (height_->status().state=="collecting" || height_->status().state=="sampled")) height_->fail("calibration interrupted");
    report_height_failure(now());
    if(height_) snapshot_.height=height_->status();
    if(hand_reset_endpoint_) hand_reset_endpoint_->request_stop();
    if(ik_) {
      ik_->request_stop();
      // A reset task borrows the IK service. Join before destroying that service,
      // even on fault or stop; no reset epoch may commit from this cleanup path.
      if(reset_future_.valid()) {
        lock.unlock();
        try { reset_future_.get(); } catch(...) {}
        lock.lock();
      }
      snapshot_.reset_pending=false;
      ik_.reset();
    }
    if(ik_factory_) ik_guard_.pause("native IK closed");
    snapshot_.ik_ready=false;
    snapshot_.simulation_ready=false;
    snapshot_.running=false;
  }
  SessionMachine machine_;
  bool height_requested_=false,reset_height_=false;
  bool xz_requested_=false;
  bool common_x_requested_=false;
  bool height_reply_pending_=false;
  std::unique_ptr<HeightCalibration> height_;
  SessionEvent height_event_{0,"","",false};
  IkFactory ik_factory_;
  std::unique_ptr<IkEndpoint> ik_;
  SessionProposal ik_proposal_;
  std::optional<WorkerTick> pending_ik_input_;
  ExecutionGuard ik_guard_{100000000,1};
  SimulationFactory simulation_factory_;
  Authority simulation_authority_;
  std::int64_t simulation_sequence_=0;
  std::unique_ptr<SessionCommands> commands_;
  std::unique_ptr<SessionHealth> health_;
  std::unique_ptr<SessionHandDomain> hand_domain_;
  std::unique_ptr<HandResetEndpoint> hand_reset_endpoint_;
  HandPipelineEndpoint* hand_pipeline_=nullptr; // Borrowed; owned by hand_reset_endpoint_.
  std::string hand_result_run_;
  Authority hand_result_producer_;
  std::unique_ptr<ResetEndpoint> reset_endpoint_;
  std::future<std::int64_t> reset_future_;
  SessionEvent reset_event_{0,"","",false};
  std::unique_ptr<RawInputEndpoint> raw_endpoint_;
  Authority raw_authority_;
  RawProgress raw_progress_;
  std::uint64_t raw_revision_=0,barrier_revision_=0,barrier_epoch_=0,barrier_generation_=0;
  std::uint64_t reset_raw_epoch_=0,reset_raw_generation_=0;
  std::int64_t raw_age_=200000000,raw_received_=-1,raw_cutoff_=0;
  bool raw_barrier_=false;
  bool auto_home_rearm_=false,auto_home_pending_=false;
  const std::int64_t period_, freshness_;
  const std::size_t capacity_;
  mutable std::mutex mutex_;
  std::mutex lifecycle_;
  std::condition_variable wake_;
  std::thread worker_;
  std::deque<SessionEvent> events_;
  std::deque<SessionReply> replies_;
  std::deque<SessionCommandResult> command_receipts_;
  std::deque<SessionCycleSnapshot> cycles_;
  std::deque<SessionRawSnapshot> raws_;
  std::deque<ManusIngressRecord> manus_raws_;
  std::size_t manus_capacity_=0;
  std::size_t cycle_capacity_=0;
  std::size_t raw_capacity_=0;
  SessionFacts facts_;
  SessionSnapshot snapshot_;
  std::int64_t received_=-1;
  bool started_=false, stopped_=false, overflow_=false;
  std::string reported_failure_;
  bool failure_pending_=false;
};
}  // namespace tianji_control
