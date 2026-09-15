#pragma once
#include "hand_pipeline_endpoint.hpp"
#include "../hand/worker_client.hpp"
#include <deque>
#include <future>
namespace tianji_control {
// One application IPC owner for samples, phases AND reset. The separate native
// Hand2 child retains solver/filter ownership; neither thread calls Python per tick.
class NativeHandPipeline final:public HandPipelineEndpoint {
 public:
  NativeHandPipeline(const std::vector<std::string>& command,int timeout_ms,std::size_t capacity)
    :capacity_(checked_capacity(capacity)),client_(command,timeout_ms),thread_([this]{run();}){}
  ~NativeHandPipeline() override {request_stop();if(thread_.joinable()) thread_.join();}
  std::int64_t epoch() const override {return epoch_.load();}
  void request_stop() noexcept override {stopped_.store(true);client_.request_stop();wake_.notify_all();}
  bool phase(std::int64_t epoch,tianji_hand::HandPhase phase) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if(stopped_ || resetting_) return false;
    if(epoch!=epoch_ || static_cast<unsigned>(phase)>3) return fail_locked("hand phase/epoch mismatch");
    if(phase==queued_phase_) return true;
    // SessionHandDomain retires all pending input associations at a phase
    // boundary. Do not solve their queued samples before delivering the new
    // phase: a startup backlog can otherwise exhaust the freshness window.
    // An already in-flight result is still fenced by the domain's phase check.
    jobs_.clear();outputs_.clear();
    Job job;job.phase=phase;job.kind=Kind::phase;
    if(!enqueue_locked(std::move(job))) return false;
    queued_phase_=phase;return true;
  }
  bool submit(const tianji_hand::HandInput& input) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if(stopped_ || resetting_) return false;
    // Validate even samples superseded before solving: malformed input must
    // never disappear merely because a newer callback arrived.
    for(double value:input.points) if(!std::isfinite(value)) return fail_locked("nonfinite hand input");
    Job job;job.kind=Kind::sample;job.input=input;job.phase=queued_phase_;
    // Match the scheduler's latest-waiting-input semantics. The in-flight solve
    // remains untouched, and SessionRuntime has already captured every raw
    // callback independently. Never coalesce across a phase/reset job.
    if(!jobs_.empty() && jobs_.back().kind==Kind::sample) {
      jobs_.back()=std::move(job);return true;
    }
    return enqueue_locked(std::move(job));
  }
  bool pop(tianji_hand::HandCommand& result) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if(stopped_ || resetting_ || outputs_.empty()) return false;
    result=std::move(outputs_.front());outputs_.pop_front();return true;
  }
  std::string failure() const override {std::lock_guard<std::mutex> lock(mutex_);return error_;}
  void reset(std::int64_t next_epoch) override {
    auto promise=std::make_shared<std::promise<void>>();auto future=promise->get_future();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if(stopped_ || resetting_ || epoch_==std::numeric_limits<std::int64_t>::max() || next_epoch!=epoch_+1)
        throw std::runtime_error("invalid hand pipeline reset");
      resetting_=true;jobs_.clear();outputs_.clear();
      Job job;job.kind=Kind::reset;job.epoch=next_epoch;job.done=promise;jobs_.push_back(std::move(job));wake_.notify_all();
    }
    // At most one in-flight sample precedes this reset; both IPC calls have
    // bounded deadlines/cancellation. Only the reset task, never main tick, waits.
    future.get();
  }
 private:
  enum class Kind {sample,phase,reset};
  struct Job {
    Kind kind=Kind::sample;
    tianji_hand::HandInput input;
    tianji_hand::HandPhase phase=tianji_hand::HandPhase::idle;
    std::int64_t epoch=1;
    std::shared_ptr<std::promise<void>> done;
  };
  static std::size_t checked_capacity(std::size_t n) {
    if(!n || n>8192) throw std::invalid_argument("bounded hand pipeline capacity required");
    return n;
  }
  bool fail_locked(const std::string& reason) {
    if(error_.empty()) error_=reason;
    request_stop();return false;
  }
  bool enqueue_locked(Job job) {
    if(jobs_.size()==capacity_) return fail_locked("hand pipeline input queue overflow");
    jobs_.push_back(std::move(job));wake_.notify_all();return true;
  }
  void run() noexcept {
    Job job;
    try {
      while(true) {
        {
          std::unique_lock<std::mutex> lock(mutex_);
          wake_.wait(lock,[&]{return stopped_ || !jobs_.empty();});
          if(stopped_) break;
          job=std::move(jobs_.front());jobs_.pop_front();
        }
        if(job.kind==Kind::reset) {
          client_.reset_at_idle(job.epoch);
          std::lock_guard<std::mutex> lock(mutex_);
          if(stopped_) throw std::runtime_error("hand reset cancelled before hand commit");
          epoch_.store(client_.epoch());queued_phase_=tianji_hand::HandPhase::idle;
          outputs_.clear();resetting_=false;job.done->set_value();job.done.reset();
        } else {
          client_.sync_phase(job.phase);
          if(job.kind==Kind::sample) {
            client_.input(job.input);auto result=client_.result();
            std::lock_guard<std::mutex> lock(mutex_);
            if(stopped_) break;
            if(resetting_) continue; // Pre-reset solve is not a post-reset result.
            if(outputs_.size()==capacity_) throw std::runtime_error("hand pipeline output queue overflow");
            outputs_.push_back(std::move(result));
          }
        }
      }
    } catch(const std::exception& e) {
      std::lock_guard<std::mutex> lock(mutex_);fail_locked(e.what());
    } catch(...) {
      std::lock_guard<std::mutex> lock(mutex_);fail_locked("unknown hand pipeline failure");
    }
    // Close/reap here, not only in the endpoint destructor: a faulted session
    // may retain the endpoint for diagnostics. This is resource shutdown, not
    // a successful Home or recording-drain acknowledgement.
    try {client_.shutdown();} catch(...) {} // Client closes on cancelled/failed IPC too.
    // Release any pending reset waiter, including a job not yet taken by owner.
    const auto failure=std::make_exception_ptr(std::runtime_error("hand pipeline closed during reset"));
    std::lock_guard<std::mutex> lock(mutex_);
    if(job.done) job.done->set_exception(failure);
    for(auto& pending:jobs_) if(pending.done) pending.done->set_exception(failure);
    jobs_.clear();outputs_.clear();resetting_=false;
  }
  const std::size_t capacity_;
  tianji_hand::NativeHandWorkerClient client_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<Job> jobs_;
  std::deque<tianji_hand::HandCommand> outputs_;
  std::string error_;
  std::atomic<bool> stopped_{false};
  std::atomic<std::int64_t> epoch_{1};
  bool resetting_=false;
  tianji_hand::HandPhase queued_phase_=tianji_hand::HandPhase::idle;
  std::thread thread_;
};
}
