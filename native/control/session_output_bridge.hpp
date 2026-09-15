#pragma once
#include "session_runtime.hpp"
#include "bounded_output.hpp"
#include <atomic>
#include <variant>

namespace tianji_control {
using SessionOutputItem=std::variant<SessionReply,SessionCommandResult,SessionCycleSnapshot,SessionRawSnapshot,ManusIngressRecord>;
struct SessionOutputStats {
  OutputStats output;
  std::string failure;
  bool session_complete=false;
};
// Sole consumer of runtime reply/receipt/opt-in cycle queues. Runtime must outlive this owner;
// other threads must not pop those queues. Separate queues retain their own FIFO
// order; no total chronology across the queues is claimed.
// The output implementation must meet BoundedOutput's bounded/cancellable I/O
// contract. This adapter neither declares publishers nor creates HDF5 schemas.
class SessionOutputBridge {
 public:
  using Write=BoundedOutput<SessionOutputItem>::Write;
  using Finalize=std::function<void(bool)>; // Complete session vs drained incomplete session.
  SessionOutputBridge(SessionRuntime& runtime,std::size_t capacity,Write write,
                      Finalize finalize,std::function<void()> cancel)
    :runtime_(runtime),finalize_(validate_finalize(std::move(finalize))),
     output_(capacity,std::move(write),[this] {
       finalize_(runtime_.snapshot().state.shutdown_complete);
     },std::move(cancel)) {
    pump_=std::thread([this]{run();});
  }
  ~SessionOutputBridge() {abort();}
  SessionOutputBridge(const SessionOutputBridge&)=delete;
  SessionOutputBridge& operator=(const SessionOutputBridge&)=delete;
  // Stop ingress producers first. Explicit operator shutdown/Home should already
  // have completed; otherwise this interrupts runtime and finalizes incomplete.
  void finish() {
    std::lock_guard<std::mutex> lock(lifecycle_);
    if(closed_) return;
    runtime_.stop(); draining_=true;
    if(pump_.joinable()) pump_.join();
    if(aborted_ || !failure().empty()) output_.abort();
    else output_.finish();
    propagate_output_failure();
    closed_=true;
  }
  void abort() noexcept {
    if(closed_) return;
    aborted_=true; output_.abort(); // Cancel blocked writes before waiting for finish.
    std::lock_guard<std::mutex> lock(lifecycle_);
    runtime_.stop(); draining_=true;
    if(pump_.joinable()) pump_.join();
    closed_=true;
  }
  SessionOutputStats stats() const {
    auto out=output_.stats(); auto error=failure();
    if(error.empty()) error=out.failure;
    return {out,error,!aborted_ && out.complete && error.empty() && runtime_.snapshot().state.shutdown_complete};
  }
 private:
  static Finalize validate_finalize(Finalize fn) {
    if(!fn) throw std::invalid_argument("session finalizer required");
    return fn;
  }
  std::string failure() const {std::lock_guard<std::mutex> lock(mutex_); return failure_;}
  void fail(std::string reason) {
    if(reason.empty()) reason="native session output failed";
    if(reason.size()>4096) reason.resize(4096);
    for(auto& ch:reason) if(ch=='\0') ch='?';
    {std::lock_guard<std::mutex> lock(mutex_); if(!failure_.empty()) return; failure_=reason;}
    runtime_.report_failure(reason);
  }
  bool propagate_output_failure() {
    const auto error=output_.stats().failure;
    if(error.empty()) return false;
    fail(error); return true;
  }
  bool submit(SessionOutputItem item) {
    if(aborted_) return false;
    if(output_.submit(std::move(item))) return true;
    if(!aborted_ && !propagate_output_failure()) fail("native session output rejected admission");
    return false;
  }
  void run() noexcept {
    try {
      while(!aborted_) {
        if(propagate_output_failure()) return;
        bool received=false;
        // Finite, fair batches; a busy reply stream cannot starve receipts or
        // failure/cancellation checks. No callback runs on the scheduler thread.
        for(int i=0;i<64;++i) {
          SessionReply reply;
          if(!runtime_.pop(reply)) break;
          received=true; if(!submit(std::move(reply))) return;
        }
        for(int i=0;i<64;++i) {
          SessionCommandResult receipt;
          if(!runtime_.pop_command_receipt(receipt)) break;
          received=true; if(!submit(std::move(receipt))) return;
        }
        for(int i=0;i<64;++i) {
          SessionCycleSnapshot cycle;
          if(!runtime_.pop_cycle(cycle)) break;
          received=true; if(!submit(std::move(cycle))) return;
        }
        for(int i=0;i<64;++i) {
          SessionRawSnapshot raw;
          if(!runtime_.pop_raw(raw)) break;
          received=true; if(!submit(std::move(raw))) return;
        }
        for(int i=0;i<64;++i) {
          ManusIngressRecord raw;
          if(!runtime_.pop_manus(raw)) break;
          received=true; if(!submit(std::move(raw))) return;
        }
        if(draining_ && !received) return;
        if(!received) std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    } catch(const std::exception& error) {fail(error.what());}
      catch(...) {fail("unknown native output bridge failure");}
  }
  SessionRuntime& runtime_;
  Finalize finalize_;
  BoundedOutput<SessionOutputItem> output_;
  std::atomic<bool> draining_{false},aborted_{false},closed_{false};
  mutable std::mutex mutex_;
  std::mutex lifecycle_;
  std::string failure_;
  std::thread pump_;
};
}
