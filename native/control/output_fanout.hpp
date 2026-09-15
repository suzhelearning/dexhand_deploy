#pragma once
#include "bounded_output.hpp"
#include <atomic>
#include <memory>
#include <vector>

namespace tianji_control {
// One immutable payload, independent FIFO consumers. No sink I/O runs in submit.
// All sinks must provide bounded I/O and concurrent, idempotent cancellation.
// finish only DRAINS: commit/complete markers belong to the caller, after every
// sink has succeeded. Never call finish/abort from a sink or failure callback.
template<class T> class OutputFanout {
 public:
  struct Sink {
    std::string name;
    std::function<void(const T&)> write;
    std::function<void()> cancel;
  };
  using Failure=std::function<void(const std::string&)>;
  OutputFanout(std::size_t capacity,std::vector<Sink> sinks,Failure failure)
      :sinks_(std::move(sinks)),on_failure_(std::move(failure)) {
    if(sinks_.empty() || !on_failure_ || !capacity || capacity>65536)
      throw std::invalid_argument("invalid output fanout configuration");
    for(std::size_t i=0;i<sinks_.size();++i) {
      const auto& sink=sinks_[i];
      if(sink.name.empty() || !sink.write || !sink.cancel)
        throw std::invalid_argument("named output sink and cancellation required");
      for(std::size_t j=0;j<i;++j) if(sinks_[j].name==sink.name)
        throw std::invalid_argument("duplicate output sink name");
    }
    for(std::size_t i=0;i<sinks_.size();++i)
      outputs_.push_back(std::make_unique<Queue>(capacity,[this,i](const Shared& item) {
        try {sinks_[i].write(*item);}
        catch(const std::exception& e){fail(sinks_[i].name+": "+e.what());throw;}
        catch(...){fail(sinks_[i].name+": unknown output failure");throw;}
      },[]{},sinks_[i].cancel));
  }
  ~OutputFanout(){abort();}
  OutputFanout(const OutputFanout&)=delete;
  OutputFanout& operator=(const OutputFanout&)=delete;
  bool submit(T value) {
    std::lock_guard<std::mutex> admission(admission_);
    if(closing_ || aborted_ || !failure().empty()) return false;
    const auto item=std::make_shared<const T>(std::move(value));
    for(std::size_t i=0;i<outputs_.size();++i) if(!outputs_[i]->submit(item)) {
      const auto error=outputs_[i]->stats().failure;
      fail(sinks_[i].name+": "+(error.empty()?"output admission rejected":error));return false;
    }
    return true;
  }
  bool finish() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_);
    {std::lock_guard<std::mutex> admission(admission_);closing_=true;}
    for(auto& output:outputs_) output->finish();
    for(std::size_t i=0;i<outputs_.size();++i) {
      const auto s=outputs_[i]->stats();
      if(!s.failure.empty()) fail(sinks_[i].name+": "+s.failure);
      if(!s.complete || s.accepted!=s.processed) return false;
    }
    return !aborted_ && failure().empty();
  }
  void abort() noexcept {
    aborted_=true;
    cancel_all(); // Cancel every sink BEFORE waiting for any worker/finish join.
    std::lock_guard<std::mutex> lifecycle(lifecycle_);
    for(auto& output:outputs_) output->abort();
  }
  std::string failure() const {std::lock_guard<std::mutex> lock(mutex_);return error_;}
  std::vector<std::pair<std::string,OutputStats>> stats() const {
    std::vector<std::pair<std::string,OutputStats>> rows;
    for(std::size_t i=0;i<outputs_.size();++i) rows.emplace_back(sinks_[i].name,outputs_[i]->stats());
    return rows;
  }
 private:
  using Shared=std::shared_ptr<const T>;
  using Queue=BoundedOutput<Shared>;
  void cancel_all() noexcept {
    for(const auto& sink:sinks_) try {sink.cancel();}catch(...){}
  }
  void fail(const std::string& error) noexcept {
    try {
      {std::lock_guard<std::mutex> lock(mutex_);if(!error_.empty()) return;error_=error;}
      cancel_all();
      on_failure_(error);
    } catch(...) {} // Failure reporting must not terminate the consumer thread.
  }
  const std::vector<Sink> sinks_;
  Failure on_failure_;
  std::vector<std::unique_ptr<Queue>> outputs_;
  mutable std::mutex mutex_;
  std::mutex admission_,lifecycle_;
  std::string error_;
  std::atomic<bool> closing_{false},aborted_{false};
};
} // namespace tianji_control
