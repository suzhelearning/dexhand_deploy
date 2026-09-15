#pragma once
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace tianji_control {
struct OutputStats {
  std::uint64_t accepted=0,processed=0;
  std::size_t pending=0,high_water=0;
  bool complete=false;
  std::string failure;
};
// One output owner: FIFO immutable value copies, bounded entry count, fail-closed
// overflow. Caller must also bound T's payload size (e.g. typed arm receipt).
// Write/finalize execute off the control thread. They must have bounded I/O and
// cancellation via cancel(), which is concurrent, idempotent and nonthrowing.
// No callback may call finish/abort on this object. This queue confers no publisher
// authority and does not create or mark an HDF5 file complete on its own.
template<class T> class BoundedOutput {
 public:
  using Write=std::function<void(const T&)>;
  using Operation=std::function<void()>;
  BoundedOutput(std::size_t capacity,Write write,Operation finalize,Operation cancel)
    :capacity_(capacity),write_(std::move(write)),finalize_(std::move(finalize)),cancel_(std::move(cancel)) {
    if(!capacity || capacity>65536 || !write_ || !finalize_ || !cancel_)
      throw std::invalid_argument("bounded output requires capacity 1..65536 and all operations");
    thread_=std::thread([this]{run();});
  }
  ~BoundedOutput() {abort();}
  BoundedOutput(const BoundedOutput&)=delete;
  BoundedOutput& operator=(const BoundedOutput&)=delete;
  bool submit(T value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(closing_ || aborted_ || !stats_.failure.empty()) return false;
    if(queue_.size()==capacity_) {
      stats_.failure="native output queue overflow"; wake_.notify_all(); return false;
    }
    queue_.push_back(std::move(value)); ++stats_.accepted;
    if(queue_.size()>stats_.high_water) stats_.high_water=queue_.size();
    wake_.notify_one(); return true;
  }
  // Call only after all producers have stopped. Completes only if every accepted
  // item was written and finalize returned successfully. Inspect stats().
  void finish() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_);
    {std::lock_guard<std::mutex> lock(mutex_); closing_=true;}
    wake_.notify_all();
    if(thread_.joinable()) thread_.join();
  }
  void abort() noexcept {
    bool cancel=false;
    {std::lock_guard<std::mutex> lock(mutex_);
      if(!stats_.complete) {aborted_=true; cancel=true;}}
    if(cancel) {try {cancel_();} catch(...) {fail("native output cancellation failed");}}
    wake_.notify_all();
    // Cancel before locking lifecycle: another thread may be joining in finish.
    std::lock_guard<std::mutex> lifecycle(lifecycle_);
    if(thread_.joinable()) thread_.join();
  }
  OutputStats stats() const {
    std::lock_guard<std::mutex> lock(mutex_); auto result=stats_; result.pending=queue_.size(); return result;
  }
 private:
  void fail(const std::string& error) {
    std::lock_guard<std::mutex> lock(mutex_); if(stats_.failure.empty()) stats_.failure=error;
  }
  void run() noexcept {
    try {
      for(;;) {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock,[&]{return aborted_ || closing_ || !stats_.failure.empty() || !queue_.empty();});
        if(aborted_ || !stats_.failure.empty()) return;
        if(queue_.empty()) {
          lock.unlock(); finalize_(); lock.lock();
          stats_.complete=!aborted_ && stats_.failure.empty(); return;
        }
        T value=std::move(queue_.front()); queue_.pop_front();
        lock.unlock(); write_(value); lock.lock(); ++stats_.processed;
      }
    } catch(const std::exception& error) {fail(error.what());}
      catch(...) {fail("unknown native output failure");}
  }
  const std::size_t capacity_;
  Write write_;
  Operation finalize_,cancel_;
  mutable std::mutex mutex_;
  std::mutex lifecycle_;
  std::condition_variable wake_;
  std::deque<T> queue_;
  OutputStats stats_;
  bool closing_=false,aborted_=false;
  std::thread thread_;
};
}
