#pragma once
#include "datagram_receiver.hpp"
#include "manus_ingress.hpp"
namespace tianji_control {
// Own only the transferred stdout descriptor, never the device/SDK process.
// Sink and notifier must be bounded and must not call start/stop on this owner.
class ManusReceiver {
 public:
  using Notifier=std::function<void(const std::string&)>;
  ManusReceiver(OwnedDescriptor input,Authority source,std::uint64_t generation,
                const std::string& right,const std::string& left,
                ManusIngress::Sink sink,Notifier notify={})
    :input_(std::move(input)),ingress_(std::move(source),generation,right,left,std::move(sink)),
     notify_(std::move(notify)) {
    const int flags=fcntl(input_.get(),F_GETFD);
    if(flags<0 || fcntl(input_.get(),F_SETFD,flags|FD_CLOEXEC)<0)
      throw std::runtime_error("invalid exclusive Manus descriptor");
  }
  ~ManusReceiver(){stop();}
  void start() {
    std::lock_guard<std::mutex> lock(lifecycle_);
    if(started_ || stopped_)throw std::logic_error("Manus receiver is single-use");
    thread_=std::thread([this]{run();});started_=true;
  }
  void stop() {
    std::lock_guard<std::mutex> lock(lifecycle_);
    stopped_=true;
    if(thread_.joinable())thread_.join();
    input_.reset();
  }
  std::string failure() const {std::lock_guard<std::mutex> lock(mutex_);return failure_;}
 private:
  void run() noexcept {
    try {
      std::array<char,4096> buffer{};
      while(!stopped_) {
        pollfd fd{input_.get(),POLLIN,0};
        const int ready=poll(&fd,1,50);
        if(stopped_)return;
        if(ready<0 && errno==EINTR)continue;
        if(ready<0 || (fd.revents&(POLLERR|POLLNVAL)))throw std::runtime_error("Manus descriptor poll failed");
        if(!ready)continue;
        const auto size=read(input_.get(),buffer.data(),buffer.size());
        if(size<0 && (errno==EINTR || errno==EAGAIN))continue;
        if(size<0)throw std::runtime_error("Manus descriptor read failed");
        if(!size){ingress_.finish();throw std::runtime_error("Manus driver EOF");}
        const auto stamp=std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        ingress_.feed(buffer.data(),static_cast<std::size_t>(size),stamp);
      }
    }catch(const std::exception& e){fail(e.what());}
     catch(...){fail("unknown Manus receive failure");}
  }
  void fail(const std::string& why) noexcept {
    if(stopped_)return;
    {std::lock_guard<std::mutex> lock(mutex_);failure_=why;}
    if(notify_)try{notify_(why);}catch(...){
      std::lock_guard<std::mutex> lock(mutex_);failure_+="; failure notification failed";
    }
  }
  OwnedDescriptor input_;
  ManusIngress ingress_;
  Notifier notify_;
  std::atomic<bool> stopped_{false};
  bool started_=false;
  std::thread thread_;
  mutable std::mutex mutex_;
  std::mutex lifecycle_;
  std::string failure_;
};
}
