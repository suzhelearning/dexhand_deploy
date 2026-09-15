#pragma once
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>
#include <fcntl.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>

namespace tianji_control {
class OwnedDescriptor {
 public:
  explicit OwnedDescriptor(int fd=-1) noexcept:fd_(fd) {}
  ~OwnedDescriptor() { reset(); }
  OwnedDescriptor(const OwnedDescriptor&)=delete;
  OwnedDescriptor& operator=(const OwnedDescriptor&)=delete;
  OwnedDescriptor(OwnedDescriptor&& other) noexcept:fd_(std::exchange(other.fd_,-1)) {}
  int get() const noexcept {return fd_;}
  void reset() noexcept {if(fd_>=0) {::close(fd_); fd_=-1;}}
 private:
  int fd_;
};
struct ReceivedDatagram {
  std::vector<std::uint8_t> bytes;
  std::int64_t received_ns=0; // Local steady clock, never packet-controlled.
};
struct DatagramStats {
  std::uint64_t datagrams=0,delivered=0,rejected_size=0;
  bool running=false;
  std::string failure;
  bool notification_failed=false;
};
// Receive-only adapter for an exclusively transferred datagram descriptor.
// Does not bind/rebind, grant source authority, decode frames or publish commands.
// TJVR decoder/gate remains downstream; size rejection is not frame validation.
// Sink/failure notifier MUST be bounded/nonblocking, and must not call start/stop
// on this object. The notifier runs without the stats mutex (may query stats).
// Socket descriptor must not have aliases/readers outside this owner.
class DatagramReceiver {
 public:
  using Sink=std::function<bool(ReceivedDatagram)>;
  using FailureNotifier=std::function<void(const std::string&)>;
  DatagramReceiver(OwnedDescriptor socket,Sink sink,FailureNotifier notify={})
    :socket_(std::move(socket)),wake_(eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK)),sink_(std::move(sink)),notify_(std::move(notify)) {
    int type=0; socklen_t size=sizeof(type);
    if(!sink_ || getsockopt(socket_.get(),SOL_SOCKET,SO_TYPE,&type,&size)<0 || type!=SOCK_DGRAM)
      throw std::invalid_argument("exclusive datagram descriptor and sink required");
    if(wake_.get()<0) throw std::system_error(errno,std::generic_category(),"eventfd");
    const int flags=fcntl(socket_.get(),F_GETFD);
    if(flags<0 || fcntl(socket_.get(),F_SETFD,flags|FD_CLOEXEC)<0)
      throw std::system_error(errno,std::generic_category(),"datagram close-on-exec");
  }
  ~DatagramReceiver() {stop();}
  DatagramReceiver(const DatagramReceiver&)=delete;
  DatagramReceiver& operator=(const DatagramReceiver&)=delete;
  void start() {
    std::lock_guard<std::mutex> lock(lifecycle_);
    if(started_ || stopped_) throw std::logic_error("datagram receiver is single-use");
    thread_=std::thread([this]{run();}); started_=true;
  }
  void stop() {
    std::lock_guard<std::mutex> lock(lifecycle_);
    stopped_=true;
    if(thread_.joinable()) {
      const std::uint64_t value=1;
      // EAGAIN means wake already pending; poll timeout also bounds wake latency.
      while(write(wake_.get(),&value,sizeof(value))<0 && errno==EINTR) {}
      thread_.join();
    }
    socket_.reset(); wake_.reset();
  }
  DatagramStats stats() const {std::lock_guard<std::mutex> lock(mutex_); return stats_;}
 private:
  void run() noexcept {
    {std::lock_guard<std::mutex> lock(mutex_); stats_.running=true;}
    try {
      while(!stopped_) {
        pollfd fds[2]={{socket_.get(),POLLIN,0},{wake_.get(),POLLIN,0}};
        const int ready=poll(fds,2,100);
        if(stopped_) break;
        if(ready<0) {if(errno==EINTR) continue; throw std::system_error(errno,std::generic_category(),"datagram poll");}
        if(fds[0].revents&(POLLERR|POLLHUP|POLLNVAL)) throw std::runtime_error("datagram socket unavailable");
        if(!(fds[0].revents&POLLIN)) continue;
        std::array<std::uint8_t,657> bytes{};
        iovec part{bytes.data(),bytes.size()}; msghdr message{};
        message.msg_iov=&part; message.msg_iovlen=1;
        const auto count=recvmsg(socket_.get(),&message,MSG_DONTWAIT);
        if(count<0) {
          if(errno==EINTR || errno==EAGAIN || errno==EWOULDBLOCK) continue;
          throw std::system_error(errno,std::generic_category(),"datagram receive");
        }
        const auto received=std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        const bool rejected=count==0 || count>656 || (message.msg_flags&MSG_TRUNC);
        {std::lock_guard<std::mutex> lock(mutex_); ++stats_.datagrams; if(rejected) ++stats_.rejected_size;}
        if(rejected) continue;
        if(stopped_) break;
        if(!sink_(ReceivedDatagram{{bytes.begin(),bytes.begin()+count},received}))
          throw std::runtime_error("datagram sink rejected admission");
        {std::lock_guard<std::mutex> lock(mutex_); ++stats_.delivered;}
      }
    } catch(const std::exception& error) {
      std::lock_guard<std::mutex> lock(mutex_); stats_.failure=error.what();
    } catch(...) {
      std::lock_guard<std::mutex> lock(mutex_); stats_.failure="unknown datagram failure";
    }
    const auto error=stats().failure;
    if(!error.empty() && notify_) {
      try {notify_(error);} catch(...) {
        std::lock_guard<std::mutex> lock(mutex_); stats_.notification_failed=true;
      }
    }
    std::lock_guard<std::mutex> lock(mutex_); stats_.running=false;
  }
  OwnedDescriptor socket_,wake_;
  Sink sink_;
  FailureNotifier notify_;
  std::atomic<bool> stopped_{false};
  bool started_=false;
  std::mutex lifecycle_;
  mutable std::mutex mutex_;
  DatagramStats stats_;
  std::thread thread_;
};
}
