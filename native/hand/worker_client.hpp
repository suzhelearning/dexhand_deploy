#pragma once
#include "scheduler_wire.hpp"
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;
namespace tianji_hand {
// Single-owner application IPC, not a hardware driver. The child command is
// an explicit cold launcher which execs the existing native Hand2 scheduler.
// Only request_stop may be concurrent. Failure closes this owned child; there
// is no automatic restart, epoch invention or publication authority here.
class NativeHandWorkerClient {
 public:
  NativeHandWorkerClient(const std::vector<std::string>& command,int timeout_ms):timeout_(timeout_ms) {
    if(command.empty() || command[0].empty() || command[0][0]!='/' || timeout_ms<=0 || timeout_ms>60000)
      throw std::invalid_argument("invalid native hand child configuration");
    for(const auto& arg:command) if(arg.find('\0')!=std::string::npos)
      throw std::invalid_argument("NUL in native hand child argv");
    try {
      spawn(command);std::array<std::uint8_t,scheduler_wire::kHandshakeSize> ready{};
      read(ready.data(),ready.size(),deadline());
      if(ready!=scheduler_wire::encode_handshake()) throw std::runtime_error("invalid native hand startup ACK");
    } catch(...) {close();throw;}
  }
  ~NativeHandWorkerClient() {close();}
  NativeHandWorkerClient(const NativeHandWorkerClient&)=delete;
  NativeHandWorkerClient& operator=(const NativeHandWorkerClient&)=delete;
  std::int64_t epoch() const {return session_.epoch;}
  void request_stop() noexcept {cancelled_.store(true);}
  void sync_phase(HandPhase phase) {
    if(session_.sequence>=static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
      throw std::overflow_error("hand session sequence exhausted");
    session({session_.epoch,session_.sequence+1,now_ns(),phase});
  }
  // Called only after the main owner verified idle/exact Home. Explicitly
  // synchronize idle before resetting; this method does not verify Home.
  HandResetAck reset_at_idle(std::int64_t next_epoch) {
    if(session_.epoch==std::numeric_limits<std::int64_t>::max() ||
       next_epoch!=session_.epoch+1 ||
       session_.sequence>static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())-2)
      throw std::invalid_argument("hand reset requires bounded next epoch/sequence");
    session({session_.epoch,session_.sequence+1,now_ns(),HandPhase::idle});
    return reset({next_epoch,session_.sequence+1,now_ns(),HandPhase::idle});
  }
  void session(const HandSession& state) {
    validate_session(state);
    if(state.epoch!=session_.epoch) throw std::invalid_argument("hand epoch changes require reset ACK");
    const auto frame=scheduler_wire::encode_session(state);
    try {send(frame.data(),frame.size(),deadline());session_=state;} catch(...) {close();throw;}
  }
  void input(const HandInput& value) {
    const auto frame=scheduler_wire::encode_input(value);
    if(value.sequence<=input_sequence_ || value.timestamp_ns<input_time_ ||
       value.timestamp_ns>now_ns() || (input_sequence_ && value.generation!=generation_))
      throw std::invalid_argument("hand input sequence/time/generation mismatch");
    try {
      send(frame.data(),frame.size(),deadline());
      input_sequence_=value.sequence;input_time_=value.timestamp_ns;generation_=value.generation;
    } catch(...) {close();throw;}
  }
  HandCommand result() {
    try {
      const auto frame=receive(deadline());
      return decode_result(frame.first.data(),frame.second);
    } catch(...) {close();throw;}
  }
  HandResetAck reset(const HandSession& request) {
    validate_session(request);
    if(session_.phase!=HandPhase::idle || request.phase!=HandPhase::idle ||
       session_.epoch==std::numeric_limits<std::int64_t>::max() || request.epoch!=session_.epoch+1)
      throw std::invalid_argument("native hand reset requires idle and next epoch");
    const auto frame=scheduler_wire::encode_reset_request(request);
    try {
      const auto until=deadline();send(frame.data(),frame.size(),until);
      while(true) {
        const auto response=receive(until);
        if(scheduler_wire::magic_is(response.first.data(),scheduler_wire::kOutputMagic)) {
          // A frame already held by the writer can cross the reset request.
          // Drain only old-epoch results; never treat one as an acknowledgement.
          decode_result(response.first.data(),response.second);continue;
        }
        const auto ack=scheduler_wire::decode_reset_ack(response.first.data(),response.second);
        if(ack.epoch!=request.epoch || ack.sequence!=request.sequence ||
           ack.requested_ns!=request.timestamp_ns || ack.completed_ns>now_ns())
          throw std::runtime_error("unassociated native hand reset ACK");
        session_=request;return ack; // Commit only after matching completed ACK.
      }
    } catch(...) {close();throw;}
  }
  void shutdown() {
    // Resource shutdown only; not an output-drain, recording or Home ACK.
    if(fd_<0) return;
    try {
      const auto frame=scheduler_wire::encode_shutdown();send(frame.data(),frame.size(),deadline());
    } catch(...) {close();throw;}
    close();
  }
 private:
  using Clock=std::chrono::steady_clock;
  using Time=Clock::time_point;
  static std::uint64_t now_ns() {return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();}
  Time deadline() const {return Clock::now()+std::chrono::milliseconds(timeout_);}
  void validate_session(const HandSession& s) const {
    scheduler_wire::encode_session(s);
    if(s.sequence<=session_.sequence || s.timestamp_ns<session_.timestamp_ns || s.timestamp_ns>now_ns())
      throw std::invalid_argument("native hand session sequence/time mismatch");
  }
  void wait(short events,Time until) {
    while(true) {
      if(cancelled_.load() || fd_<0) throw std::runtime_error("native hand child cancelled or closed");
      const auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(until-Clock::now()).count();
      if(remaining<=0) throw std::runtime_error("native hand child IPC timeout");
      pollfd p{fd_,events,0};const auto n=::poll(&p,1,static_cast<int>(std::min<std::int64_t>(remaining,50)));
      if(n<0) {if(errno==EINTR) continue;throw std::runtime_error("native hand child poll failed");}
      if(!n) continue;
      if(p.revents&(POLLNVAL|POLLERR)) throw std::runtime_error("native hand child socket failed");
      if(p.revents&events) return;
      if(p.revents&POLLHUP) throw std::runtime_error("native hand child EOF");
    }
  }
  void send(const std::uint8_t* bytes,std::size_t size,Time until) {
    std::size_t offset=0;
    while(offset<size) {
      wait(POLLOUT,until);const auto n=::send(fd_,bytes+offset,size-offset,MSG_NOSIGNAL);
      if(n<0 && (errno==EINTR || errno==EAGAIN || errno==EWOULDBLOCK)) continue;
      if(n<=0) throw std::runtime_error("native hand child write failed");
      offset+=static_cast<std::size_t>(n);
    }
  }
  void read(std::uint8_t* bytes,std::size_t size,Time until) {
    std::size_t offset=0;
    while(offset<size) {
      wait(POLLIN,until);const auto n=::recv(fd_,bytes+offset,size-offset,0);
      if(n<0 && (errno==EINTR || errno==EAGAIN || errno==EWOULDBLOCK)) continue;
      if(n<=0) throw std::runtime_error("native hand child truncated output/EOF");
      offset+=static_cast<std::size_t>(n);
    }
  }
  std::pair<std::array<std::uint8_t,scheduler_wire::kOutputSize>,std::size_t> receive(Time until) {
    std::pair<std::array<std::uint8_t,scheduler_wire::kOutputSize>,std::size_t> out{};
    read(out.first.data(),scheduler_wire::kHeaderSize,until);
    out.second=scheduler_wire::load_le<std::uint16_t>(out.first.data()+6);
    const bool result=scheduler_wire::magic_is(out.first.data(),scheduler_wire::kOutputMagic);
    const bool ack=scheduler_wire::magic_is(out.first.data(),scheduler_wire::kResetAckMagic);
    if((!result && !ack) || out.second!=(result?scheduler_wire::kOutputSize:scheduler_wire::kResetAckSize))
      throw std::runtime_error("unexpected native hand child frame");
    read(out.first.data()+scheduler_wire::kHeaderSize,out.second-scheduler_wire::kHeaderSize,until);
    return out;
  }
  HandCommand decode_result(const std::uint8_t* bytes,std::size_t size) {
    const auto result=scheduler_wire::decode_output(bytes,size);
    if(result.output_sequence<=output_sequence_ || result.input_sequence>input_sequence_ ||
       result.input_timestamp_ns>input_time_ || result.scheduler_timestamp_ns>now_ns() || result.epoch>session_.epoch)
      throw std::runtime_error("unassociated native hand child output");
    output_sequence_=result.output_sequence;return result;
  }
  void spawn(const std::vector<std::string>& command) {
    int pair[2]{-1,-1};
    if(::socketpair(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0,pair)) throw std::runtime_error("native hand child socketpair failed");
    posix_spawn_file_actions_t actions;bool actions_ready=false;
    try {
      for(auto& fd:pair) if(fd<3) {
        const int moved=fcntl(fd,F_DUPFD_CLOEXEC,3);
        if(moved<0) throw std::runtime_error("native hand child descriptor relocation failed");
        ::close(fd);fd=moved;
      }
      if(posix_spawn_file_actions_init(&actions)) throw std::runtime_error("native hand spawn actions failed");
      actions_ready=true;
      if(posix_spawn_file_actions_addclose(&actions,pair[0]) ||
         posix_spawn_file_actions_adddup2(&actions,pair[1],STDIN_FILENO) ||
         posix_spawn_file_actions_adddup2(&actions,pair[1],STDOUT_FILENO) ||
         posix_spawn_file_actions_addclose(&actions,pair[1])) throw std::runtime_error("native hand spawn descriptors failed");
      std::vector<char*> args;for(const auto& arg:command) args.push_back(const_cast<char*>(arg.c_str()));args.push_back(nullptr);
      const int error=posix_spawn(&pid_,args[0],&actions,nullptr,args.data(),environ);
      if(error) {pid_=-1;throw std::runtime_error("native hand spawn failed");}
      posix_spawn_file_actions_destroy(&actions);actions_ready=false;
      ::close(pair[1]);pair[1]=-1;fd_=pair[0];pair[0]=-1;
      const int flags=fcntl(fd_,F_GETFL);
      if(flags<0 || fcntl(fd_,F_SETFL,flags|O_NONBLOCK)<0) throw std::runtime_error("native hand nonblocking socket failed");
    } catch(...) {
      if(actions_ready) posix_spawn_file_actions_destroy(&actions);
      for(int fd:pair) if(fd>=0) ::close(fd);
      throw;
    }
  }
  bool reap_for(int milliseconds) noexcept {
    const auto until=Clock::now()+std::chrono::milliseconds(milliseconds);
    do {
      int status=0;const auto result=waitpid(pid_,&status,WNOHANG);
      if(result==pid_ || (result<0 && errno==ECHILD)) {pid_=-1;return true;}
      if(result<0 && errno!=EINTR) return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while(Clock::now()<until);
    return false;
  }
  void close() noexcept {
    cancelled_.store(true);
    if(fd_>=0) {::close(fd_);fd_=-1;}
    if(pid_>0 && !reap_for(200)) {
      ::kill(pid_,SIGTERM);
      if(!reap_for(200)) {
        ::kill(pid_,SIGKILL);int status;
        while(waitpid(pid_,&status,0)<0 && errno==EINTR) {}
        pid_=-1;
      }
    }
  }
  const int timeout_;
  int fd_=-1;pid_t pid_=-1;
  std::atomic<bool> cancelled_{false};
  HandSession session_;
  std::uint64_t input_sequence_=0,input_time_=0,generation_=0,output_sequence_=0;
};
} // namespace tianji_hand
