#pragma once
#include "reset_endpoint.hpp"
#include "worker_result.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <fcntl.h>
#include <iomanip>
#include <locale>
#include <limits>
#include <poll.h>
#include <set>
#include <signal.h>
#include <spawn.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#include <utility>
#include <vector>

extern char** environ;
namespace tianji_control {
// Single-owner Linux IPC client for reset and optional binary IK steps. No command
// publication, Home authorization, restart or control scheduling. Calls must be
// serialized by the owner; only request_stop() may be concurrent.
class WorkerResetClient final:public ResetEndpoint {
 public:
  WorkerResetClient(const std::vector<std::string>& command,std::string prefix,
                    const std::string& algorithm,int timeout_ms,bool binary_steps=false,bool deterministic=false)
      :prefix_(std::move(prefix)),timeout_(timeout_ms),binary_steps_(binary_steps),deterministic_(deterministic) {
    if(command.empty() || command[0].empty() || command[0][0]!='/' || timeout_ms<=0 ||
       (prefix_!="spark" && prefix_!="mapped_palm")) throw std::invalid_argument("invalid worker configuration");
    for(const auto& arg:command) if(arg.find('\0')!=std::string::npos)
      throw std::invalid_argument("NUL in worker argv");
    try {
      spawn(command);
      auto ready=parse(read_line(deadline()));
      const nlohmann::json expected={{"schema_version",1},{"kind",prefix_+"_worker_ready"},
                                    {"algorithm",algorithm},{"native_ticks",0}};
      if(ready!=expected || !ready.at("schema_version").is_number_integer() ||
         !ready.at("native_ticks").is_number_integer()) throw std::runtime_error("invalid native startup acknowledgement");
    } catch(...) { close(); throw; }
  }
  ~WorkerResetClient() { close(); }
  WorkerResetClient(const WorkerResetClient&)=delete;
  WorkerResetClient& operator=(const WorkerResetClient&)=delete;
  std::int64_t epoch() const override { return epoch_; }
  void request_stop() noexcept override { cancelled_.store(true); }
  bool supports_height() const noexcept override { return prefix_=="mapped_palm"; }
  void configure_height(const std::array<double,2>& offsets) override {
    if(fd_<0) throw std::runtime_error("native worker is closed; no implicit restart");
    if(!supports_height() || tick_!=0) throw std::logic_error("height requires mapped-palm worker at tick zero");
    for(double v:offsets) if(!std::isfinite(v) || std::abs(v)>1.) throw std::invalid_argument("finite height offsets within 1 m required");
    std::ostringstream request; request.imbue(std::locale::classic());
    request<<"TJMH1 "<<std::setprecision(17)<<offsets[0]<<' '<<offsets[1]<<'\n';
    try {
      const auto until=deadline(); send_request(request.str(),until);
      const auto ack=parse(read_line(until));
      const nlohmann::json expected={{"kind","mapped_palm_height_ack"},{"target_height_offsets_m",offsets}};
      if(ack!=expected) throw std::runtime_error("height acknowledgement mismatch");
      for(const auto& v:ack.at("target_height_offsets_m"))
        if(!v.is_number() || !std::isfinite(v.get<double>())) throw std::runtime_error("invalid height acknowledgement number");
    } catch(...) { close(); throw; }
  }
  void configure_xz(const std::array<double,2>& x,const std::array<double,2>& z) override {
    if(fd_<0 || !supports_height() || tick_!=0) throw std::logic_error("XZ requires mapped-palm at tick zero");
    const std::array<double,4> values{x[0],x[1],z[0],z[1]};
    for(double v:values) if(!std::isfinite(v) || std::abs(v)>1.) throw std::invalid_argument("invalid XZ offsets");
    std::ostringstream request;request.imbue(std::locale::classic());request<<"TJMX1"<<std::setprecision(17);
    for(double v:values) request<<' '<<v;
    request<<'\n';
    try {
      const auto until=deadline();send_request(request.str(),until);const auto ack=parse(read_line(until));
      if(ack!=nlohmann::json{{"kind","mapped_palm_xz_ack"},{"offsets_xxzz_m",values}})
        throw std::runtime_error("XZ acknowledgement mismatch");
      for(const auto& v:ack.at("offsets_xxzz_m")) if(!v.is_number() || !std::isfinite(v.get<double>()))
        throw std::runtime_error("invalid XZ acknowledgement");
    }catch(...) {close();throw;}
  }
  WorkerResult step(const WorkerTick& tick) {
    if(fd_<0) throw std::runtime_error("native worker is closed; no implicit restart");
    if(!binary_steps_) throw std::logic_error("binary steps not enabled");
    if(tick_==std::numeric_limits<std::uint64_t>::max() || tick.id!=tick_+1 || tick.now_ns<=now_)
      throw std::invalid_argument("consecutive ticks and increasing int64 time required");
    if(tick.packet.size()>656 || (tick.packet.empty()?(tick.received_ns!=0 || tick.generation!=0 || tick.discontinuity):
       (tick.received_ns<=0 || tick.received_ns>tick.now_ns)))
      throw std::invalid_argument("invalid sample metadata or size");
    std::ostringstream request; request.imbue(std::locale::classic());
    request<<"TJSC1 "<<tick.id<<' '<<tick.now_ns<<' '<<tick.received_ns<<' '<<tick.generation<<' '<<tick.discontinuity<<' ';
    if(tick.packet.empty()) request<<'-';
    else { request<<std::hex<<std::setfill('0'); for(auto b:tick.packet) request<<std::setw(2)<<unsigned(b); }
    request<<'\n';
    try {
      const auto until=deadline(); send_request(request.str(),until);
      const bool mapped=prefix_=="mapped_palm";
      std::vector<std::uint8_t> bytes;
      auto expected=worker_result_size(mapped); bytes.reserve(expected+16);
      while(bytes.size()<expected) {
        wait(POLLIN,until);
        std::uint8_t buffer[2048]; const auto n=recv(fd_,buffer,sizeof(buffer),0);
        if(n<0 && (errno==EINTR || errno==EAGAIN || errno==EWOULDBLOCK)) continue;
        if(n<=0) throw std::runtime_error("native worker EOF");
        bytes.insert(bytes.end(),buffer,buffer+n);
        if(bytes.size()>=8) {validate_worker_header(bytes,mapped);expected=worker_result_size(mapped,bytes[5]==3);}
        if(bytes.size()>expected) throw std::runtime_error("unsolicited native binary output");
      }
      auto result=decode_worker_result(std::move(bytes),mapped);
      if(result.tick!=tick.id || result.timestamp!=static_cast<std::uint64_t>(tick.now_ns) || result.deterministic!=deterministic_)
        throw std::runtime_error("unassociated native binary result");
      tick_=tick.id; now_=tick.now_ns;
      return result;
    } catch(...) { close(); throw; }
  }
  void reset(const std::array<double,14>& positions,std::int64_t epoch) override {
    if(fd_<0) throw std::runtime_error("native worker is closed; no implicit restart");
    if(epoch<=epoch_) throw std::invalid_argument("execution epoch must increase");
    for(double q:positions) if(!std::isfinite(q)) throw std::invalid_argument("finite reset positions required");
    std::ostringstream request;
    request.imbue(std::locale::classic());
    request<<"TJSR1 "<<epoch<<std::setprecision(17);
    for(double q:positions) request<<' '<<q;
    request<<'\n';
    try {
      const auto until=deadline();
      send_request(request.str(),until);
      auto ack=parse(read_line(until));
      std::array<double,14> zero{};
      const nlohmann::json expected={{"schema_version",1},{"kind",prefix_+"_reset_ack"},
        {"execution_epoch",epoch},{"position_rad",positions},{"velocity_rad_s",zero},{"acceleration_rad_s2",zero}};
      if(ack!=expected || !ack.at("schema_version").is_number_integer() ||
         !ack.at("execution_epoch").is_number_integer()) throw std::runtime_error("reset state acknowledgement mismatch");
      for(const char* key:{"position_rad","velocity_rad_s","acceleration_rad_s2"})
        for(const auto& value:ack.at(key)) if(!value.is_number() || !std::isfinite(value.get<double>()))
          throw std::runtime_error("invalid reset state number");
      epoch_=epoch; // Commit only after exact position and zero derivative acknowledgement.
      tick_=0; // The receive/control clock deliberately does not rewind on reset.
    } catch(...) { close(); throw; }
  }
  void close() noexcept {
    if(fd_>=0) { ::close(fd_); fd_=-1; }
    if(pid_>0) {
      // This class exclusively owns wait/reap for its child (no waitpid(-1)
      // reaper may steal it). ECHILD is not permission to signal a recycled PID.
      pid_t result;
      do { result=waitpid(pid_,nullptr,WNOHANG); } while(result<0 && errno==EINTR);
      if(result==0) {
        kill(pid_,SIGKILL);
        while(waitpid(pid_,nullptr,0)<0 && errno==EINTR) {}
      }
      pid_=-1;
    }
  }
 private:
  using Clock=std::chrono::steady_clock;
  Clock::time_point deadline() const { return Clock::now()+std::chrono::milliseconds(timeout_); }
  void send_request(const std::string& bytes,Clock::time_point until) {
    if(bytes.size()>2048) throw std::invalid_argument("oversized native request");
    char extra;
    const auto peek=recv(fd_,&extra,1,MSG_PEEK|MSG_DONTWAIT);
    if(peek>=0) throw std::runtime_error("unsolicited output or closed worker");
    if(errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR) throw std::runtime_error("worker socket failure");
    std::size_t offset=0;
    while(offset<bytes.size()) {
      wait(POLLOUT,until);
      const auto n=send(fd_,bytes.data()+offset,bytes.size()-offset,MSG_NOSIGNAL);
      if(n>0) offset+=static_cast<std::size_t>(n);
      else if(n<0 && (errno==EAGAIN || errno==EWOULDBLOCK || errno==EINTR)) continue;
      else throw std::runtime_error("native worker write failed");
    }
  }
  void wait(short events,Clock::time_point until) {
    for(;;) {
      if(cancelled_.load()) throw std::runtime_error("native worker cancelled");
      const auto left=std::chrono::duration_cast<std::chrono::milliseconds>(until-Clock::now()).count();
      if(left<0) throw std::runtime_error("native worker timeout");
      pollfd descriptor{fd_,events,0};
      const auto result=poll(&descriptor,1,static_cast<int>(std::min<std::int64_t>(
          left+1,10)));
      if(result<0 && errno==EINTR) continue;
      if(result==0) continue;
      if(result<0) throw std::runtime_error("native worker poll failure");
      if(descriptor.revents & (events|POLLHUP)) return;
      throw std::runtime_error("native worker socket error");
    }
  }
  std::string read_line(Clock::time_point until) {
    std::string result;
    for(;;) {
      wait(POLLIN,until);
      char buffer[1024];
      const auto n=recv(fd_,buffer,sizeof(buffer),0);
      if(n<0 && (errno==EINTR || errno==EAGAIN || errno==EWOULDBLOCK)) continue;
      if(n<=0) throw std::runtime_error("native worker EOF");
      result.append(buffer,static_cast<std::size_t>(n));
      if(result.size()>4096) throw std::runtime_error("oversized native worker response");
      const auto newline=result.find('\n');
      if(newline!=std::string::npos) {
        if(newline!=result.size()-1) throw std::runtime_error("unsolicited native worker output");
        return result;
      }
    }
  }
  static nlohmann::json parse(const std::string& text) {
    std::vector<std::set<std::string>> keys;
    return nlohmann::json::parse(text,[&](int depth,nlohmann::json::parse_event_t event,nlohmann::json& value) {
      if(depth>16) throw std::runtime_error("native response nesting limit");
      using Event=nlohmann::json::parse_event_t;
      if(event==Event::object_start) keys.emplace_back();
      if(event==Event::key && !keys.back().insert(value.get<std::string>()).second)
        throw std::runtime_error("duplicate native response key");
      if(event==Event::object_end) keys.pop_back();
      return true;
    });
  }
  void spawn(const std::vector<std::string>& command) {
    std::vector<char*> argv;
    for(const auto& arg:command) argv.push_back(const_cast<char*>(arg.c_str()));
    argv.push_back(nullptr);
    std::vector<std::string> environment;
    for(char** entry=environ;*entry;++entry) {
      std::string value=*entry, key=value.substr(0,value.find('='));
      if(key!="PYTHONPATH" && key!="PYTHONHOME" && key!="LD_LIBRARY_PATH" && key!="LD_PRELOAD")
        environment.push_back(std::move(value));
    }
    std::vector<char*> env;
    for(auto& value:environment) env.push_back(value.data());
    env.push_back(nullptr);
    int sockets[2];
    if(socketpair(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0,sockets)<0) throw std::runtime_error("worker socketpair failed");
    if(sockets[0]<3 || sockets[1]<3) {
      ::close(sockets[0]); ::close(sockets[1]); throw std::runtime_error("worker requires open standard descriptors");
    }
    posix_spawn_file_actions_t actions;
    int status=posix_spawn_file_actions_init(&actions);
    if(status) { ::close(sockets[0]); ::close(sockets[1]); throw std::runtime_error("spawn actions failed"); }
    status=posix_spawn_file_actions_adddup2(&actions,sockets[1],STDIN_FILENO);
    if(!status) status=posix_spawn_file_actions_adddup2(&actions,sockets[1],STDOUT_FILENO);
    if(!status) status=posix_spawn_file_actions_addclose(&actions,sockets[0]);
    if(!status) status=posix_spawn_file_actions_addclose(&actions,sockets[1]);
    if(!status) status=posix_spawn(&pid_,argv[0],&actions,nullptr,argv.data(),env.data());
    posix_spawn_file_actions_destroy(&actions);
    ::close(sockets[1]);
    if(status) { ::close(sockets[0]); pid_=-1; throw std::runtime_error("native worker spawn failed"); }
    fd_=sockets[0];
    if(fcntl(fd_,F_SETFL,O_NONBLOCK)<0) throw std::runtime_error("worker nonblocking setup failed");
  }
  std::string prefix_;
  int timeout_,fd_=-1;
  bool binary_steps_=false,deterministic_=false;
  std::uint64_t tick_=0;
  std::int64_t now_=0;
  pid_t pid_=-1;
  std::int64_t epoch_=1;
  std::atomic<bool> cancelled_{false};
};
} // namespace tianji_control
