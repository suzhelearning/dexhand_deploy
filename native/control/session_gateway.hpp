#pragma once

#include "datagram_receiver.hpp"
#include "session_gateway_wire.hpp"
#include "session_output_bridge.hpp"
#include "output_fanout.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace tianji_control {

struct NativeRecordingHooks {
  std::function<void(const SessionOutputItem&)> write;
  std::function<void(bool,const SessionSnapshot&)> finish;
  std::function<void()> cancel;
};
// Optional application ingress lifecycle (e.g. an inherited rawviz pipe).
// Captured owners must outlive the gateway. No driver process is owned here.
struct NativeInputHooks {
  std::function<void()> start;
  std::function<void()> stop;
};

// C++ owner for the application control path.  The caller supplies an already
// configured SessionRuntime; this class only wires its unchanged TJVR input
// contract, operator channel, heartbeat and bounded output bridge together.
// No PICO/Manus SDK, physical actuator or device connection is created here.
class NativeSessionGateway {
 public:
  NativeSessionGateway(std::unique_ptr<SessionRuntime> runtime,
                       OwnedDescriptor datagram,OwnedDescriptor control,
                       Authority source,std::size_t cycle_capacity=2048,
                       int io_timeout_ms=1000,
                       std::chrono::milliseconds heartbeat_period=std::chrono::milliseconds(100),
                       std::function<void(const SessionCycleSnapshot&)> publish={},
                       std::function<void()> cancel_publication=[]{},
                       NativeRecordingHooks recording={},
                       std::function<void(const SessionOutputItem&)> display={},bool summaries=false,
                       NativeInputHooks input={})
      :runtime_(std::move(runtime)),source_(std::move(source)),
       control_(std::move(control)),io_timeout_ms_(io_timeout_ms),
       heartbeat_period_(heartbeat_period),publish_(std::move(publish)),
       cancel_publication_(std::move(cancel_publication)),recording_(std::move(recording)),display_(std::move(display)),
       input_(std::move(input)) {
    if(!runtime_ || datagram.get()<0 || control_.get()<0 || !source_.bounded() ||
       !cycle_capacity || cycle_capacity>8192 || io_timeout_ms<=0 || heartbeat_period.count()<=0 || !cancel_publication_)
      throw std::invalid_argument("invalid native session gateway configuration");
    if(bool(recording_.write)!=bool(recording_.finish) || bool(recording_.write)!=bool(recording_.cancel))
      throw std::invalid_argument("recording requires write, finish and cancellation");
    if(bool(input_.start)!=bool(input_.stop)) throw std::invalid_argument("input requires paired start/stop");
    const int duplicate=fcntl(control_.get(),F_DUPFD_CLOEXEC,3);
    if(duplicate<0) throw std::runtime_error("native gateway control duplication failed");
    try {
      writer_=std::make_unique<GatewayWireWriter>(OwnedDescriptor(duplicate),io_timeout_ms_,summaries);
      runtime_->enable_cycle_capture(cycle_capacity);
      runtime_->enable_raw_capture(cycle_capacity);
      receiver_=std::make_unique<DatagramReceiver>(std::move(datagram),
        [this](ReceivedDatagram frame){ return accept_datagram(std::move(frame)); },
        [this](const std::string& error){ report_failure("TJVR receiver failed: "+error); });
    } catch(...) {
      // If construction succeeded, writer_ now owns the duplicate.  Closing
      // the raw integer again could accidentally close a reused descriptor.
      if(!writer_) ::close(duplicate);
      throw;
    }
  }
  ~NativeSessionGateway() { abort(); }
  NativeSessionGateway(const NativeSessionGateway&)=delete;
  NativeSessionGateway& operator=(const NativeSessionGateway&)=delete;

  void start() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_);
    if(started_ || closed_) throw std::logic_error("native session gateway is single-use");
    if(!submit_source_status()) throw std::runtime_error("native gateway source heartbeat admission failed");
    if(publish_ || recording_.write || display_) {
      using Fanout=OutputFanout<SessionOutputItem>;
      std::vector<Fanout::Sink> sinks{
        {"python_gateway",[this](const auto& item) {
          std::visit([this](const auto& value){writer_->write(value);},item);
        },[this]{if(writer_) writer_->request_stop();}}};
      if(publish_) sinks.push_back({"publication",[this](const auto& item) {
          if(const auto* cycle=std::get_if<SessionCycleSnapshot>(&item)) publish_(*cycle);
        },cancel_publication_});
      if(recording_.write) sinks.push_back({"recording",recording_.write,recording_.cancel});
      if(display_) sinks.push_back({"display",display_,[]{}});
      fanout_=std::make_unique<Fanout>(2048,std::move(sinks),
        [this](const std::string& error){report_failure(error);});
    }
    output_=std::make_unique<SessionOutputBridge>(*runtime_,2048,
      [this](const SessionOutputItem& item){ write_output(item); },
      [this](bool complete){
        if(fanout_ && !fanout_->finish()) throw std::runtime_error("native output drain failed: "+fanout_->failure());
        if(recording_.finish) recording_.finish(complete,runtime_->snapshot());
        writer_->complete(complete,runtime_->snapshot());
      },
      [this]{writer_->request_stop();if(fanout_) fanout_->abort();});
    try {
      runtime_->start();
      receiver_->start();
      if(input_.start) {input_active_=true;input_.start();}
      control_thread_=std::thread([this]{control_loop();});
      heartbeat_thread_=std::thread([this]{heartbeat_loop();});
      started_=true;
    } catch(...) {
      stopping_.store(true);
      stop_input();
      if(receiver_) receiver_->stop();
      if(control_thread_.joinable()) control_thread_.join();
      if(heartbeat_thread_.joinable()) heartbeat_thread_.join();
      if(output_) output_->abort();
      throw;
    }
  }

  // Blocks until the runtime reaches a terminal state.  A fault also returns;
  // finish() then emits an incomplete marker and drains only accepted output.
  void wait() {
    if(!started_) throw std::logic_error("native session gateway not started");
    while(!stopping_.load()) {
      const auto snapshot=runtime_->snapshot();
      if(snapshot.state.phase=="fault" || snapshot.state.shutdown_complete ||
         (!snapshot.running && snapshot.ticks>0)) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  void finish() {
    close(false);
  }

  void abort() noexcept {
    try { close(true); } catch(...) {}
  }

  SessionSnapshot snapshot() const { return runtime_->snapshot(); }
  // Only valid after finish/abort. Home completion alone doesn't imply outputs drained.
  bool output_complete() const {return output_complete_;}

  // Main-thread local UI follows the SAME runtime event admission/state machine
  // as terminal input. Upper-half IDs identify UI replies without collisions.
  bool request_viewer_action(GatewayAction action) {
    if(!started_ || closed_ || stopping_.load()) return false;
    const auto ordinal=viewer_sequence_++;
    if(ordinal>=(std::uint64_t(1)<<60)) throw std::overflow_error("viewer action sequence exhausted");
    const auto id=(std::uint64_t(1)<<63)|(ordinal<<3)|static_cast<std::uint8_t>(action);
    SessionEvent event{id,action_name(action),"",true};
    if(action==GatewayAction::rearm) {
      const auto epoch=runtime_->snapshot().state.epoch;
      if(epoch==std::numeric_limits<std::int64_t>::max()) return false;
      event.next_epoch=epoch+1;
    }
    if(runtime_->submit(std::move(event))) return true;
    report_failure("native viewer operator admission rejected");return false;
  }

 private:
  static std::string action_name(GatewayAction action) {
    switch(action) {
      case GatewayAction::start: return "start";
      case GatewayAction::return_home: return "return";
      case GatewayAction::shutdown: return "shutdown";
      case GatewayAction::rearm: return "rearm";
      case GatewayAction::calibrate: return "calibrate";
    }
    throw std::invalid_argument("unknown native gateway action");
  }

  bool accept_datagram(ReceivedDatagram frame) {
    if(stopping_.load()) return true;
    SessionEvent event{next_event_id_.fetch_add(1),"raw","",false};
    event.reply=false;
    event.raw=RawDatagram{source_,std::move(frame.bytes),frame.received_ns};
    if(runtime_->submit(std::move(event))) return true;
    report_failure("native gateway raw input admission rejected");
    return false;
  }

  bool submit_source_status() {
    SessionEvent event{0,"status","",false};
    event.reply=false;
    HealthStatus status; status.role=0; status.authority=source_;
    status.sequence=heartbeat_sequence_.fetch_add(1);
    status.ready=status.healthy=status.simulation=true;
    event.status=status;
    return runtime_->submit(std::move(event));
  }

  void report_failure(const std::string& reason) noexcept {
    if(stopping_.load()) return;
    try { runtime_->report_failure(reason); } catch(...) {}
  }

  void heartbeat_loop() noexcept {
    while(!stopping_.load()) {
      std::this_thread::sleep_for(heartbeat_period_);
      if(stopping_.load()) break;
      if(!submit_source_status()) {
        report_failure("native gateway source heartbeat admission rejected");
        break;
      }
    }
  }

  bool read_command(GatewayCommand& command) {
    std::array<std::uint8_t,kGatewayCommandWireSize> bytes{};
    std::size_t offset=0;
    while(offset<bytes.size() && !stopping_.load()) {
      pollfd descriptor{control_.get(),POLLIN,0};
      const int ready=poll(&descriptor,1,50);
      if(ready<0 && errno==EINTR) continue;
      if(ready<0) throw std::runtime_error("native gateway control poll failed");
      if(ready==0) continue;
      if(descriptor.revents&(POLLERR|POLLNVAL)) throw std::runtime_error("native gateway control unavailable");
      if(descriptor.revents&POLLHUP) {
        const auto count=recv(control_.get(),bytes.data()+offset,bytes.size()-offset,MSG_DONTWAIT);
        if(count<=0) {
          if(offset==0) return false;
          throw std::runtime_error("truncated native gateway command");
        }
        offset+=static_cast<std::size_t>(count); continue;
      }
      const auto count=recv(control_.get(),bytes.data()+offset,bytes.size()-offset,MSG_DONTWAIT);
      if(count>0) { offset+=static_cast<std::size_t>(count); continue; }
      if(count==0) {
        if(offset==0) return false;
        throw std::runtime_error("truncated native gateway command");
      }
      if(errno==EINTR || errno==EAGAIN || errno==EWOULDBLOCK) continue;
      throw std::runtime_error("native gateway control read failed");
    }
    if(stopping_.load()) return false;
    command=GatewayWireCodec::decode_command(bytes);
    return true;
  }

  void control_loop() noexcept {
    try {
      GatewayCommand command;
      while(!stopping_.load() && read_command(command)) {
        SessionEvent event{command.id,action_name(command.action),"",true};
        event.next_epoch=command.next_epoch;
        if(!runtime_->submit(std::move(event))) {
          report_failure("native gateway operator admission rejected");
          break;
        }
        if(command.action==GatewayAction::shutdown) {
          // The runtime owns the Home transition.  Keep this thread alive until
          // wait()/finish() drains the final command and completion frame.
        }
      }
      if(!stopping_.load() && !runtime_->snapshot().state.shutdown_complete)
        report_failure("native gateway control channel closed");
    } catch(const std::exception& error) {
      report_failure(error.what());
    } catch(...) {
      report_failure("unknown native gateway control failure");
    }
  }

  void write_output(const SessionOutputItem& item) {
    if(fanout_) {
      if(!fanout_->submit(item)) throw std::runtime_error("native output admission failed: "+fanout_->failure());
      return;
    }
    std::visit([this](const auto& value){ writer_->write(value); },item);
  }

  void close(bool aborting) {
    std::lock_guard<std::mutex> lifecycle(lifecycle_);
    if(closed_) return;
    stopping_.store(true);
    stop_input();
    if(receiver_) receiver_->stop();
    if(control_thread_.joinable()) control_thread_.join();
    if(heartbeat_thread_.joinable()) heartbeat_thread_.join();
    if(output_) {
      if(aborting || input_stop_failed_) output_->abort();
      else output_->finish();
      output_complete_=output_->stats().session_complete;
      output_.reset();
    }
    // Join consumers while callbacks' writer/publication resources still exist.
    fanout_.reset();
    control_.reset();
    writer_.reset();
    receiver_.reset();
    closed_=true;
  }

  void stop_input() noexcept {
    if(!input_active_) return;
    input_active_=false;
    try {input_.stop();}catch(...) {input_stop_failed_=true;}
  }

  std::unique_ptr<SessionRuntime> runtime_;
  Authority source_;
  OwnedDescriptor control_;
  std::unique_ptr<GatewayWireWriter> writer_;
  std::unique_ptr<DatagramReceiver> receiver_;
  std::unique_ptr<SessionOutputBridge> output_;
  std::unique_ptr<OutputFanout<SessionOutputItem>> fanout_;
  int io_timeout_ms_;
  std::chrono::milliseconds heartbeat_period_;
  std::function<void(const SessionCycleSnapshot&)> publish_;
  std::function<void()> cancel_publication_;
  NativeRecordingHooks recording_;
  std::function<void(const SessionOutputItem&)> display_;
  NativeInputHooks input_;
  bool input_active_=false,input_stop_failed_=false;
  std::uint64_t viewer_sequence_=1;
  std::atomic<std::uint64_t> next_event_id_{1},heartbeat_sequence_{1};
  std::atomic<bool> stopping_{false};
  std::mutex lifecycle_;
  std::thread control_thread_,heartbeat_thread_;
  bool started_=false,closed_=false,output_complete_=false;
};

} // namespace tianji_control
