#include "../../native/control/session_gateway.hpp"

#include <array>
#include <atomic>
#include <cassert>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

using namespace tianji_control;

static void put_unsigned(std::array<std::uint8_t,kGatewayCommandWireSize>& bytes,
                         std::size_t offset,std::uint64_t value) {
  for(std::size_t i=0;i<8;++i) bytes[offset+i]=static_cast<std::uint8_t>(value>>(8*i));
}

static std::array<std::uint8_t,kGatewayCommandWireSize> command(GatewayAction action,
                                                                  std::uint64_t id) {
  std::array<std::uint8_t,kGatewayCommandWireSize> bytes{};
  std::memcpy(bytes.data(),"TJAC",4); bytes[4]=1;
  bytes[5]=static_cast<std::uint8_t>(action); put_unsigned(bytes,8,id);
  return bytes;
}

struct FakeRaw final:RawInputEndpoint {
  RawProgress latest;
  RawProgress ingest(const std::vector<std::uint8_t>& bytes) override {
    if(bytes.empty()) return {};
    latest.decoded=true; latest.ingress_sequence++;
    latest.accepted=true; latest.epoch=1; latest.sequence++;
    latest.generation=1; latest.skeleton_valid=true; latest.rotations_valid=true;
    latest.palms[0]={0.1,0.2,0.3}; latest.palms[1]={0.1,-0.2,0.3};
    return latest;
  }
};

struct FakeSimulation final:SimulationEndpoint {
  ArmPair positions{};
  void apply(const ArmPair& value) override { positions=value; }
  ArmPair feedback() override { return positions; }
};

struct FakeIk final:IkEndpoint {
  ArmPair home;
  explicit FakeIk(ArmPair value):home(value) {}
  WorkerResult step(const WorkerTick& request) override {
    WorkerResult result; result.tick=request.id; result.timestamp=request.now_ns;
    result.applied_epoch=1; result.applied_sequence=request.id; result.input_live=true;
    result.control_executed=true;
    result.arms[0].q=home[0]; result.arms[1].q=home[1];
    if(request.id==1) result.arms[0].q[0]+=0.01;
    for(auto& arm:result.arms) arm.accepted=true;
    return result;
  }
  void request_stop() noexcept override {}
};

static std::uint32_t get_u32(const std::array<std::uint8_t,kGatewayFrameHeaderWireSize>& h,
                             std::size_t offset) {
  std::uint32_t value=0;
  for(std::size_t i=0;i<4;++i) value|=std::uint32_t(h[offset+i])<<(8*i);
  return value;
}

int main(int argc,char** argv) {
  const bool fail_publication=argc>1 && std::string(argv[1])=="--fail-publication";
  const bool fail_final=argc>1 && (std::string(argv[1])=="--fail-final" || std::string(argv[1])=="--summary-fail-final");
  const bool fail_recording=argc>1 && std::string(argv[1])=="--fail-recording";
  const bool fail_recording_close=argc>1 && std::string(argv[1])=="--fail-recording-close";
  const bool viewer_input=argc>1 && std::string(argv[1])=="--viewer-input";
  const bool fail_input_start=argc>1 && std::string(argv[1])=="--fail-input-start";
  const bool fail_input_stop=argc>1 && std::string(argv[1])=="--fail-input-stop";
  const bool summary_mode=argc>1 && (std::string(argv[1])=="--summary" || std::string(argv[1])=="--summary-fail-final");
  SessionCommandConfig command_config;
  command_config.run="gateway-test"; command_config.producer="ik";
  command_config.instance="worker"; command_config.router="offline";
  command_config.math.maximum_step=1.; command_config.math.time_window=0.;
  command_config.math.proposal_timeout=1.; command_config.home_duration=.01;
  command_config.home_speed=100.; command_config.ingress_age=10000000000LL;
  for(auto& side:command_config.lower) side.fill(-3.2);
  for(auto& side:command_config.upper) side.fill(3.2);
  SessionHealthConfig health;
  health.authorities={Authority{"source","source-instance","offline"},
                      Authority{"ik","worker","offline"},
                      Authority{"sim","sim-instance","offline"}};
  health.home=command_config.home; health.age=10000000000LL;
  auto raw=std::make_unique<FakeRaw>();
  auto runtime=std::make_unique<SessionRuntime>(1000000,1000000000,256,false,
      command_config,health,nullptr,std::move(raw),1000000000LL,
      [] { return std::make_unique<FakeSimulation>(); },
      [home=command_config.home] { return std::make_unique<FakeIk>(home); });
  int datagram[2]; int control[2];
  assert(socketpair(AF_UNIX,SOCK_DGRAM|SOCK_CLOEXEC,0,datagram)==0);
  assert(socketpair(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0,control)==0);
  std::atomic<int> publications{0};
  const auto caller_thread=std::this_thread::get_id();
  std::atomic<int> recorded_cycles{0},recorded_raws{0},recorded_replies{0};
  bool recording_finished=false;
  NativeRecordingHooks recording;
  recording.write=[&](const SessionOutputItem& item) {
    if(fail_recording) throw std::runtime_error("injected recording failure");
    assert(std::this_thread::get_id()!=caller_thread);
    if(std::holds_alternative<SessionCycleSnapshot>(item)) ++recorded_cycles;
    if(std::holds_alternative<SessionRawSnapshot>(item)) ++recorded_raws;
    if(const auto* reply=std::get_if<SessionReply>(&item)) {
      assert(!reply->action.empty() && reply->timestamp_ns>0);++recorded_replies;
    }
  };
  recording.finish=[&](bool success,const SessionSnapshot&) {
    if(fail_recording_close) throw std::runtime_error("injected recorder close failure");
    assert(success);recording_finished=true;
  };
  recording.cancel=[]{};
  int input_started=0,input_stopped=0;
  NativeInputHooks input_hooks{[&]{++input_started;if(fail_input_start)throw std::runtime_error("input start failed");},
                              [&]{++input_stopped;if(fail_input_stop)throw std::runtime_error("input stop failed");}};
  NativeSessionGateway gateway(std::move(runtime),OwnedDescriptor(datagram[0]),
      OwnedDescriptor(control[0]),health.authorities[0],256,1000,
      std::chrono::milliseconds(10),[&](const SessionCycleSnapshot& cycle) {
        if(fail_publication || (fail_final && cycle.snapshot.state.shutdown_complete))
          throw std::runtime_error("injected publication failure");
        assert(std::this_thread::get_id()!=caller_thread);
        assert(cycle.snapshot.command && cycle.snapshot.feedback);
        ++publications;
      },[]{},recording,{},summary_mode,input_hooks);

  std::atomic<bool> reader_failed{false}; std::string reader_error;
  std::atomic<int> replies{0}; std::atomic<int> cycles{0};
  std::atomic<int> raws{0};
  std::atomic<bool> start_accepted{false},complete{false};
  std::mutex mutex; std::condition_variable wake;
  std::thread reader([&] {
    try {
      for(;;) {
        std::array<std::uint8_t,kGatewayFrameHeaderWireSize> header{};
        std::size_t offset=0;
        while(offset<header.size()) {
          const auto count=recv(control[1],header.data()+offset,header.size()-offset,0);
          if(count==0) return;
          if(count<0) throw std::runtime_error("gateway test output read failed");
          offset+=static_cast<std::size_t>(count);
        }
        if(std::memcmp(header.data(),"TJSO",4)!=0 || header[4]!=1)
          throw std::runtime_error("invalid gateway output header");
        const auto size=get_u32(header,8);
        if(size>2U*1024U*1024U) throw std::runtime_error("oversized gateway test frame");
        std::vector<std::uint8_t> payload(size); offset=0;
        while(offset<payload.size()) {
          const auto count=recv(control[1],payload.data()+offset,payload.size()-offset,0);
          if(count<=0) throw std::runtime_error("truncated gateway test output");
          offset+=static_cast<std::size_t>(count);
        }
        switch(static_cast<GatewayFrameKind>(header[5])) {
          case GatewayFrameKind::reply:
            ++replies;
            if((header[12]==10 || ((header[19]&0x80) && (header[12]&7)==1)) && payload.size()>=12) {
              start_accepted=payload[8]!=0; wake.notify_all();
            }
            break;
          case GatewayFrameKind::cycle:
            assert(!summary_mode);
            ++cycles; break;
          case GatewayFrameKind::summary:
            assert(summary_mode && payload.size()>=40);
            {std::uint64_t count=0;for(int i=0;i<8;++i) count|=std::uint64_t(payload[24+i])<<(8*i);
             cycles=static_cast<int>(count);}
            break;
          case GatewayFrameKind::raw:
            ++raws; break;
          case GatewayFrameKind::complete:
            if(!payload.empty()) complete=payload[0]!=0;
            wake.notify_all(); return;
          case GatewayFrameKind::receipt:
          case GatewayFrameKind::failure: break;
        }
      }
    } catch(const std::exception& error) {
      reader_error=error.what(); reader_failed=true; wake.notify_all();
    }
  });

  if(fail_input_start) {
    bool failed=false;
    try{gateway.start();}catch(const std::runtime_error&){failed=true;}
    gateway.abort();reader.join();close(control[1]);close(datagram[1]);
    assert(failed && input_started==1 && input_stopped==1 && !gateway.output_complete());
    return 0;
  }
  gateway.start();
  if(fail_publication || fail_recording) {
    gateway.wait(); gateway.finish();
    assert(input_started==1 && input_stopped==1);
    reader.join();close(control[1]);close(datagram[1]);
    assert(gateway.snapshot().state.phase=="fault");
    assert(!gateway.snapshot().state.shutdown_complete);
    assert(!complete);
    return 0;
  }
  const std::array<std::uint8_t,4> packet{{1,2,3,4}};
  for(int i=0;i<4;++i) assert(send(datagram[1],packet.data(),packet.size(),0)==4);
  const auto ready_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  // Datagram ingress and operator commands have independent channels. Model
  // readiness alone doesn't prove that the receiver has delivered the packets.
  while((!gateway.snapshot().simulation_ready || !gateway.snapshot().ik_ready || recorded_raws<4) &&
        std::chrono::steady_clock::now()<ready_deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  assert(gateway.snapshot().simulation_ready && gateway.snapshot().ik_ready && recorded_raws>=4);
  auto start=command(GatewayAction::start,10);
  if(viewer_input) assert(gateway.request_viewer_action(GatewayAction::start));
  else assert(send(control[1],start.data(),start.size(),0)==static_cast<ssize_t>(start.size()));
  {
    std::unique_lock<std::mutex> lock(mutex);
    wake.wait_for(lock,std::chrono::seconds(3),[&]{return start_accepted||reader_failed;});
  }
  if(reader_failed) throw std::runtime_error(reader_error);
  assert(start_accepted);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  auto shutdown_command=command(GatewayAction::shutdown,11);
  if(viewer_input) assert(gateway.request_viewer_action(GatewayAction::shutdown));
  else assert(send(control[1],shutdown_command.data(),shutdown_command.size(),0)==static_cast<ssize_t>(shutdown_command.size()));
  gateway.wait(); gateway.finish();
  assert(input_started==1 && input_stopped==1);
  // Gateway has closed its end. Drain buffered frames/EOF before closing the
  // reader's descriptor; shutdown(SHUT_RDWR) here races with the receive thread.
  reader.join();
  close(control[1]);close(datagram[1]);
  if(fail_final || fail_recording_close || fail_input_stop) {
    assert(!complete);
    assert(!gateway.output_complete());
    return 0;
  }
  assert(recording_finished);
  assert(recorded_cycles==cycles && recorded_replies==replies);
  assert(summary_mode?raws==0:recorded_raws==raws);
  if(reader_failed) throw std::runtime_error(reader_error);
  assert(complete); assert(cycles>0);
  assert(publications==cycles);
  assert(recorded_raws>=4);
  // Raw packets and the periodic source heartbeat are intentionally reply-free;
  // only the two operator intents are visible on the reply lane.
  assert(replies==2);
  assert(gateway.snapshot().state.shutdown_complete);
  assert(gateway.output_complete());
  return 0;
}
