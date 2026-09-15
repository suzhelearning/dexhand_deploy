#include "../../native/control/session_runtime.hpp"
#include "../../native/control/native_hand_reset_endpoint.hpp"
#include "../../native/control/mujoco_endpoint.hpp"
#include "../../native/control/worker_reset_client.hpp"
#include <iostream>
using namespace tianji_control;
// Arm reset fixture isolates the joint transaction; hand side uses the actual
// native Hand2 child. This does not substitute for actual-arm-worker parity.
class ArmReset final:public ResetEndpoint {
 public:
  explicit ArmReset(bool fail):fail_(fail){}
  void reset(const std::array<double,14>&,std::int64_t e) override {
    if(fail_) throw std::runtime_error("injected arm reset failure");
    epoch_=e;
  }
  std::int64_t epoch() const override {return epoch_;}
  void request_stop() noexcept override {}
 private: bool fail_;std::int64_t epoch_=1;
};
int main(int argc,char** argv) {
 if(argc<5) return 2;
 try {
  const std::string mode=argv[2];
  std::vector<std::string> command;
  int split=3;for(;split<argc && std::string(argv[split])!="--arm-worker";++split) command.emplace_back(argv[split]);
  std::unique_ptr<ResetEndpoint> arm=std::make_unique<ArmReset>(mode=="arm_fail");
  if(split<argc) {
    if(split+3>=argc) throw std::runtime_error("missing arm command");
    std::vector<std::string> arm_command;for(int i=split+3;i<argc;++i) arm_command.emplace_back(argv[i]);
    arm=std::make_unique<WorkerResetClient>(arm_command,argv[split+1],argv[split+2],20000);
  }
  auto hand=std::make_unique<NativeHandResetEndpoint>(command,3000);
  auto* peer=hand.get();
  if(mode=="missing") hand.reset();
  SessionCommandConfig c;c.run=c.router="offline";c.producer="ik";c.instance="worker";
  c.home={Joints7{1.1,-1.52,-1.52,-1.1,0,0,0},Joints7{-1.1,-1.52,1.52,-1.1,0,0,0}};
  for(auto& q:c.lower) q.fill(-3.2);
  for(auto& q:c.upper) q.fill(3.2);
  SessionHealthConfig h;h.home=c.home;h.age=10000000000LL;
  h.authorities={Authority{"src","source","offline"},Authority{"ik","worker","offline"},Authority{"sim","executor","offline"}};
  SessionHandConfig hc;hc.run=c.run;hc.source={"manus","rawviz","offline"};hc.producer={"hand","native","offline"};hc.age=10000000000LL;
  for(auto& q:hc.lower) q.fill(-3.2);
  for(auto& q:hc.upper) q.fill(3.2);
  for(auto& q:hc.zero_tolerance) q.fill(.01);
  SessionRuntime runtime(5000000,10000000000LL,64,true,c,h,
    std::move(arm),nullptr,200000000,
    [&]{return std::make_unique<MujocoEndpoint>(argv[1],true);},{},false,hc,std::move(hand));
  for(int role=0;role<2;++role) {
    SessionEvent e{std::uint64_t(role+1),"status","",false};
    HealthStatus s;s.role=role;s.sequence=1;s.authority=h.authorities[role];s.ready=s.healthy=s.simulation=true;
    e.status=s;runtime.submit(e);
  }
  runtime.start();
  auto wait=[&](auto predicate) {
    const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while(std::chrono::steady_clock::now()<until) {
      if(predicate()) return;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    throw std::runtime_error("timeout: "+runtime.snapshot().state.reason);
  };
  wait([&]{return runtime.snapshot().simulation_ready;});
  runtime.submit({10,"rearm","",true,2,false});
  if(mode=="cancel" || mode=="source_loss") {
    wait([&]{return runtime.snapshot().reset_pending;});
    const auto ticks=runtime.snapshot().ticks;
    wait([&]{return runtime.snapshot().ticks>=ticks+3;});
    runtime.submit({12,"start","",true});
    wait([&]{SessionReply r;while(runtime.pop(r)) if(r.id==12) {
      if(r.outcome.accepted) throw std::runtime_error("start during reset accepted");
      return true;}return false;});
    if(mode=="source_loss") {
      SessionEvent lost{13,"status","",false};
      HealthStatus s;s.role=0;s.sequence=2;s.authority=h.authorities[0];
      s.ready=false;s.healthy=s.simulation=true;lost.status=s;
      if(!runtime.submit(lost)) throw std::runtime_error("source loss admission failed");
      wait([&]{const auto state=runtime.snapshot();return state.state.phase=="fault" && !state.reset_pending;});
    }
    const auto begin=std::chrono::steady_clock::now();
    runtime.stop();
    if(runtime.snapshot().state.epoch!=1 || runtime.snapshot().reset_pending ||
       std::chrono::steady_clock::now()-begin>std::chrono::seconds(2))
      throw std::runtime_error("reset cancellation failed");
    std::cout<<"joint reset cancellation preserved epoch\n";return 0;
  }
  bool accepted=false;
  wait([&]{SessionReply r;while(runtime.pop(r)) if(r.id==10){accepted=r.outcome.accepted;return true;}return false;});
  const auto s=runtime.snapshot();
  if(mode=="ok") {
    if(!accepted || s.state.epoch!=2 || peer->epoch()!=2) throw std::runtime_error("joint reset did not commit");
    runtime.submit({11,"start","",true});
    wait([&]{SessionReply r;while(runtime.pop(r)) if(r.id==11){
      if(r.outcome.accepted) throw std::runtime_error("new hand input barrier bypassed");
      return true;}return false;});
  } else if(mode=="missing") {
    if(accepted || s.state.epoch!=1 || s.state.phase!="idle")
      throw std::runtime_error("missing hand reset endpoint bypassed gate");
  } else if(accepted || s.state.epoch!=1 || s.state.phase!="fault") {
    throw std::runtime_error("partial reset advanced session");
  }
  runtime.stop();std::cout<<"joint hand reset "<<mode<<" passed\n";
 } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
