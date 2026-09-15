#include "../../native/control/session_runtime.hpp"
#include "../../native/control/mujoco_endpoint.hpp"
#include <iostream>
#include <atomic>
using namespace tianji_control;
static std::int64_t clock_ns() { return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
int main(int argc,char** argv) {
  if(argc!=3) return 2;
  try {
    std::string mode=argv[2];
    SessionCommandConfig c; c.run=c.router="offline"; c.producer="ik"; c.instance="worker";
    c.home={Joints7{1.1,-1.52,-1.52,-1.1,0,0,0},Joints7{-1.1,-1.52,1.52,-1.1,0,0,0}};
    for(auto& q:c.lower) q.fill(-3.2);
    for(auto& q:c.upper) q.fill(3.2);
    c.home_duration=.05; c.home_speed=100.; c.math.maximum_step=.5;
    SessionHealthConfig h; h.home=c.home; h.age=10000000000LL;
    h.authorities={Authority{"src","source","offline"},Authority{"ik","worker","offline"},Authority{"sim","executor","offline"}};
    std::atomic<int> created{0},destroyed{0}; const auto caller=std::this_thread::get_id();
    struct Endpoint:SimulationEndpoint {
      MujocoEndpoint sim;
      std::atomic<int>& destroyed;
      std::thread::id owner=std::this_thread::get_id();
      bool fail,invalid_feedback; int calls=0;
      Endpoint(const std::string& path,std::atomic<int>& d,bool f,bool bad):sim(path),destroyed(d),fail(f),invalid_feedback(bad) {}
      ~Endpoint() { if(std::this_thread::get_id()==owner) ++destroyed; }
      void apply(const ArmPair& q) override { if(fail && ++calls>1) throw std::runtime_error("injected execution failure"); sim.apply(q); }
      ArmPair feedback() override {
        auto q=sim.feedback(); if(invalid_feedback) q[1][0]=std::numeric_limits<double>::quiet_NaN(); return q;
      }
    };
    SessionRuntime runtime(5000000,100000000,64,false,c,h,nullptr,nullptr,200000000,[&]() -> std::unique_ptr<SimulationEndpoint> {
      if(caller==std::this_thread::get_id()) throw std::runtime_error("factory on caller thread");
      ++created;
      if(mode=="load_fail") throw std::runtime_error("injected load failure");
      if(mode=="null_fail") return nullptr;
      if(mode=="stop_loading") std::this_thread::sleep_for(std::chrono::milliseconds(100));
      return std::make_unique<Endpoint>(argv[1],destroyed,mode=="apply_fail",mode=="feedback_fail");
    });
    SessionEvent forged{30,"feedback","",false}; ArmFeedback f; f.authority=h.authorities[2]; f.sequence=1; f.names=arm_names(); f.positions=c.home;
    forged.feedback=f;
    if(runtime.submit(forged)) throw std::runtime_error("external feedback accepted");
    SessionEvent status{31,"status","",false}; HealthStatus s; s.role=2; s.sequence=1; s.authority=h.authorities[2]; s.ready=s.healthy=s.simulation=true; status.status=s;
    if(runtime.submit(status)) throw std::runtime_error("external executor status accepted");
    for(int role=0;role<2;++role) {
      status.id=role+1; s.role=role; s.authority=h.authorities[role]; status.status=s; runtime.submit(status);
    }
    runtime.start();
    auto until=clock_ns()+5000000000LL;
    if(mode=="stop_loading") {
      while(created==0 && clock_ns()<until) std::this_thread::sleep_for(std::chrono::milliseconds(1));
      runtime.stop(); auto snap=runtime.snapshot();
      if(created!=1 || destroyed!=1 || snap.ticks || snap.simulation_ready || snap.feedback || snap.state.shutdown_complete) return 1;
      std::cout<<"native_thread_simulation_complete\n"; return 0;
    }
    while(!runtime.snapshot().simulation_ready && runtime.snapshot().state.phase!="fault" && clock_ns()<until)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if(mode.find("_fail")!=std::string::npos) {
      while(runtime.snapshot().state.phase!="fault" && clock_ns()<until) std::this_thread::sleep_for(std::chrono::milliseconds(1));
      runtime.stop();
      if(runtime.snapshot().state.phase!="fault" || runtime.snapshot().state.shutdown_complete || created!=1 || destroyed!=((mode=="apply_fail" || mode=="feedback_fail")?1:0)) return 1;
      std::cout<<"fault_cleaned\n"; return 0;
    }
    auto reply=[&](std::uint64_t id) {
      const auto deadline=clock_ns()+2000000000LL;
      while(clock_ns()<deadline) {
        SessionReply r;
        while(runtime.pop(r)) if(r.id==id) { if(!r.outcome.accepted) throw std::runtime_error(r.outcome.reason); return; }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      throw std::runtime_error("reply timeout");
    };
    runtime.submit({3,"start","",true}); reply(3);
    SessionProposal p; p.run=c.run; p.router=c.router; p.producer=c.producer; p.instance=c.instance;
    p.epoch=1; p.tick=1; p.timestamp=clock_ns(); p.positions=c.home; p.positions[0][0]+=.1;
    runtime.submit_proposal(4,p); reply(4);
    auto snap=runtime.snapshot();
    if(!snap.feedback || *snap.feedback!=p.positions) throw std::runtime_error("owned feedback mismatch");
    if(mode=="interrupt") runtime.stop();
    else {
      runtime.submit({5,"shutdown","",true}); reply(5);
      while(!runtime.snapshot().state.shutdown_complete && clock_ns()<until) std::this_thread::sleep_for(std::chrono::milliseconds(1));
      runtime.stop();
      snap=runtime.snapshot();
      if(!snap.state.shutdown_complete || snap.state.phase!="idle" || !snap.feedback || *snap.feedback!=c.home) throw std::runtime_error("Home incomplete");
    }
    if(mode=="interrupt" && runtime.snapshot().state.shutdown_complete) throw std::runtime_error("interrupt falsely completed Home");
    if(created!=1 || destroyed!=1 || runtime.snapshot().simulation_ready) throw std::runtime_error("wrong-thread cleanup or dangling readiness");
    std::cout<<"native_thread_simulation_complete\n";
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
