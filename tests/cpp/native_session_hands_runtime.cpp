#include "../../native/control/session_runtime.hpp"
#include "../../native/control/mujoco_endpoint.hpp"
#include <iostream>
using namespace tianji_control;
static std::int64_t clock_ns() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
int main(int argc,char** argv) {
 if(argc!=3) return 2;
 try {
  const std::string mode=argv[2];
  SessionCommandConfig c;c.run=c.router="offline";c.producer="ik";c.instance="worker";
  for(auto& q:c.lower) q.fill(-3.);
  for(auto& q:c.upper) q.fill(3.);
  c.home_duration=.02;c.home_speed=100.;
  SessionHealthConfig h;h.home=c.home;h.age=5000000000LL;
  h.authorities={Authority{"src","source","offline"},Authority{"ik","worker","offline"},Authority{"sim","executor","offline"}};
  SessionHandConfig hc;hc.run=c.run;hc.source={"manus","rawviz","offline"};hc.producer={"hand","native","offline"};hc.age=5000000000LL;
  if(mode=="stale") hc.age=100000000;
  for(auto& q:hc.lower) q.fill(-1.);
  for(auto& q:hc.upper) q.fill(1.);
  for(auto& q:hc.zero_tolerance) q.fill(.01);
  SessionRuntime r(5000000,5000000000LL,256,true,c,h,nullptr,nullptr,200000000,
     [&]{return std::make_unique<MujocoEndpoint>(argv[1],true);},{},false,hc);
  // Exercise the real native scheduler thread/queue boundary, with explicit
  // constant numerical fixtures (not an optimizer-equivalence/device test).
  auto solve=[](const double*,std::uint64_t,double* out) {std::fill_n(out,20,.3);};
  tianji_hand::HandScheduler scheduler({5000000,5000000000LL,16},solve,solve);
  if(mode=="scheduler") scheduler.start();
  for(int role=0;role<2;++role) {
    SessionEvent e{std::uint64_t(role+1),"status","",false};
    HealthStatus v;v.authority=h.authorities[role];v.role=role;v.sequence=1;v.ready=v.healthy=v.simulation=true;e.status=v;r.submit(e);
  }
  auto wait=[&](auto predicate) {
    auto deadline=clock_ns()+3000000000LL;
    while(clock_ns()<deadline) {if(predicate()) return;std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    throw std::runtime_error("timeout: "+r.snapshot().state.reason);
  };
  auto reply=[&](std::uint64_t id,bool accepted) {
    wait([&]{SessionReply out;while(r.pop(out)) if(out.id==id) {
      if(out.outcome.accepted!=accepted) throw std::runtime_error(out.outcome.reason);
      return true;
    }return false;});
  };
  auto sample=[&](std::uint64_t seq,bool teleop,bool invalid=false) {
    auto stamp=clock_ns();
    SessionEvent in{seq*10,"hand_input","",false};in.reply=false;
    in.hand_input=HandInputReceipt{hc.source,seq,static_cast<std::uint64_t>(stamp),1,3};
    if(!r.submit(in)) throw std::runtime_error("input admission rejected");
    SessionEvent out{seq*10+1,"hand_result","",false};out.reply=false;
    HandResultEnvelope result;result.run=hc.run;result.producer=hc.producer;
    auto& v=result.value;v.output_sequence=seq;v.input_sequence=seq;v.input_timestamp_ns=stamp;
    v.scheduler_timestamp_ns=clock_ns();v.valid_flags=3;
    v.phase=teleop?tianji_hand::HandPhase::teleop:tianji_hand::HandPhase::idle;
    v.status=teleop?tianji_hand::HandOutputStatus::command:tianji_hand::HandOutputStatus::processed;
    v.positions.fill(invalid?2.:.3);
    if(mode=="scheduler") {
      tianji_hand::HandSession session;session.sequence=seq;session.timestamp_ns=stamp;session.phase=v.phase;
      tianji_hand::HandInput points;points.sequence=seq;points.timestamp_ns=stamp;points.flags=3;points.generation=1;
      if(!scheduler.submit_session(session) || !scheduler.submit_input(points) ||
         !scheduler.wait_pop(v,std::chrono::milliseconds(1000))) throw std::runtime_error("native hand scheduler did not produce result");
    }
    out.hand_result=result;
    if(!r.submit(out)) throw std::runtime_error("result admission rejected");
  };
  r.start();wait([&]{return r.snapshot().simulation_ready;});
  r.submit({100,"start","",true});reply(100,false); // No fabricated readiness.
  sample(1,false);r.submit({101,"start","",true});reply(101,true);
  sample(2,true);wait([&]{auto s=r.snapshot();return s.hand_feedback && (*s.hand_feedback)[0][0]==.3;});
  if(mode=="fault" || mode=="stale") {
    if(mode=="fault") sample(3,true,true);
    wait([&]{return r.snapshot().state.phase=="fault";});
    r.stop();auto s=r.snapshot();
    if(!s.hand_feedback || (*s.hand_feedback)[0][0]!=.3 || (*s.hand_feedback)[1][0]!=.3 || s.state.shutdown_complete)
      throw std::runtime_error("fault did not hold actual hand position");
    if(mode=="stale" && s.state.reason!="producer_hand stale or unhealthy") throw std::runtime_error(s.state.reason);
  } else {
    // Queue a valid command followed by return in the same tick: only zero may apply.
    sample(3,true);r.submit({102,"shutdown","",true});reply(102,true);
    wait([&]{return r.snapshot().state.shutdown_complete;});r.stop();auto s=r.snapshot();
    if(!s.hand_feedback || *s.hand_feedback!=hc.zero || s.state.phase!="idle") throw std::runtime_error("hand Home incomplete");
  }
  std::cout<<"native hands runtime "<<mode<<" passed\n";
 } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
