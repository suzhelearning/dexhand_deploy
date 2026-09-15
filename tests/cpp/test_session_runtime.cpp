#include "../../native/control/session_runtime.hpp"
#include <cassert>
#include <chrono>
#include <thread>
using namespace tianji_control;
int main() {
  {
    SessionRuntime capture(1000000,100000000,8,false);
    for(auto capacity:{std::size_t(0),std::size_t(8193)}) {
      bool rejected=false;
      try {capture.enable_cycle_capture(capacity);}catch(const std::invalid_argument&){rejected=true;}
      assert(rejected);
    }
    capture.enable_cycle_capture(2);
    capture.start();
    for(int i=0;i<1000 && !capture.snapshot().cycle_capture_failed;++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    capture.stop();
    assert(capture.snapshot().cycle_capture_failed);
    assert(capture.snapshot().state.phase=="fault");
    SessionCycleSnapshot first,second;
    assert(capture.pop_cycle(first) && capture.pop_cycle(second));
    assert(first.snapshot.ticks==1 && second.snapshot.ticks==2);
    assert(first.timestamp_ns<=second.timestamp_ns);
    assert(!first.request && !first.result && !first.ik_adopted);
    assert(first.snapshot.state.phase=="idle");
    assert(!capture.pop_cycle(second));
    bool rejected=false;
    try {capture.enable_cycle_capture(4);}catch(const std::logic_error&){rejected=true;}
    assert(rejected);
    SessionRuntime disabled(1000000,100000000,8,false);
    disabled.start();std::this_thread::sleep_for(std::chrono::milliseconds(5));disabled.stop();
    assert(!disabled.pop_cycle(second) && !disabled.snapshot().cycle_capture_failed);
  }
  AbsoluteDeadline d(100, 5);
  assert(d.next()==105); d.advance(); assert(d.next()==110);
  d.reanchor(200); assert(d.next()==205);
  bool invalid=false;
  try { AbsoluteDeadline bad(0, 0); } catch (const std::invalid_argument&) { invalid=true; }
  assert(invalid);
  invalid=false;
  try { AbsoluteDeadline bad(INT64_MAX,1); } catch (const std::overflow_error&) { invalid=true; }
  assert(invalid);
  SessionFacts f;
  f.source_ready=f.producer_ready=f.executor_ready=f.arm_fresh=true;
  f.arm_home=f.arm_exact_home=f.command_home=true;
  SessionMachine reducer(10,20,false);
  assert(!reducer.intent(1,"start","",false,f).accepted);
  assert(!reducer.rearm(2,2,f,false).accepted);
  f.arm_exact_home=false;
  assert(!reducer.rearm(3,2,f,true).accepted);
  f.arm_exact_home=true;
  assert(reducer.rearm(4,2,f,true).accepted);
  assert(!reducer.intent(5,"start","",true,f).accepted);
  ++f.input_revision;
  assert(reducer.intent(6,"start","",true,f).accepted);
  reducer.tick(5,f);
  assert(reducer.state().phase=="fault");
  SessionMachine hand(10,20,true);
  f.hand_producer_ready=f.hand_zero=true;
  assert(hand.intent(10,"start","",true,f).accepted);
  hand.tick(20,f); assert(hand.state().phase=="teleop");
  hand.tick(21,f); assert(hand.state().phase=="fault");
  SessionRuntime runtime(1000000, 100000000, 4, false);
  runtime.update(f);
  assert(runtime.submit({1, "start", "", true}));
  runtime.start();
  invalid=false;
  try { runtime.start(); } catch (const std::logic_error&) { invalid=true; }
  assert(invalid);
  auto wait_for=[&](auto pred) {
    for (int i=0; i<1000; ++i) {
      if (pred()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  };
  assert(wait_for([&]{ return runtime.snapshot().state.phase=="teleop"; }));
  SessionReply reply;
  assert(runtime.pop(reply) && reply.id==1 && reply.outcome.accepted);
  assert(reply.action=="start" && reply.timestamp_ns>0);
  // Aggregate facts expire even if no new ingress callback arrives.
  assert(wait_for([&]{ return runtime.snapshot().state.phase=="fault"; }));
  runtime.stop(); runtime.stop();
  assert(!runtime.snapshot().state.shutdown_complete);
  assert(!runtime.submit({2,"start","",true}));
  SessionRuntime overflow(1000000,100000000,1,false);
  assert(overflow.submit({1,"start","",true}));
  assert(!overflow.submit({2,"start","",true}));
  overflow.start();
  assert(wait_for([&]{ return overflow.snapshot().state.phase=="fault"; }));
  assert(overflow.snapshot().queue_overflow);
  overflow.stop();
  SessionRuntime reply_overflow(1000000,100000000,1,false);
  reply_overflow.update(f); reply_overflow.submit({1,"unsupported","",true});
  reply_overflow.start();
  assert(wait_for([&]{ return reply_overflow.snapshot().ticks>0; }));
  assert(reply_overflow.submit({2,"start","",true}));
  assert(wait_for([&]{ return reply_overflow.snapshot().queue_overflow; }));
  assert(reply_overflow.snapshot().state.phase=="fault");
  reply_overflow.stop();
  SessionRuntime home(1000000,100000000,4,false);
  home.update(f); home.submit({1,"shutdown","",true}); home.start();
  assert(wait_for([&]{ return home.snapshot().state.shutdown_complete; }));
  home.stop();
  SessionRuntime sleeping(10000000000LL,100000000,4,false);
  sleeping.start();
  assert(wait_for([&]{ return sleeping.snapshot().running; }));
  const auto stop_start=std::chrono::steady_clock::now();
  sleeping.stop();
  assert(std::chrono::steady_clock::now()-stop_start<std::chrono::seconds(1));
  assert(!sleeping.snapshot().state.shutdown_complete);
  SessionCommandConfig config;
  config.run="run"; config.producer="ik"; config.instance="worker"; config.router="router";
  for (auto& q:config.lower) q.fill(-2);
  for (auto& q:config.upper) q.fill(2);
  SessionRuntime commands(1000000,100000000,8,false,config);
  commands.update(f); commands.submit({1,"start","",true}); commands.start();
  assert(wait_for([&]{return commands.snapshot().state.phase=="teleop";}));
  SessionProposal p;
  p.run="run"; p.producer="ik"; p.instance="worker"; p.router="router";
  p.epoch=1; p.tick=1;
  p.timestamp=std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  for (auto& q:p.positions) q.fill(.01);
  assert(commands.submit_proposal(2,p));
  assert(wait_for([&]{ auto s=commands.snapshot(); return s.command && s.command->positions==p.positions; }));
  assert(wait_for([&]{ return commands.snapshot().ticks>20; }));
  SessionCommandResult disposition;
  assert(commands.pop_command_receipt(disposition));
  assert(disposition.receipt && disposition.receipt->tick==1 && disposition.receipt->accepted);
  assert(disposition.positions==p.positions);
  commands.stop();
}
