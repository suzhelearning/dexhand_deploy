#include "../../native/control/session_output_bridge.hpp"
#include "../../native/control/datagram_receiver.hpp"
#include <cassert>
#include <condition_variable>
#include <vector>
using namespace tianji_control;
template<class F> void until(F predicate) {
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(!predicate()) {
    assert(std::chrono::steady_clock::now()<deadline);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
SessionFacts ready() {
  SessionFacts f; f.source_ready=f.producer_ready=f.executor_ready=f.arm_fresh=true;
  f.arm_home=f.arm_exact_home=f.command_home=true; return f;
}
SessionCommandConfig config() {
  SessionCommandConfig c; c.run="offline"; c.producer="ik"; c.instance="worker"; c.router="offline";
  for(auto& q:c.lower) q.fill(-3);
  for(auto& q:c.upper) q.fill(3);
  return c;
}
int main() {
  // Reporting does not depend on space in the ordinary event queue, and cannot
  // manufacture an operator start or rewrite an earlier local failure.
  SessionRuntime prestart(1000000,1000000000,1,false);
  assert(prestart.submit({1,"start","",true}));
  assert(prestart.report_failure("input reader failed"));
  assert(!prestart.report_failure("replacement"));
  prestart.start(); until([&]{return prestart.snapshot().state.phase=="fault";});
  assert(prestart.snapshot().state.reason=="input reader failed"); prestart.stop();
  assert(!prestart.report_failure("after stop"));
  SessionRuntime sleeping(10000000000LL,1000000000,8,false);
  sleeping.start(); until([&]{return sleeping.snapshot().running;});
  assert(!sleeping.report_failure(""));
  assert(!sleeping.report_failure(std::string(4097,'x')));
  assert(sleeping.report_failure("output disconnected"));
  until([&]{return sleeping.snapshot().state.phase=="fault";}); sleeping.stop();

  SessionRuntime runtime(1000000,1000000000,64,false,config());
  runtime.enable_cycle_capture(64);
  runtime.update(ready());
  std::vector<SessionOutputItem> written;
  bool finalized=false,complete=false;
  std::atomic<int> received_commands{0};
  SessionOutputBridge bridge(runtime,64,[&](const SessionOutputItem& item){
    written.push_back(item);if(std::holds_alternative<SessionCommandResult>(item)) ++received_commands;
  },
    [&](bool success){finalized=true; complete=success;},[]{});
  runtime.start(); assert(runtime.submit({1,"start","",true}));
  until([&]{return runtime.snapshot().state.phase=="teleop";});
  SessionProposal p; p.run="offline"; p.producer="ik"; p.instance="worker"; p.router="offline";
  p.epoch=1; p.tick=1; p.timestamp=std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  // Home-valued proposal permits a complete shutdown without artificial feedback.
  assert(runtime.submit_proposal(2,p));
  until([&]{return received_commands==1;});
  assert(runtime.submit({3,"shutdown","",true}));
  until([&]{return runtime.snapshot().state.shutdown_complete;});
  bridge.finish(); bridge.finish();
  assert(finalized && complete && bridge.stats().session_complete && bridge.stats().output.complete);
  std::vector<std::uint64_t> reply_ids; int receipts=0;std::uint64_t cycles=0;bool last_home=false;
  for(const auto& item:written) {
    if(const auto* reply=std::get_if<SessionReply>(&item)) reply_ids.push_back(reply->id);
    else if(const auto* cycle=std::get_if<SessionCycleSnapshot>(&item)) {
      assert(cycle->snapshot.ticks==++cycles);last_home=cycle->snapshot.state.shutdown_complete;
    } else {const auto& r=std::get<SessionCommandResult>(item); assert(r.receipt->tick==1); ++receipts;}
  }
  assert((reply_ids==std::vector<std::uint64_t>{1,2,3}) && receipts==1);
  assert(cycles==runtime.snapshot().ticks && last_home && !runtime.snapshot().cycle_capture_failed);
  bridge.abort(); assert(bridge.stats().session_complete); // Closing again is idempotent.

  // Failure of a real consumer cancels command acceptance through the local
  // failure lane, independently of operator identities and reply queues.
  SessionRuntime failed(1000000,1000000000,64,false,config()); failed.update(ready());
  SessionOutputBridge failure(failed,4,[](const SessionOutputItem&){throw std::runtime_error("disk failed");},
    [](bool){assert(false);},[]{});
  failed.start(); assert(failed.submit({1,"start","",true}));
  until([&]{return failed.snapshot().state.phase=="fault";});
  assert(!failure.stats().failure.empty());
  failure.finish(); assert(!failure.stats().session_complete);
  assert(!failed.snapshot().state.shutdown_complete);

  // A stopped/interrupted session may drain but must finalize as incomplete.
  SessionRuntime interrupted(1000000,1000000000,64,false);
  interrupted.update(ready()); complete=true; finalized=false;
  SessionOutputBridge partial(interrupted,4,[](const SessionOutputItem&){},
    [&](bool success){finalized=true; complete=success;},[]{});
  interrupted.start(); partial.finish();
  assert(finalized && !complete && !partial.stats().session_complete);

  // Queue overflow: control keeps ticking while the output is blocked, then
  // enters fault. Abort wakes the blocked sink before joining output threads.
  std::mutex mutex; std::condition_variable wake; bool entered=false,released=false;
  SessionRuntime overflow(1000000,1000000000,64,false); overflow.update(ready());
  SessionOutputBridge slow(overflow,1,[&](const SessionOutputItem&) {
    std::unique_lock<std::mutex> lock(mutex); entered=true; wake.notify_all();
    wake.wait(lock,[&]{return released;});
  },[](bool){assert(false);},[&]{std::lock_guard<std::mutex> lock(mutex); released=true; wake.notify_all();});
  overflow.start(); assert(overflow.submit({1,"unsupported","",true}));
  {std::unique_lock<std::mutex> lock(mutex); wake.wait(lock,[&]{return entered;});}
  const auto before=overflow.snapshot().ticks;
  assert(overflow.submit({2,"unsupported","",true}));
  assert(overflow.submit({3,"unsupported","",true}));
  until([&]{return overflow.snapshot().state.phase=="fault";});
  assert(overflow.snapshot().ticks>before && !slow.stats().failure.empty());
  slow.abort(); assert(!slow.stats().session_complete);

  SessionRuntime raw_failure(1000000,1000000000,8,false); raw_failure.update(ready());
  int pair[2]; assert(socketpair(AF_UNIX,SOCK_DGRAM|SOCK_CLOEXEC,0,pair)==0);
  DatagramReceiver input(OwnedDescriptor(pair[0]),[](ReceivedDatagram){throw std::runtime_error("raw audit unavailable"); return false;},
    [&](const std::string& error){assert(raw_failure.report_failure(error));});
  raw_failure.start(); input.start(); assert(send(pair[1],"data",4,0)==4);
  until([&]{return raw_failure.snapshot().state.phase=="fault";});
  assert(raw_failure.snapshot().state.reason=="raw audit unavailable");
  input.stop(); raw_failure.stop(); close(pair[1]);

  SessionRuntime close_failed(1000000,1000000000,8,false); close_failed.update(ready());
  SessionOutputBridge bad_close(close_failed,8,[](const SessionOutputItem&){},
    [](bool){throw std::runtime_error("disk close failed");},[]{});
  close_failed.start(); assert(close_failed.submit({1,"shutdown","",true}));
  until([&]{return close_failed.snapshot().state.shutdown_complete;});
  bad_close.finish();
  assert(!bad_close.stats().session_complete && bad_close.stats().failure=="disk close failed");
}
