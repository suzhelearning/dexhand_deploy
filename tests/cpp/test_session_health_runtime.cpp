#include "../../native/control/session_runtime.hpp"
#include <cassert>
using namespace tianji_control;
int main() {
  SessionCommandConfig command;
  command.run="run"; command.producer="ik"; command.instance="worker"; command.router="router";
  for(auto& q:command.lower) q.fill(-2);
  for(auto& q:command.upper) q.fill(2);
  SessionHealthConfig health;
  health.authorities={Authority{"src","source","router"},Authority{"ik","worker","router"},
                      Authority{"sim","executor","router"}};
  SessionRuntime runtime(1000000,100000000,16,false,command,health);
  SessionFacts fake;
  fake.source_ready=fake.producer_ready=fake.executor_ready=fake.arm_fresh=fake.arm_home=true;
  assert(!runtime.update(fake));
  for(int role=0;role<3;++role) {
    SessionEvent event{static_cast<unsigned>(role+1),"status","",false};
    HealthStatus value;
    value.role=role; value.authority=health.authorities[role]; value.sequence=1;
    value.ready=value.healthy=value.simulation=true;
    event.status=value; assert(runtime.submit(event));
  }
  SessionEvent feedback{4,"feedback","",false};
  ArmFeedback value;
  value.authority=health.authorities[2]; value.sequence=1; value.names=arm_names();
  feedback.feedback=value; assert(runtime.submit(feedback));
  runtime.submit({5,"start","",true});
  runtime.start();
  auto wait_for=[&](auto predicate) {
    for(int i=0;i<1000;++i) {
      if(predicate()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  };
  assert(wait_for([&]{return runtime.snapshot().state.phase=="teleop";}));
  assert(runtime.submit(feedback)); // replayed measured-state sequence
  assert(wait_for([&]{return runtime.snapshot().state.phase=="fault";}));
  assert(runtime.snapshot().state.reason=="arm state sequence rollback");
  runtime.stop();
  bool invalid=false;
  try { SessionRuntime bad(1000000,100000000,16,true,command,health); }
  catch(const std::invalid_argument&) { invalid=true; }
  assert(invalid);
  health.age=10000000;
  SessionRuntime queued(1000000,100000000,16,false,command,health);
  feedback.received_ns=INT64_MAX; // Sender cannot manufacture a future admission time.
  assert(queued.submit(feedback));
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  for(int role=0;role<3;++role) {
    SessionEvent event{static_cast<unsigned>(role+1),"status","",false};
    HealthStatus status;
    status.role=role; status.authority=health.authorities[role]; status.sequence=1;
    status.ready=status.healthy=status.simulation=true;
    event.status=status; assert(queued.submit(event));
  }
  queued.submit({5,"start","",true}); queued.start();
  assert(wait_for([&]{return queued.snapshot().ticks>1;}));
  SessionReply reply;
  bool denied=false;
  while(queued.pop(reply)) if(reply.id==5) denied=!reply.outcome.accepted;
  assert(denied && queued.snapshot().state.phase=="idle");
  queued.stop();
}
