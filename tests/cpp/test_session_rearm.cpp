#include "../../native/control/session_runtime.hpp"
#include <atomic>
#include <cassert>
using namespace tianji_control;
class ResetTestPeer final:public ResetEndpoint {
 public:
  std::atomic<bool> cancelled{false};
  explicit ResetTestPeer(int delay,bool wrong_epoch=false):delay_(delay),wrong_epoch_(wrong_epoch) {}
  void reset(const std::array<double,14>&,std::int64_t epoch) override {
    for(int i=0;i<delay_;++i) {
      if(cancelled) throw std::runtime_error("cancelled");
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    epoch_=epoch+(wrong_epoch_?1:0);
  }
  std::int64_t epoch() const override { return epoch_; }
  void request_stop() noexcept override { cancelled=true; }
 private:
  int delay_; bool wrong_epoch_; std::int64_t epoch_=1;
};
int main() {
  SessionCommandConfig command;
  command.run="run"; command.producer="ik"; command.instance="worker"; command.router="router";
  for(auto& q:command.lower) q.fill(-2);
  for(auto& q:command.upper) q.fill(2);
  SessionHealthConfig health;
  health.authorities={Authority{"src","source","router"},Authority{"ik","worker","router"},Authority{"sim","executor","router"}};
  auto enqueue=[&](SessionRuntime& runtime) {
    for(int role=0;role<3;++role) {
      SessionEvent e{static_cast<unsigned>(role+1),"status","",false};
      HealthStatus s; s.role=role; s.sequence=1; s.authority=health.authorities[role];
      s.ready=s.healthy=s.simulation=true; e.status=s; assert(runtime.submit(e));
    }
    SessionEvent e{4,"feedback","",false};
    ArmFeedback feedback; feedback.sequence=1; feedback.authority=health.authorities[2]; feedback.names=arm_names();
    e.feedback=feedback; assert(runtime.submit(e));
    assert(runtime.submit({5,"rearm","",true,2,false})); // Boolean ack is false: use actual endpoint.
  };
  auto wait_for=[](auto predicate) {
    for(int i=0;i<1000;++i) { if(predicate()) return true; std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
    return false;
  };
  SessionRuntime runtime(1000000,100000000,32,false,command,health,std::make_unique<ResetTestPeer>(40));
  enqueue(runtime); runtime.start();
  assert(wait_for([&]{return runtime.snapshot().reset_pending;}));
  const auto before=runtime.snapshot().ticks;
  runtime.submit({6,"start","",true});
  assert(wait_for([&]{return runtime.snapshot().state.epoch==2;}));
  assert(runtime.snapshot().ticks>before+5); // reset must not freeze the control scheduler
  SessionReply reply; bool reset_ok=false, start_denied=false;
  while(runtime.pop(reply)) {
    if(reply.id==5) reset_ok=reply.outcome.accepted;
    if(reply.id==6) start_denied=!reply.outcome.accepted;
  }
  assert(reset_ok && start_denied);
  SessionEvent heartbeat{9,"status","",false};
  HealthStatus alive; alive.role=0; alive.sequence=2; alive.authority=health.authorities[0];
  alive.ready=alive.healthy=alive.simulation=true; heartbeat.status=alive;
  assert(runtime.submit(heartbeat)); // New heartbeat is not a new source frame.
  runtime.submit({7,"start","",true});
  assert(wait_for([&]{ while(runtime.pop(reply)) if(reply.id==7) return !reply.outcome.accepted; return false; }));
  runtime.stop();
  health.age=10000000;
  SessionRuntime stale(1000000,100000000,32,false,command,health,std::make_unique<ResetTestPeer>(40));
  enqueue(stale); stale.start();
  assert(wait_for([&]{return stale.snapshot().state.phase=="fault";}));
  assert(stale.snapshot().state.epoch==1); stale.stop();
  health.age=1000000000;
  SessionRuntime stopped(1000000,100000000,32,false,command,health,std::make_unique<ResetTestPeer>(10000));
  enqueue(stopped); stopped.start();
  assert(wait_for([&]{return stopped.snapshot().reset_pending;}));
  const auto stop_time=std::chrono::steady_clock::now(); stopped.stop();
  assert(std::chrono::steady_clock::now()-stop_time<std::chrono::seconds(1));
  assert(stopped.snapshot().state.epoch==1 && !stopped.snapshot().state.shutdown_complete);
  SessionRuntime mismatch(1000000,100000000,32,false,command,health,std::make_unique<ResetTestPeer>(10,true));
  enqueue(mismatch); mismatch.start();
  assert(wait_for([&]{return mismatch.snapshot().state.phase=="fault";}));
  assert(mismatch.snapshot().state.epoch==1); mismatch.stop();
  SessionRuntime drift(1000000,100000000,32,false,command,health,std::make_unique<ResetTestPeer>(100));
  enqueue(drift); drift.start();
  assert(wait_for([&]{return drift.snapshot().reset_pending;}));
  SessionEvent moved{8,"feedback","",false};
  ArmFeedback off_home; off_home.sequence=2; off_home.authority=health.authorities[2]; off_home.names=arm_names();
  off_home.positions[0][0]=.001; // Within tolerance is NOT exact Home for rearm.
  moved.feedback=off_home; assert(drift.submit(moved));
  assert(wait_for([&]{return drift.snapshot().state.phase=="fault";}));
  assert(drift.snapshot().state.epoch==1); drift.stop();
}
