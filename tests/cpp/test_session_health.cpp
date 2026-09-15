#include "../../native/control/session_health.hpp"
#include <cassert>
using namespace tianji_control;
int main() {
  SessionHealthConfig config;
  config.authorities={Authority{"src","source","router"},Authority{"ik","worker","router"},
                      Authority{"sim","executor","router"}};
  config.age=100; config.home_tolerance=.01;
  SessionHealth health(config);
  for(int role=0;role<3;++role) {
    HealthStatus status;
    status.role=role; status.authority=config.authorities[role]; status.sequence=1;
    status.ready=status.healthy=status.simulation=true;
    assert(health.status(100,status).accepted);
  }
  ArmFeedback feedback;
  feedback.authority=config.authorities[2]; feedback.sequence=1; feedback.names=arm_names();
  assert(health.feedback(100,feedback).accepted);
  auto facts=health.facts(200);
  assert(facts.source_ready && facts.producer_ready && facts.executor_ready && facts.arm_home);
  assert(facts.arm_exact_home && !health.facts(201).arm_fresh);
  // A fresh status cannot rejuvenate old measured feedback.
  HealthStatus fresh;
  fresh.role=2; fresh.authority=config.authorities[2]; fresh.sequence=2;
  fresh.ready=fresh.healthy=fresh.simulation=true;
  assert(health.status(201,fresh).accepted);
  assert(health.facts(201).executor_ready && !health.facts(201).arm_home);
  feedback.sequence=2; feedback.positions[0][0]=.005;
  assert(health.feedback(201,feedback).accepted);
  assert(health.facts(201).arm_home && !health.facts(201).arm_exact_home);
  assert(!health.feedback(202,feedback).accepted);
  assert(health.error()=="arm state sequence rollback");
  SessionHealth order(config);
  std::swap(feedback.names[0],feedback.names[1]);
  assert(!order.feedback(100,feedback).accepted);
  assert(order.error()=="arm state identity/order mismatch");
  SessionHealth observer(config);
  fresh.role=0; fresh.authority={"observer","other","router"}; fresh.observation_only=true;
  assert(observer.status(100,fresh).accepted);
  assert(!observer.facts(100).source_ready && observer.error().empty());
  fresh.observation_only=false;
  assert(!observer.status(101,fresh).accepted);
  SessionHealth rollback(config);
  feedback.names=arm_names();
  assert(rollback.feedback(200,feedback).accepted);
  assert(!rollback.feedback(199,feedback).accepted);
  assert(rollback.error()=="monotonic clock rollback");
}
