#include "../../native/control/session_hand_domain.hpp"
#include <iostream>
#include <limits>
using namespace tianji_control;
#define CHECK(x) do { if(!(x)) throw std::runtime_error(#x); } while(false)
SessionHandConfig config() {
  SessionHandConfig c; c.run="run";
  c.source={"manus","input","router"}; c.producer={"hand","worker","router"};
  c.age=100; c.capacity=4;
  for(auto& q:c.lower) q.fill(-1.);
  for(auto& q:c.upper) q.fill(1.);
  for(auto& q:c.zero_tolerance) q.fill(.01);
  return c;
}
HandInputReceipt input(std::uint64_t seq,std::int64_t time) {
  return {config().source,seq,static_cast<std::uint64_t>(time),7,3};
}
HandResultEnvelope result(std::uint64_t seq,std::int64_t time,std::uint64_t out,const char* phase="idle") {
  HandResultEnvelope r; r.run="run";r.producer=config().producer;
  r.value.input_sequence=seq;r.value.input_timestamp_ns=time;r.value.output_sequence=out;
  r.value.scheduler_timestamp_ns=time+1;r.value.valid_flags=3;
  r.value.phase=std::string(phase)=="idle"?tianji_hand::HandPhase::idle:tianji_hand::HandPhase::teleop;
  r.value.status=std::string(phase)=="idle"?tianji_hand::HandOutputStatus::processed:tianji_hand::HandOutputStatus::command;
  r.value.positions.fill(.3);return r;
}
int main() {
 try {
  SessionHandDomain d(config());SessionState s;
  d.sync(s,10);d.feedback(10,HandPair{});
  CHECK(!d.facts(10).hand_producer_ready);CHECK(d.facts(10).hand_zero);
  CHECK(d.input(10,input(1,10)).accepted);
  CHECK(d.result(12,result(1,10,1)).accepted);
  CHECK(d.facts(12).hand_producer_ready);CHECK(!d.take(12)[0]);
  // An idle result arriving after start is discarded, not a teleop command.
  CHECK(d.input(13,input(2,13)).accepted);
  CHECK(d.facts(13).hand_producer_ready); // A newer waiting frame must not starve admission.
  CHECK(!d.facts(36).hand_producer_ready); // Still reject backed-up idle results.
  s.phase="teleop";d.sync(s,14);
  CHECK(!d.result(15,result(2,13,2)).accepted);CHECK(d.error().empty());
  CHECK(!d.take(15)[0]);
  CHECK(d.input(16,input(3,16)).accepted);
  CHECK(d.result(18,result(3,16,3,"teleop")).accepted);
  auto q=d.take(18);CHECK(q[0] && (*q[0])[0]==.3 && q[1]);CHECK(!d.take(19)[0]);
  d.feedback(19,HandPair{*q[0],*q[1]});CHECK(!d.facts(19).hand_zero);
  CHECK(d.input(20,input(4,20)).accepted);CHECK(d.result(22,result(4,20,4,"teleop")).accepted);
  s.phase="returning";d.sync(s,23);auto zero=d.take(23);CHECK(zero[0] && (*zero[0])[0]==0.);
  s.phase="fault";d.sync(s,24);CHECK(!d.take(24)[0]);
  // Epoch fences require a newly admitted input; cached readiness cannot survive.
  SessionHandDomain epoch(config());s={};epoch.sync(s,10);epoch.feedback(10,HandPair{});
  CHECK(epoch.input(10,input(1,10)).accepted);CHECK(epoch.result(12,result(1,10,1)).accepted);
  s.epoch=2;epoch.sync(s,20);CHECK(!epoch.facts(20).hand_producer_ready);
  CHECK(!epoch.input(21,input(2,19)).accepted);CHECK(epoch.error().empty());
  auto delayed=result(2,19,2);delayed.value.epoch=2;
  CHECK(!epoch.result(21,delayed).accepted);CHECK(epoch.error().empty());
  CHECK(epoch.input(22,input(3,22)).accepted);
  auto next=result(3,22,3);next.value.epoch=2;CHECK(epoch.result(24,next).accepted);
  CHECK(epoch.facts(24).hand_producer_ready);
  CHECK(!epoch.facts(123).hand_producer_ready);
  // Pending results are rechecked at execution, not rejuvenated by polling.
  SessionHandDomain aged(config());s={};s.phase="teleop";aged.sync(s,10);
  CHECK(aged.input(10,input(1,10)).accepted);CHECK(aged.result(12,result(1,10,1,"teleop")).accepted);
  CHECK(!aged.take(111)[0]);
  // Scheduler can finish an old admitted input in the new phase. Drop it
  // without treating this expected transition race as an unknown producer.
  SessionHandDomain crossing(config());s={};crossing.sync(s,10);
  CHECK(crossing.input(10,input(1,10)).accepted);s.phase="teleop";crossing.sync(s,11);
  CHECK(!crossing.result(12,result(1,10,1,"teleop")).accepted);
  CHECK(crossing.error().empty());CHECK(!crossing.take(12)[0]);
  SessionHandDomain phase(config());phase.sync(s,10);CHECK(phase.input(10,input(1,10)).accepted);
  auto wrong_phase=result(1,10,1,"teleop");wrong_phase.value.phase=static_cast<tianji_hand::HandPhase>(255);
  CHECK(!phase.result(12,wrong_phase).accepted);CHECK(!phase.error().empty());
  // Invalid numeric data faults before either side is eligible for execution.
  SessionHandDomain bad(config());bad.sync(s,10);CHECK(bad.input(10,input(1,10)).accepted);
  auto invalid=result(1,10,1,"teleop");invalid.value.positions[39]=1.1;
  CHECK(!bad.result(12,invalid).accepted);CHECK(!bad.error().empty());CHECK(!bad.take(12)[0]);
  SessionHandDomain forged(config());forged.sync(s,10);CHECK(forged.input(10,input(1,10)).accepted);
  invalid=result(1,10,1,"teleop");invalid.producer.instance="other";
  CHECK(!forged.result(12,invalid).accepted);CHECK(!forged.error().empty());
  SessionHandDomain inactive(config());inactive.sync(s,10);CHECK(inactive.input(10,input(1,10)).accepted);
  invalid=result(1,10,1,"teleop");invalid.value.valid_flags=1;
  invalid.value.positions[39]=std::numeric_limits<double>::quiet_NaN();
  CHECK(!inactive.result(12,invalid).accepted);CHECK(!inactive.error().empty());
  SessionHandDomain overflow(config());overflow.sync(s,10);
  for(int i=1;i<=4;++i) CHECK(overflow.input(10+i,input(i,10+i)).accepted);
  CHECK(!overflow.input(15,input(5,15)).accepted);CHECK(!overflow.error().empty());
  std::cout<<"native hand admission gates passed\n";
 } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
