#include "../../native/control/session_commands.hpp"
#include <cassert>
using namespace tianji_control;
int main() {
  SessionMachine machine(1000000000,2000000000,false);
  SessionCommandConfig config;
  config.run="run"; config.producer="ik"; config.instance="worker"; config.router="router";
  for (auto& q:config.lower) q.fill(-2);
  for (auto& q:config.upper) q.fill(2);
  SessionCommands commands(machine,config);
  SessionFacts facts;
  facts.source_ready=facts.producer_ready=facts.executor_ready=facts.arm_fresh=true;
  facts.arm_home=facts.arm_exact_home=true;
  // Caller-supplied command_home is deliberately false: only owner knows it.
  assert(commands.intent(1000000000,"start","",true,facts).accepted);
  SessionProposal proposal;
  proposal.run="run"; proposal.producer="ik"; proposal.instance="worker"; proposal.router="router";
  proposal.tick=1; proposal.epoch=1; proposal.timestamp=1000000000;
  for(auto& q:proposal.positions) q.fill(.01);
  assert(commands.accept(1000000000,proposal));
  auto result=commands.tick(1000000000,facts);
  assert(result.positions==proposal.positions && result.receipt && result.receipt->accepted);
  assert(result.receipt->epoch==1 && result.receipt->timestamp==1000000000 && result.receipt->run=="run" && result.receipt->router=="router");
  assert(!commands.accept(1000000000,proposal)); // old tick ignored
  proposal.tick=2; proposal.positions[1][0]=99;
  assert(commands.accept(1005000000,proposal));
  auto bad=commands.tick(1005000000,facts);
  assert(machine.state().phase=="fault" && bad.positions==result.positions);
  assert(bad.receipt && !bad.receipt->accepted);
  assert(!commands.intent(1006000000,"start","",true,facts).accepted);
  SessionMachine other(1000000000,2000000000,false);
  SessionCommands pending(other,config);
  pending.intent(1000000000,"start","",true,facts);
  proposal.positions=result.positions; proposal.tick=1;
  assert(pending.accept(1000000000,proposal));
  proposal.tick=2;
  assert(!pending.accept(1000000000,proposal));
  assert(other.state().phase=="fault");
  SessionMachine timed(1000000000,2000000000,false);
  SessionCommands time_owner(timed,config);
  assert(time_owner.intent(2000000000,"start","",true,facts).accepted);
  proposal.tick=1; proposal.timestamp=1999999999;
  assert(!time_owner.accept(1999999999,proposal));
  assert(timed.state().phase=="fault");
  SessionMachine epochs(1000000000,2000000000,false);
  SessionCommands restarted(epochs,config);
  assert(restarted.rearm(3000000000,2,facts,true).accepted);
  facts.input_revision=1;
  assert(restarted.intent(3000000001,"start","",true,facts).accepted);
  proposal.epoch=2; proposal.tick=1; proposal.timestamp=3000000001; proposal.positions=config.home;
  assert(restarted.accept(3000000001,proposal));
  auto second=restarted.tick(3000000002,facts);
  assert(second.receipt && second.receipt->accepted && second.receipt->tick==1 && second.receipt->epoch==2 && second.receipt->timestamp==3000000002);
}
