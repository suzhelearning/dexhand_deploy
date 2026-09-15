// Offline deterministic integration fixture, not a production session/authority.
#include "../../native/control/owned_mujoco.hpp"
#include "../../native/control/session_commands.hpp"
#include "../../native/control/execution_guard.hpp"
#include "../../native/control/worker_reset_client.hpp"
#include <iostream>
using namespace tianji_control;
int main(int argc,char** argv) {
  if(argc<8) return 2;
  try {
    std::vector<std::string> command;
    for(int i=4;i<argc;++i) command.emplace_back(argv[i]);
    WorkerResetClient worker(command,argv[1],argv[2],std::stoi(argv[3]),true,true);
    std::string line; std::getline(std::cin,line); auto config=nlohmann::json::parse(line);
    OwnedMujoco sim(command[2],config.at("groups").get<std::vector<std::vector<std::string>>>());
    SessionCommandConfig c; c.run=c.router="offline"; c.producer="ik"; c.instance="worker";
    c.home=config.at("home").get<ArmPair>(); c.lower=config.at("lower").get<ArmPair>(); c.upper=config.at("upper").get<ArmPair>();
    c.math.maximum_step=10.; c.math.time_window=0.; c.home_duration=.05; c.home_speed=100.;
    SessionMachine machine(1000000000,2000000000,false); SessionCommands coordinator(machine,c);
    ExecutionGuard guard(100000000,1);
    auto apply=[&](const ArmPair& q) {
      OwnedMujoco::Batch batch;
      for(const auto& side:q) batch.emplace_back(std::vector<double>(side.begin(),side.end()));
      sim.apply(batch);
    };
    auto facts=[&] {
      SessionFacts f; f.source_ready=f.producer_ready=f.executor_ready=f.arm_fresh=true;
      auto feedback=sim.snapshot(); f.arm_home=f.arm_exact_home=true;
      for(int s=0;s<2;++s) for(int j=0;j<7;++j)
        if(feedback.groups[s][j]!=c.home[s][j]) f.arm_home=f.arm_exact_home=false;
      return f;
    };
    apply(c.home);
    std::int64_t now=1000000000;
    if(!coordinator.intent(now,"start","",true,facts()).accepted) throw std::runtime_error("start rejected");
    while(std::getline(std::cin,line)) {
      std::istringstream in(line); std::string magic,hex; unsigned discontinuity; WorkerTick tick;
      if(!(in>>magic>>tick.id>>tick.now_ns>>tick.received_ns>>tick.generation>>discontinuity>>hex)) return 2;
      tick.discontinuity=discontinuity!=0;
      if(hex!="-") for(std::size_t i=0;i<hex.size();i+=2) tick.packet.push_back(std::stoul(hex.substr(i,2),nullptr,16));
      now=tick.now_ns; auto result=worker.step(tick);
      SessionProposal proposal; proposal.run=c.run; proposal.router=c.router; proposal.producer=c.producer; proposal.instance=c.instance;
      proposal.epoch=machine.state().epoch; proposal.tick=tick.id; proposal.timestamp=now;
      Positions expected{};
      for(int s=0;s<2;++s) for(int j=0;j<7;++j) expected[s*7+j]=proposal.positions[s][j]=result.arms[s].q[j];
      guard.register_tick(tick.id,now,expected);
      if(!coordinator.accept(now,proposal)) throw std::runtime_error("proposal rejected");
      const auto final=coordinator.tick(now,facts()); Positions sent{};
      for(int s=0;s<2;++s) for(int j=0;j<7;++j) sent[s*7+j]=final.positions[s][j];
      if(!final.receipt || !guard.observe(final.receipt->tick,final.receipt->accepted,final.receipt->reason,sent))
        throw std::runtime_error("execution receipt rejected");
      apply(final.positions);
      auto state=sim.snapshot();
      if(state.groups[0]!=std::vector<double>(expected.begin(),expected.begin()+7) ||
         state.groups[1]!=std::vector<double>(expected.begin()+7,expected.end())) throw std::runtime_error("feedback mismatch");
      std::cout<<nlohmann::json{{"q",state.groups},{"state",machine.state().phase}}<<'\n';
    }
    if(!coordinator.intent(now,"shutdown","",true,facts()).accepted) throw std::runtime_error("shutdown rejected");
    for(int i=0;i<100 && !machine.state().shutdown_complete;++i) {
      now+=5000000; apply(coordinator.tick(now,facts()).positions);
    }
    if(!machine.state().shutdown_complete || !facts().arm_exact_home || guard.in_flight()) throw std::runtime_error("Home incomplete");
    std::cout<<nlohmann::json{{"home",true},{"state",machine.state().phase}}<<'\n';
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
