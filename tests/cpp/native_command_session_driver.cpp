#include "../../native/control/session_commands.hpp"
#include "../../native/control/receipt_wire.hpp"
#include <iomanip>
#include <iostream>
using namespace tianji_control;
int main(int argc,char** argv) {
  SessionCommandConfig config;
  config.run="run-1"; config.producer="ik"; config.instance="ik-instance"; config.router="router-1";
  config.clipping=argc>1 && std::string(argv[1])=="clip";
  for(auto* pair:{&config.home,&config.lower,&config.upper})
    for(auto& side:*pair) for(auto& x:side) if(!(std::cin>>x)) return 2;
  SessionMachine machine(1000000000,2000000000,false);
  SessionCommands commands(machine,config);
  SessionFacts facts;
  facts.source_ready=facts.producer_ready=facts.executor_ready=facts.arm_fresh=true;
  facts.arm_home=facts.arm_exact_home=true;
  std::string op;
  std::int64_t now;
  while(std::cin>>op>>now) {
    bool accepted=false;
    std::optional<CommandReceipt> receipt;
    if(op=="proposal") {
      SessionProposal p;
      p.run=config.run; p.producer=config.producer; p.instance=config.instance; p.router=config.router;
      p.epoch=1;
      std::cin>>p.tick>>p.timestamp;
      for(auto& q:p.positions) for(auto& x:q) std::cin>>x;
      accepted=commands.accept(now,p);
    } else if(op=="tick") {
      auto result=commands.tick(now,facts); receipt=result.receipt;
      if(receipt) std::cerr<<encode_bilateral_receipt(result,"co").dump()<<'\n';
    }
    else accepted=commands.intent(now,op,op,true,facts).accepted;
    std::cout<<accepted<<' '<<machine.state().phase<<' '<<std::quoted(machine.state().reason)<<' '
             <<(receipt?(receipt->accepted?1:0):-1);
    for(const auto& q:commands.positions()) for(double x:q) std::cout<<' '<<std::setprecision(17)<<x;
    std::cout<<'\n';
  }
}
