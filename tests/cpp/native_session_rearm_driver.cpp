#include "../../native/control/session_runtime.hpp"
#include "../../native/control/worker_reset_client.hpp"
#include <iostream>
using namespace tianji_control;
int main(int argc,char** argv) {
  if(argc<5) return 2;
  try {
    std::vector<std::string> command;
    for(int i=4;i<argc;++i) command.emplace_back(argv[i]);
    auto peer=std::make_unique<WorkerResetClient>(command,argv[1],argv[2],std::stoi(argv[3]));
    SessionCommandConfig c;
    c.run="offline"; c.producer="ik"; c.instance="worker"; c.router="offline";
    c.home={Joints7{1.1,-1.52,-1.52,-1.1,0,0,0},Joints7{-1.1,-1.52,1.52,-1.1,0,0,0}};
    for(auto& q:c.lower) q.fill(-3.2);
    for(auto& q:c.upper) q.fill(3.2);
    SessionHealthConfig h;
    h.home=c.home; h.age=20000000000LL; // Offline model-reset check, not live acceptance.
    h.authorities={Authority{"src","source","offline"},Authority{"ik","worker","offline"},Authority{"sim","executor","offline"}};
    SessionRuntime runtime(1000000,100000000,32,false,c,h,std::move(peer));
    for(int role=0;role<3;++role) {
      SessionEvent e{static_cast<unsigned>(role+1),"status","",false};
      HealthStatus s; s.role=role; s.sequence=1; s.authority=h.authorities[role];
      s.ready=s.healthy=s.simulation=true; e.status=s; runtime.submit(e);
    }
    SessionEvent e{4,"feedback","",false};
    ArmFeedback feedback; feedback.sequence=1; feedback.authority=h.authorities[2]; feedback.names=arm_names(); feedback.positions=c.home;
    e.feedback=feedback; runtime.submit(e);
    runtime.submit({5,"rearm","",true,2,false}); runtime.start();
    bool accepted=false;
    for(int i=0;i<20000;++i) {
      SessionReply reply;
      while(runtime.pop(reply)) if(reply.id==5) {
        if(!reply.outcome.accepted) throw std::runtime_error(reply.outcome.reason);
        accepted=true;
      }
      if(accepted || runtime.snapshot().state.phase=="fault") break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if(!accepted || runtime.snapshot().state.epoch!=2) throw std::runtime_error("session reset did not commit");
    runtime.submit({6,"start","",true});
    bool denied=false;
    for(int i=0;i<1000 && !denied;++i) {
      SessionReply reply;
      while(runtime.pop(reply)) if(reply.id==6) {
        if(reply.outcome.accepted) throw std::runtime_error("start bypassed new input barrier");
        denied=true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if(!denied) throw std::runtime_error("missing start disposition");
    runtime.stop();
    std::cout<<"session_epoch=2; new_input_required\n";
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
