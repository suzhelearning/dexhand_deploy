#include "../../native/control/session_health.hpp"
#include <iostream>
using namespace tianji_control;
int main() {
  SessionHealthConfig config;
  config.authorities={Authority{"src","src-instance","router-1"},Authority{"ik","ik-instance","router-1"},
                      Authority{"mujoco","mujoco-instance","router-1"}};
  for(auto& q:config.home) for(double& x:q) if(!(std::cin>>x)) return 2;
  SessionHealth health(config);
  std::string op;
  std::int64_t now, sequence;
  while(std::cin>>op>>now>>sequence) {
    if(op=="status") {
      HealthStatus status; int role,ready;
      std::cin>>role>>ready; status.role=role; status.sequence=sequence;
      status.authority=config.authorities.at(role);
      status.ready=ready; status.healthy=status.simulation=true;
      health.status(now,status);
    } else if(op=="feedback") {
      ArmFeedback feedback; double delta;
      std::cin>>delta; feedback.authority=config.authorities[2]; feedback.sequence=sequence;
      feedback.names=arm_names(); feedback.positions=config.home; feedback.positions[0][0]+=delta;
      health.feedback(now,feedback);
    }
    auto f=health.facts(now);
    std::cout<<f.source_ready<<' '<<f.producer_ready<<' '<<f.executor_ready<<' '
             <<f.arm_fresh<<' '<<f.arm_home<<' '<<f.arm_exact_home<<'\n';
  }
}
