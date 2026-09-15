#include "../../native/control/worker_reset_client.hpp"
#include <iostream>
using namespace tianji_control;
int main(int argc,char** argv) {
  if(argc<5) return 2;
  try {
    std::vector<std::string> command;
    for(int i=4;i<argc;++i) command.emplace_back(argv[i]);
    WorkerResetClient client(command,argv[1],argv[2],std::stoi(argv[3]),true,true);
    std::array<double,14> q{};
    std::uint64_t next=1; std::int64_t last_now=0;
    for(std::string line;std::getline(std::cin,line);) {
      if(line=="reset") { client.reset(q,client.epoch()+1); next=1; continue; }
      if(line=="invalid") {
        for(int variant=0;variant<7;++variant) {
          WorkerTick bad; bad.id=next; bad.now_ns=last_now+1;
          if(variant==0) bad.id=next+1;
          if(variant==1) bad.now_ns=last_now;
          if(variant==2) bad.received_ns=1;
          if(variant==3) { bad.packet={0}; bad.received_ns=bad.now_ns+1; }
          if(variant==4) { bad.packet.resize(657); bad.received_ns=bad.now_ns; }
          if(variant==5) bad.generation=1;
          if(variant==6) bad.now_ns=-1;
          try { client.step(bad); throw std::runtime_error("invalid request accepted"); }
          catch(const std::invalid_argument&) {}
        }
        continue;
      }
      std::istringstream in(line);
      std::string magic,hex; unsigned discontinuity;
      WorkerTick tick;
      if(!(in>>magic>>tick.id>>tick.now_ns>>tick.received_ns>>tick.generation>>discontinuity>>hex)) return 2;
      tick.discontinuity=discontinuity!=0;
      if(hex!="-") for(std::size_t i=0;i<hex.size();i+=2)
        tick.packet.push_back(std::stoul(hex.substr(i,2),nullptr,16));
      try {
        const auto result=client.step(tick);
        nlohmann::json out={{"tick",result.tick},{"now",result.timestamp},{"live",result.input_live},
          {"epoch",result.applied_epoch},{"sequence",result.applied_sequence},
          {"control",result.control_executed},{"deterministic",result.deterministic}};
        for(int side=0;side<2;++side) {
          const auto& a=result.arms[side];
          out[side?"right":"left"]={{"q",a.q},{"qdot",a.qdot},{"qddot",a.qddot},
            {"accepted",a.accepted},{"target_position",a.target_position},{"target_quaternion_xyzw",a.target_quaternion},
            {"qp_status",a.qp_status},{"hold_reason",a.hold_reason},{"headroom_scale",a.headroom_scale},
            {"task_scale_position",a.task_scale_position},{"task_scale_orientation",a.task_scale_orientation}};
          if(std::string(argv[1])=="spark") {
            const auto& g=a.guidance; auto& arm=out[side?"right":"left"];
            arm.update(nlohmann::json{{"stage1_q",g.stage1_q},{"ik_q",g.ik_q},{"ik_accepted",g.ik_accepted},
              {"stage1_iterations",g.stage1_iterations},{"stage2_iterations",g.stage2_iterations},{"budget_exhausted",g.budget_exhausted},
              {"feedforward_q",g.feedforward_q},{"feedforward_qdot",g.feedforward_qdot},{"feedforward_qddot",g.feedforward_qddot},
              {"feedforward_state",g.feedforward_state},{"feedforward_target_accepted",g.feedforward_target_accepted},
              {"headroom_state",g.headroom_state},{"settled_hold",g.settled_hold},{"settled_hold_reason",g.settled_hold_reason},
              {"stationary_hold",g.stationary_hold}});
          }
          for(int j=0;j<7;++j) q[side*7+j]=a.q[j];
        }
        std::ostringstream raw; raw<<std::hex<<std::setfill('0');
        for(auto byte:result.wire) raw<<std::setw(2)<<unsigned(byte);
        out["wire"]=raw.str(); std::cout<<out<<'\n';
        next=tick.id+1; last_now=tick.now_ns;
      } catch(const std::exception& e) {
        std::cerr<<e.what()<<'\n';
        try { client.step(tick); }
        catch(const std::exception& second) { std::cerr<<"second: "<<second.what()<<'\n'; }
        return 1;
      }
    }
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
