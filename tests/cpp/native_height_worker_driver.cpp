#include "../../native/control/worker_reset_client.hpp"
#include "../../native/control/mujoco_endpoint.hpp"
#include <iostream>
using namespace tianji_control;
int main(int argc,char** argv) {
  if(argc<5) return 2;
  try {
    std::vector<std::string> command(argv+1,argv+argc);
    MujocoEndpoint sim(command[2]); ArmPair home={Joints7{1.1,-1.52,-1.52,-1.1,0,0,0},Joints7{-1.1,-1.52,1.52,-1.1,0,0,0}};
    sim.apply(home); auto reference=sim.height_reference();
    auto forward=sim.forward_reference();
    if(!forward || sim.feedback()!=home) throw std::runtime_error("forward reference changed state");
    for(const auto& p:*forward) if(std::abs(p[0]-.7325)>1e-5 || std::abs(p[1]-1.121)>1e-5)
      throw std::runtime_error("unexpected J2=-90 forward TCP");
    if(!reference || sim.feedback()!=home) throw std::runtime_error("reference changed execution state");
    WorkerResetClient client(command,"mapped_palm","pico_ee_mapped_corrected_palm_velocity_qp",20000,true,true);
    std::array<double,14> q; for(int s=0;s<2;++s) for(int j=0;j<7;++j) q[s*7+j]=home[s][j];
    client.reset(q,2); client.configure_height({-.1,-.12});
    WorkerTick tick; tick.id=1; tick.now_ns=1000000000; tick.received_ns=tick.now_ns;
    std::string hex; std::cin>>hex; for(std::size_t i=0;i<hex.size();i+=2) tick.packet.push_back(std::stoul(hex.substr(i,2),nullptr,16));
    auto settle=[&]() {
      auto r=client.step(tick);
      for(int i=0;i<80;++i) {++tick.id;tick.now_ns+=10000;tick.received_ns=tick.now_ns;r=client.step(tick);}
      return r;
    };
    auto result=settle();
    if(!result.height_present || result.height_offsets!=std::array<double,2>{-.1,-.12}) throw std::runtime_error("height not applied");
    bool denied=false; try { client.configure_height({0,0}); } catch(const std::logic_error&) { denied=true; }
    if(!denied) throw std::runtime_error("height changed after tick");
    client.reset(q,3); tick.id=1; tick.now_ns+=5000000; tick.received_ns=tick.now_ns;
    result=settle();
    if(!result.height_present || result.height_offsets!=std::array<double,2>{-.1,-.12}) throw std::runtime_error("height lost across reset");
    const auto baseline=result;
    client.reset(q,4); client.configure_xz({.1,.2},{-.1,-.12});
    tick.id=1; tick.now_ns+=5000000; tick.received_ns=tick.now_ns;
    result=settle();
    if(!result.x_present || result.x_offsets!=std::array<double,2>{.1,.2} || result.wire.size()!=611)
      throw std::runtime_error("XZ result mismatch");
    for(int s=0;s<2;++s) {
      for(int a=0;a<3;++a) if(std::abs(result.arms[s].target_position[a]-baseline.arms[s].target_position[a]-(a==0?(s?.2:.1):0))>1e-10)
        throw std::runtime_error("XZ shifted wrong target axis: "+std::to_string(s)+":"+std::to_string(a)+" baseline="+std::to_string(baseline.arms[s].target_position[a])+" actual="+std::to_string(result.arms[s].target_position[a])+" live="+std::to_string(result.input_live));
      if(result.arms[s].target_quaternion!=baseline.arms[s].target_quaternion) throw std::runtime_error("XZ changed orientation");
    }
    client.reset(q,5); tick.id=1; tick.now_ns+=5000000; tick.received_ns=tick.now_ns;
    result=client.step(tick);
    if(!result.x_present || result.x_offsets!=std::array<double,2>{.1,.2}) throw std::runtime_error("X lost across reset");
    std::cout<<nlohmann::json({{"reference",*reference},{"offsets",result.height_offsets}})<<'\n';
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
