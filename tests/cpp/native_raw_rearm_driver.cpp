#include "../../native/control/session_runtime.hpp"
#include "../../native/control/tjvr_input.hpp"
#include "../../native/control/worker_reset_client.hpp"
#include <iostream>
using namespace tianji_control;
int main(int argc,char** argv) {
  if(argc<5) return 2;
  try {
    std::vector<std::vector<std::uint8_t>> packets;
    for(std::string hex;std::cin>>hex;) {
      std::vector<std::uint8_t> packet;
      for(std::size_t i=0;i+1<hex.size();i+=2) packet.push_back(std::stoul(hex.substr(i,2),nullptr,16));
      packets.push_back(std::move(packet));
    }
    if(packets.size()!=2 && packets.size()!=3) return 2;
    std::vector<std::string> command;
    for(int i=4;i<argc;++i) command.emplace_back(argv[i]);
    auto peer=std::make_unique<WorkerResetClient>(command,argv[1],argv[2],std::stoi(argv[3]));
    SessionCommandConfig c;
    c.run="offline"; c.producer="ik"; c.instance="worker"; c.router="offline";
    c.home={Joints7{1.1,-1.52,-1.52,-1.1,0,0,0},Joints7{-1.1,-1.52,1.52,-1.1,0,0,0}};
    for(auto& q:c.lower) q.fill(-3.2);
    for(auto& q:c.upper) q.fill(3.2);
    SessionHealthConfig h; h.home=c.home; h.age=20000000000LL;
    h.authorities={Authority{"src","source","offline"},Authority{"ik","worker","offline"},Authority{"sim","executor","offline"}};
    SessionRuntime runtime(1000000,100000000,64,false,c,h,std::move(peer),
      std::make_unique<TjvrInput>(std::string(argv[1])=="mapped_palm",.15,.6),20000000000LL);
    runtime.enable_raw_capture(64);
    for(int role=0;role<3;++role) {
      SessionEvent e{static_cast<unsigned>(role+1),"status","",false};
      HealthStatus s; s.role=role; s.sequence=1; s.authority=h.authorities[role];
      s.ready=s.healthy=s.simulation=true; e.status=s; runtime.submit(e);
    }
    SessionEvent e{4,"feedback","",false};
    ArmFeedback f; f.sequence=1; f.authority=h.authorities[2]; f.names=arm_names(); f.positions=c.home;
    e.feedback=f; runtime.submit(e);
    auto raw=[&](std::uint64_t id,const std::vector<std::uint8_t>& packet) {
      SessionEvent event{id,"raw","",false}; event.raw=RawDatagram{h.authorities[0],packet};
      if(!runtime.submit(event)) throw std::runtime_error("raw queue rejected");
    };
    raw(5,packets[0]); runtime.submit({6,"rearm","",true,2,false}); runtime.start();
    auto await_reply=[&](std::uint64_t id,bool expected) {
      for(int i=0;i<20000;++i) {
        SessionReply r;
        while(runtime.pop(r)) if(r.id==id) {
          if(r.outcome.accepted!=expected) throw std::runtime_error(r.outcome.reason);
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      throw std::runtime_error("reply timeout");
    };
    await_reply(6,true);
    raw(7,packets[0]); await_reply(7,false);
    runtime.submit({8,"start","",true}); await_reply(8,false);
    raw(9,packets[1]); await_reply(9,true);
    if(packets.size()==3) {
      SessionEvent spoof{11,"start","",true};
      HealthStatus s; s.role=0; s.sequence=2; s.authority=h.authorities[0];
      s.ready=s.healthy=s.simulation=true; spoof.status=s;
      runtime.submit(spoof); await_reply(11,true);
      runtime.submit({12,"start","",true}); await_reply(12,false);
      runtime.submit({13,"rearm","",true,3,false}); await_reply(13,true);
      raw(14,packets[1]); await_reply(14,false);
      runtime.submit({15,"start","",true}); await_reply(15,false);
      raw(16,packets[2]); await_reply(16,true);
    }
    runtime.submit({10,"start","",true}); await_reply(10,true);
    if(runtime.snapshot().state.phase!="teleop") throw std::runtime_error("new frame did not release start");
    runtime.stop();
    std::vector<SessionRawSnapshot> recorded;
    SessionRawSnapshot capture;
    while(runtime.pop_raw(capture)) recorded.push_back(std::move(capture));
    const auto expected=packets.size()==2?3U:5U;
    if(recorded.size()!=expected) throw std::runtime_error("decoded packets missing from raw recording");
    for(std::size_t i=0;i<recorded.size();++i) {
      if(recorded[i].sequence!=i+1 || recorded[i].accepted!=(i%2==0))
        throw std::runtime_error("raw recording ordinal/acceptance mismatch");
      const auto source_index=i/2;
      if(recorded[i].bytes!=packets[source_index]) throw std::runtime_error("raw recording bytes changed");
    }
    std::cout<<"raw_barrier_released_by_new_frame\n";
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
