#include "../../native/control/datagram_receiver.hpp"
#include "../../native/control/bounded_output.hpp"
#include "../../native/control/session_runtime.hpp"
#include "../../native/control/tjvr_input.hpp"
#include "../../native/control/receipt_wire.hpp"
#include <cassert>
#include <iostream>
#include <map>
using namespace tianji_control;
template<class F> void until(F predicate) {
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(!predicate()) {
    assert(std::chrono::steady_clock::now()<deadline);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
int main(int argc,char** argv) {
  std::vector<std::vector<std::uint8_t>> packets;
  for(std::string hex; std::cin>>hex;) {
    std::vector<std::uint8_t> packet;
    for(std::size_t i=0;i+1<hex.size();i+=2) packet.push_back(std::stoul(hex.substr(i,2),nullptr,16));
    packets.push_back(std::move(packet));
  }
  assert(packets.size()==4);
  SessionCommandConfig config;
  config.run="offline"; config.producer="ik"; config.instance="worker"; config.router="offline";
  // Isolate raw-input freshness. This fixture submits one command rather than
  // a live IK stream; do not race its default 200 ms proposal deadline against
  // the raw gate's 200 ms deadline and assert which fault happens first.
  config.math.proposal_timeout=10.; config.ingress_age=10000000000LL;
  for(auto& q:config.lower) q.fill(-3);
  for(auto& q:config.upper) q.fill(3);
  SessionHealthConfig health; health.home=config.home; health.age=10000000000LL;
  health.authorities={Authority{"source","receiver","offline"},Authority{"ik","worker","offline"},
                      Authority{"sim","executor","offline"}};
  SessionRuntime runtime(1000000,100000000,64,false,config,health,nullptr,
    std::make_unique<TjvrInput>(argc>1 && std::string(argv[1])=="mapped",.15,.6),200000000);
  for(int role=0;role<3;++role) {
    SessionEvent event{static_cast<unsigned>(role+1),"status","",false};
    HealthStatus status; status.role=role; status.sequence=1; status.authority=health.authorities[role];
    status.ready=status.healthy=status.simulation=true; event.status=status; assert(runtime.submit(event));
  }
  ArmFeedback feedback; feedback.authority=health.authorities[2]; feedback.sequence=1;
  feedback.names=arm_names(); feedback.positions=config.home;
  SessionEvent event{4,"feedback","",false}; event.feedback=feedback; assert(runtime.submit(event));
  runtime.start();
  int pair[2]; assert(socketpair(AF_UNIX,SOCK_DGRAM|SOCK_CLOEXEC,0,pair)==0);
  std::uint64_t id=100;
  DatagramReceiver receiver(OwnedDescriptor(pair[0]),[&](ReceivedDatagram frame) {
    SessionEvent raw{id++,"raw","",false};
    raw.raw=RawDatagram{health.authorities[0],std::move(frame.bytes)};
    return runtime.submit(std::move(raw));
  });
  // Side effects have their own thread. Recording/network sinks will reuse this
  // boundary later; this fixture intentionally has no external publisher/HDF5.
  std::vector<nlohmann::json> receipts;
  std::thread::id consumer;
  BoundedOutput<SessionCommandResult> output(32,[&](const SessionCommandResult& command) {
    consumer=std::this_thread::get_id();
    receipts.push_back(encode_bilateral_receipt(command,"offline-coordinator"));
  },[]{},[]{});
  receiver.start();
  std::map<std::uint64_t,IntentOutcome> replies;
  auto drain=[&] {
    SessionReply reply;
    while(runtime.pop(reply)) replies[reply.id]=reply.outcome;
    SessionCommandResult result;
    while(runtime.pop_command_receipt(result)) assert(output.submit(std::move(result)));
  };
  for(const auto& packet:packets) assert(send(pair[1],packet.data(),packet.size(),0)==static_cast<ssize_t>(packet.size()));
  until([&]{drain(); return replies.count(103);});
  assert(replies.at(100).accepted && !replies.at(101).accepted && !replies.at(102).accepted && replies.at(103).accepted);
  assert(runtime.snapshot().state.phase=="idle"); // Reception cannot start motion.
  assert(runtime.submit({200,"start","",true}));
  until([&]{drain(); return replies.count(200);}); assert(replies.at(200).accepted);
  SessionProposal proposal; proposal.run=config.run; proposal.producer=config.producer;
  proposal.instance=config.instance; proposal.router=config.router; proposal.epoch=1; proposal.tick=1;
  proposal.timestamp=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  for(auto& q:proposal.positions) q.fill(.01);
  assert(runtime.submit_proposal(201,proposal));
  until([&]{drain(); return output.stats().processed==1;});
  // Duplicates arrive, but cannot rejuvenate the last accepted raw input.
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(300);
  while(std::chrono::steady_clock::now()<deadline) {
    assert(send(pair[1],packets.back().data(),packets.back().size(),0)==static_cast<ssize_t>(packets.back().size()));
    drain(); std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  assert(runtime.snapshot().state.phase=="fault");
  if(runtime.snapshot().state.reason!="source stale or unhealthy")
    throw std::runtime_error("unexpected fault while repeating raw input: "+runtime.snapshot().state.reason);
  assert(runtime.snapshot().command->positions==proposal.positions);
  assert(runtime.submit({203,"start","",true}));
  until([&]{drain(); return replies.count(203);}); assert(!replies.at(203).accepted);
  receiver.stop(); close(pair[1]); // Stop producers before runtime/output drain.
  runtime.stop(); drain(); output.finish();
  assert(!runtime.snapshot().state.shutdown_complete);
  assert(receiver.stats().failure.empty() && output.stats().complete);
  assert(consumer!=std::this_thread::get_id());
  assert(receipts.size()==1 && receipts[0].at("tick_id")==1 && receipts[0].at("execution_epoch")==1);
  assert(receipts[0].at("command_position_rad").at("left").get<Joints7>()==proposal.positions[0]);
  std::cout<<"native_io_gate_and_receipt_ok\n";
}
