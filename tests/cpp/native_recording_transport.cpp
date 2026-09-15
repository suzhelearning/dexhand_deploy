#include "../../native/control/hdf5_stream_client.hpp"
#include "../../native/control/arm_cycle_hdf5.hpp"
#include <cassert>
#include <iostream>
using namespace tianji_control;
int main(int argc,char** argv) {
  if(argc==2) {
    Hdf5Block block;
    block.integers("/time",1,{1});
    const auto original=block.payload();
    for(int mode=0;mode<5;++mode) {
      bool rejected=false;
      try {
        if(mode==0) block.integers("/time",1,{2});
        if(mode==1) block.integers("relative",1,{2});
        if(mode==2) block.strings("/s",{std::string("a\0b",3)});
        if(mode==3) block.integers("/x",0,{});
        if(mode==4) block.attribute("/meta","complete","1");
      } catch(const std::invalid_argument&) {rejected=true;}
      assert(rejected && block.payload()==original);
    }
    Hdf5Block numbers;
    numbers.reals("/matrix",2,{1.,2.,3.,4.});
    numbers.octets("/valid",2,{0,1});
    numbers.bytes("/raw",{{1,2,3},{}});
    numbers.attribute("/meta","logical_id","test");
    assert(!numbers.payload().empty());
    SessionCycleSnapshot cycle;cycle.timestamp_ns=100;cycle.snapshot.command=SessionCommandResult{};
    cycle.snapshot.feedback=ArmPair{};
    ArmCycleRecordMetadata meta;meta.time_ns=0;meta.feedback_sequence=0;meta.command_sequences={0,0};
    meta.command_instance=meta.command_producer=meta.executor_instance=meta.executor=meta.session_instance=meta.session_source="test";
    meta.command_mode="idle";
    const auto valid=encode_arm_cycle_columns(cycle,meta).payload();
    for(int mode=0;mode<10;++mode) {
      auto bad=cycle;auto fields=meta;bool rejected=false;
      if(mode==0) bad.snapshot.feedback.reset();
      if(mode==1) bad.snapshot.command.reset();
      if(mode==2) bad.snapshot.command->positions[1][6]=std::numeric_limits<double>::infinity();
      if(mode==3) (*bad.snapshot.feedback)[0][0]=std::numeric_limits<double>::quiet_NaN();
      if(mode==4) fields.command_instance="";
      if(mode==5) fields.command_sequences[1]=-1;
      if(mode==6) fields.proposal_sequences[1]=-1;
      if(mode==7) fields.command_mode="fault";
      if(mode==8) fields.time_ns=101;
      if(mode==9) fields.command_instance=std::string(1,static_cast<char>(0xff));
      try {encode_arm_cycle_columns(bad,fields);}catch(const std::invalid_argument&){rejected=true;}
      assert(rejected);
    }
    assert(encode_arm_cycle_columns(cycle,meta).payload()==valid);
    return 0;
  }
  if(argc!=3) return 2;
  const std::string mode=argv[2];
  const bool success=mode=="complete" || mode=="incomplete" || mode=="numeric" || mode=="arm_cycle";
  try {
    Hdf5StreamClient client(OwnedDescriptor(std::stoi(argv[1])),success?2000:100);
    if(mode=="arm_cycle") {
      for(int i=0;i<2;++i) {
        SessionCycleSnapshot cycle;cycle.timestamp_ns=100+i;cycle.snapshot.ticks=i+1;
        cycle.snapshot.command=SessionCommandResult{};cycle.snapshot.feedback=ArmPair{};
        cycle.snapshot.state.phase=i?"returning":"teleop";cycle.snapshot.state.reason="记录✓";
        for(int s=0;s<2;++s) {
          cycle.snapshot.command->positions[s].fill((s?-1.:1.)+i);
          (*cycle.snapshot.feedback)[s].fill(.5+i);
        }
        ArmCycleRecordMetadata metadata;
        metadata.time_ns=i;metadata.command_instance="command";metadata.command_producer="arm";
        metadata.executor_instance="feedback";metadata.executor="mujoco";
        metadata.session_instance="session";metadata.session_source="keyboard";
        metadata.command_mode=cycle.snapshot.state.phase;
        metadata.command_sequences={3+i,4+i};metadata.feedback_sequence=5+i;
        metadata.proposal_sequences[0]=2;metadata.target_sequences[0]=1;
        if(!i) metadata.intent_sequence=3;
        client.append(encode_arm_cycle_columns(cycle,metadata));
      }
      client.close(true);return 0;
    }
    if(mode=="numeric") {
      Hdf5Block block;
      block.integers("/i",2,{-1,std::numeric_limits<std::int64_t>::min()});
      block.reals("/f",2,{1.25,-2.5,std::numeric_limits<double>::quiet_NaN(),0.});
      block.octets("/u",2,{0,255});
      block.bytes("/v",{{},{1,255}});
      block.attribute("/","logical_id","记录✓");
      client.append(block);client.close(true);return 0;
    }
    if(!success) {
      std::thread cancel;
      if(mode=="cancel") cancel=std::thread([&]{std::this_thread::sleep_for(std::chrono::milliseconds(10));client.request_stop();});
      bool failed=false;
      try {
        if(mode=="invalid_column") {
          Hdf5Block block;block.integers("/missing",1,{1});client.append(block);
        } else if(mode=="write_timeout") {
          Hdf5Block block;block.octets("/large",1,std::vector<std::uint8_t>(4*1024*1024));client.append(block);
        } else client.flush();
      } catch(const std::exception&) {failed=true;}
      if(cancel.joinable()) cancel.join();
      assert(failed && client.failed());
      try {client.close(true); assert(false);} catch(const std::exception&) {}
      return 0;
    }
    Hdf5Block block;
    block.integers("/meta/dual_audit/time_ns",2,{0,1});
    block.integers("/meta/dual_audit/received_timestamp_ns",2,{100,101});
    block.strings("/meta/dual_audit/kind",{"lifecycle","lifecycle"});
    block.strings("/meta/dual_audit/payload_json",{"{\"text\":\"记录✓\"}","{\"text\":\"记录✓\"}"});
    std::vector<std::vector<std::uint8_t>> packets;
    for(std::string hex;std::cin>>hex;) {
      std::vector<std::uint8_t> bytes;
      for(std::size_t i=0;i<hex.size();i+=2) bytes.push_back(std::stoul(hex.substr(i,2),nullptr,16));
      packets.push_back(std::move(bytes));
    }
    assert(packets.size()==2);
    block.integers("/raw/tjvr_upper_limb/time_ns",2,{2,3});
    block.integers("/raw/tjvr_upper_limb/received_timestamp_ns",2,{102,103});
    block.strings("/raw/tjvr_upper_limb/receiver_instance_id",{"offline-source","offline-source"});
    block.integers("/raw/tjvr_upper_limb/receiver_frame_sequence",2,{1,2});
    block.bytes("/raw/tjvr_upper_limb/raw_packet",packets);
    client.append(block); client.flush(); client.close(mode=="complete");
  } catch(const std::exception& e) {
    if(mode=="bad_ready") return 0;
    std::cerr<<e.what()<<'\n'; return 1;
  }
}
