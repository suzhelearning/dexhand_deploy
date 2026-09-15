#include "../../native/hand/worker_client.hpp"
#include <iostream>
using namespace tianji_hand;
std::uint64_t stamp() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
int main(int argc,char** argv) {
 try {
  std::vector<std::string> command;for(int i=1;i<argc;++i) command.emplace_back(argv[i]);
  NativeHandWorkerClient client(command,5000);
  client.session({1,1,stamp(),HandPhase::idle});
  HandInput input;input.sequence=1;input.timestamp_ns=stamp();input.generation=7;input.flags=2;
  std::size_t k=3;
  for(int finger=0;finger<5;++finger) for(int joint=0;joint<4;++joint) {
    input.points[k++]=.02*(finger-2);input.points[k++]=.02*(joint+1);input.points[k++]=.002*joint;
  }
  client.input(input);const auto first=client.result();
  const HandSession request{2,2,stamp(),HandPhase::idle};
  const auto ack=client.reset(request);
  if(ack.epoch!=2 || ack.sequence!=2 || ack.requested_ns!=request.timestamp_ns ||
     ack.completed_ns<request.timestamp_ns || client.epoch()!=2) throw std::runtime_error("bad actual reset ACK");
  input.sequence=2;input.timestamp_ns=stamp();client.input(input);const auto after=client.result();
  if(after.epoch!=2 || after.input_sequence!=2 || after.positions!=first.positions)
    throw std::runtime_error("native reset did not restore initial pipeline result");
  client.session({2,3,stamp(),HandPhase::teleop});
  input.sequence=3;input.timestamp_ns=stamp();client.input(input);
  if(client.result().status!=HandOutputStatus::command) throw std::runtime_error("missing hand command");
  client.shutdown();
  std::cout<<"native hand child reset and result parity passed\n";
 } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
