#include "../../native/control/session_hand_publication.hpp"
#include <iostream>
using namespace tianji_control;
int main() {
 try {
  for(std::string line;std::getline(std::cin,line);) {
   auto j=nlohmann::json::parse(line);SessionCycleSnapshot c;
   c.timestamp_ns=1000;c.snapshot.ticks=7;c.snapshot.state.epoch=2;c.snapshot.state.phase=j.at("phase");
   c.snapshot.hand_feedback=HandPair{};
   if(j.value("command",true))c.snapshot.hand_command[0]=HandJoints{};
   SessionHandPublication publication(j.at("authorities"));
   std::cout<<publication.encode(c).dump()<<'\n';
  }
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
