#include "../../native/control/worker_reset_client.hpp"
#include <iostream>
int main(int argc,char** argv) {
  if(argc<5) return 2;
  try {
    std::vector<std::string> command;
    for(int i=4;i<argc;++i) command.emplace_back(argv[i]);
    tianji_control::WorkerResetClient client(command,argv[1],argv[2],std::stoi(argv[3]));
    std::array<double,14> home={1.1,-1.52,-1.52,-1.1,0,0,0,-1.1,-1.52,1.52,-1.1,0,0,0};
    try { client.reset(home,2); }
    catch(...) {
      if(client.epoch()!=1) throw std::runtime_error("failed reset advanced epoch");
      bool closed=false;
      try { client.reset(home,2); }
      catch(const std::exception& e) { closed=std::string(e.what()).find("closed")!=std::string::npos; }
      if(!closed) throw std::runtime_error("failed transport was reused");
      std::cerr<<"failure_epoch=1; transport_closed\n";
      throw;
    }
    client.reset(home,3);
    std::cout<<"reset_epoch="<<client.epoch()<<'\n';
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
