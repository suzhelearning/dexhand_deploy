#include "../../native/control/worker_reset_client.hpp"
#include <iostream>
using namespace tianji_control;
int main(int argc,char** argv) {
  try {
    WorkerResetClient worker(std::vector<std::string>(argv+1,argv+argc),"mapped_palm","test",300);
    bool rejected=false;
    try { worker.configure_height({1.1,0}); } catch(const std::invalid_argument&) { rejected=true; }
    if(!rejected) throw std::runtime_error("out-of-range offset accepted");
    try { worker.configure_height({0,.1}); }
    catch(...) {
      bool closed=false;
      try { worker.configure_height({0,.1}); }
      catch(const std::exception& e) { closed=std::string(e.what()).find("closed")!=std::string::npos; }
      if(!closed) throw std::runtime_error("failed height transport reused");
      std::cerr<<"height_transport_closed\n"; return 1;
    }
    std::cout<<"height_ack_checked\n";
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 2; }
}
