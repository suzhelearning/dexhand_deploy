#include "../../native/hand/worker_client.hpp"
#include <iostream>
using namespace tianji_hand;
std::uint64_t stamp() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
int main(int argc,char** argv) {
  if(argc!=4) return 2;
  try {
    const std::string mode=argv[3];
    NativeHandWorkerClient client({argv[1],argv[2],argv[3]},mode=="timeout"?200:5000);
    client.session({1,1,stamp(),HandPhase::idle});
    const auto begin=std::chrono::steady_clock::now();
    bool rejected=false;
    auto reset=[&] {try {client.reset({2,2,stamp(),HandPhase::idle});} catch(const std::runtime_error&) {rejected=true;}};
    if(mode=="cancel") {
      std::thread task(reset);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));client.request_stop();task.join();
    } else reset();
    if(!rejected || client.epoch()!=1) throw std::runtime_error("failed reset committed epoch");
    if(std::chrono::steady_clock::now()-begin>std::chrono::seconds(2)) throw std::runtime_error("unbounded reset failure/cancellation");
    std::cout<<"failed hand reset stayed in original epoch: "<<mode<<'\n';
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
