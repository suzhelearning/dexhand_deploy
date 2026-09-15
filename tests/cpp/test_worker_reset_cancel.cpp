#include "../../native/control/worker_reset_client.hpp"
#include <future>
#include <thread>
#include <iostream>
int main(int argc,char** argv) {
  try {
    std::vector<std::string> command;
    const bool step=argc>1 && std::string(argv[1])=="--step";
    for(int i=step?2:1;i<argc;++i) command.emplace_back(argv[i]);
    tianji_control::WorkerResetClient client(command,"mapped_palm","test",5000,step);
    auto result=std::async(std::launch::async,[&] {
      try {
        if(step) { tianji_control::WorkerTick tick; tick.id=1; tick.now_ns=1; client.step(tick); }
        else client.reset({},2);
        return false;
      }
      catch(const std::exception&) { return true; }
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    client.request_stop();
    if(result.wait_for(std::chrono::seconds(1))!=std::future_status::ready || !result.get() || client.epoch()!=1)
      return 1;
    std::cout<<"cancelled_without_epoch_commit\n";
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
