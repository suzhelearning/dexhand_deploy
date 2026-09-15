#include "../../native/control/manus_receiver.hpp"
#include <iostream>
using namespace tianji_control;
int main(int argc,char** argv) {
  try {
    if(argc<2)return 2;
    int fds[2];if(pipe(fds))return 2;
    OwnedDescriptor writer(fds[1]);
    std::atomic<unsigned> records{0},samples{0};
    ManusReceiver receiver(OwnedDescriptor(fds[0]),{"manus","source","offline"},1,"","",
      [&](const ManusIngressRecord& record){++records;if(record.sample)++samples;return true;});
    receiver.start();
    const std::string mode=argv[1];
    if(mode=="idle") {
      const auto before=std::chrono::steady_clock::now();receiver.stop();
      if(std::chrono::steady_clock::now()-before>std::chrono::seconds(1))throw std::runtime_error("stop blocked");
      if(!receiver.failure().empty())throw std::runtime_error("intentional stop is not failure");
    }else {
      std::string text((std::istreambuf_iterator<char>(std::cin)),{});
      for(char c:text)if(write(writer.get(),&c,1)!=1)throw std::runtime_error("fixture write failed");
      writer.reset();
      const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(3);
      while(receiver.failure().empty() && std::chrono::steady_clock::now()<until)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      receiver.stop();
      if(receiver.failure().find(mode=="partial"?"truncated":"EOF")==std::string::npos)
        throw std::runtime_error("unexpected failure: "+receiver.failure());
      if(mode!="partial" && !samples)throw std::runtime_error("no parsed callbacks");
    }
    std::cout<<records<<' '<<samples<<'\n';
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
