#include "../../native/control/native_hand_pipeline.hpp"
#include <fstream>
#include <iostream>
int main(int argc,char** argv) {
 try {
  std::vector<std::string> command;for(int i=1;i<argc;++i) command.emplace_back(argv[i]);
  tianji_control::NativeHandPipeline pipeline(command,2000,2);
  // This test process owns exactly this one child. Do not inspect or signal
  // unrelated device processes; cancellation must reap the owned child itself.
  std::ifstream children("/proc/self/task/"+std::to_string(getpid())+"/children");
  pid_t child=-1;children>>child;
  if(child<=0) throw std::runtime_error("missing owned child");
  pipeline.request_stop();
  const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(std::chrono::steady_clock::now()<until) {
    if(kill(child,0)<0 && errno==ESRCH) {std::cout<<"cancel reaped child before endpoint destruction\n";return 0;}
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  throw std::runtime_error("cancelled pipeline retained child until destruction");
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
