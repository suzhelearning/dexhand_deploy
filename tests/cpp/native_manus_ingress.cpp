#include "../../native/control/manus_ingress.hpp"
#include <iostream>
#include <iomanip>
using namespace tianji_control;
int main(int argc,char** argv) {
  try {
    std::size_t lines=0,callbacks=0;
    ManusIngress ingress({"manus","source","offline"},1,"","",
      [&](const ManusIngressRecord& record) {
        ++lines;
        if(record.sample) {
          ++callbacks;
          const auto& v=record.sample->value;
          std::cout<<v.sequence<<' '<<v.timestamp_ns<<' '<<v.generation<<' '<<int(v.flags);
          for(auto s:record.source_sequences)std::cout<<' '<<s;
          for(double p:v.points)std::cout<<' '<<std::setprecision(17)<<p;
          std::cout<<'\n';
        }
        return !(argc>1 && std::string(argv[1])=="reject");
      });
    const bool fragmented=argc>1 && std::string(argv[1])=="fragment";
    std::string data((std::istreambuf_iterator<char>(std::cin)),{});
    if(fragmented) for(char c:data)ingress.feed(&c,1,1000);
    else ingress.feed(data.data(),data.size(),1000);
    ingress.finish();
    std::cerr<<"lines="<<lines<<" callbacks="<<callbacks<<'\n';
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
