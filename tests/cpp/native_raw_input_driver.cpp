#include "../../native/control/tjvr_input.hpp"
#include <iostream>
int main(int argc,char** argv) {
  tianji_control::TjvrInput input(argc>1 && std::string(argv[1])=="mapped",.15,.6);
  std::string hex;
  while(std::cin>>hex) {
    std::vector<std::uint8_t> bytes;
    for(std::size_t i=0;i+1<hex.size();i+=2) bytes.push_back(std::stoul(hex.substr(i,2),nullptr,16));
    const auto r=input.ingest(bytes);
    if(argc>2) std::cout<<r.accepted<<' '<<r.decoded<<' '<<r.ingress_sequence<<'\n';
    else std::cout<<r.accepted<<' '<<r.epoch<<' '<<r.generation<<'\n';
  }
}
