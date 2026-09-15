#include "../../native/control/manus_hdf5.hpp"
#include <iostream>
using namespace tianji_control;
int main(int argc,char** argv) {
  try {
    if(argc!=2) return 2;
    Hdf5StreamClient disk(OwnedDescriptor(std::stoi(argv[1])),2000);
    std::string line;
    while(std::getline(std::cin,line)) {
      const auto row=nlohmann::json::parse(line);
      ManusIngressRecord r;r.line_sequence=row.at("sequence");r.received_ns=row.at("timestamp");
      r.raw_line=row.at("text");
      if(row.contains("points")) {
        r.sample.emplace();r.sample->source={"manus","manus-source","router"};
        auto& s=r.sample->value;s.sequence=1;s.timestamp_ns=r.received_ns;s.generation=1;s.flags=3;
        s.points=row.at("points").get<std::array<double,126>>();
        r.source_sequences={4,5};r.source_timestamps={100,200};
      }
      disk.append(encode_manus_ingress_columns(r,100,"manus-source","test-run"));
    }
    disk.close(true);
  }catch(const std::exception& e){std::cerr<<e.what();return 1;}
}
