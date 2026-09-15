#include "../../native/control/raw_tjvr_hdf5.hpp"
#include <iostream>
using namespace tianji_control;
int main(int argc,char** argv) {
  try {
    if(argc!=2) return 2;
    Hdf5StreamClient disk(OwnedDescriptor(std::stoi(argv[1])),2000);
    std::string line;
    while(std::getline(std::cin,line)) {
      const auto row=nlohmann::json::parse(line);
      SessionRawSnapshot raw;
      raw.sequence=row.at("sequence");raw.received_ns=row.at("received_ns");
      raw.accepted=row.at("accepted");raw.bytes=row.at("bytes").get<std::vector<std::uint8_t>>();
      disk.append(encode_raw_tjvr_columns(raw,row.at("origin_ns"),row.at("receiver"),"offline-run"));
    }
    disk.close(true);return 0;
  } catch(const std::exception& e) {std::cerr<<e.what();return 1;}
}
