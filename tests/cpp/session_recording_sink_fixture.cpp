#include "../../native/control/session_recording_sink.hpp"
#include <iostream>
using namespace tianji_control;
int main(int argc,char** argv) {
  if(argc!=3) return 2;
  try {
    nlohmann::json input;std::cin>>input;
    if(std::string(argv[2])=="manus_batch") input["manifest"]["hand_runtime"]=nlohmann::json::object();
    SessionRecordingSink sink(OwnedDescriptor(std::stoi(argv[1])),input.at("manifest"),100,1000);
    SessionReply reply{7,{true,"accepted"},"start",101};
    sink.write(reply);
    SessionRawSnapshot raw{1,99,true,input.at("bytes").get<std::vector<std::uint8_t>>()};
    sink.write(raw);
    if(std::string(argv[2])=="manus" || std::string(argv[2])=="manus_batch" || std::string(argv[2])=="missing_manus") {
      ManusIngressRecord line;line.line_sequence=1;line.received_ns=101;line.raw_line="SDK ready";
      if(std::string(argv[2])=="missing_manus") {
        bool rejected=false;
        try{sink.write(line);}catch(const std::invalid_argument&){rejected=true;}
        if(!rejected) return 5;
        try{sink.finish(true,102,"must not commit");return 6;}catch(const std::logic_error&){}
        return 0;
      } else {
        const int count=std::string(argv[2])=="manus_batch"?130:1;
        for(int i=0;i<count;++i) {line.line_sequence=i+1;sink.write(SessionOutputItem(line));}
      }
    }
    const std::string mode=argv[2];
    if(mode=="abandon") return 0;
    if(mode=="cancel") sink.request_stop();
    bool failed=false;
    if(mode=="invalid") {
      reply.action="";
      try {sink.write(reply);}catch(const std::exception&){failed=true;}
      if(!failed) return 3;
    }
    try {
      sink.finish(mode!="incomplete",102,"test finished");
      if(mode=="invalid" || mode=="cancel") return 4;
    }catch(const std::exception&) {
      if(mode!="invalid" && mode!="cancel") throw;
    }
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what();return 1;}
}
