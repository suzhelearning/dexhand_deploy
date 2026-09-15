#include "../../native/control/viewer_state.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
using namespace tianji_control;
int main(int argc,char** argv) {
  try {
    if(argc!=2) return 2;
    ViewerState state(std::string(argv[1])=="mapped_palm");
    {
      ViewerState hands(false);
      SessionCycleSnapshot c;c.snapshot.state.epoch=2;c.snapshot.hand_feedback=HandPair{};
      (*c.snapshot.hand_feedback)[1].fill(.2);
      hands.ingest(c);
      if(hands.snapshot().hand_feedback!=c.snapshot.hand_feedback)
        throw std::runtime_error("viewer lost measured hand feedback without IK result");
      auto old=c;old.snapshot.state.epoch=1;(*old.snapshot.hand_feedback)[1].fill(.9);
      hands.ingest(old);
      if(hands.snapshot().hand_feedback!=c.snapshot.hand_feedback)
        throw std::runtime_error("old epoch replaced hand feedback");
      c.snapshot.hand_feedback.reset();hands.ingest(c);
      if(hands.snapshot().hand_feedback) throw std::runtime_error("viewer retained absent hand feedback");
    }
    std::string line;
    while(std::getline(std::cin,line)) {
      const auto row=nlohmann::json::parse(line);
      if(row.contains("raw")) state.ingest(SessionRawSnapshot{row.at("sequence"),row.at("received_ns"),true,
        row.at("raw").get<std::vector<std::uint8_t>>()});
      if(row.contains("wire")) {
        SessionCycleSnapshot c;c.snapshot.state.epoch=row.at("epoch");
        c.snapshot.state.phase=row.at("active").get<bool>()?"teleop":"idle";
        c.snapshot.feedback=ArmPair{};
        c.result=decode_worker_result(row.at("wire").get<std::vector<std::uint8_t>>(),std::string(argv[1])=="mapped_palm");
        if(row.contains("packet")) {
          c.request.emplace();c.request->packet=row.at("packet").get<std::vector<std::uint8_t>>();
          c.request->received_ns=row.at("received_ns");
        }
        state.ingest(c);
      }
      const auto geometry=state.geometry(row.at("now_ns"));
      nlohmann::json markers=nlohmann::json::array();
      for(const auto& m:geometry.markers) markers.push_back({{"label",m.label},{"position",m.position},
        {"color",m.color},{"rotation",m.rotation?nlohmann::json(*m.rotation):nlohmann::json(nullptr)}});
      std::cout<<nlohmann::json({{"markers",markers},{"bones",geometry.bones}}).dump()<<'\n';
    }
  }catch(const std::exception& e){std::cerr<<e.what();return 1;}
}
