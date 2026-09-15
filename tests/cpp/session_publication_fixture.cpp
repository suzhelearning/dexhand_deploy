#include "../../native/control/session_publication.hpp"
#include <iostream>
#include <chrono>
using namespace tianji_control;
int main(int argc,char**) {
  try {
    std::vector<std::int64_t> elapsed;
    std::string line;
    while(std::getline(std::cin,line)) {
      const auto input=nlohmann::json::parse(line);
      const auto& row=input.at("cycle");
      SessionCycleSnapshot cycle;
      cycle.timestamp_ns=row.at("timestamp_ns");
      cycle.source_revision=row.at("source").value("revision",0ULL);
      cycle.source.sequence=row.at("source").value("sequence",0ULL);
      cycle.source.accepted=row.at("source").value("accepted",false);
      cycle.source.skeleton_valid=row.at("source").value("skeleton_valid",false);
      cycle.source.rotations_valid=row.at("source").value("rotations_valid",false);
      auto& s=cycle.snapshot;
      s.state.phase=row.at("state");s.state.reason=row.value("reason","");
      s.state.epoch=row.at("state_epoch");s.ticks=row.at("ticks");
      s.cycle_capture_failed=row.value("capture_failed",false);
      s.command.emplace();s.feedback.emplace();
      if(row.contains("hand_feedback")) {
        s.hand_feedback=row.at("hand_feedback").get<HandPair>();
        if(row.contains("left_hand_command"))s.hand_command[0]=row.at("left_hand_command").get<HandJoints>();
      }
      for(int side=0;side<2;++side) {
        const auto key=side?"right":"left";
        s.command->positions[side]=row.at("command").at(key).get<Joints7>();
        (*s.feedback)[side]=row.at("feedback").at(key).get<Joints7>();
      }
      if(row.contains("result") && !row.at("result").is_null()) {
        const auto& r=row.at("result");cycle.result.emplace();
        cycle.result->tick=r.at("tick_id");
        cycle.result->applied_sequence=r.at("applied_sequence");
        cycle.result->input_live=r.value("input_live",false);
      }
      cycle.ik_adopted=row.value("ik_adopted",false);
      SessionPublication encoder(input.at("manifest"));
      const auto begin=std::chrono::steady_clock::now();
      const auto rows=encoder.encode(cycle);
      // Measure per-message serialization, just like the real publisher.
      std::vector<std::string> payloads;
      for(const auto& row:rows) payloads.push_back(row.at(1).dump());
      elapsed.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now()-begin).count());
      std::cout << rows.dump() << '\n';
    }
    if(argc>1) std::cerr << nlohmann::json({{"encode_serialize_ns",elapsed}}).dump() << '\n';
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
