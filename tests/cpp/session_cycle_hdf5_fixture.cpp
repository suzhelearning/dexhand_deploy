#include "../../native/control/session_cycle_hdf5.hpp"
#include <iostream>
using namespace tianji_control;
int main(int argc,char** argv) {
  try {
    if(argc!=2) return 2;
    Hdf5StreamClient disk(OwnedDescriptor(std::stoi(argv[1])),2000);
    std::string line;
    while(std::getline(std::cin,line)) {
      const auto input=nlohmann::json::parse(line);
      const auto& row=input.at("cycle");
      SessionCycleSnapshot c;c.timestamp_ns=row.at("timestamp_ns");
      c.source_revision=row.at("source").at("revision");c.source.sequence=row.at("source").at("sequence");
      c.source.accepted=row.at("source").at("accepted");
      c.source.skeleton_valid=row.at("source").at("skeleton_valid");
      c.source.rotations_valid=row.at("source").at("rotations_valid");
      c.snapshot.state.phase=row.at("state");c.snapshot.state.reason=row.at("reason");
      c.snapshot.state.epoch=row.at("state_epoch");c.snapshot.ticks=row.at("ticks");
      c.snapshot.command.emplace();c.snapshot.feedback.emplace();
      if(row.contains("hand_feedback")) c.snapshot.hand_feedback=row.at("hand_feedback").get<HandPair>();
      if(row.contains("hand_commands")) for(int s=0;s<2;++s) {
        const auto key=s?"right":"left";
        if(row.at("hand_commands").contains(key))
          c.snapshot.hand_command[s]=row.at("hand_commands").at(key).get<HandJoints>();
      }
      if(row.contains("hand_results")) for(const auto& a:row.at("hand_results")) {
        SessionHandResultAudit audit;audit.observed_ns=a.at("observed_timestamp_ns");
        audit.outcome={a.at("accepted"),a.at("reason")};audit.result.run=a.at("run_id");
        const auto& identity=a.at("producer_authority");
        audit.result.producer={identity.at("logical"),identity.at("instance"),identity.at("router")};
        auto& v=audit.result.value;const auto& result=a.at("result");
        v.output_sequence=result.at("output_sequence");v.input_sequence=result.at("input_sequence");
        v.input_timestamp_ns=result.at("input_timestamp_ns");v.epoch=result.at("epoch");
        v.scheduler_timestamp_ns=result.at("scheduler_timestamp_ns");v.valid_flags=result.at("valid_flags");
        v.phase=static_cast<tianji_hand::HandPhase>(result.at("phase").get<int>());
        v.status=static_cast<tianji_hand::HandOutputStatus>(result.at("status").get<int>());
        v.positions=result.at("positions_rad").get<std::array<double,40>>();
        c.hand_results.push_back(audit);
      }
      for(int s=0;s<2;++s) {
        const auto key=s?"right":"left";
        c.snapshot.command->positions[s]=row.at("command").at(key).get<Joints7>();
        (*c.snapshot.feedback)[s]=row.at("feedback").at(key).get<Joints7>();
      }
      const bool mapped=input.at("manifest").at("worker_prefix")=="mapped_palm";
      if(input.contains("wire")) c.result=decode_worker_result(input.at("wire").get<std::vector<std::uint8_t>>(),mapped);
      if(row.contains("request")) {
        const auto& r=row.at("request");c.request.emplace();
        c.request->packet=r.at("packet").get<std::vector<std::uint8_t>>();
        c.request->source_sequence=r.at("source_sequence");c.request->received_ns=r.at("received_ns");
        c.request->generation=r.at("generation");c.request->discontinuity=r.at("discontinuity");
      }
      c.ik_adopted=row.at("ik_adopted");
      disk.append(SessionCycleRecording(input.at("manifest")).encode(c,input.at("origin_ns")));
    }
    disk.close(true);return 0;
  } catch(const std::exception& e) {std::cerr<<e.what();return 1;}
}
