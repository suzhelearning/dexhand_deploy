#pragma once
#include "worker_result.hpp"
#include <nlohmann/json.hpp>

namespace tianji_control {
// Audit-only adapter for the existing versioned worker ABI. Decode at this
// boundary so malformed/non-finite values cannot silently become JSON null.
// Does not grant command authority or alter the worker's result/state.
inline nlohmann::json worker_result_json(const std::vector<std::uint8_t>& wire, bool mapped) {
  const auto r=decode_worker_result(wire,mapped);
  using Json=nlohmann::json;
  Json out={{"schema_version",1},
    {"kind",mapped?"mapped_palm_bilateral_result":"spark_bilateral_result"},
    {"algorithm",mapped?"pico_ee_mapped_corrected_palm_velocity_qp":
      "spark_upper_qpoases_headroom_feedforward_velocity_qp"},
    {"state_source","model_reference"},{"simulation_only",true},
    {"deterministic_test",r.deterministic},{"tick_id",r.tick},{"timestamp_ns",r.timestamp},
    {"applied_epoch",r.applied_epoch},{"applied_sequence",r.applied_sequence},
    {"epoch_reset",r.epoch_reset},{"input_live",r.input_live},
    {"button_action",r.button_action},{"control_executed",r.control_executed}};
  if(mapped) {
    if(r.height_present) out["target_height_offsets_m"]=r.height_offsets;
    if(r.x_present) out["target_x_offsets_m"]=r.x_offsets;
  } else {
    out["joint_takeover_cycle"]=r.joint_takeover_cycle;
    out["guidance_accepted"]=r.guidance_accepted;
    out["guidance_updates"]=r.guidance_updates;
    out["headroom_updates"]=r.headroom_updates;
  }
  for(std::size_t side=0;side<2;++side) {
    const auto& a=r.arms[side];
    Json arm={{"q",a.q},{"qdot",a.qdot},{"qddot",a.qddot},{"accepted",a.accepted},
      {"qp_status",a.qp_status},{"hold_reason",a.hold_reason},{"headroom_scale",a.headroom_scale},
      {"task_scale_position",a.task_scale_position},{"task_scale_orientation",a.task_scale_orientation},
      {"target_position",a.target_position},{"target_quaternion_xyzw",a.target_quaternion}};
    if(!mapped) {
      const auto& g=a.guidance;
      arm.update(Json{{"stage1_q",g.stage1_q},{"ik_q",g.ik_q},{"ik_accepted",g.ik_accepted},
        {"stage1_iterations",g.stage1_iterations},{"stage2_iterations",g.stage2_iterations},
        {"budget_exhausted",g.budget_exhausted},{"feedforward_q",g.feedforward_q},
        {"feedforward_qdot",g.feedforward_qdot},{"feedforward_qddot",g.feedforward_qddot},
        {"feedforward_state",g.feedforward_state},{"feedforward_target_accepted",g.feedforward_target_accepted},
        {"headroom_state",g.headroom_state},{"settled_hold",g.settled_hold},
        {"settled_hold_reason",g.settled_hold_reason},{"stationary_hold",g.stationary_hold}});
    }
    out[side==0?"left":"right"]=std::move(arm);
  }
  // Native timing counters intentionally remain outside the legacy audit schema.
  return out;
}
} // namespace tianji_control
