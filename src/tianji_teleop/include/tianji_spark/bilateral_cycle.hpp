#pragma once

#include "tianji_spark/controller.hpp"
#include "tianji_spark/pico_teleop_session.hpp"
#include "tianji_spark/spark_guidance.hpp"
#include <memory>
#include <optional>

namespace tianji_spark {

struct BilateralCycleResult {
  std::uint64_t tick_id{0};
  std::uint64_t applied_epoch{0};
  std::uint64_t applied_sequence{0};
  std::uint64_t guidance_updates{0};
  std::uint64_t headroom_updates{0};
  bool epoch_reset{false};
  bool control_executed{false};
  bool joint_takeover_cycle{false};
  PicoTeleopButtonAction button_action{PicoTeleopButtonAction::kNone};
  PicoTeleopFreshness freshness;
  SparkGuidanceDiagnostics guidance;
  ControllerDiagnostics control;
  ArmMotionState left;
  ArmMotionState right;
};

// One atomic dual-arm control cycle. Full SPARK Headroom/velocity/model-only
// route extracted from the pinned Viewer, not two single-arm solve() calls.
// No transport, device publishing, wall-clock sleep or external authorization.
// Own this object on one thread. Feed only the latest stream-gated frame per
// tick; nullopt means no NEW frame, not that the last target is immediately stale.
class NativeSparkCycle {
 public:
  NativeSparkCycle(QpIkConfig config, const std::string& model_path,
                   const std::string& urdf_path, bool initially_enabled,
                   bool resume_same_epoch = false);
  ~NativeSparkCycle();
  NativeSparkCycle(const NativeSparkCycle&) = delete;
  NativeSparkCycle& operator=(const NativeSparkCycle&) = delete;
  BilateralCycleResult step(std::uint64_t tick_id, std::int64_t now_ns,
                           const std::optional<PicoTeleopFrame>& frame);
  void set_enabled(bool enabled);
  // Explicit deployment reset ONLY after the owner verifies trusted bilateral
  // feedback at rest. Rebuilds histories; does not authorize or execute a tick.
  // Input tracking-epoch changes must NOT use this operation.
  void reset_at_rest(const Vec7& left, const Vec7& right);
  ArmMotionState reference_state(ArmSide side) const;
  double period_seconds() const noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace tianji_spark
