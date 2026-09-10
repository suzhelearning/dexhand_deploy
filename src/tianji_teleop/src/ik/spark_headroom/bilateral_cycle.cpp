#include "tianji_spark/bilateral_cycle.hpp"
#include "tianji_spark/so3.hpp"
#include <cmath>
#include <stdexcept>
#include <array>

namespace tianji_spark {
namespace {
QpIkConfig checked(QpIkConfig config) {
  if (config.ik_algorithm != IkAlgorithm::kSparkUpperQpoasesHeadroomFeedforwardVelocityQp ||
      config.control_level != ControlLevel::kVelocity || !config.controller.model_state_only) {
    throw std::invalid_argument("SPARK cycle requires headroom velocity QP and model_state_only");
  }
  if (!std::isfinite(config.controller.rate_hz) || config.controller.rate_hz <= 0) {
    throw std::invalid_argument("SPARK rate must be finite and positive");
  }
  return config;
}
}

struct NativeSparkCycle::Impl {
  QpIkConfig config;
  std::string model_path, urdf_path;
  MujocoRobot robot;
  std::unique_ptr<DualArmController> controller;
  std::unique_ptr<DualArmSparkGuidance> guidance;
  PicoTeleopSession session;
  ArmDirectionReferenceManager directions;
  DualArmDirectionReferences latest_directions;
  DualArmTargets last_targets;
  DualArmReferences last_references;
  bool takeover{false};
  bool pico_paused{false};
  std::uint64_t last_tick{0}, applied_epoch{0}, applied_sequence{0};
  std::uint64_t guidance_updates{0}, headroom_updates{0};
  std::int64_t last_now{0};
  const double dt;

  Impl(QpIkConfig c, const std::string& model, const std::string& urdf, bool enabled,
       const std::optional<std::array<Vec7, 2>>& initial = std::nullopt)
      : config(checked(std::move(c))), model_path(model), urdf_path(urdf), robot(model),
        session(config.cartesian_servo.target_timeout_seconds),
        directions(config.arm_angle.reference_rate_limit_rad_s),
        dt(1.0 / config.controller.rate_hz) {
    for (auto side : {ArmSide::kLeft, ArmSide::kRight}) {
      const auto& limits = robot.mapping(side).limits;
      const Vec7 q = initial ? (*initial)[side == ArmSide::kLeft ? 0 : 1]
                            : configuredInitialPosture(config.controller, limits, side);
      if (!q.allFinite() || (q.array() < limits.lower_position.array()).any() ||
          (q.array() > limits.upper_position.array()).any()) {
        throw std::invalid_argument("reset state is nonfinite or outside model joint limits");
      }
      robot.setArmState(side, q, Vec7::Zero());
    }
    robot.forward();
    last_targets = {robot.tcpPose(ArmSide::kLeft), robot.tcpPose(ArmSide::kRight)};
    last_references = directReferences(last_targets);
    controller = std::make_unique<DualArmController>(robot, config);
    guidance = std::make_unique<DualArmSparkGuidance>(robot, config, urdf,
        SparkPostureGuideMode::kHeadroomFeedforwardJointReferenceVelocity);
    session.setEnabled(enabled);
    pico_paused = !enabled;
  }

  BilateralCycleResult step(std::uint64_t tick, std::int64_t now,
                            const std::optional<PicoTeleopFrame>& frame) {
    if (tick != last_tick + 1 || now <= 0 || now <= last_now) {
      throw std::invalid_argument("SPARK tick must be consecutive with increasing monotonic time");
    }
    BilateralCycleResult out;
    out.tick_id = tick;
    if (frame) {
      out.button_action = session.observeButton(*frame);
      if (out.button_action == PicoTeleopButtonAction::kPause) {
        pico_paused = true;
        guidance->cancelJointSpaceTakeover();
        takeover = false;
      } else {
        if (out.button_action == PicoTeleopButtonAction::kResume) pico_paused = false;
        const auto classification = session.classify(*frame, now);
        if (classification.action == PicoTeleopAction::kApply ||
            classification.action == PicoTeleopAction::kResetEpochAndApply) {
          const bool reset = classification.action == PicoTeleopAction::kResetEpochAndApply;
          if (reset) {
            takeover = false;
            guidance->cancelJointSpaceTakeover();
            (void)guidance->reset(controller->referenceState(ArmSide::kLeft),
                                  controller->referenceState(ArmSide::kRight));
          }
          const auto targets = guidance->updatePicoFrame(*frame);
          if (targets.valid) {
            if (reset) {
              takeover = guidance->startJointSpaceTakeover(targets,
                  controller->referenceState(ArmSide::kLeft),
                  controller->referenceState(ArmSide::kRight));
            }
            // model_state_only: never synchronize controller/OTG to feedback
            // merely because the source epoch/resynchronization changed.
            session.commitApplied(*frame);
            if (frame->left_arm_direction.valid) latest_directions.left = frame->left_arm_direction;
            if (frame->right_arm_direction.valid) latest_directions.right = frame->right_arm_direction;
            applied_epoch = frame->tracking_epoch;
            applied_sequence = frame->sequence;
            out.epoch_reset = reset;
          }
        }
      }
    }
    out.freshness = session.freshness(now);
    const auto arm_directions = directions.update(selectArmDirectionReferences(
        ArmAngleReferenceMode::kOutwardOnly, out.freshness.live, latest_directions), dt);
    if (!out.freshness.live) {
      if (takeover) {
        guidance->cancelJointSpaceTakeover();
        takeover = false;
      }
      guidance->invalidateTarget("spark_pico_stale");
    }
    out.joint_takeover_cycle = takeover;
    const auto left_model = controller->referenceState(ArmSide::kLeft);
    const auto right_model = controller->referenceState(ArmSide::kRight);
    out.guidance = takeover ? guidance->stepJointSpaceTakeover(left_model, right_model, dt)
                             : guidance->step(left_model, right_model, dt);
    ++guidance_updates;
    if (out.guidance.accepted) {
      last_targets = out.guidance.cartesian_targets;
      if (out.guidance.cartesian_references_valid) last_references = out.guidance.cartesian_references;
    }
    if (out.joint_takeover_cycle && (!out.guidance.accepted || out.guidance.joint_takeover_finished)) {
      takeover = false;
    }
    auto desired = last_targets;
    desired.left_stale = !out.freshness.live || !out.guidance.accepted;
    desired.right_stale = desired.left_stale;
    const auto references = (out.joint_takeover_cycle && out.guidance.accepted &&
                             out.guidance.cartesian_references_valid)
        ? out.guidance.cartesian_references : last_references;
    controller->setArmAngleReferenceMode(ArmAngleReferenceMode::kOutwardOnly);
    if (!pico_paused) {
      out.control_executed = true;
      if (out.guidance.accepted) {
        out.control = controller->step(references, arm_directions, out.guidance.posture_tasks, dt);
      } else {
        out.control = config.cartesian_otg.enabled
            ? controller->step(references, arm_directions, dt)
            : controller->step(desired, arm_directions, dt);
      }
      if (!out.joint_takeover_cycle) {
        const auto feedback = [&](ArmSide side, const ArmControllerDiagnostics& arm) {
          SparkConstraintHeadroomFeedback value;
          value.accepted = out.freshness.live && out.guidance.accepted && arm.accepted &&
                           arm.ik.status == SolverStatus::kSolved;
          value.qdot = arm.ik.qdot;
          value.qddot = controller->previousAcceleration(side);
          value.task_scale_position = arm.ik.task_scale_position;
          value.task_scale_orientation = arm.ik.task_scale_orientation;
          return value;
        };
        guidance->updateHeadroomFeedback(feedback(ArmSide::kLeft, out.control.left),
                                         feedback(ArmSide::kRight, out.control.right), dt);
        ++headroom_updates;
      }
    } else {
      robot.forward();
      const auto paused_diagnostics = [&](ArmSide side, const CartesianReference& reference,
                                          const Pose& target, ArmControllerDiagnostics& arm) {
        arm.q_ref = controller->reference(side);
        arm.q_actual = robot.armPosition(side);
        arm.tcp_actual = robot.tcpPose(side);
        arm.current = robot.armKinematicsAt(side, arm.q_ref).tcp_pose;
        arm.reference = config.cartesian_otg.enabled ? reference : CartesianReference{target};
        arm.target = arm.reference.pose;
        arm.pose_error = poseErrorWorld(arm.target, arm.current);
        arm.actual_pose_error = poseErrorWorld(arm.target, arm.tcp_actual);
      };
      paused_diagnostics(ArmSide::kLeft, references.left, desired.left, out.control.left);
      paused_diagnostics(ArmSide::kRight, references.right, desired.right, out.control.right);
      out.control.hold_reason = HoldReason::kNone;
    }
    out.left = controller->referenceState(ArmSide::kLeft);
    out.right = controller->referenceState(ArmSide::kRight);
    out.applied_epoch = applied_epoch;
    out.applied_sequence = applied_sequence;
    out.guidance_updates = guidance_updates;
    out.headroom_updates = headroom_updates;
    last_tick = tick;
    last_now = now;
    return out;
  }
};

NativeSparkCycle::NativeSparkCycle(QpIkConfig config, const std::string& model,
    const std::string& urdf, bool enabled)
    : impl_(std::make_unique<Impl>(std::move(config), model, urdf, enabled)) {}
NativeSparkCycle::~NativeSparkCycle() = default;
BilateralCycleResult NativeSparkCycle::step(std::uint64_t tick, std::int64_t now,
    const std::optional<PicoTeleopFrame>& frame) { return impl_->step(tick, now, frame); }
void NativeSparkCycle::set_enabled(bool enabled) {
  impl_->session.setEnabled(enabled);
  impl_->pico_paused = !enabled;
}
void NativeSparkCycle::reset_at_rest(const Vec7& left, const Vec7& right) {
  // Construct both sides before replacing the live object. No partial reset
  // survives a validation/model/solver initialization exception.
  auto replacement = std::make_unique<Impl>(impl_->config, impl_->model_path,
      impl_->urdf_path, true, std::array<Vec7, 2>{left, right});
  impl_ = std::move(replacement);
}
ArmMotionState NativeSparkCycle::reference_state(ArmSide side) const {
  return impl_->controller->referenceState(side);
}
double NativeSparkCycle::period_seconds() const noexcept { return impl_->dt; }
}  // namespace tianji_spark
