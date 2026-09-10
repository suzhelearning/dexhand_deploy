// TEST ORACLE ONLY. Original Viewer control block from c022b177 is verbatim.
// Only the clock/input exchange and rendering/telemetry are replaced by the
// deterministic test harness. Never link this into a production backend.
#pragma once
#include "tianji_qp_ik/acceleration_controller.hpp"
#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/pico_teleop_session.hpp"
#include "tianji_qp_ik/pico_udp_receiver.hpp"
#include "tianji_qp_ik/spark_guidance.hpp"
#include "tianji_qp_ik/spark_qpoases_diagnostic.hpp"
#include "tianji_qp_ik/so3.hpp"
#include <optional>

namespace tianji_qp_ik {
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


struct TestInbox {
  std::optional<PicoTeleopFrame> pending;
  bool tryReadLatest(PicoTeleopFrame& output) {
    if (!pending) return false;
    output = *pending;
    pending.reset();
    return true;
  }
};
inline DualArmTargets currentTargets(MujocoRobot& robot) {
  robot.forward();
  return {robot.tcpPose(ArmSide::kLeft), robot.tcpPose(ArmSide::kRight)};
}
class ReferenceViewerCycle {
  QpIkConfig config;
  MujocoRobot robot;
  std::unique_ptr<TargetManager> target_storage;
  std::unique_ptr<DualArmController> controller;
  std::unique_ptr<DualArmAccelerationController> acceleration_controller;
  std::unique_ptr<DualArmSparkGuidance> spark_guidance;
  CartesianReferenceGenerator left_otg, right_otg;
  PicoTeleopSession pico_session;
  ArmDirectionReferenceManager arm_direction_manager;
  TestInbox inbox;
  TestInbox* pico_frames{&inbox};
  PicoUdpReceiver* pico_receiver{nullptr};  // stats only; never start a socket
  DualArmTargets last_spark_targets;
  DualArmReferences last_spark_references;
  SparkGuidanceDiagnostics spark_diagnostics;
  bool joint_takeover_active{false}, pico_paused{false}, paused{false};
  bool plot_reset_requested{false};
  ArmMotionState direct_left_state, direct_right_state;
  ArmAngleReferenceMode arm_angle_reference_mode{ArmAngleReferenceMode::kOutwardOnly};
  DualArmDirectionReferences latest_pico_arm_directions;
  PicoUpperLimbSkeleton latest_pico_upper_limb_skeleton;
  std::uint64_t pico_applied_epoch{0}, pico_applied_sequence{0}, pico_reset_applies{0};
  std::int64_t pico_left_source_timestamp_ns{0}, pico_right_source_timestamp_ns{0};
  double pico_receive_to_control_us{0}, pico_bridge_to_control_us{0};
  std::uint64_t control_failures{0}, guidance_count{0}, headroom_count{0};
  double dt;
 public:
  ReferenceViewerCycle(QpIkConfig c, const std::string& model, const std::string& urdf, bool enabled)
      : config(std::move(c)), robot(model),
        left_otg(cartesianOtgConfigForControlLevel(config, config.control_level), 1.0/config.controller.rate_hz),
        right_otg(cartesianOtgConfigForControlLevel(config, config.control_level), 1.0/config.controller.rate_hz),
        pico_session(config.cartesian_servo.target_timeout_seconds),
        arm_direction_manager(config.arm_angle.reference_rate_limit_rad_s),
        dt(1.0/config.controller.rate_hz) {
    if (config.ik_algorithm != IkAlgorithm::kSparkUpperQpoasesHeadroomFeedforwardVelocityQp ||
        config.control_level != ControlLevel::kVelocity || !config.controller.model_state_only)
      throw std::invalid_argument("oracle is only the pinned SPARK model-only route");
    for (const auto side : {ArmSide::kLeft, ArmSide::kRight})
      robot.setArmState(side, configuredInitialPosture(config.controller, robot.mapping(side).limits, side), Vec7::Zero());
    robot.forward();
    const auto initial = currentTargets(robot);
    target_storage = std::make_unique<TargetManager>(config, initial);
    controller = std::make_unique<DualArmController>(robot, config);
    acceleration_controller = std::make_unique<DualArmAccelerationController>(robot, config);
    left_otg.reset(initial.left); right_otg.reset(initial.right);
    spark_guidance = std::make_unique<DualArmSparkGuidance>(robot, config, urdf,
        SparkPostureGuideMode::kHeadroomFeedforwardJointReferenceVelocity);
    last_spark_targets = initial;
    last_spark_references = directReferences(initial);
    direct_left_state.q = robot.armPosition(ArmSide::kLeft);
    direct_right_state.q = robot.armPosition(ArmSide::kRight);
    pico_session.setEnabled(enabled);
    pico_paused = !enabled;
  }
  BilateralCycleResult step(std::uint64_t tick, std::int64_t monotonic_now_ns,
                            const std::optional<PicoTeleopFrame>& incoming) {
    auto& targets = *target_storage;
    // Preserve the original block's explicit local-variable lambda captures.
    auto& robot = this->robot;
    auto& controller = this->controller;
    const double dt = this->dt;
    // Offline input is already stream-gated; receiver statistics are not part
    // of this numerical oracle and no live UDP receiver is constructed.
    PicoUdpReceiver* const pico_receiver = nullptr;
    const double target_time = monotonicTimestampSeconds(monotonic_now_ns);
    const bool previous_pause = pico_paused;
    const auto previous_resets = pico_reset_applies;
    inbox.pending = incoming;
    for (;;) {
// BEGIN VERBATIM REFERENCE VIEWER CONTROL BLOCK
    PicoTeleopFrame pico_frame;
    if (pico_frames != nullptr && pico_frames->tryReadLatest(pico_frame)) {
      const PicoTeleopButtonAction button_action =
          pico_session.observeButton(pico_frame);
      if (button_action == PicoTeleopButtonAction::kPause) {
        pico_paused = true;
        if (spark_guidance != nullptr) {
          spark_guidance->cancelJointSpaceTakeover();
        }
        joint_takeover_active = false;
        targets.setMode(TargetMode::kHold, target_time);
      } else {
        if (button_action == PicoTeleopButtonAction::kResume) {
          pico_paused = false;
        }
        const PicoTeleopClassification classification =
            pico_session.classify(pico_frame, monotonic_now_ns);
        if (classification.action == PicoTeleopAction::kApply ||
            classification.action == PicoTeleopAction::kResetEpochAndApply) {
          const double source_seconds =
              static_cast<double>(pico_frame.source_timestamp_ns) * 1.0e-9;
          const bool reset_epoch =
              classification.action == PicoTeleopAction::kResetEpochAndApply;
          const DualArmTargets current = currentTargets(robot);
          bool target_accepted = false;
          TargetManager candidate = reset_epoch
                                        ? TargetManager(config, current)
                                        : targets;
          if (spark_guidance != nullptr &&
              usesSparkGuidance(controller->algorithm())) {
            const bool spark_direct_qpos =
                usesSparkUpperQpoasesDirect(controller->algorithm());
            if (reset_epoch) {
              joint_takeover_active = false;
              spark_guidance->cancelJointSpaceTakeover();
              (void)spark_guidance->reset(
                  spark_direct_qpos
                      ? direct_left_state
                      : controller->referenceState(ArmSide::kLeft),
                  spark_direct_qpos
                      ? direct_right_state
                      : controller->referenceState(ArmSide::kRight));
            }
            const SparkUpperTargets spark_targets =
                spark_guidance->updatePicoFrame(pico_frame);
            target_accepted = spark_targets.valid;
            if (target_accepted && reset_epoch && !spark_direct_qpos &&
                controller->algorithm() != IkAlgorithm::kSparkPoseVelocityQp) {
              const ArmMotionState left_alignment_model =
                  spark_direct_qpos
                      ? direct_left_state
                      : controller->referenceState(ArmSide::kLeft);
              const ArmMotionState right_alignment_model =
                  spark_direct_qpos
                      ? direct_right_state
                      : controller->referenceState(ArmSide::kRight);
              joint_takeover_active = spark_guidance->startJointSpaceTakeover(
                  spark_targets, left_alignment_model, right_alignment_model);
            }
          } else {
            candidate.setMode(TargetMode::kManual, target_time);
            target_accepted = candidate.setManualTargets(
                pico_frame.left, pico_frame.right, source_seconds,
                monotonicTimestampSeconds(pico_frame.receive_monotonic_ns));
          }
          if (target_accepted) {
            if (reset_epoch && !config.controller.model_state_only) {
              const bool synchronized =
                  config.control_level == ControlLevel::kAcceleration
                      ? acceleration_controller->synchronizeReferencesToActual()
                      : controller->synchronizeReferencesToActual();
              if (!synchronized) {
                continue;
              }
            }
            if (spark_guidance == nullptr ||
                !usesSparkGuidance(controller->algorithm())) {
              targets = std::move(candidate);
            }
            if (reset_epoch) {
              // A PICO epoch reset changes the target stream, not the robot's
              // commanded Cartesian state.  Keep the OTG state continuous and
              // let it move toward the newly accepted target under its normal
              // velocity, acceleration, and jerk limits.  Resetting from the
              // MuJoCo feedback pose here teleported the reference whenever
              // the model reference and simulated feedback had separated.
              plot_reset_requested = true;
              ++pico_reset_applies;
            }
            pico_session.commitApplied(pico_frame);
            latest_pico_upper_limb_skeleton = pico_frame.upper_limb_skeleton;
            if (pico_frame.left_arm_direction.valid) {
              latest_pico_arm_directions.left = pico_frame.left_arm_direction;
            }
            if (pico_frame.right_arm_direction.valid) {
              latest_pico_arm_directions.right = pico_frame.right_arm_direction;
            }
            pico_applied_epoch = pico_frame.tracking_epoch;
            pico_applied_sequence = pico_frame.sequence;
            pico_left_source_timestamp_ns = pico_frame.source_timestamp_ns;
            pico_right_source_timestamp_ns = pico_frame.source_timestamp_ns;
            pico_receive_to_control_us = 1.0e-3 * static_cast<double>(
                std::max<std::int64_t>(
                    0, monotonic_now_ns - pico_frame.receive_monotonic_ns));
            pico_bridge_to_control_us = 1.0e-3 * static_cast<double>(
                std::max<std::int64_t>(
                    0, monotonic_now_ns -
                           pico_frame.bridge_send_monotonic_ns));
          }
        }
      }
    }

    const PicoTeleopFreshness pico_freshness =
        pico_session.freshness(monotonic_now_ns);
    const PicoReceiverStats pico_stats =
        pico_receiver != nullptr ? pico_receiver->stats() : PicoReceiverStats{};
    if (spark_guidance != nullptr &&
        usesSparkGuidance(controller->algorithm())) {
      arm_angle_reference_mode = ArmAngleReferenceMode::kOutwardOnly;
    }
    const DualArmDirectionReferences requested_arm_directions =
        selectArmDirectionReferences(arm_angle_reference_mode,
                                     pico_freshness.live,
                                     latest_pico_arm_directions);
    const DualArmDirectionReferences arm_directions =
        arm_direction_manager.update(requested_arm_directions, dt);

    DualArmTargets desired = targets.sample(target_time);
    const bool spark_mode = spark_guidance != nullptr &&
                            usesSparkGuidance(controller->algorithm());
    bool joint_takeover_cycle = false;
    if (spark_mode) {
      if (!pico_freshness.live) {
        if (joint_takeover_active) {
          spark_guidance->cancelJointSpaceTakeover();
          joint_takeover_active = false;
        }
        spark_guidance->invalidateTarget("spark_pico_stale");
      }
      joint_takeover_cycle = joint_takeover_active;
      const bool spark_direct_qpos =
          usesSparkUpperQpoasesDirect(controller->algorithm());
      const ArmMotionState left_spark_model =
          spark_direct_qpos ? direct_left_state
                            : controller->referenceState(ArmSide::kLeft);
      const ArmMotionState right_spark_model =
          spark_direct_qpos ? direct_right_state
                            : controller->referenceState(ArmSide::kRight);
      spark_diagnostics = joint_takeover_active
                              ? spark_guidance->stepJointSpaceTakeover(
                                    left_spark_model, right_spark_model, dt)
                              : spark_guidance->step(
                                    left_spark_model, right_spark_model, dt);
      if (spark_diagnostics.accepted) {
        last_spark_targets = spark_diagnostics.cartesian_targets;
        if (spark_diagnostics.cartesian_references_valid) {
          last_spark_references = spark_diagnostics.cartesian_references;
        }
      }
      if (joint_takeover_cycle &&
          (!spark_diagnostics.accepted ||
           spark_diagnostics.joint_takeover_finished)) {
        joint_takeover_active = false;
      }
      desired = last_spark_targets;
      desired.left_stale = !pico_freshness.live ||
                           !spark_diagnostics.accepted;
      desired.right_stale = desired.left_stale;
    }
    const bool spark_internal_otg =
        usesSparkOtgConsistentVelocityQp(controller->algorithm());
    const bool spark_internal_feedforward =
        usesSparkFeedforwardVelocityQp(controller->algorithm());
    const bool spark_internal_reference =
        spark_internal_otg || spark_internal_feedforward;
    DualArmReferences references =
        (joint_takeover_cycle && spark_diagnostics.accepted &&
         spark_diagnostics.cartesian_references_valid)
            ? spark_diagnostics.cartesian_references
            : (spark_internal_reference ? last_spark_references
                                        : directReferences(desired));
    const bool spark_direct_qpos =
        usesSparkUpperQpoasesDirect(controller->algorithm());
    const bool spark_joint_reference_velocity =
        controller->algorithm() ==
        IkAlgorithm::kSparkUpperQpoasesVelocityQp;
    if (config.cartesian_otg.enabled && !spark_direct_qpos &&
        !spark_joint_reference_velocity && !spark_internal_reference) {
      references.left = left_otg.update(
          desired.left, desired.left_twist, desired.left_stale, dt);
      references.right = right_otg.update(
          desired.right, desired.right_twist, desired.right_stale, dt);
    }
    ControllerDiagnostics diagnostics;
    AccelerationControllerDiagnostics acceleration_diagnostics;
    controller->setArmAngleReferenceMode(arm_angle_reference_mode);
    acceleration_controller->setArmAngleReferenceMode(
        arm_angle_reference_mode);
    if (!paused && !pico_paused) {
      if (spark_direct_qpos) {
        const SparkQpoasesDirectCommand direct_command =
            makeDirectCommand(spark_diagnostics);
        const auto update_direct_state = [dt](const Vec7& joint_command,
                                               ArmMotionState& state) {
          const Vec7 velocity = (joint_command - state.q) / dt;
          const Vec7 acceleration = (velocity - state.qdot) / dt;
          state.q = joint_command;
          state.qdot = velocity;
          state.qddot = acceleration;
        };
        if (direct_command.accepted) {
          update_direct_state(direct_command.left, direct_left_state);
          update_direct_state(direct_command.right, direct_right_state);
        } else {
          update_direct_state(direct_left_state.q, direct_left_state);
          update_direct_state(direct_right_state.q, direct_right_state);
        }
        robot.setArmState(ArmSide::kLeft, direct_left_state.q,
                          direct_left_state.qdot);
        robot.setArmState(ArmSide::kRight, direct_right_state.q,
                          direct_right_state.qdot);
        robot.forward();

        const auto fill_direct_diagnostics =
            [&robot](ArmSide side, const ArmMotionState& state,
                     const CartesianReference& reference,
                     ArmControllerDiagnostics& arm) {
              arm.accepted = true;
              arm.hold_reason = HoldReason::kNone;
              arm.reference = reference;
              arm.target = reference.pose;
              arm.q_ref = state.q;
              arm.q_actual = robot.armPosition(side);
              arm.current = robot.tcpPose(side);
              arm.tcp_actual = arm.current;
              arm.pose_error = poseErrorWorld(arm.target, arm.current);
              arm.actual_pose_error = arm.pose_error;
              arm.ik.status = SolverStatus::kSolved;
              arm.ik.detail = "spark_qpoases_direct_qpos";
              arm.ik.qdot = state.qdot;
            };
        fill_direct_diagnostics(ArmSide::kLeft, direct_left_state,
                                references.left, diagnostics.left);
        fill_direct_diagnostics(ArmSide::kRight, direct_right_state,
                                references.right, diagnostics.right);
        diagnostics.accepted = true;
        diagnostics.hold_reason = HoldReason::kNone;
      } else if (config.control_level == ControlLevel::kAcceleration) {
        acceleration_diagnostics = acceleration_controller->step(
            references, arm_directions, dt);
      } else {
        if (spark_mode && spark_diagnostics.accepted) {
          diagnostics = controller->step(
              references, arm_directions, spark_diagnostics.posture_tasks,
              dt);
        } else {
          diagnostics = config.cartesian_otg.enabled
                            ? controller->step(references, arm_directions, dt)
                            : controller->step(desired, arm_directions, dt);
        }
      }
      const bool accepted = config.control_level == ControlLevel::kAcceleration
                                ? acceleration_diagnostics.accepted
                                : diagnostics.accepted;
      if (!accepted) {
        ++control_failures;
      }
      if (spark_guidance != nullptr &&
          usesSparkHeadroomFeedforwardVelocityQp(controller->algorithm()) &&
          config.control_level == ControlLevel::kVelocity &&
          !joint_takeover_cycle) {
        const bool pico_headroom_feedback_valid =
            pico_freshness.live && spark_diagnostics.accepted;
        const auto feedback = [&controller, pico_headroom_feedback_valid](
                                  ArmSide side,
                                  const ArmControllerDiagnostics& arm) {
          SparkConstraintHeadroomFeedback value;
          value.accepted = pico_headroom_feedback_valid && arm.accepted &&
                           arm.ik.status == SolverStatus::kSolved;
          value.qdot = arm.ik.qdot;
          value.qddot = controller->previousAcceleration(side);
          value.task_scale_position = arm.ik.task_scale_position;
          value.task_scale_orientation = arm.ik.task_scale_orientation;
          return value;
        };
        spark_guidance->updateHeadroomFeedback(
            feedback(ArmSide::kLeft, diagnostics.left),
            feedback(ArmSide::kRight, diagnostics.right), dt);
      }
    } else {
      robot.forward();
      diagnostics.left.q_ref =
          config.control_level == ControlLevel::kAcceleration
              ? acceleration_controller->positionReference(ArmSide::kLeft)
              : controller->reference(ArmSide::kLeft);
      diagnostics.right.q_ref =
          config.control_level == ControlLevel::kAcceleration
              ? acceleration_controller->positionReference(ArmSide::kRight)
              : controller->reference(ArmSide::kRight);
      diagnostics.left.q_actual = robot.armPosition(ArmSide::kLeft);
      diagnostics.right.q_actual = robot.armPosition(ArmSide::kRight);
      diagnostics.left.tcp_actual = robot.tcpPose(ArmSide::kLeft);
      diagnostics.right.tcp_actual = robot.tcpPose(ArmSide::kRight);
      diagnostics.left.current = robot.armKinematicsAt(
          ArmSide::kLeft, diagnostics.left.q_ref).tcp_pose;
      diagnostics.right.current = robot.armKinematicsAt(
          ArmSide::kRight, diagnostics.right.q_ref).tcp_pose;
      diagnostics.left.reference = config.cartesian_otg.enabled
                                       ? references.left
                                       : CartesianReference{desired.left};
      diagnostics.right.reference = config.cartesian_otg.enabled
                                        ? references.right
                                        : CartesianReference{desired.right};
      diagnostics.left.target = diagnostics.left.reference.pose;
      diagnostics.right.target = diagnostics.right.reference.pose;
      diagnostics.left.pose_error = poseErrorWorld(
          diagnostics.left.target, diagnostics.left.current);
      diagnostics.right.pose_error = poseErrorWorld(
          diagnostics.right.target, diagnostics.right.current);
      diagnostics.left.actual_pose_error =
          poseErrorWorld(diagnostics.left.target, diagnostics.left.tcp_actual);
      diagnostics.right.actual_pose_error =
          poseErrorWorld(diagnostics.right.target, diagnostics.right.tcp_actual);
      diagnostics.hold_reason = HoldReason::kNone;
    }

// END VERBATIM REFERENCE VIEWER CONTROL BLOCK
      BilateralCycleResult out;
      out.tick_id = tick;
      out.applied_epoch = pico_applied_epoch;
      out.applied_sequence = pico_applied_sequence;
      out.guidance_updates = ++guidance_count;
      if (!paused && !pico_paused && !joint_takeover_cycle) ++headroom_count;
      out.headroom_updates = headroom_count;
      out.epoch_reset = pico_reset_applies != previous_resets;
      out.control_executed = !paused && !pico_paused;
      out.joint_takeover_cycle = joint_takeover_cycle;
      out.button_action = previous_pause == pico_paused ? PicoTeleopButtonAction::kNone
          : (pico_paused ? PicoTeleopButtonAction::kPause : PicoTeleopButtonAction::kResume);
      out.freshness = pico_freshness;
      out.guidance = spark_diagnostics;
      out.control = diagnostics;
      out.left = controller->referenceState(ArmSide::kLeft);
      out.right = controller->referenceState(ArmSide::kRight);
      return out;
    }
  }
};
} // namespace tianji_qp_ik
