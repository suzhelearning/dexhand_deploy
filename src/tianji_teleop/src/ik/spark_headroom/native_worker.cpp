// Bounded stdin/stdout IPC only. This executable cannot publish robot commands.
#include "tianji_spark/bilateral_cycle.hpp"
#include "../native_binary_results.hpp"
#include "../native_home_config.hpp"
#include <charconv>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {
using namespace tianji_spark;
template <class T> T number(const std::string& token) {
  T value{};
  if (token.empty() || token.front() == '-') throw std::invalid_argument("negative/empty IPC integer");
  const auto result = std::from_chars(token.data(), token.data() + token.size(), value);
  if (result.ec != std::errc{} || result.ptr != token.data() + token.size()) {
    throw std::invalid_argument("invalid IPC integer");
  }
  return value;
}

template <class Derived> void array(std::ostream& out, const Eigen::MatrixBase<Derived>& value) {
  if (!value.allFinite()) throw std::runtime_error("non-finite SPARK result");
  out << '[';
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (i) out << ',';
    out << value(i);
  }
  out << ']';
}

void arm(std::ostream& out, const ArmMotionState& state,
         const ArmControllerDiagnostics& control, const SparkGuidanceArmDiagnostics& guide) {
  out << "{\"q\":"; array(out, state.q);
  out << ",\"qdot\":"; array(out, state.qdot);
  out << ",\"qddot\":"; array(out, state.qddot);
  out << ",\"accepted\":" << control.accepted;
  out << ",\"qp_status\":" << static_cast<int>(control.ik.status);
  out << ",\"hold_reason\":" << static_cast<int>(control.hold_reason);
  out << ",\"stage1_q\":"; array(out, guide.ik.stage1_q);
  out << ",\"ik_q\":"; array(out, guide.ik.q);
  out << ",\"ik_accepted\":" << guide.ik.accepted;
  out << ",\"stage1_iterations\":" << guide.ik.stage1_iterations;
  out << ",\"stage2_iterations\":" << guide.ik.stage2_iterations;
  out << ",\"budget_exhausted\":" << guide.ik.budget_exhausted;
  out << ",\"feedforward_q\":"; array(out, guide.feedforward.q);
  out << ",\"feedforward_qdot\":"; array(out, guide.feedforward.qdot);
  out << ",\"feedforward_qddot\":"; array(out, guide.feedforward.qddot);
  out << ",\"feedforward_state\":" << static_cast<int>(guide.feedforward.state);
  out << ",\"feedforward_target_accepted\":" << guide.feedforward_target.accepted;
  out << ",\"headroom_scale\":" << guide.headroom.scale;
  out << ",\"headroom_state\":" << static_cast<int>(guide.headroom.state);
  out << ",\"task_scale_position\":" << control.ik.task_scale_position;
  out << ",\"task_scale_orientation\":" << control.ik.task_scale_orientation;
  out << ",\"settled_hold\":" << guide.settled_hold_active;
  out << ",\"settled_hold_reason\":" << static_cast<int>(guide.settled_hold_reason);
  out << ",\"stationary_hold\":" << guide.stationary_joint_reference_held;
  out << ",\"target_position\":"; array(out, control.target.position);
  out << ",\"target_quaternion_xyzw\":";
  array(out, Eigen::Quaterniond(control.target.rotation).coeffs());
  out << '}';
}

std::string encode(const BilateralCycleResult& result, std::int64_t now, bool deterministic) {
  std::ostringstream out;
  out << std::setprecision(17) << std::boolalpha;
  out << "{\"schema_version\":1,\"kind\":\"spark_bilateral_result\","
         "\"algorithm\":\"spark_upper_qpoases_headroom_feedforward_velocity_qp\","
         "\"state_source\":\"model_reference\",\"simulation_only\":true,\"tick_id\":" << result.tick_id;
  out << ",\"timestamp_ns\":" << now << ",\"deterministic_test\":" << deterministic;
  out << ",\"applied_epoch\":" << result.applied_epoch;
  out << ",\"applied_sequence\":" << result.applied_sequence;
  out << ",\"epoch_reset\":" << result.epoch_reset;
  out << ",\"input_live\":" << result.freshness.live;
  out << ",\"button_action\":" << static_cast<int>(result.button_action);
  out << ",\"control_executed\":" << result.control_executed;
  out << ",\"joint_takeover_cycle\":" << result.joint_takeover_cycle;
  out << ",\"guidance_accepted\":" << result.guidance.accepted;
  out << ",\"guidance_updates\":" << result.guidance_updates;
  out << ",\"headroom_updates\":" << result.headroom_updates;
  out << ",\"left\":"; arm(out, result.left, result.control.left, result.guidance.left);
  out << ",\"right\":"; arm(out, result.right, result.control.right, result.guidance.right);
  out << '}';
  return out.str();
}
}

int main(int argc, char** argv) {
  try {
    if (argc < 4 || argc > 10) throw std::invalid_argument(
        "usage: spark_native_worker CONFIG MODEL URDF [--deterministic-test] [--startup-handshake] [--binary-results] [--resume-same-epoch]");
    bool deterministic = false, handshake = false, binary = false;
    bool resume_same_epoch = false;
    std::string home_config;
    for (int i = 4; i < argc; ++i) {
      const std::string option(argv[i]);
      if (option == "--home-config" && home_config.empty() && i+1<argc) home_config=argv[++i];
      else if (option == "--deterministic-test" && !deterministic) deterministic = true;
      else if (option == "--startup-handshake" && !handshake) handshake = true;
      else if (option == "--binary-results" && !binary) binary = true;
      else if (option == "--resume-same-epoch" && !resume_same_epoch) resume_same_epoch = true;
      else throw std::invalid_argument("unknown or duplicate worker option");
    }
    if (binary) {
      // This worker uses only C++ streams. Responses explicitly flush; avoid
      // per-character stdio synchronization while reading the bounded request.
      std::ios::sync_with_stdio(false);
      std::cin.tie(nullptr);
    }
    auto config = loadConfig(argv[1]);
    apply_deployment_home(config, home_config);
    if (deterministic) {
      // Only the explicit offline test mode relaxes wall-clock budgets. Keep
      // iteration counts/constraints/objectives unchanged. Never use this mode
      // as a real-time or physical-device performance claim.
      config.spark_upper_qpoases.ik_cycle_budget_seconds = 3600.0;
      config.qpoases.cpu_time_limit_seconds = 3600.0;
    }
    NativeSparkCycle cycle(config, argv[2], argv[3], true, resume_same_epoch);
    if (handshake) {
      std::cout << "{\"schema_version\":1,\"kind\":\"spark_worker_ready\","
                   "\"algorithm\":\"spark_upper_qpoases_headroom_feedforward_velocity_qp\","
                   "\"native_ticks\":0}" << std::endl;
    }
    char buffer[2048];
    std::int64_t execution_epoch = 1;
    while (std::cin.getline(buffer, sizeof(buffer))) {
      if (std::cin.eof()) throw std::invalid_argument("incomplete IPC frame without newline");
      std::istringstream line(buffer);
      std::vector<std::string> tokens;
      for (std::string token; line >> token;) tokens.push_back(token);
      if (tokens.size() == 16 && tokens[0] == "TJSR1") {
        const auto next_epoch = number<std::int64_t>(tokens[1]);
        if (next_epoch <= execution_epoch) throw std::invalid_argument("reset epoch must increase");
        Eigen::Matrix<double, 14, 1> q;
        for (int i = 0; i < 14; ++i) {
          std::size_t consumed = 0;
          q[i] = std::stod(tokens[i + 2], &consumed);
          if (consumed != tokens[i + 2].size() || !std::isfinite(q[i])) {
            throw std::invalid_argument("invalid reset position");
          }
        }
        cycle.reset_at_rest(q.head<7>(), q.tail<7>());
        const auto left = cycle.reference_state(ArmSide::kLeft);
        const auto right = cycle.reference_state(ArmSide::kRight);
        // Acknowledgement reads initialized controller state, not input echo.
        q << left.q, right.q;
        Eigen::Matrix<double, 14, 1> velocity, acceleration;
        velocity << left.qdot, right.qdot;
        acceleration << left.qddot, right.qddot;
        execution_epoch = next_epoch;
        std::ostringstream ack;
        ack << std::setprecision(17) << "{\"schema_version\":1,\"kind\":\"spark_reset_ack\","
            << "\"execution_epoch\":" << execution_epoch << ",\"position_rad\":";
        array(ack, q);
        ack << ",\"velocity_rad_s\":";
        array(ack, velocity);
        ack << ",\"acceleration_rad_s2\":";
        array(ack, acceleration);
        ack << '}';
        std::cout << ack.str() << std::endl;
        continue;
      }
      if (tokens.size() != 7 || tokens[0] != "TJSC1") throw std::invalid_argument("invalid IPC frame");
      const auto tick = number<std::uint64_t>(tokens[1]);
      const auto now = number<std::int64_t>(tokens[2]);
      const auto received = number<std::int64_t>(tokens[3]);
      if (received > now) throw std::invalid_argument("receive time is later than control time");
      const auto generation = number<std::uint64_t>(tokens[4]);
      const auto discontinuity = number<unsigned>(tokens[5]);
      if (discontinuity > 1) throw std::invalid_argument("invalid discontinuity flag");
      std::optional<PicoTeleopFrame> frame;
      const auto& hex = tokens[6];
      if (hex == "-") {
        if (received || generation || discontinuity) throw std::invalid_argument("metadata without packet");
      } else {
        if (received <= 0 || hex.size() > 1312 || hex.size() % 2) throw std::invalid_argument("invalid packet framing");
        std::vector<std::uint8_t> packet;
        for (std::size_t i = 0; i < hex.size(); i += 2) {
          unsigned byte{};
          const auto parsed = std::from_chars(hex.data() + i, hex.data() + i + 2, byte, 16);
          if (parsed.ec != std::errc{} || parsed.ptr != hex.data() + i + 2) throw std::invalid_argument("invalid packet hex");
          packet.push_back(static_cast<std::uint8_t>(byte));
        }
        auto decoded = decodePicoTeleopPacket(packet.data(), packet.size());
        if (!decoded.frame) throw std::invalid_argument("invalid TJVR packet");
        frame = *decoded.frame;
        frame->receive_monotonic_ns = received;
        frame->resynchronization_generation = generation;
        frame->stream_discontinuity = discontinuity != 0;
      }
      const auto started = tianji_native_wire::Clock::now();
      const auto result = cycle.step(tick, now, frame);
      const auto solve_ns = tianji_native_wire::elapsed_ns(started);
      if (binary) {
        tianji_native_wire::ResultWriter out(1);
        out.common(result, now, deterministic);
        out.boolean(result.joint_takeover_cycle); out.boolean(result.guidance.accepted);
        out.integer(result.guidance_updates); out.integer(result.headroom_updates);
        out.arm(result.left, result.control.left, result.guidance.left.headroom.scale);
        out.guidance(result.guidance.left);
        out.arm(result.right, result.control.right, result.guidance.right.headroom.scale);
        out.guidance(result.guidance.right);
        out.finish(solve_ns);
      } else {
        std::cout << encode(result, now, deterministic) << std::endl;
      }
    }
    if (!std::cin.eof()) throw std::invalid_argument("oversized/incomplete IPC frame");
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "spark_native_worker: " << error.what() << '\n';
    return 1;
  }
}
