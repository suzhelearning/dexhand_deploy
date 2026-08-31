#include "tianji_teleop/ik/arm_ik_factory.hpp"
#include "tianji_teleop/control/joint_trajectory_limiter.hpp"
#include "tianji_teleop/protocol/json_parser.hpp"

#include <zenoh.hxx>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cctype>
#include <limits>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <thread>

namespace tianji_teleop {
namespace {

using Clock = std::chrono::steady_clock;
constexpr std::int64_t kFreshnessNs = 500000000;

std::string env_or(const char *name, const std::string &fallback = {}) {
  const char *value = std::getenv(name);
  return value == nullptr ? fallback : std::string(value);
}

double env_double(const char *name, double fallback) {
  const auto value = env_or(name);
  if (value.empty()) return fallback;
  std::size_t used = 0;
  const double result = std::stod(value, &used);
  if (used != value.size() || !std::isfinite(result)) {
    throw std::invalid_argument(std::string("invalid numeric environment variable: ") + name);
  }
  return result;
}

std::int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}

using ArmJointNames = std::array<std::array<std::string, 7>, 2>;

ArmJointVector parse_vector(const std::string &text, const std::string &field) {
  const auto start = text.find(field);
  const auto open = start == std::string::npos ? std::string::npos : text.find('[', start);
  const auto close = open == std::string::npos ? std::string::npos : text.find(']', open);
  if (open == std::string::npos || close == std::string::npos) {
    throw std::invalid_argument("vector missing " + field);
  }
  ArmJointVector result;
  std::size_t count = 0;
  std::size_t cursor = open + 1;
  while (cursor < close) {
    while (cursor < close && (std::isspace(static_cast<unsigned char>(text[cursor])) || text[cursor] == ',')) ++cursor;
    if (cursor >= close) break;
    std::size_t used = 0;
    const auto value = std::stod(text.substr(cursor, close - cursor), &used);
    if (!std::isfinite(value) || count >= 7) {
      throw std::invalid_argument("vector contains invalid values: " + field);
    }
    result[static_cast<Eigen::Index>(count++)] = value;
    cursor += used;
  }
  if (count != 7) throw std::invalid_argument("vector must contain seven values: " + field);
  return result;
}

ArmJointVector env_vector(const char *name, const ArmJointVector &fallback) {
  const auto value = env_or(name);
  return value.empty() ? fallback : parse_vector(value, "");
}

struct ArmConfig {
  ArmJointNames names;
  ArmJointVector lower_limits{ArmJointVector::Zero()};
  ArmJointVector upper_limits{ArmJointVector::Zero()};
};

ArmConfig load_arm_config(const std::string &path) {
  if (path.empty()) throw std::invalid_argument("TIANJI_ARM_CONFIG is required");
  std::ifstream file(path);
  if (!file) throw std::invalid_argument("unable to read TIANJI_ARM_CONFIG: " + path);
  const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  ArmConfig result;
  for (std::size_t side = 0; side < 2; ++side) {
    const std::string field = side == 0 ? "left_joint_names:" : "right_joint_names:";
    const auto start = text.find(field);
    if (start == std::string::npos) throw std::invalid_argument("arm config missing " + field);
    std::size_t cursor = text.find('\n', start);
    for (std::size_t index = 0; index < 7; ++index) {
      if (cursor == std::string::npos) throw std::invalid_argument("arm config joint names truncated");
      const auto end = text.find('\n', cursor + 1);
      const auto line = text.substr(cursor + 1, end == std::string::npos ? std::string::npos : end - cursor - 1);
      const auto dash = line.find('-');
      if (dash == std::string::npos) throw std::invalid_argument("arm config joint name is malformed");
      auto begin = dash + 1;
      while (begin < line.size() && std::isspace(static_cast<unsigned char>(line[begin]))) ++begin;
      auto finish = begin;
      while (finish < line.size() && !std::isspace(static_cast<unsigned char>(line[finish])) && line[finish] != '#') ++finish;
      result.names[side][index] = line.substr(begin, finish - begin);
      if (result.names[side][index] != "Joint" + std::to_string(index + 1) + (side == 0 ? "_L" : "_R")) {
        throw std::invalid_argument("arm config joint order mismatch");
      }
      cursor = end;
    }
  }
  (void)parse_vector(text, "left_home_rad:");
  (void)parse_vector(text, "right_home_rad:");
  result.lower_limits = parse_vector(text, "lower_limits_rad:");
  result.upper_limits = parse_vector(text, "upper_limits_rad:");
  if ((result.lower_limits.array() >= result.upper_limits.array()).any()) {
    throw std::invalid_argument("arm config limits are invalid");
  }
  return result;
}

std::string quote(const std::string &value) {
  std::ostringstream out;
  out << '"';
  for (char c : value) {
    if (c == '"' || c == '\\') out << '\\';
    out << c;
  }
  out << '"';
  return out.str();
}

std::string number(double value) {
  if (!std::isfinite(value)) return "null";
  std::ostringstream out;
  out << std::setprecision(12) << value;
  return out.str();
}

template<typename T>
std::string array_json(const T &array) {
  std::ostringstream out;
  out << '[';
  for (std::size_t i = 0; i < static_cast<std::size_t>(array.size()); ++i) {
    if (i != 0) out << ',';
    out << number(array[i]);
  }
  out << ']';
  return out.str();
}

struct Target {
  std::string instance;
  std::string router;
  std::string source;
  std::string side;
  std::string frame;
  std::uint64_t sequence{0};
  std::int64_t timestamp_ns{0};
  std::optional<std::int64_t> source_timestamp_ns;
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d elbow{Eigen::Vector3d::UnitZ()};
};

class JsonTargetParser {
public:
  explicit JsonTargetParser(const std::string &text) : text_(text) {}

  Target parse() const {
    const auto root = protocol::StrictJsonParser::parse(text_);
    protocol::require_exact_fields(root, {
      "schema_version", "publisher_instance_id", "router_zid", "sequence",
      "timestamp_ns", "source_timestamp_ns", "source", "side", "frame_id",
      "position_m", "orientation_xyzw", "elbow_reference_direction"
    });
    if (protocol::field(root, "schema_version").as_uint("schema_version") != 1) {
      throw std::invalid_argument("unsupported arm target schema");
    }
    Target target;
    target.instance = protocol::field(root, "publisher_instance_id").as_string("publisher_instance_id");
    target.router = protocol::field(root, "router_zid").as_string("router_zid");
    target.source = protocol::field(root, "source").as_string("source");
    target.side = protocol::field(root, "side").as_string("side");
    target.frame = protocol::field(root, "frame_id").as_string("frame_id");
    target.sequence = protocol::field(root, "sequence").as_uint("sequence");
    const auto timestamp = protocol::field(root, "timestamp_ns").as_uint("timestamp_ns");
    if (timestamp > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
      throw std::invalid_argument("timestamp_ns is outside monotonic range");
    }
    target.timestamp_ns = static_cast<std::int64_t>(timestamp);
    const auto source_time = protocol::field(root, "source_timestamp_ns");
    if (!source_time.is_null()) {
      const auto value = source_time.as_uint("source_timestamp_ns");
      if (value > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
        throw std::invalid_argument("source_timestamp_ns is outside range");
      }
      target.source_timestamp_ns = static_cast<std::int64_t>(value);
    }
    if (target.side != "left" && target.side != "right") throw std::invalid_argument("invalid arm side");
    if (target.frame != (target.side == "left" ? "Base_L" : "Base_R")) {
      throw std::invalid_argument("arm frame does not match side");
    }
    const auto p = protocol::vector_field(root, "position_m", 3);
    const auto q = protocol::vector_field(root, "orientation_xyzw", 4);
    const auto e = protocol::vector_field(root, "elbow_reference_direction", 3);
    const double qnorm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    const double enorm = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
    if (!std::isfinite(qnorm) || qnorm < 0.999 || qnorm > 1.001) throw std::invalid_argument("invalid quaternion norm");
    if (!std::isfinite(enorm) || enorm < 1e-8) throw std::invalid_argument("invalid elbow direction");
    target.pose.translation() = Eigen::Vector3d(p[0], p[1], p[2]);
    target.pose.linear() = Eigen::Quaterniond(q[3], q[0], q[1], q[2]).normalized().toRotationMatrix();
    target.elbow = Eigen::Vector3d(e[0], e[1], e[2]) / enorm;
    return target;
  }

private:
  const std::string &text_;
};

struct CurrentCommand {
  std::string instance;
  std::string router;
  std::string side;
  std::uint64_t sequence{0};
  std::int64_t timestamp_ns{0};
  std::string mode;
  ArmJointVector joints{ArmJointVector::Zero()};
};

class JsonCommandParser {
public:
  explicit JsonCommandParser(const std::string &text) : text_(text) {}

  CurrentCommand parse(const std::string &expected_instance, const std::string &expected_router, const std::string &expected_side, const std::array<std::string, 7> &expected_names) const {
    const auto root = protocol::StrictJsonParser::parse(text_);
    protocol::require_exact_fields(root, {
      "schema_version", "publisher_instance_id", "router_zid", "sequence",
      "timestamp_ns", "producer", "side", "mode", "proposal_sequence",
      "target_sequence", "names", "position_rad"
    });
    if (protocol::field(root, "schema_version").as_uint("schema_version") != 1) {
      throw std::invalid_argument("unsupported arm command schema");
    }
    CurrentCommand command;
    command.instance = protocol::field(root, "publisher_instance_id").as_string("publisher_instance_id");
    command.router = protocol::field(root, "router_zid").as_string("router_zid");
    if (command.instance != expected_instance || command.router != expected_router) {
      throw std::invalid_argument("final command authority mismatch");
    }
    (void)protocol::field(root, "producer").as_string("producer");
    command.side = protocol::field(root, "side").as_string("side");
    if (command.side != expected_side) throw std::invalid_argument("final command side does not match topic");
    command.mode = protocol::field(root, "mode").as_string("mode");
    if (command.mode != "idle" && command.mode != "teleop" && command.mode != "returning") {
      throw std::invalid_argument("invalid arm command mode");
    }
    command.sequence = protocol::field(root, "sequence").as_uint("sequence");
    const auto timestamp = protocol::field(root, "timestamp_ns").as_uint("timestamp_ns");
    if (timestamp > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
      throw std::invalid_argument("timestamp_ns is outside monotonic range");
    }
    command.timestamp_ns = static_cast<std::int64_t>(timestamp);
    if (command.timestamp_ns > now_ns() || now_ns() - command.timestamp_ns > kFreshnessNs) {
      throw std::invalid_argument("final command is stale");
    }
    const auto proposal_sequence = protocol::field(root, "proposal_sequence");
    if (!proposal_sequence.is_null()) (void)proposal_sequence.as_uint("proposal_sequence");
    const auto target_sequence = protocol::field(root, "target_sequence");
    if (!target_sequence.is_null()) (void)target_sequence.as_uint("target_sequence");
    const auto names = protocol::string_array_field(root, "names", 7);
    for (std::size_t index = 0; index < names.size(); ++index) {
      if (names[index] != expected_names[index]) throw std::invalid_argument("final command joint order mismatch");
    }
    const auto position = protocol::vector_field(root, "position_rad", 7);
    for (std::size_t index = 0; index < position.size(); ++index) command.joints[static_cast<int>(index)] = position[index];
    return command;
  }

private:
  const std::string &text_;
};

std::string proposal_json(const Target &target, const IkResult &result, const ArmJointVector &joints, const std::array<std::string, 7> &names, const std::string &instance, const std::string &router, std::uint64_t sequence, bool hold = false) {
  std::ostringstream out;
  out << "{\"schema_version\":1,\"publisher_instance_id\":" << quote(instance)
      << ",\"router_zid\":" << quote(router) << ",\"sequence\":" << sequence
      << ",\"timestamp_ns\":" << now_ns() << ",\"producer\":\"arm_ik_producer\",\"side\":" << quote(target.side)
      << ",\"target_sequence\":" << target.sequence << ",\"names\":[";
  for (std::size_t i = 0; i < names.size(); ++i) { if (i) out << ','; out << quote(names[i]); }
  out << "],\"position_rad\":" << array_json(joints) << ",\"diagnostics\":{\"accepted\":true,\"converged\":" << (result.converged ? "true" : "false")
      << ",\"hold\":" << (hold ? "true" : "false") << "}}";
  return out.str();
}

std::string solved_json(const Target &target, const Eigen::Isometry3d &pose, const std::string &instance, const std::string &router, std::uint64_t sequence) {
  Eigen::Quaterniond q(pose.rotation());
  std::array<double, 3> p{pose.translation().x(), pose.translation().y(), pose.translation().z()};
  std::array<double, 4> quat{q.x(), q.y(), q.z(), q.w()};
  std::ostringstream out;
  out << "{\"schema_version\":1,\"publisher_instance_id\":" << quote(instance) << ",\"router_zid\":" << quote(router)
      << ",\"sequence\":" << sequence << ",\"timestamp_ns\":" << now_ns() << ",\"producer\":\"arm_ik_producer\",\"side\":"
      << quote(target.side) << ",\"frame_id\":" << quote(target.frame) << ",\"target_sequence\":" << target.sequence
      << ",\"position_m\":" << array_json(p) << ",\"orientation_xyzw\":" << array_json(quat) << "}";
  return out.str();
}  // namespace
class ArmIkProducer {
public:
  ArmIkProducer(
    zenoh::Session &session,
    std::string backend,
    std::string instance,
    std::string router,
    std::string coordinator,
    std::string source,
    std::string source_instance,
    ArmJointNames names,
    IkSettings settings,
    JointTrajectoryLimits trajectory_limits,
    double rate_hz,
    double freshness_timeout_s,
    double reject_grace_s)
  : session_(session), backend_(std::move(backend)), instance_(std::move(instance)),
    router_(std::move(router)), coordinator_(std::move(coordinator)),
    source_(std::move(source)), source_instance_(std::move(source_instance)),
    settings_(std::move(settings)), joint_names_(std::move(names)),
    joint_limits_(trajectory_limits),
    trajectory_limiters_{
      JointTrajectoryLimiter7(trajectory_limits, 1.0 / rate_hz),
      JointTrajectoryLimiter7(trajectory_limits, 1.0 / rate_hz)},
    failure_windows_{
      ConsecutiveFailureWindow(static_cast<std::int64_t>(reject_grace_s * 1.0e9)),
      ConsecutiveFailureWindow(static_cast<std::int64_t>(reject_grace_s * 1.0e9))},
    rate_hz_(rate_hz),
    freshness_timeout_ns_(static_cast<std::int64_t>(freshness_timeout_s * 1.0e9)),
    real_capability_(env_or("TIANJI_REQUIRED_CAPABILITY", "simulation") == "real") {
    if (source_.empty() || source_instance_.empty()) {
      throw std::invalid_argument("source logical and publisher identities are required");
    }
    if (!(rate_hz_ > 0.0) || !(freshness_timeout_ns_ > 0) || !(reject_grace_s > 0.0)) {
      throw std::invalid_argument("IK rate, freshness timeout and reject grace must be positive");
    }
    ArmIkBackendOptions options;
    options.urdf_path = env_or("TIANJI_ARM_URDF");
    options.official_library_path = env_or("TIANJI_OFFICIAL_IK_LIBRARY");
    options.official_config_path = env_or("TIANJI_OFFICIAL_IK_CONFIG");
    solver_ = create_arm_ik_solver(backend_, options, settings_);
    status_publisher_ = session_.declare_publisher(zenoh::KeyExpr("tianji/producer/status"));
    for (const std::string side : {"left", "right"}) {
      const auto index = side == "left" ? 0U : 1U;
      proposal_publishers_[index] = session_.declare_publisher(zenoh::KeyExpr("tianji/proposal/arm/" + side));
      solved_publishers_[index] = session_.declare_publisher(zenoh::KeyExpr("tianji/producer/arm/" + side + "/solved_pose"));
      target_subscribers_[index] = session_.declare_subscriber(
        zenoh::KeyExpr("tianji/target/arm/" + side),
        [this, index](const zenoh::Sample &sample) { on_target(index, sample.get_payload().as_string()); }, []() {});
      command_subscribers_[index] = session_.declare_subscriber(
        zenoh::KeyExpr("tianji/command/arm/" + side),
        [this, index](const zenoh::Sample &sample) { on_command(index, sample.get_payload().as_string()); }, []() {});
    }
    liveliness_token_ = session_.liveliness_declare_token(zenoh::KeyExpr("tj/live/producer/arm/arm_ik_producer/" + instance_));
    publish_status("");
  }

  void run() {
    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    auto next_tick = Clock::now();
    while (true) {
      next_tick += std::chrono::duration_cast<Clock::duration>(period);
      tick();
      std::this_thread::sleep_until(next_tick);
    }
  }

private:
  void on_target(std::size_t index, const std::string &payload) {
    try {
      auto parsed = JsonTargetParser(payload).parse();
      const std::string expected_side = index == 0 ? "left" : "right";
      if (parsed.side != expected_side) throw std::invalid_argument("target side does not match topic");
      if (parsed.router != router_) throw std::invalid_argument("target router mismatch");
      if (parsed.source != source_ || parsed.instance != source_instance_) {
        throw std::invalid_argument("target source authority mismatch");
      }
      if (parsed.timestamp_ns > now_ns()) throw std::invalid_argument("target timestamp is in the future");
      if (now_ns() - parsed.timestamp_ns > freshness_timeout_ns_) {
        throw std::invalid_argument("target is stale");
      }
      std::lock_guard<std::mutex> lock(mutex_);
      if (last_target_baseline_[index].has_value()) {
        const auto [instance, sequence] = *last_target_baseline_[index];
        if (parsed.instance != instance || parsed.sequence <= sequence) {
          throw std::invalid_argument("target source instance/sequence rollback");
        }
      }
      last_target_baseline_[index] = std::make_pair(parsed.instance, parsed.sequence);
      targets_[index] = std::move(parsed);
      target_input_errors_[index].reset();
    } catch (const std::exception &error) {
      const std::string message = std::string("target rejected: ") + error.what();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        target_input_errors_[index] = message;
      }
      publish_status(message, false, false);
    }
  }

  void on_command(std::size_t index, const std::string &payload) {
    try {
      const std::string expected_side = index == 0 ? "left" : "right";
      const auto command = JsonCommandParser(payload).parse(coordinator_, router_, expected_side, joint_names_[index]);
      if (
        (command.joints.array() < joint_limits_.lower_position.array()).any() ||
        (command.joints.array() > joint_limits_.upper_position.array()).any())
      {
        throw std::invalid_argument("final command is outside hard joint limits");
      }
      std::lock_guard<std::mutex> lock(mutex_);
      if (last_command_sequence_[index].has_value() && command.sequence <= *last_command_sequence_[index]) {
        throw std::invalid_argument("final command sequence rollback");
      }
      last_command_sequence_[index] = command.sequence;
      current_[index] = command.joints;
      const bool resynchronize =
        command.mode != "teleop" || !trajectory_limiters_[index].initialized() ||
        (trajectory_limiters_[index].state().position - command.joints)
        .cwiseAbs().maxCoeff() > 2.0 * settings_.maximum_joint_step_rad;
      if (resynchronize) {
        ArmMotionState state;
        state.position = command.joints;
        if (!trajectory_limiters_[index].reset(state)) {
          throw std::invalid_argument("unable to reset Ruckig from final command");
        }
        failure_windows_[index].recovered();
      }
      command_input_errors_[index].reset();
    } catch (const std::exception &error) {
      const std::string message = std::string("current command rejected: ") + error.what();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        command_input_errors_[index] = message;
      }
      publish_status(message, false, false);
    }
  }

  void tick() {
    const auto tick_ns = now_ns();
    std::string status_error;
    bool healthy = true;
    bool degraded = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (std::size_t index = 0; index < 2; ++index) {
        const auto & input_error = target_input_errors_[index].has_value() ?
          target_input_errors_[index] : command_input_errors_[index];
        if (input_error.has_value()) {
          healthy = false;
          if (status_error.empty()) status_error = *input_error;
        }
      }
      for (std::size_t index = 0; index < 2; ++index) {
        if (!targets_[index].has_value()) continue;
        const auto &target = *targets_[index];
        if (tick_ns - target.timestamp_ns > kFreshnessNs) continue;
        const auto side = index == 0 ? ArmSide::kLeft : ArmSide::kRight;
        const auto reject = [&](const std::string & detail, bool hard_failure) {
          const bool continuous = hard_failure || failure_windows_[index].failed(tick_ns);
          healthy = healthy && !continuous;
          degraded = true;
          if (status_error.empty()) {
            std::ostringstream message;
            message << (hard_failure ? "hard IK safety reject" :
              (continuous ? "continuous IK reject" : "transient IK reject; holding final command"))
                    << ": side=" << target.side << " detail=" << detail
                    << " failures=" << failure_windows_[index].count()
                    << " age_ms=" << failure_windows_[index].age_ns(tick_ns) / 1000000.0;
            status_error = message.str();
          }
          ArmMotionState hold_state;
          hold_state.position = current_[index];
          (void)trajectory_limiters_[index].reset(hold_state);
          IkResult hold_result;
          try {
            hold_result.achieved_pose = solver_->forward(side, current_[index]);
            publish_proposal(index, target, hold_result, current_[index], hold_result.achieved_pose, true);
          } catch (const std::exception &) {
            healthy = false;
          }
        };

        IkResult result;
        try {
          result = solver_->solve(side, target.pose, current_[index], target.elbow);
        } catch (const std::exception & error) {
          reject(std::string("solver exception: ") + error.what(), false);
          continue;
        }
        if (!result.joints_rad.allFinite() || !result.achieved_pose.matrix().allFinite()) {
          reject("solver returned non-finite output", true);
          continue;
        }
        if (!result.accepted) {
          reject("solver status=" + (result.status.empty() ? std::string("rejected") : result.status),
            result.status == "qp_contract_violation");
          continue;
        }
        if (
          (result.joints_rad.array() < joint_limits_.lower_position.array()).any() ||
          (result.joints_rad.array() > joint_limits_.upper_position.array()).any())
        {
          reject("QP output is outside hard joint limits", true);
          continue;
        }

        const ArmJointVector target_velocity =
          (result.joints_rad - current_[index]) / settings_.control_period_s;
        const auto limited = trajectory_limiters_[index].update(target_velocity);
        if (!limited.accepted) {
          reject(std::string(limited.detail), limited.hard_failure);
          continue;
        }
        const double step =
          (limited.state.position - current_[index]).cwiseAbs().maxCoeff();
        if (!std::isfinite(step)) {
          reject("Ruckig output step is non-finite", true);
          continue;
        }
        if (step > settings_.maximum_joint_step_rad + 1.0e-10) {
          reject("Ruckig output exceeds distributed command step", false);
          continue;
        }

        Eigen::Isometry3d achieved;
        try {
          achieved = solver_->forward(side, limited.state.position);
        } catch (const std::exception & error) {
          reject(std::string("FK exception: ") + error.what(), true);
          continue;
        }
        if (!achieved.matrix().allFinite()) {
          reject("FK returned non-finite output", true);
          continue;
        }
        failure_windows_[index].recovered();
        result.joints_rad = limited.state.position;
        result.achieved_pose = achieved;
        result.converged = result.converged &&
          (limited.state.position - current_[index]).cwiseAbs().maxCoeff() < 1.0e-8;
        publish_proposal(index, target, result, limited.state.position, achieved, false);
      }
    }
    publish_status(status_error, healthy, degraded);
  }

  void publish_proposal(
    std::size_t index,
    const Target & target,
    const IkResult & result,
    const ArmJointVector & joints,
    const Eigen::Isometry3d & achieved,
    bool hold)
  {
    std::lock_guard<std::mutex> publish_lock(publish_mutex_);
    const auto wire_sequence = sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
    proposal_publishers_[index]->put(zenoh::Bytes(proposal_json(
      target, result, joints, joint_names_[index], instance_, router_, wire_sequence, hold)));
    solved_publishers_[index]->put(zenoh::Bytes(solved_json(
      target, achieved, instance_, router_, wire_sequence)));
  }

  void publish_status(
    const std::string &error,
    bool healthy = true,
    bool degraded = false)
  {
    std::lock_guard<std::mutex> publish_lock(publish_mutex_);
    if (!status_publisher_) return;
    const std::string signature = error.empty() ? "healthy" :
      (healthy ? "degraded" : "unhealthy");
    if (signature != last_status_signature_) {
      if (!error.empty()) {
        std::cerr << "arm_ik_producer " << (healthy ? "degraded" : "unhealthy")
                  << ": " << error << std::endl;
      } else if (!last_status_signature_.empty() && last_status_signature_ != "healthy") {
        std::cerr << "arm_ik_producer recovered" << std::endl;
      }
      last_status_signature_ = signature;
    }
    const auto wire_sequence = sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
    const auto capabilities = real_capability_ ? "[\"simulation\",\"real\"]" : "[\"simulation\"]";
    status_publisher_->put(zenoh::Bytes("{\"schema_version\":1,\"publisher_instance_id\":" + quote(instance_) + ",\"router_zid\":" + quote(router_) +
      ",\"sequence\":" + std::to_string(wire_sequence) + ",\"timestamp_ns\":" + std::to_string(now_ns()) +
      ",\"component_role\":\"producer_arm\",\"component_id\":\"arm_ik_producer\",\"phase\":\"ready\",\"ready\":true,\"healthy\":" +
      (healthy ? "true" : "false") + ",\"capabilities\":" + capabilities + ",\"error\":" + (error.empty() ? "null" : quote(error)) +
      ",\"diagnostics\":{\"backend\":" + quote(backend_) + ",\"degraded\":" + (degraded ? "true" : "false") +
      ",\"rate_hz\":" + number(rate_hz_) + "}}"));
  }

  zenoh::Session &session_;
  std::string backend_, instance_, router_, coordinator_, source_, source_instance_;
  IkSettings settings_{};
  std::unique_ptr<ArmIkSolver> solver_;
  ArmJointNames joint_names_{};
  JointTrajectoryLimits joint_limits_;
  std::array<JointTrajectoryLimiter7, 2> trajectory_limiters_;
  std::array<ConsecutiveFailureWindow, 2> failure_windows_;
  std::array<std::optional<zenoh::Publisher>, 2> proposal_publishers_;
  std::array<std::optional<std::pair<std::string, std::uint64_t>>, 2> last_target_baseline_;
  std::array<std::optional<zenoh::Publisher>, 2> solved_publishers_;
  std::array<std::optional<zenoh::Subscriber<void>>, 2> target_subscribers_;
  std::array<std::optional<zenoh::Subscriber<void>>, 2> command_subscribers_;
  std::array<std::optional<std::uint64_t>, 2> last_command_sequence_;
  std::array<std::optional<std::string>, 2> target_input_errors_;
  std::array<std::optional<std::string>, 2> command_input_errors_;
  std::optional<zenoh::LivelinessToken> liveliness_token_;
  std::optional<zenoh::Publisher> status_publisher_;
  std::mutex publish_mutex_;
  std::string last_status_signature_;
  std::array<std::optional<Target>, 2> targets_;
  std::array<ArmJointVector, 2> current_{ArmJointVector::Zero(), ArmJointVector::Zero()};
  std::mutex mutex_;
  std::atomic<std::uint64_t> sequence_{0};
  double rate_hz_{200.0};
  std::int64_t freshness_timeout_ns_{500000000};
  bool real_capability_{false};
};

}  // namespace
}  // namespace tianji_teleop

int main() {
  try {
    const auto read_env = [](const char *name, const std::string &fallback = std::string()) {
      const char *value = std::getenv(name);
      return value == nullptr ? fallback : std::string(value);
    };
    const auto instance = read_env("TIANJI_COMPONENT_INSTANCE_ID");
    const auto coordinator = read_env("TIANJI_COORDINATOR_INSTANCE_ID");
    const auto router = read_env("TIANJI_ROUTER_ZID");
    const auto source = read_env("TIANJI_SOURCE_LOGICAL_ID", "source");
    const auto source_instance = read_env("TIANJI_SOURCE_INSTANCE_ID");
    if (instance.empty() || coordinator.empty() || router.empty() || source_instance.empty()) {
      throw std::invalid_argument("component, coordinator, router and source identities are required");
    }
    const auto endpoint = read_env("TIANJI_ROUTER_ENDPOINT", "tcp/127.0.0.1:7447");
    const auto arm_config_path = read_env("TIANJI_ARM_CONFIG");
    const auto arm_config = tianji_teleop::load_arm_config(arm_config_path);
    const double rate_hz = tianji_teleop::env_double("TIANJI_IK_RATE_HZ", 200.0);
    const double freshness_timeout_s = tianji_teleop::env_double("TIANJI_IK_FRESHNESS_TIMEOUT_S", 0.5);
    const double reject_grace_s = tianji_teleop::env_double("TIANJI_IK_SOLVER_REJECT_GRACE_S", 0.15);
    tianji_teleop::IkSettings settings;
    settings.maximum_joint_step_rad = tianji_teleop::env_double("TIANJI_IK_MAXIMUM_JOINT_STEP_RAD", settings.maximum_joint_step_rad);
    settings.position_tolerance_m = tianji_teleop::env_double("TIANJI_IK_POSITION_TOLERANCE_M", settings.position_tolerance_m);
    settings.orientation_tolerance_rad = tianji_teleop::env_double("TIANJI_IK_ORIENTATION_TOLERANCE_RAD", settings.orientation_tolerance_rad);
    settings.control_period_s = 1.0 / rate_hz;
    settings.qp_joint_velocity_limits_rad_s = tianji_teleop::env_vector(
      "TIANJI_IK_QP_JOINT_VELOCITY_LIMITS_RAD_S", settings.qp_joint_velocity_limits_rad_s);
    settings.official_worker_timeout_ms = static_cast<int>(tianji_teleop::env_double("TIANJI_IK_WORKER_TIMEOUT_MS", settings.official_worker_timeout_ms));
    settings.official_worker_restart_attempts = static_cast<int>(tianji_teleop::env_double("TIANJI_IK_WORKER_RESTART_ATTEMPTS", settings.official_worker_restart_attempts));
    if (settings.maximum_joint_step_rad <= 0.0 || settings.position_tolerance_m <= 0.0 ||
        settings.orientation_tolerance_rad <= 0.0 || settings.official_worker_timeout_ms <= 0 ||
        settings.official_worker_restart_attempts < 0 || reject_grace_s <= 0.0) {
      throw std::invalid_argument("invalid canonical IK producer settings");
    }
    tianji_teleop::JointTrajectoryLimits trajectory_limits;
    trajectory_limits.lower_position = arm_config.lower_limits;
    trajectory_limits.upper_position = arm_config.upper_limits;
    tianji_teleop::ArmJointVector default_velocity;
    default_velocity << 0.8, 0.8, 1.0, 1.0, 1.2, 1.2, 1.2;
    tianji_teleop::ArmJointVector default_acceleration;
    default_acceleration << 7.854, 7.854, 15.708, 15.708, 15.708, 15.708, 15.708;
    tianji_teleop::ArmJointVector default_jerk;
    default_jerk << 600.0, 600.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0;
    trajectory_limits.maximum_velocity = tianji_teleop::env_vector(
      "TIANJI_IK_RUCKIG_MAX_VELOCITY_RAD_S", default_velocity);
    trajectory_limits.maximum_acceleration = tianji_teleop::env_vector(
      "TIANJI_IK_RUCKIG_MAX_ACCELERATION_RAD_S2", default_acceleration);
    trajectory_limits.maximum_jerk = tianji_teleop::env_vector(
      "TIANJI_IK_RUCKIG_MAX_JERK_RAD_S3", default_jerk);
    trajectory_limits.validation_tolerance = tianji_teleop::env_double(
      "TIANJI_IK_RUCKIG_VALIDATION_TOLERANCE", 1.0e-8);
    if (endpoint.find('\"') != std::string::npos) throw std::invalid_argument("invalid router endpoint");
    auto config = zenoh::Config::create_default();
    config.insert_json5("mode", "\"client\"");
    config.insert_json5("connect/endpoints", "[\"" + endpoint + "\"]");
    zenoh::Session session = zenoh::Session::open(std::move(config));
    const auto routers = session.get_routers_z_id();
    if (routers.size() != 1 || routers.front().to_string() != router) {
      throw std::runtime_error("expected exactly one router with matching TIANJI_ROUTER_ZID");
    }
    tianji_teleop::ArmIkProducer node(
      session, read_env("TIANJI_IK_BACKEND", "pinocchio_qp"), instance, router,
      coordinator, source, source_instance, arm_config.names, settings,
      trajectory_limits, rate_hz, freshness_timeout_s, reject_grace_s);
    node.run();
  } catch (const std::exception &error) {
    std::cerr << "arm_ik_producer failed: " << error.what() << std::endl;
    return 1;
  }
  return 0;
}
