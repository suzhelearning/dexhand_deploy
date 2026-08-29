#include "pico_body_tianji/ik/arm_ik_factory.hpp"
#include "pico_body_tianji/protocol/json_parser.hpp"

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
#include <string>
#include <thread>

namespace pico_body_tianji {
namespace {

using Clock = std::chrono::steady_clock;
constexpr std::int64_t kFreshnessNs = 500000000;

std::string env_or(const char *name, const std::string &fallback = {}) {
  const char *value = std::getenv(name);
  return value == nullptr ? fallback : std::string(value);
}

std::int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}

using ArmJointNames = std::array<std::array<std::string, 7>, 2>;

ArmJointNames load_arm_joint_names(const std::string &path) {
  if (path.empty()) throw std::invalid_argument("TIANJI_ARM_CONFIG is required");
  std::ifstream file(path);
  if (!file) throw std::invalid_argument("unable to read TIANJI_ARM_CONFIG: " + path);
  const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  ArmJointNames result;
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
      result[side][index] = line.substr(begin, finish - begin);
      if (result[side][index] != "Joint" + std::to_string(index + 1) + (side == 0 ? "_L" : "_R")) {
        throw std::invalid_argument("arm config joint order mismatch");
      }
      cursor = end;
    }
  }
  for (const std::string field : {"left_home_rad:", "right_home_rad:", "lower_limits_rad:", "upper_limits_rad:"}) {
    const auto start = text.find(field);
    const auto open = start == std::string::npos ? std::string::npos : text.find('[', start);
    const auto close = open == std::string::npos ? std::string::npos : text.find(']', open);
    if (open == std::string::npos || close == std::string::npos) throw std::invalid_argument("arm config vector missing " + field);
    std::size_t count = 0;
    std::size_t cursor = open + 1;
    while (cursor < close) {
      while (cursor < close && (std::isspace(static_cast<unsigned char>(text[cursor])) || text[cursor] == ',')) ++cursor;
      if (cursor >= close) break;
      std::size_t used = 0;
      const auto value = std::stod(text.substr(cursor, close - cursor), &used);
      if (!std::isfinite(value)) throw std::invalid_argument("arm config vector contains non-finite value");
      cursor += used;
      ++count;
    }
    if (count != 7) throw std::invalid_argument("arm config vector must contain seven values");
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
    const auto mode = protocol::field(root, "mode").as_string("mode");
    if (mode != "idle" && mode != "teleop" && mode != "returning") {
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

std::string proposal_json(const Target &target, const IkResult &result, const ArmJointVector &joints, const std::array<std::string, 7> &names, const std::string &instance, const std::string &router, std::uint64_t sequence) {
  std::ostringstream out;
  out << "{\"schema_version\":1,\"publisher_instance_id\":" << quote(instance)
      << ",\"router_zid\":" << quote(router) << ",\"sequence\":" << sequence
      << ",\"timestamp_ns\":" << now_ns() << ",\"producer\":\"arm_ik_producer\",\"side\":" << quote(target.side)
      << ",\"target_sequence\":" << target.sequence << ",\"names\":[";
  for (std::size_t i = 0; i < names.size(); ++i) { if (i) out << ','; out << quote(names[i]); }
  out << "],\"position_rad\":" << array_json(joints) << ",\"diagnostics\":{\"accepted\":true,\"converged\":" << (result.converged ? "true" : "false") << "}}";
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
  ArmIkProducer(zenoh::Session &session, std::string backend, std::string instance, std::string router, std::string coordinator, ArmJointNames names)
  : session_(session), backend_(std::move(backend)), instance_(std::move(instance)), router_(std::move(router)), coordinator_(std::move(coordinator)) {
    joint_names_ = std::move(names);
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
    // Do not expose an authority until every input and output is declared.
    // If any declaration above throws, the status publisher is destroyed with
    // this object and no liveliness token or ready status has escaped.
    liveliness_token_ = session_.liveliness_declare_token(zenoh::KeyExpr("tj/live/producer/arm/arm_ik_producer/" + instance_));
    publish_status("");
  }

  void run() {
    while (true) {
      tick();
      std::this_thread::sleep_for(std::chrono::milliseconds(11));
    }
  }

private:
  void on_target(std::size_t index, const std::string &payload) {
    try {
      auto parsed = JsonTargetParser(payload).parse();
      const std::string expected_side = index == 0 ? "left" : "right";
      if (parsed.side != expected_side) throw std::invalid_argument("target side does not match topic");
      if (parsed.router != router_) throw std::invalid_argument("target router mismatch");
      if (parsed.timestamp_ns > now_ns()) throw std::invalid_argument("target timestamp is in the future");
      std::lock_guard<std::mutex> lock(mutex_);
      if (last_target_sequence_[index].has_value() && parsed.sequence <= *last_target_sequence_[index]) {
        throw std::invalid_argument("target sequence rollback");
      }
      last_target_sequence_[index] = parsed.sequence;
      targets_[index] = std::move(parsed);
    } catch (const std::exception &error) {
      publish_status(std::string("target rejected: ") + error.what());
    }
  }

  void on_command(std::size_t index, const std::string &payload) {
    try {
      const std::string expected_side = index == 0 ? "left" : "right";
      const auto command = JsonCommandParser(payload).parse(coordinator_, router_, expected_side, joint_names_[index]);
      std::lock_guard<std::mutex> lock(mutex_);
      if (last_command_sequence_[index].has_value() && command.sequence <= *last_command_sequence_[index]) {
        throw std::invalid_argument("final command sequence rollback");
      }
      last_command_sequence_[index] = command.sequence;
      current_[index] = command.joints;
    } catch (const std::exception &error) {
      publish_status(std::string("current command rejected: ") + error.what());
    }
  }

  void tick() {
    publish_status("");
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t index = 0; index < 2; ++index) {
      if (!targets_[index].has_value()) continue;
      const auto &target = *targets_[index];
      if (now_ns() - target.timestamp_ns > kFreshnessNs) continue;
      const auto side = index == 0 ? ArmSide::kLeft : ArmSide::kRight;
      const auto result = solver_->solve(side, target.pose, current_[index], target.elbow);
      if (!result.accepted || !result.joints_rad.allFinite()) {
        publish_status("solver rejected target");
        continue;
      }
      const double step = (result.joints_rad - current_[index]).cwiseAbs().maxCoeff();
      if (!std::isfinite(step) || step > settings_.maximum_joint_step_rad) {
        publish_status("solver result exceeds maximum_joint_step_rad");
        continue;
      }
      {
        std::lock_guard<std::mutex> publish_lock(publish_mutex_);
        const auto wire_sequence = sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
        const auto proposal = proposal_json(target, result, result.joints_rad, joint_names_[index], instance_, router_, wire_sequence);
        const auto solved = solved_json(target, result.achieved_pose, instance_, router_, wire_sequence);
        proposal_publishers_[index]->put(zenoh::Bytes(proposal));
        solved_publishers_[index]->put(zenoh::Bytes(solved));
      }
    }
  }

  void publish_status(const std::string &error) {
    std::lock_guard<std::mutex> publish_lock(publish_mutex_);
    if (!status_publisher_) return;
    const auto wire_sequence = sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
    status_publisher_->put(zenoh::Bytes("{\"schema_version\":1,\"publisher_instance_id\":" + quote(instance_) + ",\"router_zid\":" + quote(router_) +
      ",\"sequence\":" + std::to_string(wire_sequence) + ",\"timestamp_ns\":" + std::to_string(now_ns()) +
      ",\"component_role\":\"producer_arm\",\"component_id\":\"arm_ik_producer\",\"phase\":\"ready\",\"ready\":true,\"healthy\":" +
      (error.empty() ? "true" : "false") + ",\"capabilities\":[\"simulation\"],\"error\":" + (error.empty() ? "null" : quote(error)) + ",\"diagnostics\":{}}"));
  }

  zenoh::Session &session_;
  std::string backend_, instance_, router_, coordinator_;
  IkSettings settings_{};
  std::unique_ptr<ArmIkSolver> solver_;
  ArmJointNames joint_names_{};
  std::array<std::optional<zenoh::Publisher>, 2> proposal_publishers_;
  std::array<std::optional<std::uint64_t>, 2> last_target_sequence_;
  std::array<std::optional<zenoh::Publisher>, 2> solved_publishers_;
  std::array<std::optional<zenoh::Subscriber<void>>, 2> target_subscribers_;
  std::array<std::optional<zenoh::Subscriber<void>>, 2> command_subscribers_;
  std::array<std::optional<std::uint64_t>, 2> last_command_sequence_;
  std::optional<zenoh::LivelinessToken> liveliness_token_;
  std::optional<zenoh::Publisher> status_publisher_;
  std::mutex publish_mutex_;
  std::array<std::optional<Target>, 2> targets_;
  std::array<ArmJointVector, 2> current_{ArmJointVector::Zero(), ArmJointVector::Zero()};
  std::mutex mutex_;
  std::atomic<std::uint64_t> sequence_{0};
};

}  // namespace
}  // namespace pico_body_tianji

int main() {
  try {
    const auto read_env = [](const char *name, const std::string &fallback = std::string()) {
      const char *value = std::getenv(name);
      return value == nullptr ? fallback : std::string(value);
    };
    const auto instance = read_env("TIANJI_COMPONENT_INSTANCE_ID");
    const auto coordinator = read_env("TIANJI_COORDINATOR_INSTANCE_ID");
    const auto router = read_env("TIANJI_ROUTER_ZID");
    if (instance.empty() || coordinator.empty() || router.empty()) {
      throw std::invalid_argument("TIANJI_COMPONENT_INSTANCE_ID, TIANJI_COORDINATOR_INSTANCE_ID and TIANJI_ROUTER_ZID are required");
    }
    const auto endpoint = read_env("TIANJI_ROUTER_ENDPOINT", "tcp/127.0.0.1:7447");
    const auto arm_config_path = read_env("TIANJI_ARM_CONFIG");
    const auto joint_names = pico_body_tianji::load_arm_joint_names(arm_config_path);
    if (endpoint.find('\"') != std::string::npos) throw std::invalid_argument("invalid router endpoint");
    auto config = zenoh::Config::create_default();
    config.insert_json5("mode", "\"client\"");
    config.insert_json5("connect/endpoints", "[\"" + endpoint + "\"]");
    zenoh::Session session = zenoh::Session::open(std::move(config));
    const auto routers = session.get_routers_z_id();
    if (routers.size() != 1 || routers.front().to_string() != router) {
      throw std::runtime_error("expected exactly one router with matching TIANJI_ROUTER_ZID");
    }
    pico_body_tianji::ArmIkProducer node(
      session, read_env("TIANJI_IK_BACKEND", "pinocchio_cpp"), instance, router, coordinator, joint_names);
    node.run();
  } catch (const std::exception &error) {
    std::cerr << "arm_ik_producer failed: " << error.what() << std::endl;
    return 1;
  }
  return 0;
}
