#include "pico_body_tianji/ik/arm_ik_factory.hpp"

#include <zenoh.hxx>
#include <Eigen/Geometry>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
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
  for (std::size_t i = 0; i < array.size(); ++i) {
    if (i != 0) out << ',';
    out << number(array[i]);
  }
  out << ']';
  return out.str();
}

struct Target {
  std::string instance;
  std::string router;
  std::string side;
  std::string frame;
  std::uint64_t sequence{0};
  std::int64_t timestamp_ns{0};
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d elbow{Eigen::Vector3d::UnitZ()};
};

// The producer accepts only the small, typed subset required by ArmTargetCommand.
// It intentionally rejects malformed/unknown geometry instead of guessing fields.
class JsonTargetParser {
public:
  explicit JsonTargetParser(const std::string &text) : text_(text) {}

  Target parse() const {
    if (integer_field("schema_version") != 1) throw std::invalid_argument("unsupported arm target schema");
    Target target;
    target.instance = string_field("publisher_instance_id");
    (void)string_field("source");
    target.side = string_field("side");
    target.frame = string_field("frame_id");
    target.sequence = integer_field("sequence");
    target.timestamp_ns = static_cast<std::int64_t>(integer_field("timestamp_ns"));
    if (target.side != "left" && target.side != "right") throw std::invalid_argument("invalid arm side");
    if (target.frame != (target.side == "left" ? "Base_L" : "Base_R")) throw std::invalid_argument("arm frame does not match side");
    const auto p = vector_field("position_m", 3);
    const auto q = vector_field4("orientation_xyzw");
    const auto e = vector_field("elbow_reference_direction", 3);
    const double qnorm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    const double enorm = std::sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
    if (!std::isfinite(qnorm) || qnorm < 0.999 || qnorm > 1.001) throw std::invalid_argument("invalid quaternion norm");
    if (!std::isfinite(enorm) || enorm < 1e-8) throw std::invalid_argument("invalid elbow direction");
    target.pose.translation() = Eigen::Vector3d(p[0], p[1], p[2]);
    target.pose.linear() = Eigen::Quaterniond(q[3], q[0], q[1], q[2]).normalized().toRotationMatrix();
    target.elbow = Eigen::Vector3d(e[0], e[1], e[2]) / enorm;
    return target;
  }

private:
  std::string field_text(const std::string &field) const {
    const std::string needle = quote(field) + ":";
    const auto start = text_.find(needle);
    if (start == std::string::npos) throw std::invalid_argument("missing field " + field);
    auto begin = start + needle.size();
    while (begin < text_.size() && std::isspace(static_cast<unsigned char>(text_[begin]))) ++begin;
    if (begin >= text_.size()) throw std::invalid_argument("missing value " + field);
    auto end = begin;
    if (text_[begin] == '"') {
      ++end;
      while (end < text_.size() && text_[end] != '"') ++end;
      if (end == text_.size()) throw std::invalid_argument("unterminated string " + field);
      return text_.substr(begin, end - begin + 1);
    }
    if (text_[begin] == '[' || text_[begin] == '{') {
      const char opening = text_[begin];
      const char closing = opening == '[' ? ']' : '}';
      int depth = 0;
      bool quoted = false;
      bool escaped = false;
      for (end = begin; end < text_.size(); ++end) {
        const char c = text_[end];
        if (quoted) {
          if (escaped) escaped = false;
          else if (c == '\\') escaped = true;
          else if (c == '"') quoted = false;
        } else if (c == '"') {
          quoted = true;
        } else if (c == opening) {
          ++depth;
        } else if (c == closing && --depth == 0) {
          ++end;
          break;
        }
      }
      if (depth != 0) throw std::invalid_argument("unterminated array " + field);
      return text_.substr(begin, end - begin);
    }
    while (end < text_.size() && text_[end] != ',' && text_[end] != '}') ++end;
    while (end > begin && std::isspace(static_cast<unsigned char>(text_[end - 1]))) --end;
    return text_.substr(begin, end - begin);
  }

  std::string string_field(const std::string &field) const {
    const auto raw = field_text(field);
    if (raw.size() < 2 || raw.front() != '"' || raw.back() != '"') throw std::invalid_argument("field is not string " + field);
    return raw.substr(1, raw.size() - 2);
  }

  std::uint64_t integer_field(const std::string &field) const {
    const auto raw = field_text(field);
    std::size_t used = 0;
    const auto value = std::stoull(raw, &used);
    if (used != raw.size()) throw std::invalid_argument("field is not integer " + field);
    return value;
  }

  std::array<double, 4> vector4(const std::string &raw) const {
    std::array<double, 4> out{};
    std::size_t pos = 0;
    for (double &value : out) {
      while (pos < raw.size() && (std::isspace(static_cast<unsigned char>(raw[pos])) || raw[pos] == ',' || raw[pos] == '[')) ++pos;
      std::size_t used = 0;
      value = std::stod(raw.substr(pos), &used);
      if (!std::isfinite(value)) throw std::invalid_argument("nonfinite vector");
      pos += used;
    }
    return out;
  }

  std::array<double, 3> vector3(const std::string &raw) const {
    std::array<double, 3> out{};
    std::size_t pos = 0;
    for (double &value : out) {
      while (pos < raw.size() && (std::isspace(static_cast<unsigned char>(raw[pos])) || raw[pos] == ',' || raw[pos] == '[')) ++pos;
      std::size_t used = 0;
      value = std::stod(raw.substr(pos), &used);
      if (!std::isfinite(value)) throw std::invalid_argument("nonfinite vector");
      pos += used;
    }
    return out;
  }

  std::array<double, 3> vector_field(const std::string &field, std::size_t size) const {
    const auto raw = field_text(field);
    if (raw.empty() || raw.front() != '[' || size != 3) throw std::invalid_argument("invalid vector " + field);
    return vector3(raw);
  }

  std::array<double, 4> vector_field4(const std::string &field) const {
    const auto raw = field_text(field);
    if (raw.empty() || raw.front() != '[') throw std::invalid_argument("invalid vector " + field);
    return vector4(raw);
  }


  const std::string &text_;
};

std::string proposal_json(const Target &target, const IkResult &result, const ArmJointVector &joints, const std::string &instance, const std::string &router, std::uint64_t sequence) {
  std::ostringstream out;
  out << "{\"schema_version\":1,\"publisher_instance_id\":" << quote(instance)
      << ",\"router_zid\":" << quote(router) << ",\"sequence\":" << sequence
      << ",\"timestamp_ns\":" << now_ns() << ",\"producer\":\"arm_ik_producer\",\"side\":" << quote(target.side)
      << ",\"target_sequence\":" << target.sequence << ",\"names\":[";
  for (int i = 0; i < 7; ++i) { if (i) out << ','; out << quote("Joint" + std::to_string(i + 1) + (target.side == "left" ? "_L" : "_R")); }
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
  ArmIkProducer(zenoh::Session &session, std::string backend, std::string instance, std::string router, std::string coordinator)
  : session_(session), backend_(std::move(backend)), instance_(std::move(instance)), router_(std::move(router)), coordinator_(std::move(coordinator)) {
    ArmIkBackendOptions options;
    options.urdf_path = env_or("TIANJI_ARM_URDF");
    options.official_library_path = env_or("TIANJI_OFFICIAL_IK_LIBRARY");
    options.official_config_path = env_or("TIANJI_OFFICIAL_IK_CONFIG");
    solver_ = create_arm_ik_solver(backend_, options, settings_);
    for (const std::string side : {"left", "right"}) {
      const auto index = side == "left" ? 0U : 1U;
      proposal_publishers_[index] = session_.declare_publisher(zenoh::KeyExpr("tianji/proposal/arm/" + side));
      solved_publishers_[index] = session_.declare_publisher(zenoh::KeyExpr("tianji/producer/arm/" + side + "/solved_pose"));
      target_subscribers_[index] = session_.declare_subscriber(
        zenoh::KeyExpr("tianji/target/arm/" + side),
        [this, index](const zenoh::Sample &sample) { on_target(index, sample.get_payload().as_string()); }, []() {});
    }
    command_subscriber_ = session_.declare_subscriber(
      "tianji/command/arm/**", [this](const zenoh::Sample &sample) { on_command(sample.get_payload().as_string()); }, []() {});
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

  void on_command(const std::string &payload) {
    try {
      const auto side = payload.find("\"side\":\"right\"") != std::string::npos ? 1U : 0U;
      const auto marker = payload.find("\"position_rad\":[");
      if (payload.find("\"publisher_instance_id\":" + quote(coordinator_)) == std::string::npos ||
          payload.find("\"router_zid\":" + quote(router_)) == std::string::npos) {
        throw std::invalid_argument("final command authority mismatch");
      }
      const auto raw = payload.substr(marker + 17);
      std::size_t pos = 0;
      ArmJointVector joints = ArmJointVector::Zero();
      for (int i = 0; i < 7; ++i) {
        while (pos < raw.size() && (raw[pos] == '[' || raw[pos] == ',' || std::isspace(static_cast<unsigned char>(raw[pos])))) ++pos;
        std::size_t used = 0;
        joints[i] = std::stod(raw.substr(pos), &used);
        if (!std::isfinite(joints[i])) throw std::invalid_argument("nonfinite current joints");
        pos += used;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      current_[side] = joints;
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
      current_[index] = result.joints_rad;
      ++sequence_;
      proposal_publishers_[index]->put(zenoh::Bytes(proposal_json(target, result, result.joints_rad, instance_, router_, sequence_)));
      solved_publishers_[index]->put(zenoh::Bytes(solved_json(target, result.achieved_pose, instance_, router_, sequence_)));
    }
  }

  void publish_status(const std::string &error) {
    if (!status_publisher_) return;
    status_publisher_->put(zenoh::Bytes("{\"schema_version\":1,\"publisher_instance_id\":" + quote(instance_) + ",\"router_zid\":" + quote(router_) +
      ",\"sequence\":" + std::to_string(++sequence_) + ",\"timestamp_ns\":" + std::to_string(now_ns()) +
      ",\"component_role\":\"producer_arm\",\"component_id\":\"arm_ik_producer\",\"phase\":\"ready\",\"ready\":true,\"healthy\":" +
      (error.empty() ? "true" : "false") + ",\"capabilities\":[\"simulation\"],\"error\":" + (error.empty() ? "null" : quote(error)) + ",\"diagnostics\":{}}"));
  }

  zenoh::Session &session_;
  std::string backend_, instance_, router_, coordinator_;
  IkSettings settings_{};
  std::unique_ptr<ArmIkSolver> solver_;
  std::array<std::optional<zenoh::Publisher>, 2> proposal_publishers_;
  std::array<std::optional<std::uint64_t>, 2> last_target_sequence_;
  std::array<std::optional<zenoh::Publisher>, 2> solved_publishers_;
  std::array<std::optional<zenoh::Subscriber<void>>, 2> target_subscribers_;
  std::optional<zenoh::Subscriber<void>> command_subscriber_;
  std::optional<zenoh::LivelinessToken> liveliness_token_;
  std::optional<zenoh::Publisher> status_publisher_;
  std::array<std::optional<Target>, 2> targets_;
  std::array<ArmJointVector, 2> current_{ArmJointVector::Zero(), ArmJointVector::Zero()};
  std::mutex mutex_;
  std::uint64_t sequence_{0};
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
      session, read_env("TIANJI_IK_BACKEND", "pinocchio_cpp"), instance, router, coordinator);
    node.run();
  } catch (const std::exception &error) {
    std::cerr << "arm_ik_producer failed: " << error.what() << std::endl;
    return 1;
  }
  return 0;
}
