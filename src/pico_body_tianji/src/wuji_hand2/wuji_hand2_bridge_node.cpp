/* Canonical Wuji Hand 2 executor.
 *
 * retarget: tianji/target/hand/{side} -> tianji/command/hand/{side}
 * direct:   tianji/command/hand/{side} (authorized publisher only)
 * state/status/safety all use protocol v1 JSON. The executor never publishes
 * SessionState or an arm/final command authority.
 */
#include "pico_body_tianji/protocol/json_parser.hpp"
#include "pico_body_tianji/wuji_hand2/wuji_hand2_control.hpp"

#include <zenoh.hxx>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace pico_body_tianji {
namespace {

using protocol::JsonValue;
using protocol::StrictJsonParser;
using pico_body_tianji::WujiHand2Device;
using pico_body_tianji::WujiRetargeter;
constexpr std::size_t kJointCount = kWujiJointCount;
constexpr std::size_t kKeypointCount = kWujiKeypointCount;
constexpr std::int64_t kFreshnessNs = 500000000;
volatile std::sig_atomic_t g_stop = 0;

void on_sigint(int) { g_stop = 1; }

std::string env_or(const char *name, const std::string &fallback = {}) {
  const char *value = std::getenv(name);
  return value == nullptr ? fallback : std::string(value);
}

std::int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string quote(const std::string &value) {
  std::ostringstream out;
  out << '"';
  for (const char c : value) {
    if (c == '"' || c == '\\') out << '\\';
    out << c;
  }
  out << '"';
  return out.str();
}

template<typename T>
std::string array_json(const T &values) {
  std::ostringstream out;
  out << '[';
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) out << ',';
    out << std::setprecision(12) << values[index];
  }
  out << ']';
  return out.str();
}
std::array<double, kJointCount> load_yaml_vector(const std::string &path, const std::string &field_name) {
  std::ifstream input(path);
  if (!input) throw std::invalid_argument("unable to read Wuji config: " + path);
  const std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  const auto start = text.find(field_name + ":");
  const auto open = start == std::string::npos ? std::string::npos : text.find('[', start);
  const auto close = open == std::string::npos ? std::string::npos : text.find(']', open);
  if (open == std::string::npos || close == std::string::npos) throw std::invalid_argument("Wuji config missing " + field_name);
  std::array<double, kJointCount> values{};
  std::size_t count = 0;
  std::size_t cursor = open + 1;
  while (cursor < close && count < kJointCount) {
    while (cursor < close && (std::isspace(static_cast<unsigned char>(text[cursor])) || text[cursor] == ',')) ++cursor;
    if (cursor >= close) break;
    std::size_t used = 0;
    const double value = std::stod(text.substr(cursor, close - cursor), &used);
    if (!std::isfinite(value)) throw std::invalid_argument("Wuji config contains nonfinite value");
    values[count++] = value;
    cursor += used;
  }
  if (count != kJointCount) throw std::invalid_argument("Wuji config vector must contain 20 values");
  return values;
}

std::string side_prefix(const std::string &side) {
  if (side == "left") return "l_";
  if (side == "right") return "r_";
  throw std::invalid_argument("side must be left or right");
}

struct ParsedTarget {
  std::string instance;
  std::string router;
  std::string source;
  std::uint64_t sequence{0};
  std::int64_t timestamp_ns{0};
  std::array<float, kKeypointCount * 3> keypoints{};
};

std::array<float, kJointCount> parse_joint_array(
  const JsonValue &root, const std::string &expected_side,
  const std::string &expected_producer, const std::string &expected_instance,
  const std::string &expected_router, const std::string &config_path,
  std::uint64_t *sequence)
{
  protocol::require_exact_fields(root, {
    "schema_version", "publisher_instance_id", "router_zid", "sequence",
    "timestamp_ns", "producer", "side", "names", "position_rad"
  });
  if (protocol::field(root, "schema_version").as_uint("schema_version") != 1) {
    throw std::invalid_argument("unsupported hand command schema");
  }
  if (protocol::field(root, "publisher_instance_id").as_string("publisher_instance_id") != expected_instance ||
      protocol::field(root, "router_zid").as_string("router_zid") != expected_router ||
      protocol::field(root, "producer").as_string("producer") != expected_producer ||
      protocol::field(root, "side").as_string("side") != expected_side) {
    throw std::invalid_argument("hand command authority mismatch");
  }
  const auto seq = protocol::field(root, "sequence").as_uint("sequence");
  const auto timestamp = protocol::field(root, "timestamp_ns").as_uint("timestamp_ns");
  if (timestamp > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
    throw std::invalid_argument("hand command timestamp outside range");
  }
  if (static_cast<std::int64_t>(timestamp) > now_ns() || now_ns() - static_cast<std::int64_t>(timestamp) > kFreshnessNs) {
    throw std::invalid_argument("hand command is stale");
  }
  const auto names = protocol::string_array_field(root, "names", kJointCount);
  const auto prefix = side_prefix(expected_side);
  static constexpr std::array<const char *, 20> base_names = {
    "thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip",
    "index_mcp_flex", "index_mcp_abd", "index_pip", "index_dip",
    "middle_mcp_flex", "middle_mcp_abd", "middle_pip", "middle_dip",
    "ring_mcp_flex", "ring_mcp_abd", "ring_pip", "ring_dip",
    "pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip"};
  for (std::size_t index = 0; index < kJointCount; ++index) {
    if (names[index] != prefix + base_names[index]) {
      throw std::invalid_argument("hand command joint order mismatch");
    }
  }
  const auto position = protocol::vector_field(root, "position_rad", kJointCount);
  const auto lower = load_yaml_vector(config_path, "lower_limits_rad");
  const auto upper = load_yaml_vector(config_path, "upper_limits_rad");
  for (std::size_t index = 0; index < kJointCount; ++index) {
    if (!std::isfinite(position[index]) || position[index] < lower[index] || position[index] > upper[index]) {
      throw std::invalid_argument("hand command exceeds finite hard limits");
    }
  }
  std::array<float, kJointCount> result{};
  for (std::size_t index = 0; index < kJointCount; ++index) {
    result[index] = static_cast<float>(position[index]);
  }
  *sequence = seq;
  return result;
}

ParsedTarget parse_target(
  const std::string &payload, const std::string &expected_side,
  const std::string &expected_router, const std::string &expected_instance)
{
  const auto root = StrictJsonParser::parse(payload);
  protocol::require_exact_fields(root, {
    "schema_version", "publisher_instance_id", "router_zid", "sequence",
    "timestamp_ns", "source_timestamp_ns", "source", "side", "frame_id", "keypoints_m"});
  if (protocol::field(root, "schema_version").as_uint("schema_version") != 1) {
    throw std::invalid_argument("unsupported hand target schema");
  }
  ParsedTarget result;
  result.instance = protocol::field(root, "publisher_instance_id").as_string("publisher_instance_id");
  result.router = protocol::field(root, "router_zid").as_string("router_zid");
  result.source = protocol::field(root, "source").as_string("source");
  if (result.router != expected_router || result.instance != expected_instance ||
      protocol::field(root, "side").as_string("side") != expected_side ||
      protocol::field(root, "frame_id").as_string("frame_id") != "wrist_relative_mediapipe") {
    throw std::invalid_argument("hand target identity/frame mismatch");
  }
  result.sequence = protocol::field(root, "sequence").as_uint("sequence");
  const auto timestamp = protocol::field(root, "timestamp_ns").as_uint("timestamp_ns");
  if (timestamp > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
    throw std::invalid_argument("hand target timestamp outside range");
  }
  result.timestamp_ns = static_cast<std::int64_t>(timestamp);
  if (result.timestamp_ns > now_ns() || now_ns() - result.timestamp_ns > kFreshnessNs) {
    throw std::invalid_argument("hand target is stale");
  }
  const auto rows = protocol::field(root, "keypoints_m").as_array("keypoints_m");
  if (rows.size() != kKeypointCount) throw std::invalid_argument("keypoints_m has invalid shape");
  for (std::size_t row = 0; row < kKeypointCount; ++row) {
    const auto &values = rows[row].as_array("keypoints_m row");
    if (values.size() != 3) throw std::invalid_argument("keypoints_m row has invalid shape");
    for (std::size_t column = 0; column < 3; ++column) {
      result.keypoints[row * 3 + column] = static_cast<float>(values[column].as_number());
    }
  }
  if (std::abs(result.keypoints[0]) > 1e-8F || std::abs(result.keypoints[1]) > 1e-8F || std::abs(result.keypoints[2]) > 1e-8F) {
    throw std::invalid_argument("keypoints_m wrist must be zero");
  }
  return result;
}

class WujiHand2Bridge {
public:
  struct Params {
    std::string side{"right"};
    std::string mode{"retarget"};
    std::string serial;
    std::string address;
    std::string instance;
    std::string router;
    std::string authorized_producer;
    std::string authorized_instance;
    std::string logical_producer{"wuji_retarget"};
    std::string run_id;
    std::string supervisor_instance;
    std::string coordinator_instance;
    std::string config_path;
    bool dry_run{false};
    int rate_hz{100};
    float command_slew_rate_rad_s{1.0F};
    float tracking_slew_rate_rad_s{6.0F};
    float keypoint_timeout_s{0.5F};
    float kp{3.0F};
    float kd{0.05F};
    float effort_limit_amps{1.5F};
    float enable_timeout_s{5.0F};
  };

  WujiHand2Bridge(zenoh::Session &session, Params params)
  : session_(session), params_(std::move(params)) {}

  int run();

private:
  void on_target(const zenoh::Sample &sample);
  void on_command(const zenoh::Sample &sample);
  void on_session_state(const zenoh::Sample &sample);
  void on_safety_stop(const zenoh::Sample &sample);
  void publish_command(const std::array<float, kJointCount> &values, std::uint64_t sequence);
  void publish_state(const std::array<float, kJointCount> &values, std::uint64_t sequence);
  void publish_status(const std::string &error = {});

  zenoh::Session &session_;
  Params params_;
  std::unique_ptr<zenoh::Publisher> command_pub_;
  std::unique_ptr<zenoh::Publisher> state_pub_;
  std::unique_ptr<zenoh::Publisher> status_pub_;
  std::unique_ptr<zenoh::Publisher> safety_ack_pub_;
  std::unique_ptr<zenoh::LivelinessToken> live_token_;
  std::unique_ptr<zenoh::Subscriber<void>> input_sub_;
  std::unique_ptr<zenoh::Subscriber<void>> state_sub_;
  std::unique_ptr<zenoh::Subscriber<void>> safety_sub_;
  std::mutex mutex_;
  std::array<float, kKeypointCount * 3> keypoints_{};
  std::array<float, kJointCount> direct_command_{};
  bool have_target_{false};
  bool have_direct_{false};
  std::int64_t input_received_ns_{0};
  std::uint64_t input_sequence_{0};
  std::uint64_t last_input_sequence_{0};
  std::string session_state_{"idle"};
  bool safety_locked_{false};
  std::uint64_t last_safety_sequence_{0};
  std::uint64_t wire_sequence_{0};
  std::uint64_t status_sequence_{0};
  std::uint64_t last_session_sequence_{0};
  std::string last_error_;
  WujiHand2Device * active_device_{nullptr};
  std::array<float, kJointCount> measured_{};
  bool measured_valid_{false};
  bool tracking_allowed_{false};
};

void WujiHand2Bridge::on_target(const zenoh::Sample &sample) {
  try {
    const auto parsed = parse_target(sample.get_payload().as_string(), params_.side, params_.router, params_.authorized_instance);
    std::lock_guard<std::mutex> guard(mutex_);
    if (session_state_ == "returning" || session_state_ == "fault") throw std::invalid_argument("hand target rejected outside teleop");
    if (parsed.sequence <= last_input_sequence_) throw std::invalid_argument("hand target sequence rollback");
    keypoints_ = parsed.keypoints;
    input_received_ns_ = now_ns();
    input_sequence_ = parsed.sequence;
    last_input_sequence_ = parsed.sequence;
    have_target_ = true;
  } catch (const std::exception &error) {
    std::lock_guard<std::mutex> guard(mutex_);
    last_error_ = std::string("target rejected: ") + error.what();
    have_target_ = false;
  }
}

void WujiHand2Bridge::on_command(const zenoh::Sample &sample) {
  try {
    const auto root = StrictJsonParser::parse(sample.get_payload().as_string());
    std::uint64_t sequence = 0;
    const auto command = parse_joint_array(root, params_.side, params_.authorized_producer, params_.authorized_instance, params_.router, params_.config_path, &sequence);
    std::lock_guard<std::mutex> guard(mutex_);
    if (session_state_ == "returning" || session_state_ == "fault") throw std::invalid_argument("hand command rejected outside teleop");
    if (sequence <= last_input_sequence_) throw std::invalid_argument("hand command sequence rollback");
    direct_command_ = command;
    input_received_ns_ = now_ns();
    input_sequence_ = sequence;
    last_input_sequence_ = sequence;
    have_direct_ = true;
  } catch (const std::exception &error) {
    std::lock_guard<std::mutex> guard(mutex_);
    last_error_ = std::string("command rejected: ") + error.what();
    have_direct_ = false;
  }
}

void WujiHand2Bridge::on_session_state(const zenoh::Sample &sample) {
  try {
    const auto root = StrictJsonParser::parse(sample.get_payload().as_string());
    const auto publisher = protocol::field(root, "publisher_instance_id").as_string("publisher_instance_id");
    const auto timestamp = protocol::field(root, "timestamp_ns").as_uint("timestamp_ns");
    const auto sequence = protocol::field(root, "sequence").as_uint("sequence");
    if (protocol::field(root, "schema_version").as_uint("schema_version") != 1 ||
        protocol::field(root, "router_zid").as_string("router_zid") != params_.router ||
        publisher != params_.coordinator_instance ||
        timestamp > static_cast<std::uint64_t>(now_ns()) ||
        now_ns() - static_cast<std::int64_t>(timestamp) > kFreshnessNs ||
        sequence <= last_session_sequence_) {
      throw std::invalid_argument("session state identity/sequence/freshness mismatch");
    }
    last_session_sequence_ = sequence;
    const auto state = protocol::field(root, "state").as_string("state");
    if (state != "idle" && state != "teleop" && state != "returning" && state != "fault") throw std::invalid_argument("invalid session state");
    std::lock_guard<std::mutex> guard(mutex_);
    session_state_ = state;
    if (state == "returning" || state == "fault") {
      have_target_ = false;
      have_direct_ = false;
    }
  } catch (const std::exception &error) {
    std::lock_guard<std::mutex> guard(mutex_);
    last_error_ = std::string("session state rejected: ") + error.what();
  }
}

void WujiHand2Bridge::on_safety_stop(const zenoh::Sample &sample) {
  try {
    const auto root = StrictJsonParser::parse(sample.get_payload().as_string());
    protocol::require_exact_fields(root, {"schema_version", "publisher_instance_id", "router_zid", "sequence", "timestamp_ns", "run_id", "reason", "latch"});
    const auto supervisor = protocol::field(root, "publisher_instance_id").as_string("publisher_instance_id");
    const auto router = protocol::field(root, "router_zid").as_string("router_zid");
    const auto sequence = protocol::field(root, "sequence").as_uint("sequence");
    const auto run_id = protocol::field(root, "run_id").as_string("run_id");
    const auto reason = protocol::field(root, "reason").as_string("reason");
    if (supervisor != params_.supervisor_instance || router != params_.router || run_id != params_.run_id || !protocol::field(root, "latch").as_bool()) throw std::invalid_argument("safety stop authority mismatch");
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (sequence <= last_safety_sequence_) throw std::invalid_argument("safety stop sequence rollback");
      last_safety_sequence_ = sequence;
      safety_locked_ = true;
      last_error_ = reason;
      if (active_device_ != nullptr) active_device_->close();
    }
    if (safety_ack_pub_) {
      const auto timestamp = now_ns();
      safety_ack_pub_->put(zenoh::Bytes(
        "{\"schema_version\":1,\"publisher_instance_id\":" + quote(params_.instance) +
        ",\"router_zid\":" + quote(params_.router) + ",\"sequence\":" +
        std::to_string(sequence) + ",\"timestamp_ns\":" + std::to_string(timestamp) +
        ",\"executor_id\":" + quote(params_.instance) + ",\"run_id\":" +
        quote(params_.run_id) + ",\"latched\":true,\"reason\":" + quote(reason) + "}"));
    }
  } catch (const std::exception &error) {
    std::lock_guard<std::mutex> guard(mutex_);
    last_error_ = std::string("safety stop rejected: ") + error.what();
  }
}

void WujiHand2Bridge::publish_command(const std::array<float, kJointCount> &values, std::uint64_t sequence) {
  if (!command_pub_) return;
  static constexpr std::array<const char *, 20> base_names = {
    "thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip", "index_mcp_flex", "index_mcp_abd", "index_pip", "index_dip", "middle_mcp_flex", "middle_mcp_abd", "middle_pip", "middle_dip", "ring_mcp_flex", "ring_mcp_abd", "ring_pip", "ring_dip", "pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip"};
  const auto prefix = side_prefix(params_.side);
  std::ostringstream out;
  out << "{\"schema_version\":1,\"publisher_instance_id\":" << quote(params_.instance) << ",\"router_zid\":" << quote(params_.router) << ",\"sequence\":" << sequence << ",\"timestamp_ns\":" << now_ns() << ",\"producer\":" << quote(params_.logical_producer) << ",\"side\":" << quote(params_.side) << ",\"names\":[";
  for (std::size_t index = 0; index < kJointCount; ++index) { if (index) out << ','; out << quote(prefix + base_names[index]); }
  out << "],\"position_rad\":" << array_json(values) << '}';
  command_pub_->put(zenoh::Bytes(out.str()));
}

void WujiHand2Bridge::publish_state(const std::array<float, kJointCount> &values, std::uint64_t sequence) {
  if (!state_pub_) return;
  static constexpr std::array<const char *, 20> base_names = {
    "thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip", "index_mcp_flex", "index_mcp_abd", "index_pip", "index_dip", "middle_mcp_flex", "middle_mcp_abd", "middle_pip", "middle_dip", "ring_mcp_flex", "ring_mcp_abd", "ring_pip", "ring_dip", "pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip"};
  const auto prefix = side_prefix(params_.side);
  std::ostringstream out;
  out << "{\"schema_version\":1,\"publisher_instance_id\":" << quote(params_.instance) << ",\"router_zid\":" << quote(params_.router) << ",\"sequence\":" << sequence << ",\"timestamp_ns\":" << now_ns() << ",\"executor\":\"wuji_hand2\",\"side\":" << quote(params_.side) << ",\"names\":[";
  for (std::size_t index = 0; index < kJointCount; ++index) { if (index) out << ','; out << quote(prefix + base_names[index]); }
  out << "],\"position_rad\":" << array_json(values) << ",\"velocity_rad_s\":null}";
  state_pub_->put(zenoh::Bytes(out.str()));
}

void WujiHand2Bridge::publish_status(const std::string &error) {
  if (!status_pub_) return;
  std::lock_guard<std::mutex> guard(mutex_);
  const auto status_error = error.empty() ? last_error_ : error;
  const bool healthy = !safety_locked_ && status_error.empty();
  bool at_zero = measured_valid_;
  for (const auto value : measured_) at_zero = at_zero && std::isfinite(value) && std::abs(value) <= 0.05F;
  std::ostringstream out;
  out << "{\"schema_version\":1,\"publisher_instance_id\":" << quote(params_.instance) << ",\"router_zid\":" << quote(params_.router) << ",\"sequence\":" << ++status_sequence_ << ",\"timestamp_ns\":" << now_ns() << ",\"side\":" << quote(params_.side) << ",\"ready\":" << (safety_locked_ ? "false" : "true") << ",\"healthy\":" << (healthy ? "true" : "false") << ",\"at_zero\":" << (at_zero ? "true" : "false") << ",\"tracking_allowed\":" << (tracking_allowed_ && healthy ? "true" : "false") << ",\"error\":" << (status_error.empty() ? "null" : quote(status_error)) << '}';
  status_pub_->put(zenoh::Bytes(out.str()));
}

int WujiHand2Bridge::run() {
  if (params_.instance.empty() || params_.router.empty() || params_.authorized_producer.empty() || params_.authorized_instance.empty()) throw std::invalid_argument("Wuji executor identities are required");
  if (params_.mode != "direct" && params_.mode != "retarget") throw std::invalid_argument("mode must be direct or retarget");
  if (params_.rate_hz < 1 || params_.rate_hz > 500) throw std::invalid_argument("rate_hz out of range");
  const auto hand_key = "tianji/target/hand/" + params_.side;
  const auto command_key = "tianji/command/hand/" + params_.side;
  const auto state_key = "tianji/state/hand/" + params_.side;
  const auto status_key = "tianji/executor/hand/" + params_.side + "/status";
  const auto state_topic = "tianji/session/state";
  const auto safety_topic = "tianji/safety/stop";
  if (params_.mode == "retarget") command_pub_ = std::make_unique<zenoh::Publisher>(session_.declare_publisher(zenoh::KeyExpr(command_key)));
  state_pub_ = std::make_unique<zenoh::Publisher>(session_.declare_publisher(zenoh::KeyExpr(state_key)));
  status_pub_ = std::make_unique<zenoh::Publisher>(session_.declare_publisher(zenoh::KeyExpr(status_key)));
  safety_ack_pub_ = std::make_unique<zenoh::Publisher>(session_.declare_publisher(zenoh::KeyExpr("tianji/safety/ack/" + params_.instance)));
  state_sub_ = std::make_unique<zenoh::Subscriber<void>>(session_.declare_subscriber(zenoh::KeyExpr(state_topic), [this](const zenoh::Sample &sample) { on_session_state(sample); }, []() {}));
  safety_sub_ = std::make_unique<zenoh::Subscriber<void>>(session_.declare_subscriber(zenoh::KeyExpr(safety_topic), [this](const zenoh::Sample &sample) { on_safety_stop(sample); }, []() {}));
  if (params_.mode == "retarget") input_sub_ = std::make_unique<zenoh::Subscriber<void>>(session_.declare_subscriber(zenoh::KeyExpr(hand_key), [this](const zenoh::Sample &sample) { on_target(sample); }, []() {}));
  else input_sub_ = std::make_unique<zenoh::Subscriber<void>>(session_.declare_subscriber(zenoh::KeyExpr(command_key), [this](const zenoh::Sample &sample) { on_command(sample); }, []() {}));
  const auto live_key = params_.mode == "retarget"
    ? "tj/live/producer/hand/" + params_.logical_producer + "/" + params_.instance
    : "tj/live/executor/hand/" + params_.side + "/" + params_.instance;
  live_token_ = std::make_unique<zenoh::LivelinessToken>(session_.liveliness_declare_token(zenoh::KeyExpr(live_key)));

  std::unique_ptr<WujiRetargeter> retargeter;
  if (params_.mode == "retarget") {
    std::string error;
    retargeter = std::make_unique<WujiRetargeter>(params_.side == "right" ? WUJI_HANDEDNESS_RIGHT : WUJI_HANDEDNESS_LEFT, &error);
    if (!error.empty()) throw std::runtime_error("retarget session failed: " + error);
  }
  if (params_.dry_run) wuji_init(nullptr);
  std::unique_ptr<WujiHand2Device> device;
  if (!params_.dry_run) {
    if (wuji_init(nullptr) != WUJI_STATUS_OK) throw std::runtime_error("wuji_init failed");
    device = std::make_unique<WujiHand2Device>();
    active_device_ = device.get();
    WujiHand2Device::Options options;
    options.serial = params_.serial; options.address = params_.address; options.kp = params_.kp; options.kd = params_.kd; options.effort_limit_amps = params_.effort_limit_amps; options.enable_timeout_s = params_.enable_timeout_s;
    std::string error;
    if (!device->connect_device(options, &error) || !device->enable_and_wait(&error) || !device->open_publisher(&error)) throw std::runtime_error("Wuji device setup failed: " + error);
  }
  publish_status();
  std::array<float, kJointCount> output{};
  std::array<float, kJointCount> desired{};
  const auto lower_limits = load_yaml_vector(params_.config_path, "lower_limits_rad");
  const auto upper_limits = load_yaml_vector(params_.config_path, "upper_limits_rad");
  auto next_tick = std::chrono::steady_clock::now();
  while (!g_stop) {
    next_tick += std::chrono::nanoseconds(static_cast<std::int64_t>(1.0e9 / params_.rate_hz));
    bool locked = false, tracking = false;
    std::string state;
    std::array<float, kKeypointCount * 3> keypoints{};
    {
      std::lock_guard<std::mutex> guard(mutex_);
      locked = safety_locked_; state = session_state_; keypoints = keypoints_;
      tracking = !locked && state == "teleop" && now_ns() - input_received_ns_ <= static_cast<std::int64_t>(params_.keypoint_timeout_s * 1.0e9);
      tracking_allowed_ = tracking;
      if (params_.mode == "direct" && have_direct_) desired = direct_command_;
    }
    if (locked || state == "returning" || state == "fault" || !tracking) desired.fill(0.0F);
    else if (params_.mode == "retarget") {
      std::string error;
      if (!retargeter->step(keypoints.data(), desired.data(), &error)) { desired.fill(0.0F); publish_status(std::string("retarget rejected: ") + error); }
    }
    for (std::size_t index = 0; index < kJointCount; ++index) {
      if (!std::isfinite(desired[index]) || desired[index] < lower_limits[index] || desired[index] > upper_limits[index]) {
        std::lock_guard<std::mutex> guard(mutex_);
        desired.fill(0.0F);
        safety_locked_ = true;
        tracking_allowed_ = false;
        last_error_ = "retarget output exceeds Wuji config limits";
        locked = true;
        break;
      }
    }
    const float step = (tracking ? params_.tracking_slew_rate_rad_s : params_.command_slew_rate_rad_s) / params_.rate_hz;
    for (std::size_t index = 0; index < kJointCount; ++index) output[index] += std::clamp(desired[index] - output[index], -step, step);
    if (!locked) {
      if (device) {
        std::string error;
        if (!device->send(output.data(), &error)) {
          publish_status("device send failed: " + error);
        }
        float measured[kJointCount]{};
        float velocity[kJointCount]{};
        float effort[kJointCount]{};
        if (device->latest_states(measured, velocity, effort)) {
          std::array<float, kJointCount> measured_array{};
          std::copy(std::begin(measured), std::end(measured), measured_array.begin());
          publish_state(measured_array, ++wire_sequence_);
          {
            std::lock_guard<std::mutex> guard(mutex_);
            measured_ = measured_array;
            measured_valid_ = true;
          }
        } else {
          publish_status("measured hand state unavailable");
          std::lock_guard<std::mutex> guard(mutex_);
          measured_valid_ = false;
          tracking_allowed_ = false;
          safety_locked_ = true;
          last_error_ = "measured hand state unavailable";
          device->close();
        }
      } else {
        publish_state(output, ++wire_sequence_);
      }
      if (params_.mode == "retarget" && tracking) publish_command(output, ++wire_sequence_);
    }
    std::this_thread::sleep_until(next_tick);
  }
  if (device) device->close();
  if (!params_.dry_run) wuji_shutdown();
  return 0;
}

}  // namespace
void canonical_on_sigint(int signal) { on_sigint(signal); }
std::string canonical_env_or(const char *name, const std::string &fallback = {}) {
  return env_or(name, fallback);
}
}  // namespace pico_body_tianji

int main(int argc, char **argv) {
  struct sigaction action{};
  action.sa_handler = pico_body_tianji::canonical_on_sigint;
  if (sigaction(SIGINT, &action, nullptr) != 0) return 1;
  try {
    pico_body_tianji::WujiHand2Bridge::Params params;
    for (int index = 1; index < argc; ++index) {
      const std::string arg = argv[index];
      const auto value = [&]() -> std::string { if (++index >= argc) throw std::invalid_argument(arg + " needs a value"); return argv[index]; };
      if (arg == "--side") params.side = value();
      else if (arg == "--mode") params.mode = value();
      else if (arg == "--dry-run") params.dry_run = true;
      else if (arg == "--serial") params.serial = value();
      else if (arg == "--address") params.address = value();
      else if (arg == "--rate") params.rate_hz = std::stoi(value());
      else if (arg == "--keypoint-timeout") params.keypoint_timeout_s = std::stof(value());
      else if (arg == "--command-slew-rate") params.command_slew_rate_rad_s = std::stof(value());
      else if (arg == "--tracking-slew-rate") params.tracking_slew_rate_rad_s = std::stof(value());
      else if (arg == "-h" || arg == "--help") { std::cout << "wuji_hand2_bridge --mode direct|retarget --side left|right [--dry-run]\n"; return 0; }
      else throw std::invalid_argument("unknown argument: " + arg);
    }
    params.instance = pico_body_tianji::canonical_env_or("TIANJI_COMPONENT_INSTANCE_ID");
    params.router = pico_body_tianji::canonical_env_or("TIANJI_ROUTER_ZID");
    params.config_path = pico_body_tianji::canonical_env_or("TIANJI_WUJI_CONFIG");
    params.coordinator_instance = pico_body_tianji::canonical_env_or("TIANJI_COORDINATOR_INSTANCE_ID");
    params.authorized_producer = pico_body_tianji::canonical_env_or("TIANJI_HAND_PRODUCER_ID");
    params.authorized_instance = pico_body_tianji::canonical_env_or("TIANJI_HAND_PRODUCER_INSTANCE_ID");
    params.logical_producer = pico_body_tianji::canonical_env_or("TIANJI_HAND_LOGICAL_PRODUCER_ID", params.mode == "retarget" ? "wuji_retarget" : "h5_direct");
    params.run_id = pico_body_tianji::canonical_env_or("TIANJI_RUN_ID");
    params.supervisor_instance = pico_body_tianji::canonical_env_or("TIANJI_SAFETY_SUPERVISOR_INSTANCE_ID");
    if (params.instance.empty() || params.router.empty() || params.coordinator_instance.empty() || params.config_path.empty() || params.supervisor_instance.empty() || params.run_id.empty()) throw std::invalid_argument("executor, router, coordinator, run and safety identities are required");
    auto config = zenoh::Config::create_default();
    config.insert_json5("mode", "\"client\"");
    config.insert_json5("connect/endpoints", "[\"" + pico_body_tianji::canonical_env_or("TIANJI_ROUTER_ENDPOINT", "tcp/127.0.0.1:7447") + "\"]");
    zenoh::Session session = zenoh::Session::open(std::move(config));
    const auto routers = session.get_routers_z_id();
    if (routers.size() != 1 || routers.front().to_string() != params.router) throw std::runtime_error("expected exactly one router ZID");
    if (params.authorized_producer.empty() || params.authorized_instance.empty()) throw std::invalid_argument("hand producer identity is required");
    pico_body_tianji::WujiHand2Bridge bridge(session, std::move(params));
    return bridge.run();
  } catch (const std::exception &error) {
    std::cerr << "wuji_hand2_bridge failed: " << error.what() << std::endl;
    return 1;
  }
}
