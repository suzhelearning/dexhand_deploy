#include "pico_body_tianji/ik/arm_ik_factory.hpp"

#include <zenoh.hxx>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace pico_body_tianji
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kLeftIndex = 0;
constexpr std::size_t kRightIndex = 1;

double radians(double degrees)
{
  return degrees * kPi / 180.0;
}

double degrees(double radians_value)
{
  return radians_value * 180.0 / kPi;
}

ArmJointVector joint_vector_from_degrees(
  const std::vector<double> & values,
  const std::string & parameter_name)
{
  if (values.size() != 7) {
    throw std::invalid_argument(
            parameter_name + " 必须恰好包含 7 个关节角");
  }
  ArmJointVector result;
  for (Eigen::Index index = 0; index < result.size(); ++index) {
    const double value = values[static_cast<std::size_t>(index)];
    if (!std::isfinite(value)) {
      throw std::invalid_argument(parameter_name + " 含有非有限值");
    }
    result[index] = radians(value);
  }
  return result;
}

std::string json_quote(const std::string & value)
{
  std::ostringstream stream;
  stream << '"';
  for (const char character : value) {
    switch (character) {
      case '"':
        stream << "\\\"";
        break;
      case '\\':
        stream << "\\\\";
        break;
      case '\n':
        stream << "\\n";
        break;
      case '\r':
        stream << "\\r";
        break;
      case '\t':
        stream << "\\t";
        break;
      default:
        stream << character;
        break;
    }
  }
  stream << '"';
  return stream.str();
}

std::string json_optional_string(
  const std::optional<std::string> & value)
{
  return value.has_value() ? json_quote(*value) : "null";
}

std::string json_optional_number(const std::optional<double> & value)
{
  if (!value.has_value() || !std::isfinite(*value)) {
    return "null";
  }
  std::ostringstream stream;
  stream << std::setprecision(10) << *value;
  return stream.str();
}

const char * json_bool(bool value)
{
  return value ? "true" : "false";
}

// ---------------------------------------------------------------- 极简 JSON

class JsonValue
{
public:
  enum class Type { kNull, kBool, kNumber, kString, kArray, kObject };

  Type type{Type::kNull};
  bool boolean{false};
  double number{0.0};
  std::string string;
  std::vector<JsonValue> array;
  std::map<std::string, JsonValue> object;

  const JsonValue * find(const std::string & member) const
  {
    if (type != Type::kObject) {
      return nullptr;
    }
    const auto iterator = object.find(member);
    return iterator == object.end() ? nullptr : &iterator->second;
  }
};

class JsonParser
{
public:
  explicit JsonParser(const std::string & text) : text_(text) {}

  JsonValue parse()
  {
    skip_whitespace();
    JsonValue value = parse_value();
    skip_whitespace();
    if (position_ != text_.size()) {
      throw std::invalid_argument("JSON 尾部存在多余内容");
    }
    return value;
  }

private:
  const std::string & text_;
  std::size_t position_{0};

  void skip_whitespace()
  {
    while (
      position_ < text_.size() &&
      (text_[position_] == ' ' || text_[position_] == '\t' ||
       text_[position_] == '\n' || text_[position_] == '\r'))
    {
      ++position_;
    }
  }

  char peek()
  {
    if (position_ >= text_.size()) {
      throw std::invalid_argument("JSON 意外结束");
    }
    return text_[position_];
  }

  void consume(char expected)
  {
    if (peek() != expected) {
      throw std::invalid_argument(
              std::string("JSON 期望字符 ") + expected);
    }
    ++position_;
  }

  JsonValue parse_value()
  {
    const char current = peek();
    switch (current) {
      case '{':
        return parse_object();
      case '[':
        return parse_array();
      case '"':
        return parse_string();
      case 't':
      case 'f':
        return parse_bool();
      case 'n':
        return parse_null();
      default:
        if (current == '-' || (current >= '0' && current <= '9')) {
          return parse_number();
        }
        throw std::invalid_argument("JSON 未知值起始字符");
    }
  }

  JsonValue parse_object()
  {
    JsonValue result;
    result.type = JsonValue::Type::kObject;
    consume('{');
    skip_whitespace();
    if (peek() == '}') {
      ++position_;
      return result;
    }
    while (true) {
      skip_whitespace();
      JsonValue key = parse_string();
      skip_whitespace();
      consume(':');
      skip_whitespace();
      JsonValue value = parse_value();
      result.object[key.string] = std::move(value);
      skip_whitespace();
      const char separator = peek();
      if (separator == '}') {
        ++position_;
        break;
      }
      consume(',');
    }
    return result;
  }

  JsonValue parse_array()
  {
    JsonValue result;
    result.type = JsonValue::Type::kArray;
    consume('[');
    skip_whitespace();
    if (peek() == ']') {
      ++position_;
      return result;
    }
    while (true) {
      skip_whitespace();
      result.array.push_back(parse_value());
      skip_whitespace();
      const char separator = peek();
      if (separator == ']') {
        ++position_;
        break;
      }
      consume(',');
    }
    return result;
  }

  JsonValue parse_string()
  {
    JsonValue result;
    result.type = JsonValue::Type::kString;
    consume('"');
    std::string value;
    while (true) {
      const char current = peek();
      if (current == '"') {
        ++position_;
        break;
      }
      if (current == '\\') {
        ++position_;
        const char escaped = peek();
        switch (escaped) {
          case '"':
            value.push_back('"');
            break;
          case '\\':
            value.push_back('\\');
            break;
          case '/':
            value.push_back('/');
            break;
          case 'b':
            value.push_back('\b');
            break;
          case 'f':
            value.push_back('\f');
            break;
          case 'n':
            value.push_back('\n');
            break;
          case 'r':
            value.push_back('\r');
            break;
          case 't':
            value.push_back('\t');
            break;
          case 'u': {
            if (position_ + 4 >= text_.size()) {
              throw std::invalid_argument("JSON \\u 转义不完整");
            }
            unsigned int code = 0;
            for (int index = 0; index < 4; ++index) {
              ++position_;
              const char hex = text_[position_];
              code <<= 4;
              if (hex >= '0' && hex <= '9') {
                code |= static_cast<unsigned int>(hex - '0');
              } else if (hex >= 'a' && hex <= 'f') {
                code |= static_cast<unsigned int>(hex - 'a' + 10);
              } else if (hex >= 'A' && hex <= 'F') {
                code |= static_cast<unsigned int>(hex - 'A' + 10);
              } else {
                throw std::invalid_argument("JSON \\u 含非法十六进制");
              }
            }
            if (code <= 0x7F) {
              value.push_back(static_cast<char>(code));
            } else if (code <= 0x7FF) {
              value.push_back(static_cast<char>(0xC0 | (code >> 6)));
              value.push_back(static_cast<char>(0x80 | (code & 0x3F)));
            } else {
              value.push_back(static_cast<char>(0xE0 | (code >> 12)));
              value.push_back(
                static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
              value.push_back(static_cast<char>(0x80 | (code & 0x3F)));
            }
            break;
          }
          default:
            throw std::invalid_argument("JSON 非法转义字符");
        }
        ++position_;
        continue;
      }
      value.push_back(current);
      ++position_;
    }
    result.string = std::move(value);
    return result;
  }

  JsonValue parse_number()
  {
    JsonValue result;
    result.type = JsonValue::Type::kNumber;
    const std::size_t start = position_;
    if (peek() == '-') {
      ++position_;
    }
    while (
      position_ < text_.size() &&
      ((text_[position_] >= '0' && text_[position_] <= '9') ||
       text_[position_] == '.' || text_[position_] == 'e' ||
       text_[position_] == 'E' || text_[position_] == '+' ||
       text_[position_] == '-'))
    {
      ++position_;
    }
    if (position_ == start) {
      throw std::invalid_argument("JSON 数字为空");
    }
    try {
      result.number = std::stod(text_.substr(start, position_ - start));
    } catch (const std::exception &) {
      throw std::invalid_argument("JSON 数字非法");
    }
    return result;
  }

  JsonValue parse_bool()
  {
    JsonValue result;
    result.type = JsonValue::Type::kBool;
    if (text_.compare(position_, 4, "true") == 0) {
      result.boolean = true;
      position_ += 4;
      return result;
    }
    if (text_.compare(position_, 5, "false") == 0) {
      result.boolean = false;
      position_ += 5;
      return result;
    }
    throw std::invalid_argument("JSON 布尔值非法");
  }

  JsonValue parse_null()
  {
    if (text_.compare(position_, 4, "null") == 0) {
      position_ += 4;
      return JsonValue{};
    }
    throw std::invalid_argument("JSON null 非法");
  }
};

JsonValue json_parse(const std::string & text)
{
  return JsonParser(text).parse();
}

// ---------------------------------------------------------------- 参数

class ParamMap
{
public:
  explicit ParamMap(
    const std::vector<std::string> & assignments,
    const std::map<std::string, std::string> & defaults = {})
    : values_(defaults)
  {
    for (const std::string & assignment : assignments) {
      const std::size_t separator = assignment.find(":=");
      if (separator == std::string::npos) {
        throw std::invalid_argument(
                "非法参数：" + assignment + "（需要 key:=value）");
      }
      values_[assignment.substr(0, separator)] =
        assignment.substr(separator + 2);
    }
  }

  bool has(const std::string & name) const
  {
    return values_.find(name) != values_.end();
  }

  std::string get_string(
    const std::string & name, const std::string & fallback) const
  {
    const auto iterator = values_.find(name);
    return iterator == values_.end() ? fallback : iterator->second;
  }

  double get_double(const std::string & name, double fallback) const
  {
    const auto iterator = values_.find(name);
    if (iterator == values_.end()) {
      return fallback;
    }
    try {
      return std::stod(iterator->second);
    } catch (const std::exception &) {
      throw std::invalid_argument(
              name + " 需要浮点数，实际为 " + iterator->second);
    }
  }

  int get_int(const std::string & name, int fallback) const
  {
    const auto iterator = values_.find(name);
    if (iterator == values_.end()) {
      return fallback;
    }
    try {
      return std::stoi(iterator->second);
    } catch (const std::exception &) {
      throw std::invalid_argument(
              name + " 需要整数，实际为 " + iterator->second);
    }
  }

  bool get_bool(const std::string & name, bool fallback) const
  {
    const auto iterator = values_.find(name);
    if (iterator == values_.end()) {
      return fallback;
    }
    const std::string & value = iterator->second;
    if (value == "true" || value == "1" || value == "yes") {
      return true;
    }
    if (value == "false" || value == "0" || value == "no") {
      return false;
    }
    throw std::invalid_argument(
            name + " 需要布尔值，实际为 " + value);
  }

  std::vector<double> get_double_list(
    const std::string & name,
    const std::vector<double> & fallback) const
  {
    const auto iterator = values_.find(name);
    if (iterator == values_.end()) {
      return fallback;
    }
    const JsonValue parsed = json_parse(iterator->second);
    if (parsed.type != JsonValue::Type::kArray) {
      throw std::invalid_argument(name + " 需要 JSON 数组");
    }
    std::vector<double> result;
    result.reserve(parsed.array.size());
    for (const JsonValue & item : parsed.array) {
      if (item.type != JsonValue::Type::kNumber) {
        throw std::invalid_argument(name + " 数组元素必须是数字");
      }
      result.push_back(item.number);
    }
    return result;
  }

private:
  std::map<std::string, std::string> values_;
};

// ---------------------------------------------------------------- 时间戳

struct Stamp
{
  std::int64_t sec{0};
  std::int32_t nanosec{0};
};

Stamp stamp_now()
{
  timespec now;
  clock_gettime(CLOCK_REALTIME, &now);
  return Stamp{static_cast<std::int64_t>(now.tv_sec),
    static_cast<std::int32_t>(now.tv_nsec)};
}

std::string json_stamp(const Stamp & stamp)
{
  std::ostringstream stream;
  stream << "{\"sec\":" << stamp.sec << ",\"nanosec\":" << stamp.nanosec
         << '}';
  return stream.str();
}

// ---------------------------------------------------------------- 消息转换

Eigen::Isometry3d pose_from_json(const JsonValue & message)
{
  const JsonValue * position = message.find("position");
  const JsonValue * orientation = message.find("orientation");
  if (
    position == nullptr || position->type != JsonValue::Type::kObject ||
    orientation == nullptr || orientation->type != JsonValue::Type::kObject)
  {
    throw std::invalid_argument("Pose JSON 缺少 position/orientation");
  }
  const JsonValue * px = position->find("x");
  const JsonValue * py = position->find("y");
  const JsonValue * pz = position->find("z");
  const JsonValue * ow = orientation->find("w");
  const JsonValue * ox = orientation->find("x");
  const JsonValue * oy = orientation->find("y");
  const JsonValue * oz = orientation->find("z");
  if (
    px == nullptr || py == nullptr || pz == nullptr ||
    ow == nullptr || ox == nullptr || oy == nullptr || oz == nullptr)
  {
    throw std::invalid_argument("Pose JSON 字段不完整");
  }
  const Eigen::Quaterniond quaternion(
    ow->number, ox->number, oy->number, oz->number);
  if (
    !quaternion.coeffs().allFinite() ||
    quaternion.norm() < 1.0e-8)
  {
    throw std::invalid_argument("末端目标四元数无效");
  }
  const Eigen::Vector3d translation(
    px->number, py->number, pz->number);
  if (!translation.allFinite()) {
    throw std::invalid_argument("末端目标位置含有非有限值");
  }

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = quaternion.normalized().toRotationMatrix();
  pose.translation() = translation;
  return pose;
}

std::string pose_json(const Eigen::Isometry3d & pose)
{
  const Eigen::Quaterniond quaternion(pose.rotation());
  std::ostringstream stream;
  stream << std::setprecision(10)
         << "{\"position\":{\"x\":" << pose.translation().x()
         << ",\"y\":" << pose.translation().y()
         << ",\"z\":" << pose.translation().z() << "},"
         << "\"orientation\":{\"x\":" << quaternion.x()
         << ",\"y\":" << quaternion.y()
         << ",\"z\":" << quaternion.z()
         << ",\"w\":" << quaternion.w() << "}}";
  return stream.str();
}

// ---------------------------------------------------------------- latched 键

class LatchedValue
{
public:
  LatchedValue(
    zenoh::Session & session,
    std::string key,
    std::string initial)
  : session_(session),
    key_expr_(std::move(key)),
    value_(std::move(initial))
  {
    queryable_ = session_.declare_queryable(
      key_expr_,
      [this](const zenoh::Query & query) {
        std::lock_guard<std::mutex> lock(mutex_);
        query.reply(key_expr_, zenoh::Bytes(value_));
      });
    subscriber_ = session_.declare_subscriber(
      key_expr_,
      [this](const zenoh::Sample & sample) {
        std::lock_guard<std::mutex> lock(mutex_);
        value_ = sample.get_payload().as_string();
      });
  }

  void put(const std::string & value)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      value_ = value;
    }
    session_.put(key_expr_, zenoh::Bytes(value));
  }

private:
  zenoh::Session & session_;
  zenoh::KeyExpr key_expr_;
  std::mutex mutex_;
  std::string value_;
  zenoh::Queryable<void> queryable_;
  zenoh::Subscriber<void> subscriber_;
};

// ---------------------------------------------------------------- 消息键

std::string topic_key(const std::string & ros_topic)
{
  if (!ros_topic.empty() && ros_topic.front() == '/') {
    return ros_topic.substr(1);
  }
  return ros_topic;
}

}  // namespace

class TianjiKinematicSimNode
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TianjiKinematicSimNode(zenoh::Session & session, const ParamMap & params)
  : session_(session)
  {
    const std::string backend = params.get_string("ik_backend", "pinocchio_cpp");
    ik_backend_ = backend;

    rate_hz_ = params.get_double("rate", 30.0);
    home_minimum_duration_s_ =
      params.get_double("home_minimum_duration", 2.0);
    home_max_speed_deg_s_ =
      params.get_double("home_max_speed_deg_s", 25.0);
    if (
      rate_hz_ <= 0.0 ||
      home_minimum_duration_s_ <= 0.0 ||
      home_max_speed_deg_s_ <= 0.0)
    {
      throw std::invalid_argument("频率与回零轨迹参数必须为正数");
    }

    IkSettings settings;
    settings.max_iterations =
      params.get_int("max_iterations", 24);
    settings.position_tolerance_m =
      params.get_double("position_tolerance_m", 1.0e-3);
    settings.orientation_tolerance_rad =
      radians(params.get_double("orientation_tolerance_deg", 0.6));
    settings.minimum_damping =
      params.get_double("minimum_damping", 1.0e-3);
    settings.maximum_damping =
      params.get_double("maximum_damping", 0.15);
    settings.singular_value_threshold =
      params.get_double("singular_value_threshold", 0.05);
    settings.maximum_iteration_step_rad =
      radians(params.get_double("max_iteration_step_deg", 4.5));
    settings.maximum_joint_step_rad =
      radians(params.get_double("max_joint_step_deg", 3.0));
    maximum_joint_step_rad_ = settings.maximum_joint_step_rad;
    settings.joint_limit_margin_rad =
      radians(params.get_double("joint_limit_margin_deg", 5.0));
    settings.arm_angle_gain =
      params.get_double("arm_angle_gain", 0.8);
    settings.arm_angle_tolerance_rad =
      radians(params.get_double("arm_angle_tolerance_deg", 2.0));
    settings.arm_angle_merit_weight =
      params.get_double("arm_angle_merit_weight", 1.0e-3);
    settings.nullspace_damping =
      params.get_double("nullspace_damping", 1.0e-3);
    settings.joint_center_gain =
      params.get_double("joint_center_gain", 0.3);
    settings.joint_center_activation_margin_rad = radians(
      params.get_double("joint_center_activation_margin_deg", 15.0));
    settings.joint_center_merit_weight =
      params.get_double("joint_center_merit_weight", 1.0e-3);
    settings.singularity_avoidance_gain =
      params.get_double("singularity_avoidance_gain", 0.2);
    settings.singularity_merit_weight =
      params.get_double("singularity_merit_weight", 1.0e-2);
    settings.control_period_s = 1.0 / rate_hz_;
    settings.qp_position_time_constant_s =
      params.get_double("qp_position_time_constant_s", 0.30);
    settings.qp_orientation_time_constant_s =
      params.get_double("qp_orientation_time_constant_s", 0.40);
    settings.qp_max_linear_speed_m_s =
      params.get_double("qp_max_linear_speed_m_s", 0.25);
    settings.qp_max_angular_speed_rad_s =
      params.get_double("qp_max_angular_speed_rad_s", 1.00);
    settings.qp_joint_velocity_limits_rad_s = joint_vector_from_degrees(
      params.get_double_list(
        "qp_joint_velocity_limits_deg_s",
        {55.0, 55.0, 55.0, 55.0, 55.0, 55.0, 55.0}),
      "qp_joint_velocity_limits_deg_s");
    settings.qp_position_weight =
      params.get_double("qp_position_weight", 1.0);
    settings.qp_orientation_weight =
      params.get_double("qp_orientation_weight", 0.45);
    settings.qp_velocity_regularization_weight =
      params.get_double("qp_velocity_regularization_weight", 2.0e-2);
    settings.qp_continuity_weight =
      params.get_double("qp_continuity_weight", 6.0e-2);
    settings.qp_posture_weight =
      params.get_double("qp_posture_weight", 8.0e-3);
    settings.qp_posture_time_constant_s =
      params.get_double("qp_posture_time_constant_s", 2.5);
    settings.qp_joint_limit_activation_margin_rad = radians(
      params.get_double("qp_joint_limit_activation_margin_deg", 15.0));
    settings.qp_joint_limit_velocity_damper_gain =
      params.get_double("qp_joint_limit_velocity_damper_gain", 4.0);
    settings.qp_singularity_critical_threshold =
      params.get_double("qp_singularity_critical_threshold", 1.5e-2);
    settings.qp_singularity_orientation_scale =
      params.get_double("qp_singularity_orientation_scale", 0.15);
    settings.qp_singularity_posture_multiplier =
      params.get_double("qp_singularity_posture_multiplier", 8.0);
    settings.qp_singularity_velocity_multiplier =
      params.get_double("qp_singularity_velocity_multiplier", 4.0);
    settings.qp_singularity_escape_weight =
      params.get_double("qp_singularity_escape_weight", 3.0e-2);
    settings.qp_singularity_escape_speed_rad_s = radians(
      params.get_double("qp_singularity_escape_speed_deg_s", 8.6));
    settings.qp_max_active_set_iterations =
      params.get_int("qp_max_active_set_iterations", 48);
    settings.qp_active_set_tolerance =
      params.get_double("qp_active_set_tolerance", 1.0e-9);
    settings.qp_left_nominal_rad = joint_vector_from_degrees(
      params.get_double_list(
        "left_home_deg", {55.0, -65.0, -70.0, -60.0, 60.0, 0.0, 0.0}),
      "left_home_deg");
    settings.qp_right_nominal_rad = joint_vector_from_degrees(
      params.get_double_list(
        "right_home_deg", {-55.0, -65.0, 70.0, -60.0, -60.0, 0.0, 0.0}),
      "right_home_deg");
    settings.official_use_zsp =
      params.get_bool("official_use_zsp", false);
    settings.official_dgr1 =
      params.get_double("official_dgr1", 0.05);
    settings.official_dgr2 =
      params.get_double("official_dgr2", 0.05);
    settings.official_dgr3 =
      params.get_double("official_dgr3", 0.0);
    settings.official_joint_limit_soft_margin_rad = radians(
      params.get_double("official_joint_limit_soft_margin_deg", 5.0));
    settings.official_candidate_continuity_weight =
      params.get_double("official_candidate_continuity_weight", 1.0);
    settings.official_candidate_limit_weight =
      params.get_double("official_candidate_limit_weight", 0.20);
    settings.official_candidate_posture_weight =
      params.get_double("official_candidate_posture_weight", 0.02);
    settings.official_orientation_relaxation_steps =
      params.get_int("official_orientation_relaxation_steps", 3);
    settings.official_workspace_backoff_iterations =
      params.get_int("official_workspace_backoff_iterations", 8);
    settings.official_worker_timeout_ms =
      params.get_int("official_worker_timeout_ms", 25);
    settings.official_worker_restart_attempts =
      params.get_int("official_worker_restart_attempts", 1);
    settings.official_left_nominal_rad = settings.qp_left_nominal_rad;
    settings.official_right_nominal_rad = settings.qp_right_nominal_rad;
    arm_angle_required_ = backend == "tianji_official" ?
      settings.official_use_zsp : settings.arm_angle_gain > 0.0;

    const std::string urdf_path =
      params.get_string("urdf_path", "");
    if (urdf_path.empty()) {
      throw std::invalid_argument(
              "urdf_path 必须通过 --param urdf_path:=<文件> 提供");
    }
    ArmIkBackendOptions backend_options;
    backend_options.urdf_path = urdf_path;
    backend_options.official_library_path =
      params.get_string("official_ik_library", "");
    backend_options.official_config_path =
      params.get_string("official_ik_config", "");
    solver_ = create_arm_ik_solver(backend, backend_options, settings);
    std::cerr << "已选择 IK 后端：" << backend << std::endl;

    arms_[kLeftIndex].side = ArmSide::kLeft;
    arms_[kLeftIndex].side_name = "left";
    arms_[kLeftIndex].suffix = "L";
    arms_[kLeftIndex].home = joint_vector_from_degrees(
      params.get_double_list(
        "left_home_deg", {55.0, -65.0, -70.0, -60.0, 60.0, 0.0, 0.0}),
      "left_home_deg");
    arms_[kRightIndex].side = ArmSide::kRight;
    arms_[kRightIndex].side_name = "right";
    arms_[kRightIndex].suffix = "R";
    arms_[kRightIndex].home = joint_vector_from_degrees(
      params.get_double_list(
        "right_home_deg", {-55.0, -65.0, 70.0, -60.0, -60.0, 0.0, 0.0}),
      "right_home_deg");
    for (ArmState & arm : arms_) {
      arm.current = arm.home;
      arm.achieved = solver_->forward(arm.side, arm.current);
    }

    create_zenoh_interfaces();
    at_home_.put("true");
    std::cerr << "双臂纯运动学节点已启动，IK 后端=" << ik_backend_
              << "；未连接实体机械臂" << std::endl;
  }

private:
  struct ReturnTrajectory
  {
    ArmJointVector start{ArmJointVector::Zero()};
    std::chrono::steady_clock::time_point start_time{};
    double duration_s{0.0};
    bool active{false};
  };

  struct ArmState
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ArmSide side{ArmSide::kLeft};
    std::string side_name;
    std::string suffix;
    ArmJointVector home{ArmJointVector::Zero()};
    ArmJointVector current{ArmJointVector::Zero()};
    std::optional<Eigen::Isometry3d> target;
    std::optional<Eigen::Vector3d> elbow_direction;
    std::optional<Eigen::Isometry3d> achieved;
    std::optional<IkResult> result;
    std::optional<std::string> error;
    int consecutive_rejections{0};
    ReturnTrajectory return_trajectory;
  };

  void create_zenoh_interfaces()
  {
    for (std::size_t index = 0; index < arms_.size(); ++index) {
      const std::string & side = arms_[index].side_name;
      joint_publishers_[index] = session_.declare_publisher(
        zenoh::KeyExpr(
          topic_key("/pico_body_sim/" + side + "_arm/joint_commands")));
      solved_pose_publishers_[index] = session_.declare_publisher(
        zenoh::KeyExpr(
          topic_key("/pico_body_sim/" + side + "_arm/solved_pose")));
      pose_subscriptions_[index] = session_.declare_subscriber(
        zenoh::KeyExpr(topic_key("/pico_body/" + side + "_arm_target_pose")),
        [this, index](const zenoh::Sample & sample) {
          on_pose(index, sample.get_payload().as_string());
        });
      direction_subscriptions_[index] = session_.declare_subscriber(
        zenoh::KeyExpr(
          topic_key("/pico_body/" + side + "_arm_elbow_direction")),
        [this, index](const zenoh::Sample & sample) {
          on_direction(index, sample.get_payload().as_string());
        });
    }

    model_joint_publisher_ = session_.declare_publisher(
      zenoh::KeyExpr(topic_key("/pico_body_sim/model_joint_states")));
    status_publisher_ = session_.declare_publisher(
      zenoh::KeyExpr(topic_key("/pico_body_sim/status")));
    teleop_state_subscription_ = session_.declare_subscriber(
      zenoh::KeyExpr(topic_key("/pico_body/teleop_state")),
      [this](const zenoh::Sample & sample) {
        on_teleop_state(sample.get_payload().as_string());
      });
  }

  void on_pose(std::size_t arm_index, const std::string & payload)
  {
    try {
      const JsonValue message = json_parse(payload);
      std::lock_guard<std::mutex> lock(mutex_);
      arms_[arm_index].target = pose_from_json(message);
      arms_[arm_index].error.reset();
    } catch (const std::exception & exception) {
      std::lock_guard<std::mutex> lock(mutex_);
      arms_[arm_index].error = exception.what();
    }
  }

  void on_direction(std::size_t arm_index, const std::string & payload)
  {
    try {
      const JsonValue message = json_parse(payload);
      const JsonValue * vector = message.find("vector");
      if (
        vector == nullptr || vector->type != JsonValue::Type::kObject)
      {
        throw std::invalid_argument("臂角方向 JSON 缺少 vector");
      }
      const JsonValue * vx = vector->find("x");
      const JsonValue * vy = vector->find("y");
      const JsonValue * vz = vector->find("z");
      if (vx == nullptr || vy == nullptr || vz == nullptr) {
        throw std::invalid_argument("臂角方向 JSON 字段不完整");
      }
      const Eigen::Vector3d direction(vx->number, vy->number, vz->number);
      if (!direction.allFinite() || direction.norm() < 1.0e-8) {
        arms_[arm_index].error = "SMPL 臂角方向无效";
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      arms_[arm_index].elbow_direction = direction.normalized();
    } catch (const std::exception & exception) {
      std::lock_guard<std::mutex> lock(mutex_);
      arms_[arm_index].error = exception.what();
    }
  }

  void on_teleop_state(const std::string & state)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state == "teleop") {
      mode_ = "teleop";
      for (ArmState & arm : arms_) {
        arm.return_trajectory.active = false;
        arm.error.reset();
        arm.consecutive_rejections = 0;
      }
      at_home_.put("false");
      return;
    }
    if (state == "returning" && mode_ != "returning") {
      begin_return_locked();
      return;
    }
    if (state == "idle" && mode_ != "returning") {
      mode_ = "idle";
    }
  }

  void begin_return_locked()
  {
    const auto start_time = std::chrono::steady_clock::now();
    for (ArmState & arm : arms_) {
      const double maximum_delta_deg =
        degrees((arm.home - arm.current).cwiseAbs().maxCoeff());
      const double speed_limited_duration =
        1.5 * maximum_delta_deg / home_max_speed_deg_s_;
      arm.return_trajectory.start = arm.current;
      arm.return_trajectory.start_time = start_time;
      arm.return_trajectory.duration_s =
        std::max(home_minimum_duration_s_, speed_limited_duration);
      arm.return_trajectory.active = true;
      arm.target.reset();
      arm.elbow_direction.reset();
      arm.result.reset();
      arm.error.reset();
    }
    mode_ = "returning";
    at_home_.put("false");
    std::cerr << "开始按零端速 smoothstep 缓慢回安全初始位" << std::endl;
  }

  void tick()
  {
    if (mode_ == "teleop") {
      solve_targets();
    } else if (mode_ == "returning") {
      sample_return();
    }
    publish_joint_states();
  }

  void solve_targets()
  {
    for (std::size_t index = 0; index < arms_.size(); ++index) {
      ArmState snapshot;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot.side = arms_[index].side;
        snapshot.side_name = arms_[index].side_name;
        snapshot.current = arms_[index].current;
        snapshot.target = arms_[index].target;
        snapshot.elbow_direction = arms_[index].elbow_direction;
      }
      if (
        !snapshot.target.has_value() ||
        (arm_angle_required_ && !snapshot.elbow_direction.has_value()))
      {
        continue;
      }
      try {
        const Eigen::Vector3d elbow_direction =
          snapshot.elbow_direction.value_or(Eigen::Vector3d::Zero());
        const auto solve_started = std::chrono::steady_clock::now();
        IkResult result = solver_->solve(
          snapshot.side,
          *snapshot.target,
          snapshot.current,
          elbow_direction);
        if (!std::isfinite(result.solve_time_ms)) {
          result.solve_time_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - solve_started).count();
        }
        if (!result.joints_rad.allFinite()) {
          throw std::runtime_error("IK 后端返回非有限关节角");
        }
        const double actual_maximum_step =
          (result.joints_rad - snapshot.current).cwiseAbs().maxCoeff();
        result.maximum_joint_step_rad = actual_maximum_step;
        if (
          result.accepted &&
          actual_maximum_step > maximum_joint_step_rad_ + 1.0e-10)
        {
          throw std::runtime_error("IK 后端返回值超过公共关节步长安全限制");
        }
        std::lock_guard<std::mutex> lock(mutex_);
        ArmState & arm = arms_[index];
        if (result.accepted) {
          arm.current = result.joints_rad;
          arm.consecutive_rejections = 0;
        } else {
          ++arm.consecutive_rejections;
        }
        arm.achieved = solver_->forward(arm.side, arm.current);
        if (!arm.achieved->matrix().allFinite()) {
          throw std::runtime_error("IK 后端 FK 返回非有限位姿");
        }
        arm.result = result;
        arm.error = result.saturated ?
          std::optional<std::string>(
          "IK 降级跟踪：" + result.status) :
          std::nullopt;
        publish_solved_pose_locked(index);
      } catch (const std::exception & exception) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++arms_[index].consecutive_rejections;
        arms_[index].error = exception.what();
        arms_[index].achieved =
          solver_->forward(arms_[index].side, arms_[index].current);
      }
    }
  }

  void sample_return()
  {
    const auto now = std::chrono::steady_clock::now();
    bool complete = true;
    std::lock_guard<std::mutex> lock(mutex_);
    for (ArmState & arm : arms_) {
      ReturnTrajectory & trajectory = arm.return_trajectory;
      if (!trajectory.active) {
        continue;
      }
      const double elapsed_s =
        std::chrono::duration<double>(now - trajectory.start_time).count();
      const double progress = std::clamp(
        elapsed_s / trajectory.duration_s, 0.0, 1.0);
      const double blend =
        progress * progress * (3.0 - 2.0 * progress);
      arm.current =
        trajectory.start + blend * (arm.home - trajectory.start);
      arm.achieved = solver_->forward(arm.side, arm.current);
      trajectory.active = progress < 1.0;
      complete = complete && !trajectory.active;
    }

    if (complete) {
      mode_ = "idle";
      at_home_.put("true");
      return_complete_.put("true");
      std::cerr << "IK 双臂已回到安全初始位" << std::endl;
    }
  }

  void publish_solved_pose_locked(std::size_t arm_index)
  {
    const ArmState & arm = arms_[arm_index];
    if (!arm.achieved.has_value()) {
      return;
    }
    std::ostringstream stream;
    stream << "{\"stamp\":" << json_stamp(stamp_now())
           << ",\"frame_id\":" << json_quote(arm.side_name + "_chest")
           << ',' << pose_json(*arm.achieved) << '}';
    solved_pose_publishers_[arm_index].put(zenoh::Bytes(stream.str()));
  }

  void publish_joint_states()
  {
    const Stamp stamp = stamp_now();
    std::string model_name;
    std::string model_position;
    model_name.reserve(256);
    model_position.reserve(256);

    for (std::size_t arm_index = 0; arm_index < arms_.size(); ++arm_index) {
      std::lock_guard<std::mutex> lock(mutex_);
      const ArmState & arm = arms_[arm_index];
      std::ostringstream stream;
      stream << "{\"stamp\":" << json_stamp(stamp)
             << ",\"frame_id\":"
             << json_quote(arm.side_name + "_base_marvin_degrees")
             << ",\"name\":[";
      for (Eigen::Index joint_index = 0; joint_index < 7; ++joint_index) {
        if (joint_index > 0) {
          stream << ',';
          model_name.push_back(',');
          model_position.push_back(',');
        }
        stream << json_quote(
          arm.side_name + "_joint_" + std::to_string(joint_index + 1));
        model_name += "\"Joint" + std::to_string(joint_index + 1) +
          "_" + arm.suffix + "\"";
        model_position += std::to_string(arm.current[joint_index]);
      }
      stream << "],\"position\":[";
      for (Eigen::Index joint_index = 0; joint_index < 7; ++joint_index) {
        if (joint_index > 0) {
          stream << ',';
        }
        stream << std::setprecision(10)
               << degrees(arm.current[joint_index]);
      }
      stream << "]}";
      joint_publishers_[arm_index].put(zenoh::Bytes(stream.str()));
    }

    std::ostringstream stream;
    stream << "{\"stamp\":" << json_stamp(stamp)
           << ",\"name\":[" << model_name << "],\"position\":["
           << model_position << "]}";
    model_joint_publisher_.put(zenoh::Bytes(stream.str()));
  }

  bool at_safe_home() const
  {
    return std::all_of(
      arms_.begin(),
      arms_.end(),
      [](const ArmState & arm) {
        return (arm.current - arm.home).cwiseAbs().maxCoeff() <=
               radians(0.1);
      });
  }

  std::string arm_status_json(const ArmState & arm) const
  {
    const std::optional<double> position_error_mm =
      arm.result.has_value() ?
      std::optional<double>(1000.0 * arm.result->position_error_m) :
      std::nullopt;
    const std::optional<double> orientation_error_deg =
      arm.result.has_value() ?
      std::optional<double>(
      degrees(arm.result->orientation_error_rad)) :
      std::nullopt;
    const std::optional<double> limit_margin_deg =
      arm.result.has_value() ?
      std::optional<double>(
      degrees(arm.result->minimum_limit_margin_rad)) :
      std::nullopt;
    const std::optional<double> maximum_step_deg =
      arm.result.has_value() ?
      std::optional<double>(
      degrees(arm.result->maximum_joint_step_rad)) :
      std::nullopt;
    const std::optional<double> requested_step_deg =
      arm.result.has_value() ?
      std::optional<double>(
      degrees(arm.result->requested_maximum_joint_step_rad)) :
      std::nullopt;
    const std::optional<double> solve_time_ms =
      arm.result.has_value() ?
      std::optional<double>(arm.result->solve_time_ms) : std::nullopt;
    const std::optional<double> transport_time_ms =
      arm.result.has_value() ?
      std::optional<double>(arm.result->transport_time_ms) : std::nullopt;
    const std::optional<double> backoff_fraction =
      arm.result.has_value() ?
      std::optional<double>(arm.result->workspace_backoff_fraction) :
      std::nullopt;
    const std::optional<double> minimum_singular_value =
      arm.result.has_value() ?
      std::optional<double>(arm.result->minimum_singular_value) :
      std::nullopt;
    const std::optional<double> damping =
      arm.result.has_value() ?
      std::optional<double>(arm.result->damping) :
      std::nullopt;
    const std::optional<double> arm_angle_error_deg =
      arm.result.has_value() ?
      std::optional<double>(
      degrees(arm.result->arm_angle_error_rad)) :
      std::nullopt;
    const std::optional<double> position_velocity_residual =
      arm.result.has_value() ?
      std::optional<double>(arm.result->position_velocity_residual_m_s) :
      std::nullopt;
    const std::optional<double> orientation_velocity_residual =
      arm.result.has_value() ?
      std::optional<double>(arm.result->orientation_velocity_residual_rad_s) :
      std::nullopt;
    const int solver_iterations =
      arm.result.has_value() ? arm.result->solver_iterations : 0;
    const int active_joint_constraints =
      arm.result.has_value() ? arm.result->active_joint_constraints : 0;
    const bool step_limited =
      arm.result.has_value() && arm.result->joint_step_limited;
    const bool saturated =
      arm.result.has_value() && arm.result->saturated;
    const bool singularity_active =
      arm.result.has_value() && arm.result->singularity_active;
    const bool soft_limit_active =
      arm.result.has_value() && arm.result->soft_limit_active;
    const bool workspace_backoff_active =
      arm.result.has_value() && arm.result->workspace_backoff_active;
    const bool orientation_relaxed =
      arm.result.has_value() && arm.result->orientation_relaxed;
    const bool transport_recovered =
      arm.result.has_value() && arm.result->transport_recovered;
    const std::optional<std::string> elbow_source =
      arm.elbow_direction.has_value() ?
      std::optional<std::string>("smpl_nullspace") :
      std::nullopt;
    const std::string prefix = arm.side_name;

    std::ostringstream stream;
    stream
      << json_quote(prefix + "_ik_error") << ':'
      << json_optional_string(arm.error) << ','
      << json_quote(prefix + "_position_error_mm") << ':'
      << json_optional_number(position_error_mm) << ','
      << json_quote(prefix + "_orientation_error_deg") << ':'
      << json_optional_number(orientation_error_deg) << ','
      << json_quote(prefix + "_min_limit_margin_deg") << ':'
      << json_optional_number(limit_margin_deg) << ','
      << json_quote(prefix + "_max_joint_step_deg") << ':'
      << json_optional_number(maximum_step_deg) << ','
      << json_quote(prefix + "_requested_max_joint_step_deg") << ':'
      << json_optional_number(requested_step_deg) << ','
      << json_quote(prefix + "_joint_step_limited") << ':'
      << json_bool(step_limited) << ','
      << json_quote(prefix + "_elbow_constraint_source") << ':'
      << json_optional_string(elbow_source) << ','
      << json_quote(prefix + "_target_saturated") << ':'
      << json_bool(saturated) << ','
      << json_quote(prefix + "_requested_target_error") << ':'
      << (saturated ? json_optional_string(arm.error) : "null") << ','
      << json_quote(prefix + "_min_singular_value") << ':'
      << json_optional_number(minimum_singular_value) << ','
      << json_quote(prefix + "_damping") << ':'
      << json_optional_number(damping) << ','
      << json_quote(prefix + "_arm_angle_error_deg") << ':'
      << json_optional_number(arm_angle_error_deg) << ','
      << json_quote(prefix + "_position_velocity_residual_m_s") << ':'
      << json_optional_number(position_velocity_residual) << ','
      << json_quote(prefix + "_orientation_velocity_residual_rad_s") << ':'
      << json_optional_number(orientation_velocity_residual) << ','
      << json_quote(prefix + "_solver_iterations") << ':'
      << solver_iterations << ','
      << json_quote(prefix + "_active_joint_constraints") << ':'
      << active_joint_constraints << ','
      << json_quote(prefix + "_singularity_active") << ':'
      << json_bool(singularity_active) << ','
      << json_quote(prefix + "_ik_status") << ':'
      << (arm.result.has_value() ? json_quote(arm.result->status) : "null") << ','
      << json_quote(prefix + "_solve_time_ms") << ':'
      << json_optional_number(solve_time_ms) << ','
      << json_quote(prefix + "_transport_time_ms") << ':'
      << json_optional_number(transport_time_ms) << ','
      << json_quote(prefix + "_candidate_count") << ':'
      << (arm.result.has_value() ? arm.result->candidate_count : 0) << ','
      << json_quote(prefix + "_selected_candidate_index") << ':'
      << (arm.result.has_value() ? arm.result->selected_candidate_index : -1) << ','
      << json_quote(prefix + "_soft_limit_active") << ':'
      << json_bool(soft_limit_active) << ','
      << json_quote(prefix + "_workspace_backoff_active") << ':'
      << json_bool(workspace_backoff_active) << ','
      << json_quote(prefix + "_workspace_backoff_fraction") << ':'
      << json_optional_number(backoff_fraction) << ','
      << json_quote(prefix + "_orientation_relaxed") << ':'
      << json_bool(orientation_relaxed) << ','
      << json_quote(prefix + "_transport_restart_count") << ':'
      << (arm.result.has_value() ? arm.result->transport_restart_count : 0) << ','
      << json_quote(prefix + "_transport_recovered") << ':'
      << json_bool(transport_recovered) << ','
      << json_quote(prefix + "_consecutive_rejections") << ':'
      << arm.consecutive_rejections;
    return stream.str();
  }

  void publish_status()
  {
    std::ostringstream stream;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stream
        << '{'
        << "\"mode\":" << json_quote(mode_) << ','
        << "\"at_safe_home\":" << json_bool(at_safe_home()) << ','
        << arm_status_json(arms_[kLeftIndex]) << ','
        << arm_status_json(arms_[kRightIndex]) << ','
        << "\"sdk\":" << json_quote(ik_backend_) << ','
        << "\"ik_interface\":\"arm_ik_solver_v1\","
        << "\"ik_backend\":" << json_quote(ik_backend_) << ','
        << "\"robot_connected\":false,"
        << "\"scope\":\"preview_only\""
        << '}';
    }
    status_publisher_.put(zenoh::Bytes(stream.str()));
  }

  void run()
  {
    const auto tick_interval =
      std::chrono::duration<double>(1.0 / rate_hz_);
    auto next_tick = std::chrono::steady_clock::now() + tick_interval;
    auto next_status = next_tick + std::chrono::milliseconds(500);
    while (true) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_tick) {
        tick();
        next_tick += tick_interval;
      }
      if (now >= next_status) {
        publish_status();
        next_status += std::chrono::milliseconds(500);
      }
      std::this_thread::sleep_until(std::min(next_tick, next_status));
    }
  }

  zenoh::Session & session_;
  std::unique_ptr<ArmIkSolver> solver_;
  std::array<ArmState, 2> arms_;
  std::string ik_backend_;
  std::string mode_{"idle"};
  double rate_hz_{30.0};
  double home_minimum_duration_s_{2.0};
  double home_max_speed_deg_s_{25.0};
  double maximum_joint_step_rad_{radians(3.0)};
  bool arm_angle_required_{false};
  std::mutex mutex_;

  std::array<zenoh::Publisher, 2> joint_publishers_;
  std::array<zenoh::Publisher, 2> solved_pose_publishers_;
  zenoh::Publisher model_joint_publisher_;
  zenoh::Publisher status_publisher_;

  std::array<zenoh::Subscriber<void>, 2> pose_subscriptions_;
  std::array<zenoh::Subscriber<void>, 2> direction_subscriptions_;
  zenoh::Subscriber<void> teleop_state_subscription_;

  LatchedValue at_home_{session_, topic_key("/pico_body_sim/at_home"), "true"};
  LatchedValue return_complete_{
    session_, topic_key("/pico_body_sim/return_complete"), "false"};
};

}  // namespace pico_body_tianji

int main(int argc, char ** argv)
{
  try {
    std::vector<std::string> assignments;
    for (int index = 1; index < argc; ++index) {
      assignments.emplace_back(argv[index]);
    }
    const pico_body_tianji::ParamMap params(assignments);
    zenoh::Session session = zenoh::Session::open(zenoh::Config::create());
    pico_body_tianji::TianjiKinematicSimNode node(session, params);
    node.run();
  } catch (const std::exception & exception) {
    std::cerr << "tianji_kinematic_sim 节点启动或运行失败："
              << exception.what() << std::endl;
    return 1;
  }
  return 0;
}
