#include "tianji_teleop/marvin/native_driver.h"

#include <dlfcn.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

using Clock = std::chrono::steady_clock;
constexpr std::size_t kJoints = TIANJI_MARVIN_JOINTS;
constexpr std::size_t kArms = TIANJI_MARVIN_ARMS;
constexpr int kClearSetAttempts = 3;
constexpr auto kPositionModeRetryPeriod = std::chrono::milliseconds(100);

struct StateCtr {
  int current;
  int command;
  int error;
};

struct RtIn {
  int realtime_switch;
  int impedance_type;
  int input_frame_serial;
  short frame_miss_count;
  short maximum_frame_miss_count;
  int system_cycle;
  short system_cycle_miss_count;
  short maximum_system_cycle_miss_count;
  float tool_kinematics[6];
  float tool_dynamics[10];
  float joint_command_position[7];
  short joint_velocity_ratio;
  short joint_acceleration_ratio;
  float joint_stiffness[7];
  float joint_damping[7];
  int drag_space_type;
  float drag_space_parameters[6];
  int cartesian_kd_type;
  float cartesian_stiffness[6];
  float cartesian_damping[6];
  float cartesian_nullspace_stiffness;
  float cartesian_nullspace_damping;
  int force_feedback_type;
  int force_type;
  float force_direction[6];
  float force_pid_limits[7];
  float force_adjustment_limit;
  float force_command;
  unsigned char set_tags[16];
  unsigned char update_tags[16];
  unsigned char pvt_id;
  unsigned char pvt_id_update;
  unsigned char pvt_run_id;
  unsigned char pvt_run_state;
};

struct RtOut {
  int output_frame_serial;
  float feedback_joint_position[7];
  float feedback_joint_velocity[7];
  float feedback_joint_external_position[7];
  float feedback_joint_command[7];
  float feedback_joint_command_torque[7];
  float feedback_joint_sensor_torque[7];
  float feedback_joint_temperature[7];
  float estimated_joint_friction[7];
  float estimated_joint_friction_derivative[7];
  float estimated_joint_force[7];
  float estimated_cartesian_force[6];
  char tip_digital_input;
  char low_speed_flag;
  char padding;
  char trajectory_state;
};

struct Dcss {
  StateCtr state[2];
  RtIn input[2];
  RtOut output[2];
  char parameter_name[30];
  unsigned char parameter_type;
  unsigned char parameter_instruction;
  int parameter_integer;
  float parameter_float;
  short parameter_command_serial;
  short parameter_return_serial;
};

static_assert(sizeof(StateCtr) == 12);
static_assert(sizeof(RtIn) == 368);
static_assert(sizeof(RtOut) == 312);
static_assert(sizeof(Dcss) == 1428);

template<typename T>
T symbol(void *library, const char *name) {
  dlerror();
  auto *address = dlsym(library, name);
  if (const char *error = dlerror(); error != nullptr) {
    throw std::runtime_error(std::string("Marvin SDK missing ") + name + ": " + error);
  }
  return reinterpret_cast<T>(address);
}

void copy_error(char *buffer, std::size_t size, const std::string &message) {
  if (buffer == nullptr || size == 0) return;
  const auto count = std::min(size - 1, message.size());
  std::memcpy(buffer, message.data(), count);
  buffer[count] = '\0';
}

class NativeDriver {
public:
  NativeDriver(const char *sdk_path, const TianjiMarvinNativeConfig &config)
  : config_(config) {
    validate_config();
    period_ = std::chrono::nanoseconds(1000000000LL / config_.rate_hz);
    library_ = dlopen(sdk_path, RTLD_NOW | RTLD_LOCAL);
    if (library_ == nullptr) throw std::runtime_error(std::string("unable to load Marvin SDK: ") + dlerror());
    try {
      connect_ = symbol<ConnectFn>(library_, "Connect");
      get_buffer_ = symbol<GetBufferFn>(library_, "OnGetBuf");
      set_tool_ = symbol<SetToolFn>(library_, "SetTool");
      set_impedance_ = symbol<SetImpedanceFn>(library_, "SetImpJointMode");
      set_position_ = symbol<SetPositionFn>(library_, "SetJointPostionCmd");
      clear_set_ = symbol<ClearSetFn>(library_, "OnClearSet");
      set_joint_position_[0] = symbol<SetJointPositionFn>(library_, "OnSetJointCmdPos_A");
      set_joint_position_[1] = symbol<SetJointPositionFn>(library_, "OnSetJointCmdPos_B");
      set_joint_limit_[0] = symbol<SetJointLimitFn>(library_, "OnSetJointLmt_A");
      set_joint_limit_[1] = symbol<SetJointLimitFn>(library_, "OnSetJointLmt_B");
      set_target_state_[0] = symbol<SetTargetStateFn>(library_, "OnSetTargetState_A");
      set_target_state_[1] = symbol<SetTargetStateFn>(library_, "OnSetTargetState_B");
      send_ = symbol<SendFn>(library_, "OnSetSend");
      set_velocity_step_ = symbol<SetVelocityStepFn>(library_, "FX_OnSetVelEstStep");
      emergency_stop_ = symbol<EmergencyStopFn>(library_, "EStop");
      disable_ = symbol<DisableFn>(library_, "Disable");
      release_ = symbol<ReleaseFn>(library_, "OnRelease");
      servo_errors_[0] = symbol<GetServoErrorsFn>(library_, "OnGetServoErr_A");
      servo_errors_[1] = symbol<GetServoErrorsFn>(library_, "OnGetServoErr_B");
    } catch (...) {
      dlclose(library_);
      library_ = nullptr;
      throw;
    }
  }

  ~NativeDriver() {
    shutdown();
    if (library_ != nullptr) dlclose(library_);
  }

  void connect(const std::string &ip) {
    if (connected_) throw std::runtime_error("Marvin native driver is already connected");
    const auto octets = parse_ip(ip);
    if (!connect_(octets[0], octets[1], octets[2], octets[3], 0)) {
      throw std::runtime_error("Marvin SDK Connect failed");
    }
    connected_ = true;
    try {
      Dcss initial{};
      wait_for_feedback(initial, 1.0);
      require_safe_feedback(initial, false);
      require(set_tool_('A', config_.tool_kinematics, config_.tool_dynamics[0]), "SetTool(A)");
      require(set_tool_('B', config_.tool_kinematics, config_.tool_dynamics[1]), "SetTool(B)");
      Dcss tool_ready{};
      wait_for_tool(tool_ready, 2.0);
      std::array<double, kJoints> left{};
      std::array<double, kJoints> right{};
      for (std::size_t joint = 0; joint < kJoints; ++joint) {
        left[joint] = tool_ready.output[0].feedback_joint_position[joint];
        right[joint] = tool_ready.output[1].feedback_joint_position[joint];
      }
      require(set_position_('A', left.data()), "seed SetJointPostionCmd(A)");
      require(set_position_('B', right.data()), "seed SetJointPostionCmd(B)");
      require(set_impedance_('A', config_.velocity_ratio, config_.acceleration_ratio,
                             config_.joint_stiffness, config_.joint_damping),
              "SetImpJointMode(A)");
      require(set_impedance_('B', config_.velocity_ratio, config_.acceleration_ratio,
                             config_.joint_stiffness, config_.joint_damping),
              "SetImpJointMode(B)");
      require(set_velocity_step_('A', config_.velocity_estimation_step_ms),
              "FX_OnSetVelEstStep(A)");
      require(set_velocity_step_('B', config_.velocity_estimation_step_ms),
              "FX_OnSetVelEstStep(B)");
      Dcss ready{};
      wait_for_impedance(ready, left, right, 5.0);
      update_feedback(ready, true);
      {
        std::lock_guard<std::mutex> guard(target_mutex_);
        targets_[0] = left;
        targets_[1] = right;
        target_updated_ = Clock::now();
        have_target_ = true;
      }
      start_worker();
    } catch (...) {
      soft_stop("native driver prepare failed");
      shutdown();
      throw;
    }
  }

  void set_position_mode() {
    if (!connected_) throw std::runtime_error("Marvin native driver is not connected");
    if (soft_stopped_) throw std::runtime_error("Marvin native driver is soft-stopped");
    stop_worker();
    if (soft_stopped_) throw std::runtime_error("Marvin native driver is soft-stopped");
    expected_control_state_.store(1);
    try {
      Dcss current{};
      wait_for_feedback(current, 1.0);
      require_safe_feedback(current, false);
      const auto left = feedback_joints(current, 0);
      const auto right = feedback_joints(current, 1);
      send_position_mode_batch(left, right);
      Dcss ready{};
      wait_for_position(ready, left, right, 5.0);
      update_feedback(ready, true);
      refresh_target(feedback_joints(ready, 0), feedback_joints(ready, 1));
      start_worker();
    } catch (...) {
      soft_stop("native position-mode transition failed");
      throw;
    }
  }
  void set_impedance_mode() {
    if (!connected_) throw std::runtime_error("Marvin native driver is not connected");
    if (soft_stopped_) throw std::runtime_error("Marvin native driver is soft-stopped");
    stop_worker();
    if (soft_stopped_) throw std::runtime_error("Marvin native driver is soft-stopped");
    expected_control_state_.store(3);
    try {
      Dcss current{};
      wait_for_feedback(current, 1.0);
      require_safe_feedback(current, false);
      const auto left = feedback_joints(current, 0);
      const auto right = feedback_joints(current, 1);
      require(set_position_('A', left.data()), "seed impedance command(A)");
      require(set_position_('B', right.data()), "seed impedance command(B)");
      require(set_impedance_('A', config_.velocity_ratio, config_.acceleration_ratio,
                             config_.joint_stiffness, config_.joint_damping),
              "SetImpJointMode(A)");
      require(set_impedance_('B', config_.velocity_ratio, config_.acceleration_ratio,
                             config_.joint_stiffness, config_.joint_damping),
              "SetImpJointMode(B)");
      require(set_velocity_step_('A', config_.velocity_estimation_step_ms),
              "FX_OnSetVelEstStep(A)");
      require(set_velocity_step_('B', config_.velocity_estimation_step_ms),
              "FX_OnSetVelEstStep(B)");
      Dcss ready{};
      wait_for_impedance(ready, left, right, 5.0);
      update_feedback(ready, true);
      refresh_target(feedback_joints(ready, 0), feedback_joints(ready, 1));
      start_worker();
    } catch (...) {
      soft_stop("native impedance-mode transition failed");
      throw;
    }
  }

  void submit(const double *left, const double *right) {
    if (!connected_) throw std::runtime_error("Marvin native driver is not connected");
    if (soft_stopped_) throw std::runtime_error("Marvin native driver is soft-stopped");
    std::array<double, kJoints> next_left{};
    std::array<double, kJoints> next_right{};
    for (std::size_t joint = 0; joint < kJoints; ++joint) {
      if (!std::isfinite(left[joint]) || !std::isfinite(right[joint])) {
        throw std::invalid_argument("Marvin native target contains nonfinite joint");
      }
      next_left[joint] = left[joint];
      next_right[joint] = right[joint];
    }
    {
      std::lock_guard<std::mutex> guard(target_mutex_);
      targets_[0] = next_left;
      targets_[1] = next_right;
      target_updated_ = Clock::now();
      have_target_ = true;
    }
  }

  TianjiMarvinNativeFeedback read() const {
    std::lock_guard<std::mutex> guard(feedback_mutex_);
    if (!have_feedback_) throw std::runtime_error("Marvin native feedback is not ready");
    return feedback_;
  }

  void soft_stop(const std::string &reason) noexcept {
    bool expected = false;
    if (!soft_stopped_.compare_exchange_strong(expected, true)) return;
    {
      std::lock_guard<std::mutex> guard(feedback_mutex_);
      last_error_ = reason;
      feedback_.soft_stopped = 1;
      feedback_.healthy = 0;
    }
    if (connected_) emergency_stop_("AB");
  }

private:
  using ConnectFn = bool (*)(unsigned char, unsigned char, unsigned char, unsigned char, int);
  using GetBufferFn = bool (*)(Dcss *);
  using SetToolFn = bool (*)(char, const double *, const double *);
  using SetImpedanceFn = bool (*)(char, int, int, const double *, const double *);
  using SetPositionFn = bool (*)(char, const double *);
  using ClearSetFn = bool (*)();
  using SetJointPositionFn = bool (*)(const double *);
  using SetJointLimitFn = bool (*)(int, int);
  using SetTargetStateFn = bool (*)(int);
  using SendFn = bool (*)();
  using SetVelocityStepFn = bool (*)(char, long);
  using EmergencyStopFn = void (*)(const char *);
  using DisableFn = bool (*)(char);
  using ReleaseFn = bool (*)();
  using GetServoErrorsFn = long (*)(long *);

  void validate_config() const {
    if (config_.rate_hz != 200 || config_.velocity_estimation_step_ms != 5) {
      throw std::invalid_argument("Marvin native driver requires 200 Hz and 5 ms velocity estimation");
    }
    if (config_.velocity_ratio < 1 || config_.velocity_ratio > 100 ||
        config_.acceleration_ratio < 1 || config_.acceleration_ratio > 100 ||
        !(config_.command_timeout_s > 0.0)) {
      throw std::invalid_argument("Marvin native driver ratios/timeout are invalid");
    }
    for (std::size_t joint = 0; joint < kJoints; ++joint) {
      if (!std::isfinite(config_.joint_stiffness[joint]) ||
          config_.joint_stiffness[joint] < 0.0 || config_.joint_stiffness[joint] > 22.0 ||
          !std::isfinite(config_.joint_damping[joint]) ||
          config_.joint_damping[joint] < 0.0 || config_.joint_damping[joint] > 1.0) {
        throw std::invalid_argument("Marvin native driver K/D are outside SDK ranges");
      }
    }
    for (const double value : config_.tool_kinematics) {
      if (!std::isfinite(value)) throw std::invalid_argument("Marvin tool kinematics are invalid");
    }
    for (const auto &arm_dynamics : config_.tool_dynamics) {
      if (arm_dynamics[0] < 0.0) throw std::invalid_argument("Marvin tool mass must be nonnegative");
      for (const double value : arm_dynamics) {
        if (!std::isfinite(value)) throw std::invalid_argument("Marvin tool dynamics are invalid");
      }
    }
  }

  static std::array<unsigned char, 4> parse_ip(const std::string &ip) {
    std::array<unsigned char, 4> result{};
    std::size_t cursor = 0;
    for (std::size_t index = 0; index < result.size(); ++index) {
      const auto separator = ip.find('.', cursor);
      const auto end = index + 1 == result.size() ? ip.size() : separator;
      if (end == std::string::npos || end == cursor) throw std::invalid_argument("invalid Marvin robot IP");
      std::size_t used = 0;
      const auto value = std::stoi(ip.substr(cursor, end - cursor), &used);
      if (used != end - cursor || value < 0 || value > 255) throw std::invalid_argument("invalid Marvin robot IP");
      result[index] = static_cast<unsigned char>(value);
      cursor = end + 1;
    }
    return result;
  }

  static void require(bool value, const char *operation) {
    if (!value) throw std::runtime_error(std::string("Marvin SDK call failed: ") + operation);
  }

  [[noreturn]] static void throw_mode_timeout(
      const char *mode, const Dcss &last) {
    char message[512]{};
    std::snprintf(
      message, sizeof(message),
      "Marvin arms did not enter verified joint %s mode: "
      "A{current=%d,command=%d,error=%d,velocity=%d,acceleration=%d,"
      "frame_serial=%d,impedance_type=%d} "
      "B{current=%d,command=%d,error=%d,velocity=%d,acceleration=%d,"
      "frame_serial=%d,impedance_type=%d}",
      mode,
      last.state[0].current, last.state[0].command, last.state[0].error,
      last.input[0].joint_velocity_ratio,
      last.input[0].joint_acceleration_ratio,
      last.output[0].output_frame_serial, last.input[0].impedance_type,
      last.state[1].current, last.state[1].command, last.state[1].error,
      last.input[1].joint_velocity_ratio,
      last.input[1].joint_acceleration_ratio,
      last.output[1].output_frame_serial, last.input[1].impedance_type);
    throw std::runtime_error(message);
  }

  void clear_batch() {
    bool cleared = false;
    for (int attempt = 0; attempt < kClearSetAttempts; ++attempt) {
      if (clear_set_()) {
        cleared = true;
        break;
      }
      if (attempt + 1 < kClearSetAttempts) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
    require(cleared, "OnClearSet");
  }

  void stage_joint_positions(
      const std::array<double, kJoints> &left,
      const std::array<double, kJoints> &right) {
    clear_batch();
    require(set_joint_position_[0](left.data()), "OnSetJointCmdPos_A");
    require(set_joint_position_[1](right.data()), "OnSetJointCmdPos_B");
  }

  void stage_position_mode(std::size_t arm) {
    require(
      set_joint_limit_[arm](config_.velocity_ratio, config_.acceleration_ratio),
      arm == 0 ? "OnSetJointLmt_A" : "OnSetJointLmt_B");
    require(
      set_target_state_[arm](1),
      arm == 0 ? "OnSetTargetState_A" : "OnSetTargetState_B");
  }

  void send_joint_position_batch(
      const std::array<double, kJoints> &left,
      const std::array<double, kJoints> &right) {
    stage_joint_positions(left, right);
    require(send_(), "OnSetSend");
  }

  void send_position_hold_batch(
      const std::array<double, kJoints> &left,
      const std::array<double, kJoints> &right,
      const std::array<bool, kArms> &retry_mode) {
    stage_joint_positions(left, right);
    for (std::size_t arm = 0; arm < kArms; ++arm) {
      if (retry_mode[arm]) stage_position_mode(arm);
    }
    require(send_(), "OnSetSend");
  }

  void send_position_mode_batch(
      const std::array<double, kJoints> &left,
      const std::array<double, kJoints> &right) {
    stage_joint_positions(left, right);
    stage_position_mode(0);
    stage_position_mode(1);
    require(send_(), "OnSetSend");
  }

  bool position_arm_ready(const Dcss &data, std::size_t arm) const {
    return data.state[arm].current == 1 &&
      (data.state[arm].command == 1 || data.state[arm].command == -1) &&
      data.state[arm].error == 0 &&
      data.input[arm].joint_velocity_ratio >= config_.velocity_ratio - 1 &&
      data.input[arm].joint_velocity_ratio <= config_.velocity_ratio &&
      data.input[arm].joint_acceleration_ratio >= config_.acceleration_ratio - 1 &&
      data.input[arm].joint_acceleration_ratio <= config_.acceleration_ratio;
  }

  void wait_for_feedback(Dcss &result, double timeout_s) {
    const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_s);
    int previous[2] = {-1, -1};
    bool have_previous = false;
    while (Clock::now() < deadline) {
      Dcss current{};
      require(get_buffer_(&current), "OnGetBuf");
      if (have_previous && current.output[0].output_frame_serial != previous[0] &&
          current.output[1].output_frame_serial != previous[1]) {
        result = current;
        return;
      }
      previous[0] = current.output[0].output_frame_serial;
      previous[1] = current.output[1].output_frame_serial;
      have_previous = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    throw std::runtime_error("Marvin feedback frame serial did not advance");
  }

  void wait_for_tool(Dcss &result, double timeout_s) {
    const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_s);
    while (Clock::now() < deadline) {
      Dcss current{};
      require(get_buffer_(&current), "OnGetBuf");
      require_safe_feedback(current, false);
      bool ready = true;
      for (std::size_t arm = 0; arm < kArms; ++arm) {
        for (std::size_t index = 0; index < 6; ++index) {
          ready = ready && std::abs(current.input[arm].tool_kinematics[index] - config_.tool_kinematics[index]) <= 1e-3;
        }
        for (std::size_t index = 0; index < 10; ++index) {
          ready = ready && std::abs(current.input[arm].tool_dynamics[index] - config_.tool_dynamics[arm][index]) <= 1e-3;
        }
      }
      if (ready) { result = current; return; }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::runtime_error("Marvin tool dynamics readback did not match configuration");
  }

  void wait_for_impedance(
      Dcss &result, const std::array<double, kJoints> &left,
      const std::array<double, kJoints> &right, double timeout_s) {
    const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_s);
    Dcss last{};
    while (Clock::now() < deadline) {
      require(set_position_('A', left.data()), "hold SetJointPostionCmd(A)");
      require(set_position_('B', right.data()), "hold SetJointPostionCmd(B)");
      require(get_buffer_(&last), "OnGetBuf");
      require_safe_feedback(last, true);
      bool ready = true;
      for (std::size_t arm = 0; arm < kArms; ++arm) {
        ready = ready && last.state[arm].current == 3 &&
          (last.state[arm].command == 3 || last.state[arm].command == -1) &&
          last.input[arm].impedance_type == 1 &&
          last.input[arm].joint_velocity_ratio >= config_.velocity_ratio - 1 &&
          last.input[arm].joint_velocity_ratio <= config_.velocity_ratio &&
          last.input[arm].joint_acceleration_ratio >= config_.acceleration_ratio - 1 &&
          last.input[arm].joint_acceleration_ratio <= config_.acceleration_ratio;
        for (std::size_t joint = 0; joint < kJoints; ++joint) {
          ready = ready && std::abs(last.input[arm].joint_stiffness[joint] - config_.joint_stiffness[joint]) <= 1e-3 &&
            std::abs(last.input[arm].joint_damping[joint] - config_.joint_damping[joint]) <= 1e-3;
        }
      }
      if (ready) {
        result = last;
        return;
      }
      std::this_thread::sleep_for(period_);
    }
    throw_mode_timeout("impedance", last);
  }

  void wait_for_position(
      Dcss &result, const std::array<double, kJoints> &left,
      const std::array<double, kJoints> &right, double timeout_s) {
    const auto started = Clock::now();
    const auto deadline = started + std::chrono::duration<double>(timeout_s);
    std::array<Clock::time_point, kArms> next_mode_retry{};
    next_mode_retry.fill(started + kPositionModeRetryPeriod);
    std::array<bool, kArms> mode_confirmed{};
    bool have_feedback = false;
    Dcss last{};
    while (Clock::now() < deadline) {
      const auto now = Clock::now();
      std::array<bool, kArms> retry_mode{};
      for (std::size_t arm = 0; arm < kArms; ++arm) {
        retry_mode[arm] =
          have_feedback && !mode_confirmed[arm] &&
          now >= next_mode_retry[arm];
      }
      send_position_hold_batch(left, right, retry_mode);
      for (std::size_t arm = 0; arm < kArms; ++arm) {
        if (retry_mode[arm]) {
          next_mode_retry[arm] = Clock::now() + kPositionModeRetryPeriod;
        }
      }
      require(get_buffer_(&last), "OnGetBuf");
      require_safe_feedback(last, true);
      bool ready = true;
      for (std::size_t arm = 0; arm < kArms; ++arm) {
        const bool arm_ready = position_arm_ready(last, arm);
        mode_confirmed[arm] = mode_confirmed[arm] || arm_ready;
        ready = ready && arm_ready;
      }
      have_feedback = true;
      if (ready) {
        result = last;
        return;
      }
      std::this_thread::sleep_for(period_);
    }
    throw_mode_timeout("position", last);
  }

  static std::array<double, kJoints> feedback_joints(const Dcss &data, std::size_t arm) {
    std::array<double, kJoints> joints{};
    for (std::size_t joint = 0; joint < kJoints; ++joint) {
      joints[joint] = data.output[arm].feedback_joint_position[joint];
    }
    return joints;
  }

  void refresh_target(const std::array<double, kJoints> &left,
                     const std::array<double, kJoints> &right) {
    std::lock_guard<std::mutex> guard(target_mutex_);
    targets_[0] = left;
    targets_[1] = right;
    target_updated_ = Clock::now();
    have_target_ = true;
  }

  static bool state_can_still_reach_target(const StateCtr &state) {
    if (state.error == 0 &&
        ((state.current >= 0 && state.current <= 3) ||
         (state.current >= 101 && state.current <= 103))) {
      return true;
    }
    if (state.current >= 101 && state.current <= 103 &&
        (state.error == 4 || state.error == 6 || state.error == 8)) {
      return true;
    }
    return state.current == 100 && state.error == 6;
  }

  void require_safe_feedback(const Dcss &data, bool allow_transition) {
    for (std::size_t arm = 0; arm < kArms; ++arm) {
      if ((allow_transition &&
           !state_can_still_reach_target(data.state[arm])) ||
          (!allow_transition && data.state[arm].error != 0)) {
        throw std::runtime_error("Marvin arm error during native prepare");
      }
      if (!allow_transition && data.state[arm].current == 100) {
        throw std::runtime_error("Marvin arm fault during native prepare");
      }
      for (const float joint : data.output[arm].feedback_joint_position) {
        if (!std::isfinite(joint)) throw std::runtime_error("Marvin feedback contains nonfinite joint");
      }
      std::array<long, kJoints> errors{};
      servo_errors_[arm](errors.data());
      if (std::any_of(errors.begin(), errors.end(), [](long value) { return value != 0; })) {
        throw std::runtime_error("Marvin servo error during native prepare");
      }
    }
  }

  void update_feedback(const Dcss &data, bool include_servo_errors) {
    TianjiMarvinNativeFeedback next{};
    const int expected_state = expected_control_state_.load();
    bool healthy = !soft_stopped_;
    for (std::size_t arm = 0; arm < kArms; ++arm) {
      next.arm_states[arm] = data.state[arm].current;
      next.command_states[arm] = data.state[arm].command;
      next.error_codes[arm] = data.state[arm].error;
      next.frame_serials[arm] = data.output[arm].output_frame_serial;
      next.velocity_ratios[arm] = data.input[arm].joint_velocity_ratio;
      next.acceleration_ratios[arm] = data.input[arm].joint_acceleration_ratio;
      next.impedance_types[arm] = data.input[arm].impedance_type;
      const bool command_compatible = next.command_states[arm] == expected_state ||
        next.command_states[arm] == -1;
      const bool mode_matches = next.arm_states[arm] == expected_state &&
        (expected_state != 3 || next.impedance_types[arm] == 1);
      healthy = healthy && command_compatible && mode_matches &&
        next.error_codes[arm] == 0 &&
        next.velocity_ratios[arm] >= config_.velocity_ratio - 1 &&
        next.velocity_ratios[arm] <= config_.velocity_ratio &&
        next.acceleration_ratios[arm] >= config_.acceleration_ratio - 1 &&
        next.acceleration_ratios[arm] <= config_.acceleration_ratio;
      for (std::size_t joint = 0; joint < kJoints; ++joint) {
        next.joints_deg[arm][joint] = data.output[arm].feedback_joint_position[joint];
      }
      if (include_servo_errors || next.error_codes[arm] != 0) {
        servo_errors_[arm](next.servo_error_codes[arm]);
        healthy = healthy && std::none_of(
          std::begin(next.servo_error_codes[arm]), std::end(next.servo_error_codes[arm]),
          [](long value) { return value != 0; });
      }
    }
    next.control_ticks = ticks_.load();
    next.deadline_misses = deadline_misses_.load();
    next.healthy = healthy ? 1 : 0;
    next.soft_stopped = soft_stopped_ ? 1 : 0;
    std::lock_guard<std::mutex> guard(feedback_mutex_);
    feedback_ = next;
    have_feedback_ = true;
  }

  void run() noexcept {
    auto next = Clock::now();
    std::uint64_t cycle = 0;
    while (running_) {
      next += period_;
      try {
        Dcss data{};
        require(get_buffer_(&data), "OnGetBuf");
        update_feedback(data, cycle % 200 == 0);
        if (!read().healthy) throw std::runtime_error("unsafe Marvin feedback in native loop");
        std::array<std::array<double, kJoints>, kArms> target{};
        Clock::time_point updated;
        bool have_target = false;
        {
          std::lock_guard<std::mutex> guard(target_mutex_);
          target = targets_;
          updated = target_updated_;
          have_target = have_target_;
        }
        if (!have_target || Clock::now() - updated > std::chrono::duration<double>(config_.command_timeout_s)) {
          throw std::runtime_error("native command watchdog expired");
        }
        send_joint_position_batch(target[0], target[1]);
        ++ticks_;
        ++cycle;
      } catch (const std::exception &error) {
        soft_stop(error.what());
        running_ = false;
        break;
      }
      const auto now = Clock::now();
      if (now > next) {
        ++deadline_misses_;
        next = now;
      } else {
        std::this_thread::sleep_until(next);
      }
    }
  }

  void start_worker() {
    if (worker_.joinable()) throw std::runtime_error("Marvin native worker is already running");
    running_ = true;
    worker_ = std::thread([this] { run(); });
  }

  void stop_worker() noexcept {
    running_ = false;
    if (worker_.joinable()) worker_.join();
  }

  void shutdown() noexcept {
    stop_worker();
    if (!connected_) return;
    if (!soft_stopped_) {
      disable_('A');
      disable_('B');
    }
    release_();
    connected_ = false;
  }

  TianjiMarvinNativeConfig config_{};
  std::chrono::nanoseconds period_{};
  void *library_{nullptr};
  ConnectFn connect_{nullptr};
  GetBufferFn get_buffer_{nullptr};
  SetToolFn set_tool_{nullptr};
  SetImpedanceFn set_impedance_{nullptr};
  SetPositionFn set_position_{nullptr};
  ClearSetFn clear_set_{nullptr};
  SetJointPositionFn set_joint_position_[2]{};
  SetJointLimitFn set_joint_limit_[2]{};
  SetTargetStateFn set_target_state_[2]{};
  SendFn send_{nullptr};
  SetVelocityStepFn set_velocity_step_{nullptr};
  EmergencyStopFn emergency_stop_{nullptr};
  DisableFn disable_{nullptr};
  ReleaseFn release_{nullptr};
  GetServoErrorsFn servo_errors_[2]{};
  std::atomic<bool> connected_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> soft_stopped_{false};
  std::atomic<int> expected_control_state_{3};
  std::thread worker_;
  mutable std::mutex target_mutex_;
  std::array<std::array<double, kJoints>, kArms> targets_{};
  Clock::time_point target_updated_{};
  bool have_target_{false};
  mutable std::mutex feedback_mutex_;
  TianjiMarvinNativeFeedback feedback_{};
  bool have_feedback_{false};
  std::string last_error_;
  std::atomic<std::uint64_t> ticks_{0};
  std::atomic<std::uint64_t> deadline_misses_{0};
};

template<typename Function>
int invoke(void *handle, char *error, std::size_t error_size, Function function) {
  try {
    if (handle == nullptr) throw std::invalid_argument("Marvin native handle is null");
    function(*static_cast<NativeDriver *>(handle));
    copy_error(error, error_size, "");
    return 1;
  } catch (const std::exception &exception) {
    copy_error(error, error_size, exception.what());
    return 0;
  }
}

}  // namespace

extern "C" void *tianji_marvin_native_create(
  const char *sdk_library, const TianjiMarvinNativeConfig *config,
  char *error, std::size_t error_size) {
  try {
    if (sdk_library == nullptr || config == nullptr) throw std::invalid_argument("Marvin native create arguments are null");
    auto *driver = new NativeDriver(sdk_library, *config);
    copy_error(error, error_size, "");
    return driver;
  } catch (const std::exception &exception) {
    copy_error(error, error_size, exception.what());
    return nullptr;
  }
}

extern "C" int tianji_marvin_native_connect(
  void *handle, const char *robot_ip, char *error, std::size_t error_size) {
  return invoke(handle, error, error_size, [robot_ip](NativeDriver &driver) {
    if (robot_ip == nullptr) throw std::invalid_argument("Marvin robot IP is null");
    driver.connect(robot_ip);
  });
}

extern "C" int tianji_marvin_native_set_position_mode(
  void *handle, char *error, std::size_t error_size) {
  return invoke(handle, error, error_size, [](NativeDriver &driver) {
    driver.set_position_mode();
  });
}

extern "C" int tianji_marvin_native_set_impedance_mode(
  void *handle, char *error, std::size_t error_size) {
  return invoke(handle, error, error_size, [](NativeDriver &driver) {
    driver.set_impedance_mode();
  });
}

extern "C" int tianji_marvin_native_submit(
  void *handle, const double *left, const double *right,
  char *error, std::size_t error_size) {
  return invoke(handle, error, error_size, [left, right](NativeDriver &driver) {
    if (left == nullptr || right == nullptr) throw std::invalid_argument("Marvin native target is null");
    driver.submit(left, right);
  });
}

extern "C" int tianji_marvin_native_read(
  void *handle, TianjiMarvinNativeFeedback *feedback,
  char *error, std::size_t error_size) {
  return invoke(handle, error, error_size, [feedback](NativeDriver &driver) {
    if (feedback == nullptr) throw std::invalid_argument("Marvin native feedback output is null");
    *feedback = driver.read();
  });
}

extern "C" void tianji_marvin_native_soft_stop(void *handle, const char *reason) {
  if (handle != nullptr) static_cast<NativeDriver *>(handle)->soft_stop(reason == nullptr ? "operator soft stop" : reason);
}

extern "C" void tianji_marvin_native_destroy(void *handle) {
  delete static_cast<NativeDriver *>(handle);
}
