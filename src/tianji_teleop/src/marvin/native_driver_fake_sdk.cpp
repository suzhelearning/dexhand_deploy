#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <mutex>

namespace {

struct StateCtr { int current; int command; int error; };
struct RtIn {
  int realtime_switch; int impedance_type; int input_frame_serial;
  short frame_miss_count; short maximum_frame_miss_count; int system_cycle;
  short system_cycle_miss_count; short maximum_system_cycle_miss_count;
  float tool_kinematics[6]; float tool_dynamics[10]; float joint_command_position[7];
  short joint_velocity_ratio; short joint_acceleration_ratio;
  float joint_stiffness[7]; float joint_damping[7]; int drag_space_type;
  float drag_space_parameters[6]; int cartesian_kd_type;
  float cartesian_stiffness[6]; float cartesian_damping[6];
  float cartesian_nullspace_stiffness; float cartesian_nullspace_damping;
  int force_feedback_type; int force_type; float force_direction[6];
  float force_pid_limits[7]; float force_adjustment_limit; float force_command;
  unsigned char set_tags[16]; unsigned char update_tags[16];
  unsigned char pvt_id; unsigned char pvt_id_update; unsigned char pvt_run_id;
  unsigned char pvt_run_state;
};
struct RtOut {
  int output_frame_serial; float feedback_joint_position[7];
  float feedback_joint_velocity[7]; float feedback_joint_external_position[7];
  float feedback_joint_command[7]; float feedback_joint_command_torque[7];
  float feedback_joint_sensor_torque[7]; float feedback_joint_temperature[7];
  float estimated_joint_friction[7]; float estimated_joint_friction_derivative[7];
  float estimated_joint_force[7]; float estimated_cartesian_force[6];
  char tip_digital_input; char low_speed_flag; char padding; char trajectory_state;
};
struct Dcss {
  StateCtr state[2]; RtIn input[2]; RtOut output[2]; char parameter_name[30];
  unsigned char parameter_type; unsigned char parameter_instruction;
  int parameter_integer; float parameter_float;
  short parameter_command_serial; short parameter_return_serial;
};

static_assert(sizeof(Dcss) == 1428);
using Clock = std::chrono::steady_clock;
std::mutex mutex;
Dcss data{};
std::array<bool, 2> impedance{};
std::array<bool, 2> velocity_step{};
std::array<bool, 2> position_since_feedback{};
std::array<bool, 2> position_pending{};
std::array<bool, 2> position_estop{};
int position_hold_batches = 0;
int missed_position_arm = -1;
Clock::time_point position_mode_started{};
std::array<bool, 2> position_transition_stuck{};
std::array<std::array<float, 7>, 2> position_hold{};
std::array<std::array<float, 7>, 2> pending_joint_position{};
std::array<bool, 2> pending_joint_position_set{};
std::array<std::array<int, 2>, 2> pending_joint_limit{};
std::array<bool, 2> pending_joint_limit_set{};
std::array<int, 2> pending_target_state{};
std::array<bool, 2> pending_target_state_set{};
std::array<bool, 2> tool_configured{};
std::array<int, 2> transition_polls{};
constexpr std::array<float, 2> expected_tool_mass{0.0F, 0.95F};

bool position_estop_requested() {
  const char *value = std::getenv("TIANJI_MARVIN_FAKE_POSITION_ESTOP");
  return value != nullptr && std::strcmp(value, "1") == 0;
}

bool miss_position_a_requested() {
  const char *value = std::getenv("TIANJI_MARVIN_FAKE_MISS_POSITION_A");
  return value != nullptr && std::strcmp(value, "1") == 0;
}
bool stuck_position_requested() {
  const char *value = std::getenv("TIANJI_MARVIN_FAKE_STUCK_POSITION");
  return value != nullptr && std::strcmp(value, "1") == 0;
}
std::size_t arm_index(char arm) { return arm == 'A' ? 0U : 1U; }

}  // namespace

extern "C" bool Connect(unsigned char, unsigned char, unsigned char, unsigned char, int) {
  std::lock_guard<std::mutex> guard(mutex);
  data = {};
  impedance = {};
  velocity_step = {};
  position_since_feedback = {};
  position_pending = {};
  position_estop = {};
  position_hold_batches = 0;
  missed_position_arm = -1;
  position_mode_started = {};
  position_transition_stuck = {};
  position_hold = {};
  pending_joint_position = {};
  pending_joint_position_set = {};
  pending_joint_limit = {};
  pending_joint_limit_set = {};
  pending_target_state = {};
  pending_target_state_set = {};
  tool_configured = {};
  transition_polls = {};
  for (auto &state : data.state) { state.current = 1; state.command = 1; }
  return true;
}

extern "C" bool OnGetBuf(Dcss *output) {
  if (output == nullptr) return false;
  std::lock_guard<std::mutex> guard(mutex);
  for (std::size_t arm = 0; arm < 2; ++arm) {
    ++data.output[arm].output_frame_serial;
    if (impedance[arm] && velocity_step[arm]) {
      if (!position_since_feedback[arm] || !tool_configured[arm]) {
        data.output[arm].feedback_joint_position[0] -= 2.0F;
      }
      data.output[arm].feedback_joint_position[0] +=
        2.0F * (data.input[arm].tool_dynamics[0] - expected_tool_mass[arm]);
      position_since_feedback[arm] = false;
      ++transition_polls[arm];
      data.state[arm].current = transition_polls[arm] >= 5 ? 3 : 101;
      data.state[arm].command = 3;
      data.state[arm].error = transition_polls[arm] >= 5 ? 0 : 6;
      data.input[arm].impedance_type = 1;
    }
  }
  *output = data;
  return true;
}

extern "C" bool SetTool(char arm, const double *kinematics, const double *dynamics) {
  const auto index = arm_index(arm);
  std::lock_guard<std::mutex> guard(mutex);
  for (std::size_t value = 0; value < 6; ++value) {
    data.input[index].tool_kinematics[value] = static_cast<float>(kinematics[value]);
  }
  for (std::size_t value = 0; value < 10; ++value) {
    data.input[index].tool_dynamics[value] = static_cast<float>(dynamics[value]);
  }
  tool_configured[index] = true;
  return true;
}

extern "C" bool SetImpJointMode(char arm, int velocity, int acceleration,
                                  const double *stiffness, const double *damping) {
  const auto index = arm_index(arm);
  std::lock_guard<std::mutex> guard(mutex);
  impedance[index] = true;
  transition_polls[index] = 0;
  data.input[index].joint_velocity_ratio = static_cast<short>(velocity);
  data.input[index].joint_acceleration_ratio = static_cast<short>(acceleration);
  for (std::size_t joint = 0; joint < 7; ++joint) {
    data.input[index].joint_stiffness[joint] = static_cast<float>(stiffness[joint]);
    data.input[index].joint_damping[joint] = static_cast<float>(damping[joint]);
  }
  return true;
}

extern "C" bool SetJointMode(char arm, int velocity, int acceleration) {
  const auto index = arm_index(arm);
  const auto other = 1U - index;
  std::lock_guard<std::mutex> guard(mutex);
  impedance[other] = true;
  position_pending[other] = false;
  data.state[other].current = 3;
  data.state[other].command = 3;
  data.state[other].error = 0;
  data.input[other].impedance_type = 1;
  impedance[index] = false;
  transition_polls[index] = 0;
  position_pending[index] = true;
  position_estop[index] = position_estop_requested();
  position_hold_batches = 0;
  position_transition_stuck[index] = stuck_position_requested();
  std::copy(
    std::begin(data.output[index].feedback_joint_position),
    std::end(data.output[index].feedback_joint_position),
    position_hold[index].begin());
  data.state[index].current = position_estop[index] ? 100 : 101;
  data.state[index].command = 1;
  data.state[index].error = position_estop[index] ? 4 : 6;
  data.input[index].impedance_type = 0;
  data.input[index].joint_velocity_ratio = static_cast<short>(velocity);
  data.input[index].joint_acceleration_ratio = static_cast<short>(acceleration);
  return true;
}

extern "C" bool SetJointPostionCmd(char arm, const double *joints) {
  const auto index = arm_index(arm);
  std::lock_guard<std::mutex> guard(mutex);
  if (position_pending[index]) {
    bool is_hold = true;
    for (std::size_t joint = 0; joint < 7; ++joint) {
      is_hold = is_hold &&
        std::abs(joints[joint] - position_hold[index][joint]) <= 1e-6;
    }
    if (is_hold && !position_transition_stuck[index] && !position_estop[index]) {
      position_pending[index] = false;
      data.state[index].current = 1;
      data.state[index].error = 0;
    }
    return true;
  }
  for (std::size_t joint = 0; joint < 7; ++joint) {
    data.output[index].feedback_joint_position[joint] = static_cast<float>(joints[joint]);
  }
  position_since_feedback[index] = true;
  return true;
}

extern "C" bool OnClearSet() {
  std::lock_guard<std::mutex> guard(mutex);
  pending_joint_position_set = {};
  pending_joint_limit_set = {};
  pending_target_state_set = {};
  return true;
}

extern "C" bool OnSetJointCmdPos_A(const double *joints) {
  std::lock_guard<std::mutex> guard(mutex);
  std::copy(joints, joints + 7, pending_joint_position[0].begin());
  pending_joint_position_set[0] = true;
  return true;
}

extern "C" bool OnSetJointCmdPos_B(const double *joints) {
  std::lock_guard<std::mutex> guard(mutex);
  std::copy(joints, joints + 7, pending_joint_position[1].begin());
  pending_joint_position_set[1] = true;
  return true;
}

extern "C" bool OnSetJointLmt_A(int velocity, int acceleration) {
  std::lock_guard<std::mutex> guard(mutex);
  pending_joint_limit[0] = {velocity, acceleration};
  pending_joint_limit_set[0] = true;
  return true;
}

extern "C" bool OnSetJointLmt_B(int velocity, int acceleration) {
  std::lock_guard<std::mutex> guard(mutex);
  pending_joint_limit[1] = {velocity, acceleration};
  pending_joint_limit_set[1] = true;
  return true;
}

extern "C" bool OnSetTargetState_A(int state) {
  std::lock_guard<std::mutex> guard(mutex);
  pending_target_state[0] = state;
  pending_target_state_set[0] = true;
  return true;
}

extern "C" bool OnSetTargetState_B(int state) {
  std::lock_guard<std::mutex> guard(mutex);
  pending_target_state[1] = state;
  pending_target_state_set[1] = true;
  return true;
}

extern "C" bool OnSetSend() {
  std::lock_guard<std::mutex> guard(mutex);
  if (!pending_joint_position_set[0] || !pending_joint_position_set[1]) {
    return false;
  }
  const int target_count =
    static_cast<int>(pending_target_state_set[0]) +
    static_cast<int>(pending_target_state_set[1]);
  for (std::size_t arm = 0; arm < 2; ++arm) {
    if (pending_target_state_set[arm] && !pending_joint_limit_set[arm]) {
      return false;
    }
  }
  const bool stuck = stuck_position_requested();
  const bool estop = position_estop_requested();
  int skipped_arm = -1;
  // 首个双臂 mode batch 只应用一臂；过早或错误臂的 retry 直接失败。
  if (target_count == 2 && !stuck && !estop) {
    skipped_arm = miss_position_a_requested() ? 0 : 1;
    missed_position_arm = skipped_arm;
    position_mode_started = Clock::now();
  } else if (target_count == 1 && missed_position_arm >= 0) {
    const int target_arm = pending_target_state_set[0] ? 0 : 1;
    if (target_arm != missed_position_arm ||
        Clock::now() - position_mode_started < std::chrono::milliseconds(80)) {
      return false;
    }
    missed_position_arm = -1;
  }
  if (target_count > 0) {
    position_hold_batches = 0;
    for (std::size_t arm = 0; arm < 2; ++arm) {
      if (!pending_target_state_set[arm] ||
          static_cast<int>(arm) == skipped_arm) {
        continue;
      }
      impedance[arm] = false;
      transition_polls[arm] = 0;
      position_estop[arm] = estop;
      position_transition_stuck[arm] = stuck;
      std::copy(
        std::begin(data.output[arm].feedback_joint_position),
        std::end(data.output[arm].feedback_joint_position),
        position_hold[arm].begin());
      data.state[arm].current = estop ? 100 : (stuck ? 101 : 1);
      data.state[arm].command = pending_target_state[arm];
      data.state[arm].error = estop ? 4 : (stuck ? 6 : 0);
      data.input[arm].impedance_type = 0;
      data.input[arm].joint_velocity_ratio =
        static_cast<short>(pending_joint_limit[arm][0]);
      data.input[arm].joint_acceleration_ratio =
        static_cast<short>(pending_joint_limit[arm][1]);
      position_pending[arm] = stuck || estop;
    }
  } else if ((position_estop[0] && position_pending[0]) ||
             (position_estop[1] && position_pending[1])) {
    // 第二个 hold batch 表明驱动忽略了 100/4 的急停反馈。
    if (++position_hold_batches > 1) return false;
  }
  for (std::size_t arm = 0; arm < 2; ++arm) {
    if (position_pending[arm]) {
      bool is_hold = true;
      for (std::size_t joint = 0; joint < 7; ++joint) {
        is_hold = is_hold &&
          std::abs(pending_joint_position[arm][joint] -
                   position_hold[arm][joint]) <= 1e-6;
      }
      if (target_count == 0 && is_hold &&
          !position_transition_stuck[arm] && !position_estop[arm]) {
        position_pending[arm] = false;
        data.state[arm].current = 1;
        data.state[arm].error = 0;
      }
    } else {
      std::copy(
        pending_joint_position[arm].begin(),
        pending_joint_position[arm].end(),
        data.output[arm].feedback_joint_position);
    }
    position_since_feedback[arm] = true;
  }
  pending_joint_position_set = {};
  pending_joint_limit_set = {};
  pending_target_state_set = {};
  return true;
}

extern "C" bool FX_OnSetVelEstStep(char arm, long step_ms) {
  std::lock_guard<std::mutex> guard(mutex);
  velocity_step[arm_index(arm)] = step_ms == 5;
  return step_ms == 5;
}

extern "C" void EStop(const char *) {
  std::lock_guard<std::mutex> guard(mutex);
  for (auto &state : data.state) { state.current = 100; state.error = 4; }
}

extern "C" bool Disable(char) { return true; }
extern "C" bool OnRelease() { return true; }
extern "C" long OnGetServoErr_A(long *errors) {
  std::fill(errors, errors + 7, 0L);
  return 0;
}
extern "C" long OnGetServoErr_B(long *errors) {
  std::fill(errors, errors + 7, 0L);
  return 0;
}
