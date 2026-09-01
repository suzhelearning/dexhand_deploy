#include <algorithm>
#include <array>
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
std::mutex mutex;
Dcss data{};
std::array<bool, 2> impedance{};
std::array<bool, 2> velocity_step{};
std::array<bool, 2> position_since_feedback{};
std::array<bool, 2> tool_configured{};
std::array<int, 2> transition_polls{};
constexpr std::array<float, 2> expected_tool_mass{0.0F, 0.95F};

std::size_t arm_index(char arm) { return arm == 'A' ? 0U : 1U; }

}  // namespace

extern "C" bool Connect(unsigned char, unsigned char, unsigned char, unsigned char, int) {
  std::lock_guard<std::mutex> guard(mutex);
  data = {};
  impedance = {};
  velocity_step = {};
  position_since_feedback = {};
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
  data.input[index].joint_velocity_ratio = static_cast<short>(velocity);
  data.input[index].joint_acceleration_ratio = static_cast<short>(acceleration);
  for (std::size_t joint = 0; joint < 7; ++joint) {
    data.input[index].joint_stiffness[joint] = static_cast<float>(stiffness[joint]);
    data.input[index].joint_damping[joint] = static_cast<float>(damping[joint]);
  }
  return true;
}

extern "C" bool SetJointPostionCmd(char arm, const double *joints) {
  const auto index = arm_index(arm);
  std::lock_guard<std::mutex> guard(mutex);
  for (std::size_t joint = 0; joint < 7; ++joint) {
    data.output[index].feedback_joint_position[joint] = static_cast<float>(joints[joint]);
  }
  position_since_feedback[index] = true;
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
