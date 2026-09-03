#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { TIANJI_MARVIN_JOINTS = 7, TIANJI_MARVIN_ARMS = 2 };

typedef struct TianjiMarvinNativeConfig {
  int rate_hz;
  int velocity_ratio;
  int acceleration_ratio;
  long velocity_estimation_step_ms;
  double joint_stiffness[TIANJI_MARVIN_JOINTS];
  double joint_damping[TIANJI_MARVIN_JOINTS];
  double tool_kinematics[6];
  double tool_dynamics[TIANJI_MARVIN_ARMS][10];
  double command_timeout_s;
} TianjiMarvinNativeConfig;

typedef struct TianjiMarvinNativeFeedback {
  double joints_deg[TIANJI_MARVIN_ARMS][TIANJI_MARVIN_JOINTS];
  int arm_states[TIANJI_MARVIN_ARMS];
  int command_states[TIANJI_MARVIN_ARMS];
  int error_codes[TIANJI_MARVIN_ARMS];
  int frame_serials[TIANJI_MARVIN_ARMS];
  int velocity_ratios[TIANJI_MARVIN_ARMS];
  int acceleration_ratios[TIANJI_MARVIN_ARMS];
  int impedance_types[TIANJI_MARVIN_ARMS];
  long servo_error_codes[TIANJI_MARVIN_ARMS][TIANJI_MARVIN_JOINTS];
  uint64_t control_ticks;
  uint64_t deadline_misses;
  int healthy;
  int soft_stopped;
} TianjiMarvinNativeFeedback;

void *tianji_marvin_native_create(
  const char *sdk_library,
  const TianjiMarvinNativeConfig *config,
  char *error,
  size_t error_size);

int tianji_marvin_native_connect(
  void *handle,
  const char *robot_ip,
  char *error,
  size_t error_size);

int tianji_marvin_native_set_position_mode(
  void *handle,
  char *error,
  size_t error_size);

int tianji_marvin_native_set_impedance_mode(
  void *handle,
  char *error,
  size_t error_size);

int tianji_marvin_native_submit(
  void *handle,
  const double left_deg[TIANJI_MARVIN_JOINTS],
  const double right_deg[TIANJI_MARVIN_JOINTS],
  char *error,
  size_t error_size);

int tianji_marvin_native_read(
  void *handle,
  TianjiMarvinNativeFeedback *feedback,
  char *error,
  size_t error_size);

void tianji_marvin_native_soft_stop(void *handle, const char *reason);
void tianji_marvin_native_destroy(void *handle);

#ifdef __cplusplus
}
#endif
