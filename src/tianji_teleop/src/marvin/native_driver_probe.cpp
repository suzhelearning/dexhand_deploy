#include "tianji_teleop/marvin/native_driver.h"

#include <chrono>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: marvin_native_driver_probe FAKE_SDK\n";
    return 2;
  }
  TianjiMarvinNativeConfig config{};
  config.rate_hz = 200;
  config.velocity_ratio = 100;
  config.acceleration_ratio = 100;
  config.velocity_estimation_step_ms = 5;
  config.command_timeout_s = 0.05;
  const double stiffness[7] = {10, 10, 10, 1.6, 1, 1, 1};
  const double damping[7] = {0.8, 0.8, 0.8, 0.4, 0.4, 0.4, 0.4};
  for (int joint = 0; joint < 7; ++joint) {
    config.joint_stiffness[joint] = stiffness[joint];
    config.joint_damping[joint] = damping[joint];
  }
  config.tool_dynamics[1][0] = 0.95;
  config.tool_dynamics[1][3] = 90.0;
  char error[512]{};
  void *driver = tianji_marvin_native_create(argv[1], &config, error, sizeof(error));
  if (driver == nullptr) { std::cerr << error << '\n'; return 1; }
  const auto destroy = [&] { tianji_marvin_native_destroy(driver); };
  if (!tianji_marvin_native_connect(driver, "192.168.1.190", error, sizeof(error))) {
    std::cerr << error << '\n'; destroy(); return 1;
  }
  TianjiMarvinNativeFeedback feedback{};
  if (!tianji_marvin_native_read(driver, &feedback, error, sizeof(error)) ||
      std::abs(feedback.joints_deg[0][0]) > 0.5 ||
      std::abs(feedback.joints_deg[1][0]) > 0.5) {
    std::cerr << "arm moved while entering impedance mode\n";
    destroy(); return 1;
  }
  double left[7]{};
  double right[7]{};
  right[0] = 1.0;
  if (!tianji_marvin_native_submit(driver, left, right, error, sizeof(error))) {
    std::cerr << error << '\n'; destroy(); return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(35));
  if (!tianji_marvin_native_read(driver, &feedback, error, sizeof(error)) ||
      feedback.control_ticks < 5 || feedback.arm_states[0] != 3 ||
      feedback.impedance_types[0] != 1 || feedback.velocity_ratios[0] != 100 ||
      feedback.acceleration_ratios[0] != 100 ||
      std::abs(feedback.joints_deg[1][0] - right[0]) > 0.01) {
    std::cerr << "200 Hz impedance contract failed: " << error << '\n';
    destroy(); return 1;
  }
  if (!tianji_marvin_native_set_position_mode(driver, error, sizeof(error)) ||
      !tianji_marvin_native_read(driver, &feedback, error, sizeof(error)) ||
      feedback.arm_states[0] != 1 || feedback.arm_states[1] != 1 ||
      !feedback.healthy) {
    std::cerr << "position-mode transition failed: " << error << '\n';
    destroy(); return 1;
  }
  right[0] = 2.0;
  if (!tianji_marvin_native_submit(driver, left, right, error, sizeof(error))) {
    std::cerr << error << '\n'; destroy(); return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(35));
  if (!tianji_marvin_native_read(driver, &feedback, error, sizeof(error)) ||
      feedback.arm_states[0] != 1 || feedback.arm_states[1] != 1 ||
      std::abs(feedback.joints_deg[1][0] - right[0]) > 0.01) {
    std::cerr << "position-mode command failed: " << error << '\n';
    destroy(); return 1;
  }
  if (!tianji_marvin_native_set_impedance_mode(driver, error, sizeof(error)) ||
      !tianji_marvin_native_read(driver, &feedback, error, sizeof(error)) ||
      feedback.arm_states[0] != 3 || feedback.arm_states[1] != 3 ||
      feedback.impedance_types[0] != 1 || !feedback.healthy) {
    std::cerr << "impedance-mode transition failed: " << error << '\n';
    destroy(); return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(70));
  if (!tianji_marvin_native_read(driver, &feedback, error, sizeof(error)) || !feedback.soft_stopped) {
    std::cerr << "native command watchdog did not stop\n";
    destroy(); return 1;
  }
  destroy();
  if (::setenv("TIANJI_MARVIN_FAKE_MISS_POSITION_A", "1", 1) != 0) {
    std::cerr << "failed to configure missed-A position fake\n";
    return 1;
  }
  driver = tianji_marvin_native_create(argv[1], &config, error, sizeof(error));
  if (driver == nullptr ||
      !tianji_marvin_native_connect(driver, "192.168.1.190", error, sizeof(error))) {
    std::cerr << error << '\n';
    if (driver != nullptr) destroy();
    return 1;
  }
  error[0] = '\0';
  const int reverse_retry_result =
    tianji_marvin_native_set_position_mode(driver, error, sizeof(error));
  ::unsetenv("TIANJI_MARVIN_FAKE_MISS_POSITION_A");
  if (!reverse_retry_result ||
      !tianji_marvin_native_read(driver, &feedback, error, sizeof(error)) ||
      feedback.arm_states[0] != 1 || feedback.arm_states[1] != 1 ||
      !feedback.healthy) {
    std::cerr << "missed-A position retry failed: " << error << '\n';
    destroy(); return 1;
  }
  destroy();
  if (::setenv("TIANJI_MARVIN_FAKE_POSITION_ESTOP", "1", 1) != 0) {
    std::cerr << "failed to configure position EStop fake\n";
    return 1;
  }
  driver = tianji_marvin_native_create(argv[1], &config, error, sizeof(error));
  if (driver == nullptr ||
      !tianji_marvin_native_connect(driver, "192.168.1.190", error, sizeof(error))) {
    std::cerr << error << '\n';
    if (driver != nullptr) destroy();
    return 1;
  }
  error[0] = '\0';
  const auto estop_started = std::chrono::steady_clock::now();
  const int estop_result =
    tianji_marvin_native_set_position_mode(driver, error, sizeof(error));
  const double estop_elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - estop_started).count();
  ::unsetenv("TIANJI_MARVIN_FAKE_POSITION_ESTOP");
  if (estop_result != 0 || estop_elapsed >= 1.0 ||
      std::strcmp(error, "Marvin arm error during native prepare") != 0) {
    std::cerr << "position EStop was not rejected immediately: "
              << error << " after " << estop_elapsed << "s\n";
    destroy(); return 1;
  }
  destroy();
  if (::setenv("TIANJI_MARVIN_FAKE_STUCK_POSITION", "1", 1) != 0) {
    std::cerr << "failed to configure stuck-position fake\n";
    return 1;
  }
  driver = tianji_marvin_native_create(argv[1], &config, error, sizeof(error));
  if (driver == nullptr ||
      !tianji_marvin_native_connect(driver, "192.168.1.190", error, sizeof(error))) {
    std::cerr << error << '\n';
    if (driver != nullptr) destroy();
    return 1;
  }
  error[0] = '\0';
  const int position_result =
    tianji_marvin_native_set_position_mode(driver, error, sizeof(error));
  ::unsetenv("TIANJI_MARVIN_FAKE_STUCK_POSITION");
  const char *arm_a = std::strstr(
    error,
    "A{current=101,command=1,error=6,velocity=100,acceleration=100,frame_serial=");
  const char *arm_b = std::strstr(
    error,
    "B{current=101,command=1,error=6,velocity=100,acceleration=100,frame_serial=");
  const char *arm_a_end =
    arm_a == nullptr ? nullptr : std::strstr(arm_a, ",impedance_type=0}");
  const char *arm_b_end =
    arm_b == nullptr ? nullptr : std::strstr(arm_b, ",impedance_type=0}");
  if (position_result != 0 ||
      std::strstr(
        error,
        "Marvin arms did not enter verified joint position mode: ") != error ||
      arm_a == nullptr || arm_b == nullptr || arm_a_end == nullptr ||
      arm_b_end == nullptr || arm_a >= arm_a_end || arm_a_end >= arm_b ||
      arm_b >= arm_b_end) {
    std::cerr << "position timeout diagnostics failed: " << error << '\n';
    destroy(); return 1;
  }
  destroy();
  std::cout << "marvin native 200 Hz impedance probe passed\n";
  return 0;
}
