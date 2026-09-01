#include "tianji_teleop/marvin/native_driver.h"

#include <chrono>
#include <cmath>
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
  const double stiffness[7] = {2, 2, 2, 1.6, 1, 1, 1};
  const double damping[7] = {0.3, 0.3, 0.3, 0.2, 0.2, 0.2, 0.2};
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
  if (!tianji_marvin_native_submit(driver, left, right, error, sizeof(error))) {
    std::cerr << error << '\n'; destroy(); return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(35));
  if (!tianji_marvin_native_read(driver, &feedback, error, sizeof(error)) ||
      feedback.control_ticks < 5 || feedback.arm_states[0] != 3 ||
      feedback.impedance_types[0] != 1 || feedback.velocity_ratios[0] != 100 ||
      feedback.acceleration_ratios[0] != 100) {
    std::cerr << "200 Hz impedance contract failed: " << error << '\n';
    destroy(); return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(70));
  if (!tianji_marvin_native_read(driver, &feedback, error, sizeof(error)) || !feedback.soft_stopped) {
    std::cerr << "native command watchdog did not stop\n";
    destroy(); return 1;
  }
  destroy();
  std::cout << "marvin native 200 Hz impedance probe passed\n";
  return 0;
}
