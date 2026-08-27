#pragma once

#include <cstdint>
#include <limits>

namespace pico_body_tianji::official_ipc
{

constexpr std::uint32_t kMagic = 0x544a494b;
constexpr std::uint32_t kVersion = 2;

enum class Operation : std::uint32_t
{
  kForward = 1,
  kSolve = 2,
  kShutdown = 3,
};

struct Request
{
  std::uint32_t magic{kMagic};
  std::uint32_t version{kVersion};
  Operation operation{Operation::kForward};
  std::int32_t side{0};
  double joints_rad[7]{};
  double target_pose[16]{};
  double elbow_direction[3]{};
};

struct Response
{
  std::uint32_t magic{kMagic};
  std::uint32_t version{kVersion};
  std::int32_t error_code{0};
  char error[256]{};
  double pose[16]{};
  double joints_rad[7]{};
  std::uint8_t accepted{0};
  std::uint8_t converged{0};
  std::uint8_t saturated{0};
  std::uint8_t joint_step_limited{0};
  std::uint8_t singularity_active{0};
  std::uint8_t reserved[3]{};
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
  double minimum_singular_value{std::numeric_limits<double>::quiet_NaN()};
  double damping{std::numeric_limits<double>::quiet_NaN()};
  double arm_angle_error_rad{std::numeric_limits<double>::quiet_NaN()};
  double minimum_limit_margin_rad{std::numeric_limits<double>::quiet_NaN()};
  double maximum_joint_step_rad{0.0};
  double requested_maximum_joint_step_rad{0.0};
  double solve_time_ms{std::numeric_limits<double>::quiet_NaN()};
  double workspace_backoff_fraction{1.0};
  std::int32_t candidate_count{0};
  std::int32_t selected_candidate_index{-1};
  std::uint8_t soft_limit_active{0};
  std::uint8_t workspace_backoff_active{0};
  std::uint8_t orientation_relaxed{0};
  std::uint8_t reserved_diagnostics{0};
  char status[64]{};
};

}  // namespace pico_body_tianji::official_ipc
