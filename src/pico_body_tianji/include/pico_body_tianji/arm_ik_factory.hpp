#pragma once

#include "pico_body_tianji/arm_ik_solver.hpp"

#include <memory>
#include <string>
#include <vector>

namespace pico_body_tianji
{

struct ArmIkBackendOptions
{
  std::string urdf_path;
  std::string official_library_path;
  std::string official_config_path;
};

std::unique_ptr<ArmIkSolver> create_arm_ik_solver(
  const std::string & backend,
  const ArmIkBackendOptions & options,
  const IkSettings & settings);

std::vector<std::string> available_arm_ik_backends();

}  // namespace pico_body_tianji
