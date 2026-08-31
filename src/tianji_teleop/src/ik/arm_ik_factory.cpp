#include "tianji_teleop/ik/arm_ik_factory.hpp"

#include "tianji_teleop/ik/pinocchio_cpp/pinocchio_arm_ik.hpp"
#include "tianji_teleop/ik/pinocchio_qp/pinocchio_qp_arm_ik.hpp"
#include "tianji_teleop/ik/tianji_official/tianji_official_arm_ik.hpp"

#include <functional>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace tianji_teleop
{
namespace
{

using Factory = std::function<std::unique_ptr<ArmIkSolver>(
    const ArmIkBackendOptions &, const IkSettings &)>;

const std::map<std::string, Factory> & solver_factories()
{
  static const std::map<std::string, Factory> factories{
    {
      "pinocchio_cpp",
      [](const ArmIkBackendOptions & options, const IkSettings & settings) {
        return std::make_unique<PinocchioArmIk>(
          options.urdf_path, settings);
      }
    },
    {
      "pinocchio_qp",
      [](const ArmIkBackendOptions & options, const IkSettings & settings) {
        return std::make_unique<PinocchioQpArmIk>(
          options.urdf_path, settings);
      }
    },
    {
      "tianji_official",
      [](const ArmIkBackendOptions & options, const IkSettings & settings) {
        return std::make_unique<TianjiOfficialArmIk>(
          options.official_library_path,
          options.official_config_path,
          settings);
      }
    },
  };
  return factories;
}

std::string join_backend_names()
{
  std::ostringstream stream;
  bool first = true;
  for (const auto & entry : solver_factories()) {
    if (!first) {
      stream << ", ";
    }
    stream << entry.first;
    first = false;
  }
  return stream.str();
}

}  // namespace

std::unique_ptr<ArmIkSolver> create_arm_ik_solver(
  const std::string & backend,
  const ArmIkBackendOptions & options,
  const IkSettings & settings)
{
  const auto factory = solver_factories().find(backend);
  if (factory == solver_factories().end()) {
    throw std::invalid_argument(
            "不支持 ik_backend='" + backend + "'；可选值：" +
            join_backend_names());
  }
  return factory->second(options, settings);
}

std::vector<std::string> available_arm_ik_backends()
{
  std::vector<std::string> backends;
  backends.reserve(solver_factories().size());
  for (const auto & entry : solver_factories()) {
    backends.push_back(entry.first);
  }
  return backends;
}

}  // namespace tianji_teleop
