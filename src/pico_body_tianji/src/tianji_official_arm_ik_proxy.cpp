#include "pico_body_tianji/tianji_official_arm_ik.hpp"

#include "pico_body_tianji/tianji_official_ipc.hpp"

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <spawn.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <stdexcept>
#include <string>

namespace pico_body_tianji
{
namespace
{

std::string worker_path()
{
  if (const char * configured = std::getenv("TIANJI_OFFICIAL_IK_WORKER")) {
    if (configured[0] != '\0' && access(configured, X_OK) == 0) {
      return configured;
    }
    throw std::runtime_error(
            "TIANJI_OFFICIAL_IK_WORKER 不可执行：" +
            std::string(configured));
  }
  std::array<char, 4096> path{};
  const ssize_t length = readlink("/proc/self/exe", path.data(), path.size() - 1);
  if (length <= 0) {
    throw std::runtime_error("无法定位当前 IK 可执行文件");
  }
  const std::filesystem::path directory =
    std::filesystem::path(std::string(path.data(), length)).parent_path();
  for (const char * name :
    {"tianji_official_ik_worker", "tianji_official_ik_worker.bin"})
  {
    const auto candidate = directory / name;
    if (access(candidate.c_str(), X_OK) == 0) {
      return candidate.string();
    }
  }
  throw std::runtime_error(
          "找不到同目录的 tianji_official_ik_worker[.bin]");
}

void copy_pose(double output[16], const Eigen::Isometry3d & pose)
{
  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index column = 0; column < 4; ++column) {
      output[row * 4 + column] = pose.matrix()(row, column);
    }
  }
}

Eigen::Isometry3d read_pose(const double input[16])
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index column = 0; column < 4; ++column) {
      pose.matrix()(row, column) = input[row * 4 + column];
    }
  }
  return pose;
}

std::string sdk_path(
  const std::string & configured,
  const char * environment_name,
  const char * description)
{
  if (!configured.empty()) {
    return configured;
  }
  if (const char * environment = std::getenv(environment_name)) {
    if (environment[0] != '\0') {
      return environment;
    }
  }
  throw std::invalid_argument(
          "选择 tianji_official 时缺少" + std::string(description) +
          "：请配置 YAML 或 runtime 环境变量 " + environment_name);
}

}  // namespace

struct TianjiOfficialArmIk::Impl
{
  Impl(
    const std::string & library_path,
    const std::string & config_path,
    const IkSettings & settings)
  {
    const std::string resolved_library = sdk_path(
      library_path, "TIANJI_OFFICIAL_IK_LIBRARY", "官方 IK 动态库");
    const std::string resolved_config = sdk_path(
      config_path, "TIANJI_OFFICIAL_IK_CONFIG", "机型配置");
    int sockets[2]{};
    if (socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0, sockets) != 0) {
      throw std::runtime_error("无法创建官方 IK IPC：" + std::string(strerror(errno)));
    }
    const std::string executable = worker_path();
    constexpr int worker_fd = 3;
    const std::string fd = std::to_string(worker_fd);
    const std::string max_step = std::to_string(settings.maximum_joint_step_rad);
    const std::string position_tolerance =
      std::to_string(settings.position_tolerance_m);
    const std::string orientation_tolerance =
      std::to_string(settings.orientation_tolerance_rad);
    const std::string arm_angle_gain = std::to_string(settings.arm_angle_gain);
    std::array<char *, 9> arguments{
      const_cast<char *>(executable.c_str()),
      const_cast<char *>(fd.c_str()),
      const_cast<char *>(resolved_library.c_str()),
      const_cast<char *>(resolved_config.c_str()),
      const_cast<char *>(max_step.c_str()),
      const_cast<char *>(position_tolerance.c_str()),
      const_cast<char *>(orientation_tolerance.c_str()),
      const_cast<char *>(arm_angle_gain.c_str()),
      nullptr,
    };
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, sockets[1], worker_fd);
    if (sockets[0] != worker_fd) {
      posix_spawn_file_actions_addclose(&actions, sockets[0]);
    }
    if (sockets[1] != worker_fd) {
      posix_spawn_file_actions_addclose(&actions, sockets[1]);
    }
    const int spawn_error = posix_spawn(
      &child_pid, executable.c_str(), &actions, nullptr,
      arguments.data(), ::environ);
    posix_spawn_file_actions_destroy(&actions);
    close(sockets[1]);
    if (spawn_error != 0) {
      close(sockets[0]);
      child_pid = -1;
      throw std::runtime_error(
              "无法启动官方 IK worker：" +
              std::string(strerror(spawn_error)));
    }
    socket_fd = sockets[0];
    try {
      official_ipc::Request request;
      request.operation = official_ipc::Operation::kForward;
      request.joints_rad[0] = 55.0 * 3.14159265358979323846 / 180.0;
      request.joints_rad[1] = -65.0 * 3.14159265358979323846 / 180.0;
      request.joints_rad[2] = -70.0 * 3.14159265358979323846 / 180.0;
      request.joints_rad[3] = -60.0 * 3.14159265358979323846 / 180.0;
      request.joints_rad[4] = 60.0 * 3.14159265358979323846 / 180.0;
      const auto response = exchange(request);
      (void)response;
    } catch (...) {
      stop();
      throw;
    }
  }

  ~Impl() {stop();}

  void stop() noexcept
  {
    if (socket_fd >= 0) {
      official_ipc::Request request;
      request.operation = official_ipc::Operation::kShutdown;
      (void)send(socket_fd, &request, sizeof(request), MSG_NOSIGNAL);
      close(socket_fd);
      socket_fd = -1;
    }
    if (child_pid > 0) {
      int status = 0;
      if (waitpid(child_pid, &status, WNOHANG) == 0) {
        kill(child_pid, SIGTERM);
        (void)waitpid(child_pid, &status, 0);
      }
      child_pid = -1;
    }
  }

  official_ipc::Response exchange(const official_ipc::Request & request) const
  {
    const std::lock_guard<std::mutex> lock(mutex);
    const ssize_t sent = send(socket_fd, &request, sizeof(request), MSG_NOSIGNAL);
    if (sent != static_cast<ssize_t>(sizeof(request))) {
      throw std::runtime_error("官方 IK worker 请求发送失败");
    }
    official_ipc::Response response;
    const ssize_t received = recv(socket_fd, &response, sizeof(response), 0);
    if (received != static_cast<ssize_t>(sizeof(response))) {
      throw std::runtime_error("官方 IK worker 异常退出或响应不完整");
    }
    if (
      response.magic != official_ipc::kMagic ||
      response.version != official_ipc::kVersion)
    {
      throw std::runtime_error("官方 IK worker 协议版本不匹配");
    }
    if (response.error_code != 0) {
      throw std::runtime_error(
              "官方 IK worker：" + std::string(response.error));
    }
    return response;
  }

  int socket_fd{-1};
  pid_t child_pid{-1};
  mutable std::mutex mutex;
};

TianjiOfficialArmIk::TianjiOfficialArmIk(
  const std::string & library_path,
  const std::string & config_path,
  const IkSettings & settings)
: impl_(std::make_unique<Impl>(library_path, config_path, settings)) {}

TianjiOfficialArmIk::~TianjiOfficialArmIk() = default;
TianjiOfficialArmIk::TianjiOfficialArmIk(TianjiOfficialArmIk &&) noexcept = default;
TianjiOfficialArmIk & TianjiOfficialArmIk::operator=(TianjiOfficialArmIk &&) noexcept = default;

Eigen::Isometry3d TianjiOfficialArmIk::forward(
  ArmSide side, const ArmJointVector & joints_rad) const
{
  official_ipc::Request request;
  request.operation = official_ipc::Operation::kForward;
  request.side = side == ArmSide::kLeft ? 0 : 1;
  for (Eigen::Index index = 0; index < joints_rad.size(); ++index) {
    request.joints_rad[index] = joints_rad[index];
  }
  return read_pose(impl_->exchange(request).pose);
}

IkResult TianjiOfficialArmIk::solve(
  ArmSide side,
  const Eigen::Isometry3d & target_pose,
  const ArmJointVector & current_joints_rad,
  const Eigen::Vector3d & elbow_ik_direction) const
{
  official_ipc::Request request;
  request.operation = official_ipc::Operation::kSolve;
  request.side = side == ArmSide::kLeft ? 0 : 1;
  copy_pose(request.target_pose, target_pose);
  for (Eigen::Index index = 0; index < current_joints_rad.size(); ++index) {
    request.joints_rad[index] = current_joints_rad[index];
  }
  for (Eigen::Index index = 0; index < elbow_ik_direction.size(); ++index) {
    request.elbow_direction[index] = elbow_ik_direction[index];
  }
  const auto response = impl_->exchange(request);
  IkResult result;
  for (Eigen::Index index = 0; index < result.joints_rad.size(); ++index) {
    result.joints_rad[index] = response.joints_rad[index];
  }
  result.achieved_pose = read_pose(response.pose);
  result.accepted = response.accepted != 0;
  result.converged = response.converged != 0;
  result.saturated = response.saturated != 0;
  result.joint_step_limited = response.joint_step_limited != 0;
  result.singularity_active = response.singularity_active != 0;
  result.position_error_m = response.position_error_m;
  result.orientation_error_rad = response.orientation_error_rad;
  result.minimum_singular_value = response.minimum_singular_value;
  result.damping = response.damping;
  result.arm_angle_error_rad = response.arm_angle_error_rad;
  result.minimum_limit_margin_rad = response.minimum_limit_margin_rad;
  result.maximum_joint_step_rad = response.maximum_joint_step_rad;
  result.status = response.status;
  return result;
}

}  // namespace pico_body_tianji
