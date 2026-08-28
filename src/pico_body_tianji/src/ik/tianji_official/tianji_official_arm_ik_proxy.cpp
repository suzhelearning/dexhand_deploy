#include "pico_body_tianji/ik/tianji_official/tianji_official_arm_ik.hpp"

#include "pico_body_tianji/ik/tianji_official/tianji_official_ipc.hpp"

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <poll.h>
#include <spawn.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

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

class WorkerTransportError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class WorkerExecutionError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

std::string serialize_joints(const ArmJointVector & joints)
{
  std::ostringstream stream;
  stream.precision(17);
  for (Eigen::Index index = 0; index < joints.size(); ++index) {
    if (index != 0) {
      stream << ',';
    }
    stream << joints[index];
  }
  return stream.str();
}

std::string serialize_double(double value)
{
  std::ostringstream stream;
  stream << std::setprecision(std::numeric_limits<double>::max_digits10)
         << value;
  return stream.str();
}

}  // namespace

struct TianjiOfficialArmIk::Impl
{
  struct ExchangeOutcome
  {
    official_ipc::Response response;
    int restart_count{0};
    double elapsed_ms{0.0};
  };

  Impl(
    const std::string & library_path,
    const std::string & config_path,
    const IkSettings & settings)
  : resolved_library(sdk_path(
      library_path, "TIANJI_OFFICIAL_IK_LIBRARY", "官方 IK 动态库")),
    resolved_config(sdk_path(
      config_path, "TIANJI_OFFICIAL_IK_CONFIG", "机型配置")),
    executable(worker_path()),
    settings(settings)
  {
    if (settings.official_worker_timeout_ms <= 0 ||
      settings.official_worker_restart_attempts < 0)
    {
      throw std::invalid_argument("官方 IK worker 超时与重启参数非法");
    }
    const std::lock_guard<std::mutex> lock(mutex);
    std::string last_error;
    for (int attempt = 0;
      attempt <= settings.official_worker_restart_attempts; ++attempt)
    {
      try {
        start_worker_unlocked();
        return;
      } catch (const WorkerExecutionError &) {
        throw;
      } catch (const WorkerTransportError & exception) {
        last_error = exception.what();
        stop_worker_unlocked();
      }
    }
    throw WorkerTransportError(
            "官方 IK worker 恢复失败：" + last_error);
  }

  ~Impl()
  {
    const std::lock_guard<std::mutex> lock(mutex);
    stop_worker_unlocked();
  }

  std::vector<std::string> worker_arguments(int worker_fd) const
  {
    return {
      executable,
      std::to_string(worker_fd),
      resolved_library,
      resolved_config,
      serialize_double(settings.maximum_joint_step_rad),
      serialize_double(settings.position_tolerance_m),
      serialize_double(settings.orientation_tolerance_rad),
      settings.official_use_zsp ? "1" : "0",
      serialize_double(settings.official_dgr1),
      serialize_double(settings.official_dgr2),
      serialize_double(settings.official_dgr3),
      serialize_double(settings.official_joint_limit_soft_margin_rad),
      serialize_double(settings.official_candidate_continuity_weight),
      serialize_double(settings.official_candidate_limit_weight),
      serialize_double(settings.official_candidate_posture_weight),
      std::to_string(settings.official_orientation_relaxation_steps),
      std::to_string(settings.official_workspace_backoff_iterations),
      serialize_joints(settings.official_left_nominal_rad),
      serialize_joints(settings.official_right_nominal_rad),
    };
  }

  void start_worker_unlocked()
  {
    if (socket_fd >= 0 || child_pid > 0) {
      throw std::logic_error("官方 IK worker 已经启动");
    }
    int sockets[2]{};
    if (socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0, sockets) != 0) {
      throw std::runtime_error("无法创建官方 IK IPC：" + std::string(strerror(errno)));
    }
    constexpr int worker_fd = 3;
    std::vector<std::string> argument_strings = worker_arguments(worker_fd);
    std::vector<char *> arguments;
    arguments.reserve(argument_strings.size() + 1);
    for (std::string & argument : argument_strings) {
      arguments.push_back(argument.data());
    }
    arguments.push_back(nullptr);
    posix_spawn_file_actions_t actions;
    if (posix_spawn_file_actions_init(&actions) != 0) {
      close(sockets[0]);
      close(sockets[1]);
      throw std::runtime_error("无法初始化官方 IK worker 文件动作");
    }
    (void)posix_spawn_file_actions_adddup2(&actions, sockets[1], worker_fd);
    if (sockets[0] != worker_fd) {
      (void)posix_spawn_file_actions_addclose(&actions, sockets[0]);
    }
    if (sockets[1] != worker_fd) {
      (void)posix_spawn_file_actions_addclose(&actions, sockets[1]);
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
      const auto response = raw_exchange_unlocked(request);
      (void)response;
    } catch (...) {
      stop_worker_unlocked();
      throw;
    }
  }

  void stop_worker_unlocked() noexcept
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
        for (int attempt = 0; attempt < 20; ++attempt) {
          if (waitpid(child_pid, &status, WNOHANG) != 0) {
            break;
          }
          usleep(5000);
        }
        if (waitpid(child_pid, &status, WNOHANG) == 0) {
          kill(child_pid, SIGKILL);
          (void)waitpid(child_pid, &status, 0);
        }
      }
      child_pid = -1;
    }
  }

  official_ipc::Response raw_exchange_unlocked(
    const official_ipc::Request & request) const
  {
    const ssize_t sent = send(socket_fd, &request, sizeof(request), MSG_NOSIGNAL);
    if (sent != static_cast<ssize_t>(sizeof(request))) {
      throw WorkerTransportError("官方 IK worker 请求发送失败");
    }
    pollfd descriptor{};
    descriptor.fd = socket_fd;
    descriptor.events = POLLIN;
    int ready = 0;
    do {
      ready = poll(&descriptor, 1, settings.official_worker_timeout_ms);
    } while (ready < 0 && errno == EINTR);
    if (ready == 0) {
      throw WorkerTransportError("官方 IK worker 调用超时");
    }
    if (ready < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {
      throw WorkerTransportError("官方 IK worker IPC 失效");
    }
    official_ipc::Response response;
    const ssize_t received = recv(socket_fd, &response, sizeof(response), 0);
    if (received != static_cast<ssize_t>(sizeof(response))) {
      throw WorkerTransportError("官方 IK worker 异常退出或响应不完整");
    }
    if (
      response.magic != official_ipc::kMagic ||
      response.version != official_ipc::kVersion)
    {
      throw WorkerTransportError("官方 IK worker 协议版本不匹配");
    }
    if (response.error_code != 0) {
      throw WorkerExecutionError(
              "官方 IK worker：" + std::string(response.error));
    }
    return response;
  }

  ExchangeOutcome exchange(const official_ipc::Request & request)
  {
    const std::lock_guard<std::mutex> lock(mutex);
    const auto started = std::chrono::steady_clock::now();
    std::string last_error;
    int restart_count = 0;
    for (int attempt = 0;
      attempt <= settings.official_worker_restart_attempts; ++attempt)
    {
      try {
        ExchangeOutcome outcome;
        outcome.response = raw_exchange_unlocked(request);
        outcome.restart_count = restart_count;
        outcome.elapsed_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
        return outcome;
      } catch (const WorkerExecutionError &) {
        throw;
      } catch (const WorkerTransportError & exception) {
        last_error = exception.what();
        stop_worker_unlocked();
        if (attempt >= settings.official_worker_restart_attempts) {
          break;
        }
        start_worker_unlocked();
        ++restart_count;
      }
    }
    throw WorkerTransportError(
            "官方 IK worker 恢复失败：" + last_error);
  }

  std::string resolved_library;
  std::string resolved_config;
  std::string executable;
  IkSettings settings;
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
  return read_pose(impl_->exchange(request).response.pose);
}

IkResult TianjiOfficialArmIk::solve(
  ArmSide side,
  const Eigen::Isometry3d & target_pose,
  const ArmJointVector & current_joints_rad,
  const Eigen::Vector3d & elbow_reference_direction) const
{
  official_ipc::Request request;
  request.operation = official_ipc::Operation::kSolve;
  request.side = side == ArmSide::kLeft ? 0 : 1;
  copy_pose(request.target_pose, target_pose);
  for (Eigen::Index index = 0; index < current_joints_rad.size(); ++index) {
    request.joints_rad[index] = current_joints_rad[index];
  }
  for (Eigen::Index index = 0; index < elbow_reference_direction.size(); ++index) {
    request.elbow_direction[index] = elbow_reference_direction[index];
  }
  const auto exchange = impl_->exchange(request);
  const auto & response = exchange.response;
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
  result.requested_maximum_joint_step_rad =
    response.requested_maximum_joint_step_rad;
  result.solve_time_ms = response.solve_time_ms;
  result.transport_time_ms = exchange.elapsed_ms;
  result.workspace_backoff_fraction = response.workspace_backoff_fraction;
  result.candidate_count = response.candidate_count;
  result.selected_candidate_index = response.selected_candidate_index;
  result.soft_limit_active = response.soft_limit_active != 0;
  result.workspace_backoff_active =
    response.workspace_backoff_active != 0;
  result.orientation_relaxed = response.orientation_relaxed != 0;
  result.transport_restart_count = exchange.restart_count;
  result.transport_recovered = exchange.restart_count > 0;
  result.status = response.status;
  return result;
}

}  // namespace pico_body_tianji
