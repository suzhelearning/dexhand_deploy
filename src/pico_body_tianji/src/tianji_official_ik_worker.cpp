#include "pico_body_tianji/tianji_official_arm_ik.hpp"
#include "pico_body_tianji/tianji_official_ipc.hpp"

#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{

using pico_body_tianji::official_ipc::Request;
using pico_body_tianji::official_ipc::Response;

pico_body_tianji::ArmSide side(std::int32_t value)
{
  return value == 0 ? pico_body_tianji::ArmSide::kLeft :
         pico_body_tianji::ArmSide::kRight;
}

Eigen::Isometry3d pose(const double input[16])
{
  Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index column = 0; column < 4; ++column) {
      output.matrix()(row, column) = input[row * 4 + column];
    }
  }
  return output;
}

void copy_pose(double output[16], const Eigen::Isometry3d & input)
{
  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index column = 0; column < 4; ++column) {
      output[row * 4 + column] = input.matrix()(row, column);
    }
  }
}

pico_body_tianji::ArmJointVector joints(const double input[7])
{
  pico_body_tianji::ArmJointVector output;
  for (Eigen::Index index = 0; index < output.size(); ++index) {
    output[index] = input[index];
  }
  return output;
}

void respond_error(Response & response, const std::exception & exception)
{
  response.error_code = 1;
  std::strncpy(response.error, exception.what(), sizeof(response.error) - 1);
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 8) {
    return 2;
  }
  const int socket_fd = std::stoi(argv[1]);
  try {
    pico_body_tianji::IkSettings settings;
    settings.maximum_joint_step_rad = std::stod(argv[4]);
    settings.position_tolerance_m = std::stod(argv[5]);
    settings.orientation_tolerance_rad = std::stod(argv[6]);
    settings.arm_angle_gain = std::stod(argv[7]);
    pico_body_tianji::TianjiOfficialArmIk solver(argv[2], argv[3], settings);
    while (true) {
      Request request;
      const ssize_t received = recv(socket_fd, &request, sizeof(request), 0);
      if (received == 0) {
        break;
      }
      if (received != static_cast<ssize_t>(sizeof(request))) {
        return 3;
      }
      if (
        request.magic != pico_body_tianji::official_ipc::kMagic ||
        request.version != pico_body_tianji::official_ipc::kVersion)
      {
        return 4;
      }
      if (request.operation == pico_body_tianji::official_ipc::Operation::kShutdown) {
        break;
      }
      Response response;
      try {
        if (request.operation == pico_body_tianji::official_ipc::Operation::kForward) {
          copy_pose(response.pose, solver.forward(side(request.side), joints(request.joints_rad)));
        } else if (request.operation == pico_body_tianji::official_ipc::Operation::kSolve) {
          const Eigen::Vector3d elbow(
            request.elbow_direction[0], request.elbow_direction[1],
            request.elbow_direction[2]);
          const auto result = solver.solve(
            side(request.side), pose(request.target_pose),
            joints(request.joints_rad), elbow);
          copy_pose(response.pose, result.achieved_pose);
          for (Eigen::Index index = 0; index < result.joints_rad.size(); ++index) {
            response.joints_rad[index] = result.joints_rad[index];
          }
          response.accepted = result.accepted;
          response.converged = result.converged;
          response.saturated = result.saturated;
          response.joint_step_limited = result.joint_step_limited;
          response.singularity_active = result.singularity_active;
          response.position_error_m = result.position_error_m;
          response.orientation_error_rad = result.orientation_error_rad;
          response.minimum_singular_value = result.minimum_singular_value;
          response.damping = result.damping;
          response.arm_angle_error_rad = result.arm_angle_error_rad;
          response.minimum_limit_margin_rad = result.minimum_limit_margin_rad;
          response.maximum_joint_step_rad = result.maximum_joint_step_rad;
          std::strncpy(response.status, result.status.c_str(), sizeof(response.status) - 1);
        } else {
          throw std::runtime_error("未知 IPC 操作");
        }
      } catch (const std::exception & exception) {
        respond_error(response, exception);
      }
      if (send(socket_fd, &response, sizeof(response), MSG_NOSIGNAL) !=
        static_cast<ssize_t>(sizeof(response)))
      {
        return 5;
      }
    }
    close(socket_fd);
    return 0;
  } catch (const std::exception & exception) {
    Response response;
    respond_error(response, exception);
    (void)send(socket_fd, &response, sizeof(response), MSG_NOSIGNAL);
    close(socket_fd);
    return 1;
  }
}
