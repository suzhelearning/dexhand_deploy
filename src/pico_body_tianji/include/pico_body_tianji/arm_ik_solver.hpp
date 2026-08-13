#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <string>

namespace pico_body_tianji
{

enum class ArmSide
{
  kLeft,
  kRight,
};

using ArmJointVector = Eigen::Matrix<double, 7, 1>;

// 后端公共配置。所有角度均为弧度、长度均为米；后端不支持的优化项可以
// 忽略，但 maximum_joint_step_rad 和两个末端容差属于公共安全契约。
struct IkSettings
{
  int max_iterations{24};
  double position_tolerance_m{1.0e-3};
  double orientation_tolerance_rad{1.0e-2};
  double minimum_damping{1.0e-3};
  double maximum_damping{1.5e-1};
  double singular_value_threshold{5.0e-2};
  double maximum_iteration_step_rad{8.0e-2};
  double maximum_joint_step_rad{3.0 * 3.14159265358979323846 / 180.0};
  double joint_limit_margin_rad{5.0 * 3.14159265358979323846 / 180.0};
  double arm_angle_gain{0.0};
  double arm_angle_tolerance_rad{
    2.0 * 3.14159265358979323846 / 180.0};
  double arm_angle_finite_difference_rad{1.0e-4};
  double arm_angle_merit_weight{1.0e-3};
  double nullspace_damping{1.0e-3};
  double joint_center_gain{0.0};
  double joint_center_activation_margin_rad{
    15.0 * 3.14159265358979323846 / 180.0};
  double joint_center_merit_weight{1.0e-3};
  double singularity_avoidance_gain{0.0};
  double singularity_finite_difference_rad{1.0e-4};
  double singularity_merit_weight{1.0e-2};
};

struct IkResult
{
  ArmJointVector joints_rad{ArmJointVector::Zero()};
  Eigen::Isometry3d achieved_pose{Eigen::Isometry3d::Identity()};
  bool accepted{false};
  bool converged{false};
  bool saturated{false};
  bool joint_step_limited{false};
  bool singularity_active{false};
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
  // 后端无法提供的诊断量应保持 NaN；状态 JSON 会把它们发布为 null。
  double minimum_singular_value{
    std::numeric_limits<double>::quiet_NaN()};
  double damping{std::numeric_limits<double>::quiet_NaN()};
  double arm_angle_error_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double minimum_limit_margin_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double maximum_joint_step_rad{0.0};
  std::string status;
};

// 双臂 IK 后端稳定边界（arm_ik_solver_v1）。
//
// 坐标与单位契约：
// - 关节顺序固定为 Joint1_{L,R} ... Joint7_{L,R}，单位为弧度；
// - 位姿为对应 Base_{L,R} 到 TCP_Link_{L,R} 的变换，平移单位为米；
// - elbow_ik_direction 位于同一 Base 坐标系，是 libKine zsp_para 约定的
//   参考平面方向，而不是物理肩肘偏移方向。
//
// solve() 必须总是返回有限的 joints_rad 和 achieved_pose。accepted=true
// 表示上层可以安全采用 joints_rad；converged=true 表示末端主任务已满足
// 配置容差。每次被接受的输出不得超过 maximum_joint_step_rad。
class ArmIkSolver
{
public:
  virtual ~ArmIkSolver() = default;

  virtual Eigen::Isometry3d forward(
    ArmSide side,
    const ArmJointVector & joints_rad) const = 0;

  virtual IkResult solve(
    ArmSide side,
    const Eigen::Isometry3d & target_pose,
    const ArmJointVector & current_joints_rad,
    const Eigen::Vector3d & elbow_ik_direction) const = 0;
};

}  // namespace pico_body_tianji
