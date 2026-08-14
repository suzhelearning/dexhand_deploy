#include "pico_body_tianji/ik/tianji_official/tianji_official_arm_ik.hpp"

#include <dlfcn.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

namespace pico_body_tianji
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadiansToDegrees = 180.0 / kPi;
constexpr double kDegreesToRadians = kPi / 180.0;

using FxBool = std::uint8_t;
using Matrix3 = double[3][3];
using Matrix4 = double[4][4];
using Matrix8 = double[8][8];
using Vector7 = double[7];

// 该结构严格对应厂商 FXKineCommon.h 中的 FX_InvKineSolvePara。
// 静态断言防止编译器 ABI 布局变化后静默调用错误的二进制接口。
struct FxInvKineSolvePara
{
  Matrix4 input_target_tcp;
  Vector7 input_reference_joint;
  std::int32_t input_zsp_type;
  double input_zsp_parameter[6];
  double input_zsp_angle;
  double dgr1;
  double dgr2;
  double dgr3;
  Vector7 output_joint;
  Matrix8 output_all_joint;
  std::int32_t output_result_count;
  FxBool output_is_out_of_range;
  FxBool output_is_singular[7];
  FxBool output_joint_limit_tags[7];
  double output_joint_limit_excess;
  FxBool output_is_joint_limit_exceeded;
  Vector7 output_positive_limits;
  Vector7 output_negative_limits;
};

static_assert(sizeof(FxBool) == 1);
static_assert(sizeof(FxInvKineSolvePara) == 992);
static_assert(offsetof(FxInvKineSolvePara, output_joint) == 272);
static_assert(offsetof(FxInvKineSolvePara, output_result_count) == 840);
static_assert(offsetof(FxInvKineSolvePara, output_joint_limit_excess) == 864);
static_assert(offsetof(FxInvKineSolvePara, output_positive_limits) == 880);

bool finite_pose(const Eigen::Isometry3d & pose)
{
  return pose.matrix().allFinite();
}

double orientation_error(
  const Eigen::Isometry3d & achieved,
  const Eigen::Isometry3d & target)
{
  Eigen::Quaterniond difference(
    achieved.rotation().transpose() * target.rotation());
  difference.normalize();
  return 2.0 * std::atan2(
    difference.vec().norm(), std::abs(difference.w()));
}

std::int32_t serial(ArmSide side)
{
  return side == ArmSide::kLeft ? 0 : 1;
}

}  // namespace

struct TianjiOfficialArmIk::Impl
{
  using LoadConfig = FxBool (*)(
    char *, std::int32_t[2], double[2][3], double[2][8][4],
    double[2][7][4], double[2][4][3], double[2][7], double[2][7][3],
    double[2][7][6]);
  using InitType = FxBool (*)(std::int32_t, std::int32_t);
  using InitKinematics = FxBool (*)(std::int32_t, double[8][4]);
  using InitLimits = FxBool (*)(std::int32_t, double[7][4], double[4][3]);
  using ForwardKinematics = FxBool (*)(std::int32_t, double[7], Matrix4);
  using InverseKinematics = FxBool (*)(
    std::int32_t, FxInvKineSolvePara *);
  using LogSwitch = void (*)(std::int32_t);

  Impl(
    const std::string & library_path,
    const std::string & config_path,
    const IkSettings & solver_settings)
  : settings(solver_settings)
  {
    if (library_path.empty()) {
      throw std::invalid_argument(
              "选择 tianji_official 时 official_ik_library 不能为空");
    }
    if (config_path.empty()) {
      throw std::invalid_argument(
              "选择 tianji_official 时 official_ik_config 不能为空");
    }
    if (settings.maximum_joint_step_rad <= 0.0) {
      throw std::invalid_argument("maximum_joint_step_rad 必须为正数");
    }

    // 官方接口由运行时动态解析，使得 pinocchio_cpp 后端
    // 不需要厂商 SDK 也可以独立运行。保持在主 link-map 内，
    // 确保调用方分配的 C ABI 参数与 libKine 共享同一份 libc。
    handle = dlopen(
      library_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle == nullptr) {
      throw std::runtime_error(
              "无法加载天机官方 IK 库 '" + library_path + "'：" +
              dlerror());
    }

    try {
      load_config = symbol<LoadConfig>("LOADMvCfg");
      init_type = symbol<InitType>("FX_Robot_Init_Type");
      init_kinematics = symbol<InitKinematics>("FX_Robot_Init_Kine");
      init_limits = symbol<InitLimits>("FX_Robot_Init_Lmt");
      forward_kinematics =
        symbol<ForwardKinematics>("FX_Robot_Kine_FK");
      inverse_kinematics =
        symbol<InverseKinematics>("FX_Robot_Kine_IK");
      if (auto log_switch = optional_symbol<LogSwitch>("FX_LOG_SWITCH")) {
        log_switch(0);
      }
      initialize(config_path);
    } catch (...) {
      dlclose(handle);
      handle = nullptr;
      throw;
    }
  }

  ~Impl()
  {
    if (handle != nullptr) {
      dlclose(handle);
    }
  }

  template<typename Function>
  Function symbol(const char * name)
  {
    dlerror();
    void * address = dlsym(handle, name);
    const char * error = dlerror();
    if (error != nullptr || address == nullptr) {
      throw std::runtime_error(
              "天机官方 IK 库缺少符号 " + std::string(name) + "：" +
              (error == nullptr ? "unknown dlsym error" : error));
    }
    return reinterpret_cast<Function>(address);
  }

  template<typename Function>
  Function optional_symbol(const char * name)
  {
    dlerror();
    void * address = dlsym(handle, name);
    if (dlerror() != nullptr || address == nullptr) {
      return nullptr;
    }
    return reinterpret_cast<Function>(address);
  }

  void initialize(const std::string & config_path)
  {
    std::int32_t types[2]{};
    double gravity[2][3]{};
    double dh[2][8][4]{};
    double limits[2][7][4]{};
    double joint_67_limits[2][4][3]{};
    double mass[2][7]{};
    double center_of_mass[2][7][3]{};
    double inertia[2][7][6]{};
    if (!load_config(
        const_cast<char *>(config_path.c_str()),
        types, gravity, dh, limits, joint_67_limits,
        mass, center_of_mass, inertia))
    {
      throw std::runtime_error(
              "天机官方 IK 无法解析配置文件：" + config_path);
    }

    for (std::int32_t arm_serial = 0; arm_serial < 2; ++arm_serial) {
      if (!init_type(arm_serial, types[arm_serial])) {
        throw std::runtime_error(
                "天机官方 IK 初始化机型失败，serial=" +
                std::to_string(arm_serial));
      }
      if (!init_kinematics(arm_serial, dh[arm_serial])) {
        throw std::runtime_error(
                "天机官方 IK 初始化 DH 参数失败，serial=" +
                std::to_string(arm_serial));
      }
      if (!init_limits(
          arm_serial, limits[arm_serial], joint_67_limits[arm_serial]))
      {
        throw std::runtime_error(
                "天机官方 IK 初始化关节限位失败，serial=" +
                std::to_string(arm_serial));
      }
    }
  }

  Eigen::Isometry3d forward_unlocked(
    ArmSide side,
    const ArmJointVector & joints_rad) const
  {
    if (!joints_rad.allFinite()) {
      throw std::invalid_argument("官方 FK 输入关节含有非有限值");
    }
    double joints_degrees[7]{};
    for (Eigen::Index index = 0; index < joints_rad.size(); ++index) {
      joints_degrees[index] = joints_rad[index] * kRadiansToDegrees;
    }
    Matrix4 matrix{};
    if (!forward_kinematics(serial(side), joints_degrees, matrix)) {
      throw std::runtime_error("天机官方 FK 解算失败");
    }

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    for (Eigen::Index row = 0; row < 3; ++row) {
      for (Eigen::Index column = 0; column < 3; ++column) {
        pose.linear()(row, column) = matrix[row][column];
      }
      pose.translation()[row] = matrix[row][3] * 1.0e-3;
    }
    if (!finite_pose(pose)) {
      throw std::runtime_error("天机官方 FK 返回非有限位姿");
    }
    return pose;
  }

  IkSettings settings;
  void * handle{nullptr};
  LoadConfig load_config{nullptr};
  InitType init_type{nullptr};
  InitKinematics init_kinematics{nullptr};
  InitLimits init_limits{nullptr};
  ForwardKinematics forward_kinematics{nullptr};
  InverseKinematics inverse_kinematics{nullptr};
  mutable std::mutex mutex;
};

TianjiOfficialArmIk::TianjiOfficialArmIk(
  const std::string & library_path,
  const std::string & config_path,
  const IkSettings & settings)
: impl_(std::make_unique<Impl>(library_path, config_path, settings))
{
}

TianjiOfficialArmIk::~TianjiOfficialArmIk() = default;
TianjiOfficialArmIk::TianjiOfficialArmIk(TianjiOfficialArmIk &&) noexcept =
  default;
TianjiOfficialArmIk & TianjiOfficialArmIk::operator=(
  TianjiOfficialArmIk &&) noexcept = default;

Eigen::Isometry3d TianjiOfficialArmIk::forward(
  ArmSide side,
  const ArmJointVector & joints_rad) const
{
  const std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->forward_unlocked(side, joints_rad);
}

IkResult TianjiOfficialArmIk::solve(
  ArmSide side,
  const Eigen::Isometry3d & target_pose,
  const ArmJointVector & current_joints_rad,
  const Eigen::Vector3d & elbow_ik_direction) const
{
  if (!finite_pose(target_pose)) {
    throw std::invalid_argument("官方 IK 目标位姿含有非有限值");
  }
  if (!current_joints_rad.allFinite()) {
    throw std::invalid_argument("官方 IK 参考关节含有非有限值");
  }

  const std::lock_guard<std::mutex> lock(impl_->mutex);
  FxInvKineSolvePara parameters{};
  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index column = 0; column < 4; ++column) {
      parameters.input_target_tcp[row][column] =
        target_pose.matrix()(row, column);
    }
  }
  for (Eigen::Index row = 0; row < 3; ++row) {
    parameters.input_target_tcp[row][3] *= 1.0e3;
  }
  for (Eigen::Index index = 0; index < current_joints_rad.size(); ++index) {
    parameters.input_reference_joint[index] =
      current_joints_rad[index] * kRadiansToDegrees;
  }
  const bool use_elbow_direction =
    impl_->settings.arm_angle_gain > 0.0 &&
    elbow_ik_direction.allFinite() && elbow_ik_direction.norm() > 1.0e-8;
  parameters.input_zsp_type = use_elbow_direction ? 1 : 0;
  if (use_elbow_direction) {
    const Eigen::Vector3d normalized = elbow_ik_direction.normalized();
    for (Eigen::Index index = 0; index < normalized.size(); ++index) {
      parameters.input_zsp_parameter[index] = normalized[index];
    }
  }
  parameters.dgr1 = 0.05;
  parameters.dgr2 = 0.05;

  const bool solved = impl_->inverse_kinematics(
    serial(side), &parameters) != 0;
  const bool singular = std::any_of(
    std::begin(parameters.output_is_singular),
    std::end(parameters.output_is_singular),
    [](FxBool value) {return value != 0;});
  const bool out_of_range = parameters.output_is_out_of_range != 0;
  const bool joint_limit_exceeded =
    parameters.output_is_joint_limit_exceeded != 0;

  IkResult result;
  result.joints_rad = current_joints_rad;
  result.achieved_pose = impl_->forward_unlocked(side, current_joints_rad);
  result.singularity_active = singular;
  result.saturated = !solved || out_of_range || joint_limit_exceeded || singular;

  if (!solved || result.saturated) {
    result.position_error_m =
      (target_pose.translation() - result.achieved_pose.translation()).norm();
    result.orientation_error_rad =
      orientation_error(result.achieved_pose, target_pose);
    if (out_of_range) {
      result.status = "out_of_range";
    } else if (joint_limit_exceeded) {
      result.status = "joint_limit";
    } else if (singular) {
      result.status = "singular";
    } else {
      result.status = "solve_failed";
    }
    return result;
  }

  ArmJointVector candidate;
  for (Eigen::Index index = 0; index < candidate.size(); ++index) {
    candidate[index] =
      parameters.output_joint[index] * kDegreesToRadians;
  }
  if (!candidate.allFinite()) {
    throw std::runtime_error("天机官方 IK 返回非有限关节角");
  }

  ArmJointVector delta = candidate - current_joints_rad;
  const double requested_step = delta.cwiseAbs().maxCoeff();
  if (requested_step > impl_->settings.maximum_joint_step_rad) {
    delta *= impl_->settings.maximum_joint_step_rad / requested_step;
    result.joint_step_limited = true;
  }
  result.joints_rad = current_joints_rad + delta;
  result.maximum_joint_step_rad = delta.cwiseAbs().maxCoeff();
  result.achieved_pose = impl_->forward_unlocked(side, result.joints_rad);
  result.position_error_m =
    (target_pose.translation() - result.achieved_pose.translation()).norm();
  result.orientation_error_rad =
    orientation_error(result.achieved_pose, target_pose);
  result.accepted = true;
  result.converged =
    result.position_error_m <= impl_->settings.position_tolerance_m &&
    result.orientation_error_rad <=
    impl_->settings.orientation_tolerance_rad;
  result.status = result.converged ? "converged" : "tracking";

  double minimum_margin_degrees = std::numeric_limits<double>::infinity();
  for (Eigen::Index index = 0; index < result.joints_rad.size(); ++index) {
    const double joint_degrees =
      result.joints_rad[index] * kRadiansToDegrees;
    minimum_margin_degrees = std::min(
      minimum_margin_degrees,
      std::min(
        parameters.output_positive_limits[index] - joint_degrees,
        joint_degrees - parameters.output_negative_limits[index]));
  }
  if (std::isfinite(minimum_margin_degrees)) {
    result.minimum_limit_margin_rad =
      minimum_margin_degrees * kDegreesToRadians;
  }
  return result;
}

}  // namespace pico_body_tianji
