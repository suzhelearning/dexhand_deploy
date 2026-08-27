#include "pico_body_tianji/ik/tianji_official/tianji_official_arm_ik.hpp"

#include <dlfcn.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

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
    if (
      settings.official_joint_limit_soft_margin_rad < 0.0 ||
      settings.official_orientation_relaxation_steps < 0 ||
      settings.official_workspace_backoff_iterations < 0 ||
      settings.official_candidate_continuity_weight < 0.0 ||
      settings.official_candidate_limit_weight < 0.0 ||
      settings.official_candidate_posture_weight < 0.0 ||
      !std::isfinite(settings.official_dgr1) ||
      !std::isfinite(settings.official_dgr2) ||
      !std::isfinite(settings.official_dgr3))
    {
      throw std::invalid_argument("官方 IK 软限位与回退参数非法");
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
  const auto solve_started = std::chrono::steady_clock::now();
  if (!finite_pose(target_pose)) {
    throw std::invalid_argument("官方 IK 目标位姿含有非有限值");
  }
  if (!current_joints_rad.allFinite()) {
    throw std::invalid_argument("官方 IK 参考关节含有非有限值");
  }

  const std::lock_guard<std::mutex> lock(impl_->mutex);
  struct Attempt
  {
    FxInvKineSolvePara parameters{};
    ArmJointVector candidate{ArmJointVector::Zero()};
    bool success{false};
    bool singular{false};
    bool out_of_range{false};
    bool joint_limit_exceeded{false};
    int candidate_count{0};
    int selected_candidate_index{-1};
    std::string status{"solve_failed"};
  };

  const Eigen::Isometry3d current_pose =
    impl_->forward_unlocked(side, current_joints_rad);
  const ArmJointVector & nominal = side == ArmSide::kLeft ?
    impl_->settings.official_left_nominal_rad :
    impl_->settings.official_right_nominal_rad;

  auto run_attempt = [&](const Eigen::Isometry3d & requested_pose) {
      Attempt attempt;
      for (Eigen::Index row = 0; row < 4; ++row) {
        for (Eigen::Index column = 0; column < 4; ++column) {
          attempt.parameters.input_target_tcp[row][column] =
            requested_pose.matrix()(row, column);
        }
      }
      for (Eigen::Index row = 0; row < 3; ++row) {
        attempt.parameters.input_target_tcp[row][3] *= 1.0e3;
      }
      for (Eigen::Index index = 0; index < current_joints_rad.size(); ++index) {
        attempt.parameters.input_reference_joint[index] =
          current_joints_rad[index] * kRadiansToDegrees;
      }
      const bool use_elbow_direction =
        impl_->settings.official_use_zsp &&
        elbow_ik_direction.allFinite() &&
        elbow_ik_direction.norm() > 1.0e-8;
      attempt.parameters.input_zsp_type = use_elbow_direction ? 1 : 0;
      if (use_elbow_direction) {
        const Eigen::Vector3d normalized = elbow_ik_direction.normalized();
        for (Eigen::Index index = 0; index < normalized.size(); ++index) {
          attempt.parameters.input_zsp_parameter[index] = normalized[index];
        }
      }
      attempt.parameters.dgr1 = impl_->settings.official_dgr1;
      attempt.parameters.dgr2 = impl_->settings.official_dgr2;
      attempt.parameters.dgr3 = impl_->settings.official_dgr3;

      const bool solved = impl_->inverse_kinematics(
        serial(side), &attempt.parameters) != 0;
      attempt.singular = std::any_of(
        std::begin(attempt.parameters.output_is_singular),
        std::end(attempt.parameters.output_is_singular),
        [](FxBool value) {return value != 0;});
      attempt.out_of_range =
        attempt.parameters.output_is_out_of_range != 0;
      attempt.joint_limit_exceeded =
        attempt.parameters.output_is_joint_limit_exceeded != 0;
      if (!solved || attempt.out_of_range ||
        attempt.joint_limit_exceeded || attempt.singular)
      {
        attempt.status = attempt.out_of_range ? "out_of_range" :
          attempt.joint_limit_exceeded ? "joint_limit" :
          attempt.singular ? "singular" : "solve_failed";
        return attempt;
      }

      struct ScoredCandidate
      {
        ArmJointVector joints{ArmJointVector::Zero()};
        double cost{0.0};
        int source_index{-1};
      };
      std::vector<ScoredCandidate> candidates;
      const double validation_position_tolerance = std::max(
        5.0 * impl_->settings.position_tolerance_m, 5.0e-3);
      const double validation_orientation_tolerance = std::max(
        5.0 * impl_->settings.orientation_tolerance_rad,
        2.0 * kPi / 180.0);

      auto add_candidate = [&](const ArmJointVector & joints, int source_index,
          bool validate_fk) {
          if (!joints.allFinite()) {
            return;
          }
          for (Eigen::Index index = 0; index < joints.size(); ++index) {
            const double degrees_value = joints[index] * kRadiansToDegrees;
            const double lower = attempt.parameters.output_negative_limits[index];
            const double upper = attempt.parameters.output_positive_limits[index];
            if (
              std::isfinite(lower) && std::isfinite(upper) && lower < upper &&
              (degrees_value < lower - 1.0e-6 ||
              degrees_value > upper + 1.0e-6))
            {
              return;
            }
          }
          if (validate_fk) {
            const Eigen::Isometry3d pose = impl_->forward_unlocked(side, joints);
            if (
              (pose.translation() - requested_pose.translation()).norm() >
              validation_position_tolerance ||
              orientation_error(pose, requested_pose) >
              validation_orientation_tolerance)
            {
              return;
            }
          }
          for (const auto & existing : candidates) {
            if ((existing.joints - joints).cwiseAbs().maxCoeff() < 1.0e-7) {
              return;
            }
          }
          double limit_penalty = 0.0;
          for (Eigen::Index index = 0; index < joints.size(); ++index) {
            const double lower =
              attempt.parameters.output_negative_limits[index] *
              kDegreesToRadians;
            const double upper =
              attempt.parameters.output_positive_limits[index] *
              kDegreesToRadians;
            if (std::isfinite(lower) && std::isfinite(upper) && lower < upper) {
              const double margin = std::min(
                upper - joints[index], joints[index] - lower);
              const double scale = std::max(
                impl_->settings.official_joint_limit_soft_margin_rad, 1.0e-6);
              const double deficit = std::max(0.0, 2.0 - margin / scale);
              limit_penalty += deficit * deficit;
            }
          }
          const double continuity =
            (joints - current_joints_rad).squaredNorm();
          const double posture = (joints - nominal).squaredNorm();
          candidates.push_back(ScoredCandidate{
            joints,
            impl_->settings.official_candidate_continuity_weight * continuity +
            impl_->settings.official_candidate_limit_weight * limit_penalty +
            impl_->settings.official_candidate_posture_weight * posture,
            source_index});
        };

      ArmJointVector primary;
      for (Eigen::Index index = 0; index < primary.size(); ++index) {
        primary[index] =
          attempt.parameters.output_joint[index] * kDegreesToRadians;
      }
      add_candidate(primary, 0, false);

      const int reported_count = std::clamp(
        attempt.parameters.output_result_count, 0, 8);
      for (int candidate_index = 0; candidate_index < reported_count;
        ++candidate_index)
      {
        ArmJointVector row_candidate;
        ArmJointVector column_candidate;
        for (Eigen::Index joint_index = 0; joint_index < 7; ++joint_index) {
          row_candidate[joint_index] =
            attempt.parameters.output_all_joint[candidate_index][joint_index] *
            kDegreesToRadians;
          column_candidate[joint_index] =
            attempt.parameters.output_all_joint[joint_index][candidate_index] *
            kDegreesToRadians;
        }
        add_candidate(row_candidate, 1 + candidate_index, true);
        // 同时验证转置布局；只有 FK 符合目标的解释才会进入候选集合。
        add_candidate(column_candidate, 9 + candidate_index, true);
      }
      if (candidates.empty()) {
        throw std::runtime_error("天机官方 IK 返回非有限或无效候选解");
      }
      const auto selected = std::min_element(
        candidates.begin(), candidates.end(),
        [](const ScoredCandidate & left, const ScoredCandidate & right) {
          return left.cost < right.cost;
        });
      attempt.candidate = selected->joints;
      attempt.candidate_count = static_cast<int>(candidates.size());
      attempt.selected_candidate_index = selected->source_index;
      attempt.success = true;
      attempt.status = "solved";
      return attempt;
    };

  Attempt selected = run_attempt(target_pose);
  const Attempt requested_attempt = selected;
  bool orientation_relaxed = false;
  bool workspace_backoff = false;
  double backoff_fraction = 1.0;

  if (!selected.success &&
    impl_->settings.official_orientation_relaxation_steps > 0)
  {
    const Eigen::Quaterniond current_rotation(current_pose.rotation());
    const Eigen::Quaterniond requested_rotation(target_pose.rotation());
    for (int step = impl_->settings.official_orientation_relaxation_steps;
      step >= 0 && !selected.success; --step)
    {
      const double fraction = static_cast<double>(step) /
        static_cast<double>(
        impl_->settings.official_orientation_relaxation_steps + 1);
      Eigen::Isometry3d relaxed = target_pose;
      relaxed.linear() =
        current_rotation.slerp(fraction, requested_rotation).toRotationMatrix();
      Attempt attempt = run_attempt(relaxed);
      if (attempt.success) {
        selected = std::move(attempt);
        orientation_relaxed = true;
      }
    }
  }

  if (!selected.success &&
    impl_->settings.official_workspace_backoff_iterations > 0)
  {
    const Eigen::Quaterniond current_rotation(current_pose.rotation());
    const Eigen::Quaterniond requested_rotation(target_pose.rotation());
    double lower = 0.0;
    double upper = 1.0;
    Attempt best;
    for (int iteration = 0;
      iteration < impl_->settings.official_workspace_backoff_iterations;
      ++iteration)
    {
      const double fraction = 0.5 * (lower + upper);
      Eigen::Isometry3d backed_off = Eigen::Isometry3d::Identity();
      backed_off.translation() =
        current_pose.translation() +
        fraction * (target_pose.translation() - current_pose.translation());
      backed_off.linear() =
        current_rotation.slerp(fraction, requested_rotation).toRotationMatrix();
      Attempt attempt = run_attempt(backed_off);
      if (attempt.success) {
        lower = fraction;
        best = std::move(attempt);
      } else {
        upper = fraction;
      }
    }
    if (best.success && lower > 1.0e-3) {
      selected = std::move(best);
      workspace_backoff = true;
      backoff_fraction = lower;
    }
  }

  IkResult result;
  result.joints_rad = current_joints_rad;
  result.achieved_pose = current_pose;
  result.singularity_active = requested_attempt.singular;
  result.orientation_relaxed = orientation_relaxed;
  result.workspace_backoff_active = workspace_backoff;
  result.workspace_backoff_fraction = backoff_fraction;

  if (!selected.success) {
    result.saturated = true;
    result.status = requested_attempt.status;
    result.position_error_m =
      (target_pose.translation() - current_pose.translation()).norm();
    result.orientation_error_rad = orientation_error(current_pose, target_pose);
    result.solve_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - solve_started).count();
    return result;
  }

  ArmJointVector delta = selected.candidate - current_joints_rad;
  result.requested_maximum_joint_step_rad = delta.cwiseAbs().maxCoeff();
  if (
    result.requested_maximum_joint_step_rad >
    impl_->settings.maximum_joint_step_rad)
  {
    delta *= impl_->settings.maximum_joint_step_rad /
      result.requested_maximum_joint_step_rad;
    result.joint_step_limited = true;
  }
  ArmJointVector proposed = current_joints_rad + delta;
  for (Eigen::Index index = 0; index < proposed.size(); ++index) {
    const double lower =
      selected.parameters.output_negative_limits[index] * kDegreesToRadians +
      impl_->settings.official_joint_limit_soft_margin_rad;
    const double upper =
      selected.parameters.output_positive_limits[index] * kDegreesToRadians -
      impl_->settings.official_joint_limit_soft_margin_rad;
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower >= upper) {
      continue;
    }
    const double before = proposed[index];
    if (current_joints_rad[index] < lower) {
      proposed[index] = std::max(proposed[index], current_joints_rad[index]);
    } else if (current_joints_rad[index] > upper) {
      proposed[index] = std::min(proposed[index], current_joints_rad[index]);
    } else {
      proposed[index] = std::clamp(proposed[index], lower, upper);
    }
    result.soft_limit_active = result.soft_limit_active ||
      std::abs(before - proposed[index]) > 1.0e-12;
  }

  result.joints_rad = proposed;
  result.maximum_joint_step_rad =
    (result.joints_rad - current_joints_rad).cwiseAbs().maxCoeff();
  result.achieved_pose = impl_->forward_unlocked(side, result.joints_rad);
  result.position_error_m =
    (target_pose.translation() - result.achieved_pose.translation()).norm();
  result.orientation_error_rad =
    orientation_error(result.achieved_pose, target_pose);
  result.accepted = true;
  result.saturated = orientation_relaxed || workspace_backoff ||
    result.soft_limit_active;
  result.converged =
    !result.saturated &&
    result.position_error_m <= impl_->settings.position_tolerance_m &&
    result.orientation_error_rad <=
    impl_->settings.orientation_tolerance_rad;
  result.status = result.soft_limit_active ? "soft_joint_limit" :
    orientation_relaxed ? "orientation_relaxed" :
    workspace_backoff ? "workspace_backoff" :
    result.converged ? "converged" : "tracking";
  result.candidate_count = selected.candidate_count;
  result.selected_candidate_index = selected.selected_candidate_index;

  double minimum_margin_degrees = std::numeric_limits<double>::infinity();
  for (Eigen::Index index = 0; index < result.joints_rad.size(); ++index) {
    const double joint_degrees = result.joints_rad[index] * kRadiansToDegrees;
    minimum_margin_degrees = std::min(
      minimum_margin_degrees,
      std::min(
        selected.parameters.output_positive_limits[index] - joint_degrees,
        joint_degrees - selected.parameters.output_negative_limits[index]));
  }
  if (std::isfinite(minimum_margin_degrees)) {
    result.minimum_limit_margin_rad =
      minimum_margin_degrees * kDegreesToRadians;
  }
  result.solve_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - solve_started).count();
  return result;
}

}  // namespace pico_body_tianji
