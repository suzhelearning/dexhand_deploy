#include "pico_body_tianji/ik/pinocchio_cpp/pinocchio_arm_ik.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial.hpp>

#include <Eigen/Cholesky>
#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace pico_body_tianji
{

namespace pin = pinocchio;

namespace
{

using ArmJacobian = Eigen::Matrix<double, 6, 7>;
using TaskVector = Eigen::Matrix<double, 6, 1>;

struct PoseEvaluation
{
  pin::SE3 current;
  TaskVector error;
  double position_error_m;
  double orientation_error_rad;
};

struct ElbowGeometry
{
  Eigen::Vector3d axis;
  Eigen::Vector3d ik_direction;
};

pin::SE3 to_pinocchio(const Eigen::Isometry3d & pose)
{
  return pin::SE3(pose.rotation(), pose.translation());
}

Eigen::Isometry3d to_eigen(const pin::SE3 & pose)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = pose.rotation();
  result.translation() = pose.translation();
  return result;
}

double orientation_distance(
  const Eigen::Matrix3d & current,
  const Eigen::Matrix3d & target)
{
  return Eigen::AngleAxisd(current.transpose() * target).angle();
}

double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

struct PinocchioArmIk::Impl
{
  explicit Impl(
    const std::string & urdf_path,
    const IkSettings & solver_settings)
  : settings(solver_settings)
  {
    if (settings.max_iterations <= 0) {
      throw std::invalid_argument("max_iterations 必须为正数");
    }
    if (
      settings.minimum_damping <= 0.0 ||
      settings.maximum_damping < settings.minimum_damping)
    {
      throw std::invalid_argument("阻尼参数无效");
    }
    if (
      settings.maximum_iteration_step_rad <= 0.0 ||
      settings.maximum_joint_step_rad <= 0.0)
    {
      throw std::invalid_argument("关节步长参数必须为正数");
    }
    if (
      settings.arm_angle_gain < 0.0 ||
      settings.arm_angle_merit_weight < 0.0 ||
      settings.nullspace_damping <= 0.0 ||
      settings.joint_center_gain < 0.0 ||
      settings.joint_center_activation_margin_rad <= 0.0 ||
      settings.joint_center_merit_weight < 0.0 ||
      settings.singularity_avoidance_gain < 0.0 ||
      settings.singularity_finite_difference_rad <= 0.0 ||
      settings.singularity_merit_weight < 0.0)
    {
      throw std::invalid_argument("零空间参数无效");
    }

    pin::urdf::buildModel(urdf_path, model);
    data = std::make_unique<pin::Data>(model);
    configure_arm(
      "L", left_joint_q_indices, left_joint_v_indices, left_base_frame,
      left_shoulder_frame, left_elbow_frame, left_tcp_frame);
    configure_arm(
      "R", right_joint_q_indices, right_joint_v_indices, right_base_frame,
      right_shoulder_frame, right_elbow_frame, right_tcp_frame);
  }

  void configure_arm(
    const std::string & suffix,
    std::array<pin::JointIndex, 7> & q_indices,
    std::array<pin::JointIndex, 7> & v_indices,
    pin::FrameIndex & base_frame,
    pin::FrameIndex & shoulder_frame,
    pin::FrameIndex & elbow_frame,
    pin::FrameIndex & tcp_frame)
  {
    for (std::size_t index = 0; index < q_indices.size(); ++index) {
      const std::string joint_name =
        "Joint" + std::to_string(index + 1) + "_" + suffix;
      const pin::JointIndex joint_id = model.getJointId(joint_name);
      if (
        joint_id == 0 ||
        joint_id >= static_cast<pin::JointIndex>(model.njoints))
      {
        throw std::runtime_error("URDF 缺少关节 " + joint_name);
      }
      if (model.joints[joint_id].nq() != 1) {
        throw std::runtime_error("机械臂关节不是单自由度：" + joint_name);
      }
      q_indices[index] = model.joints[joint_id].idx_q();
      v_indices[index] = model.joints[joint_id].idx_v();
    }

    base_frame = model.getFrameId("Base_" + suffix);
    shoulder_frame = model.getFrameId("Link1_" + suffix);
    elbow_frame = model.getFrameId("Link4_" + suffix);
    tcp_frame = model.getFrameId("TCP_Link_" + suffix);
    if (
      base_frame >= static_cast<pin::FrameIndex>(model.nframes) ||
      shoulder_frame >= static_cast<pin::FrameIndex>(model.nframes) ||
      elbow_frame >= static_cast<pin::FrameIndex>(model.nframes) ||
      tcp_frame >= static_cast<pin::FrameIndex>(model.nframes))
    {
      throw std::runtime_error("URDF 缺少 Base/肩/肘/TCP 帧：" + suffix);
    }
  }

  const std::array<pin::JointIndex, 7> & q_indices(ArmSide side) const
  {
    return side == ArmSide::kLeft ?
           left_joint_q_indices : right_joint_q_indices;
  }

  const std::array<pin::JointIndex, 7> & v_indices(ArmSide side) const
  {
    return side == ArmSide::kLeft ?
           left_joint_v_indices : right_joint_v_indices;
  }

  pin::FrameIndex base_frame(ArmSide side) const
  {
    return side == ArmSide::kLeft ? left_base_frame : right_base_frame;
  }

  pin::FrameIndex tcp_frame(ArmSide side) const
  {
    return side == ArmSide::kLeft ? left_tcp_frame : right_tcp_frame;
  }

  pin::FrameIndex shoulder_frame(ArmSide side) const
  {
    return side == ArmSide::kLeft ?
           left_shoulder_frame : right_shoulder_frame;
  }

  pin::FrameIndex elbow_frame(ArmSide side) const
  {
    return side == ArmSide::kLeft ?
           left_elbow_frame : right_elbow_frame;
  }

  Eigen::VectorXd configuration(
    ArmSide side,
    const ArmJointVector & joints_rad) const
  {
    Eigen::VectorXd result = pin::neutral(model);
    const auto & indices = q_indices(side);
    for (std::size_t index = 0; index < indices.size(); ++index) {
      result[indices[index]] =
        joints_rad[static_cast<Eigen::Index>(index)];
    }
    return result;
  }

  PoseEvaluation evaluate(
    ArmSide side,
    const ArmJointVector & joints_rad,
    const Eigen::Isometry3d & target_pose) const
  {
    const Eigen::VectorXd full_q = configuration(side, joints_rad);
    pin::forwardKinematics(model, *data, full_q);
    pin::updateFramePlacements(model, *data);
    const pin::SE3 current =
      data->oMf[base_frame(side)].actInv(data->oMf[tcp_frame(side)]);
    const pin::SE3 target = to_pinocchio(target_pose);
    const TaskVector error = pin::log6(current.actInv(target)).toVector();
    return PoseEvaluation{
      current,
      error,
      (target_pose.translation() - current.translation()).norm(),
      orientation_distance(current.rotation(), target_pose.rotation()),
    };
  }

  ArmJacobian jacobian(ArmSide side, const ArmJointVector & joints_rad) const
  {
    const Eigen::VectorXd full_q = configuration(side, joints_rad);
    pin::computeJointJacobians(model, *data, full_q);
    pin::updateFramePlacements(model, *data);
    Eigen::Matrix<double, 6, Eigen::Dynamic> full_jacobian(6, model.nv);
    full_jacobian.setZero();
    pin::getFrameJacobian(
      model,
      *data,
      tcp_frame(side),
      pin::ReferenceFrame::LOCAL,
      full_jacobian);

    ArmJacobian result;
    const auto & indices = v_indices(side);
    for (std::size_t index = 0; index < indices.size(); ++index) {
      result.col(static_cast<Eigen::Index>(index)) =
        full_jacobian.col(indices[index]);
    }
    return result;
  }

  ElbowGeometry elbow_geometry(
    ArmSide side,
    const ArmJointVector & joints_rad) const
  {
    const Eigen::VectorXd full_q = configuration(side, joints_rad);
    pin::forwardKinematics(model, *data, full_q);
    pin::updateFramePlacements(model, *data);
    const pin::SE3 & world_from_base = data->oMf[base_frame(side)];
    const Eigen::Vector3d shoulder =
      world_from_base.actInv(
      data->oMf[shoulder_frame(side)]).translation();
    const Eigen::Vector3d elbow =
      world_from_base.actInv(
      data->oMf[elbow_frame(side)]).translation();
    const Eigen::Vector3d tcp =
      world_from_base.actInv(data->oMf[tcp_frame(side)]).translation();

    const Eigen::Vector3d shoulder_to_tcp = tcp - shoulder;
    const double axis_norm = shoulder_to_tcp.norm();
    if (axis_norm < 1.0e-8) {
      throw std::runtime_error("肩—TCP 轴退化，无法计算臂角");
    }
    const Eigen::Vector3d axis = shoulder_to_tcp / axis_norm;
    const Eigen::Vector3d shoulder_to_elbow = elbow - shoulder;
    const Eigen::Vector3d elbow_offset =
      shoulder_to_elbow - shoulder_to_elbow.dot(axis) * axis;
    const double elbow_offset_norm = elbow_offset.norm();
    if (elbow_offset_norm < 1.0e-8) {
      throw std::runtime_error("机械臂接近完全伸直，肘平面退化");
    }
    const Eigen::Vector3d physical_direction =
      elbow_offset / elbow_offset_norm;
    return ElbowGeometry{axis, -physical_direction};
  }

  Eigen::Vector3d shoulder_position(
    ArmSide side,
    const ArmJointVector & joints_rad) const
  {
    const Eigen::VectorXd full_q = configuration(side, joints_rad);
    pin::forwardKinematics(model, *data, full_q);
    pin::updateFramePlacements(model, *data);
    return data->oMf[base_frame(side)].actInv(
      data->oMf[shoulder_frame(side)]).translation();
  }

  double arm_angle_error(
    ArmSide side,
    const ArmJointVector & joints_rad,
    const Eigen::Vector3d & desired_ik_direction) const
  {
    const ElbowGeometry geometry = elbow_geometry(side, joints_rad);
    Eigen::Vector3d desired =
      desired_ik_direction -
      desired_ik_direction.dot(geometry.axis) * geometry.axis;
    const double desired_norm = desired.norm();
    if (desired_norm < 1.0e-8) {
      throw std::invalid_argument("SMPL 臂角方向与肩—TCP 轴平行");
    }
    desired /= desired_norm;
    const double sine = geometry.axis.dot(
      geometry.ik_direction.cross(desired));
    const double cosine = std::clamp(
      geometry.ik_direction.dot(desired), -1.0, 1.0);
    return std::atan2(sine, cosine);
  }

  std::optional<double> try_arm_angle_error(
    ArmSide side,
    const ArmJointVector & joints_rad,
    const Eigen::Vector3d & desired_ik_direction) const
  {
    try {
      return arm_angle_error(side, joints_rad, desired_ik_direction);
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }

  std::optional<ArmJointVector> try_arm_angle_error_gradient(
    ArmSide side,
    const ArmJointVector & joints_rad,
    const Eigen::Vector3d & desired_ik_direction) const
  {
    ArmJointVector gradient;
    const ArmJointVector lower = lower_limits(side);
    const ArmJointVector upper = upper_limits(side);
    const double epsilon = settings.arm_angle_finite_difference_rad;
    for (Eigen::Index index = 0; index < gradient.size(); ++index) {
      ArmJointVector positive = joints_rad;
      ArmJointVector negative = joints_rad;
      positive[index] = std::min(upper[index], positive[index] + epsilon);
      negative[index] = std::max(lower[index], negative[index] - epsilon);
      const double denominator = positive[index] - negative[index];
      if (denominator <= std::numeric_limits<double>::epsilon()) {
        gradient[index] = 0.0;
        continue;
      }
      const std::optional<double> positive_error =
        try_arm_angle_error(side, positive, desired_ik_direction);
      const std::optional<double> negative_error =
        try_arm_angle_error(side, negative, desired_ik_direction);
      if (!positive_error.has_value() || !negative_error.has_value()) {
        return std::nullopt;
      }
      gradient[index] =
        wrap_angle(*positive_error - *negative_error) / denominator;
    }
    return gradient;
  }

  double minimum_singular_value(
    ArmSide side,
    const ArmJointVector & joints_rad) const
  {
    const ArmJacobian arm_jacobian = jacobian(side, joints_rad);
    return Eigen::JacobiSVD<ArmJacobian>(
      arm_jacobian).singularValues().minCoeff();
  }

  ArmJointVector singularity_gradient(
    ArmSide side,
    const ArmJointVector & joints_rad) const
  {
    ArmJointVector gradient;
    const ArmJointVector lower = lower_limits(side);
    const ArmJointVector upper = upper_limits(side);
    const double epsilon =
      settings.singularity_finite_difference_rad;
    for (Eigen::Index index = 0; index < gradient.size(); ++index) {
      ArmJointVector positive = joints_rad;
      ArmJointVector negative = joints_rad;
      positive[index] = std::min(upper[index], positive[index] + epsilon);
      negative[index] = std::max(lower[index], negative[index] - epsilon);
      const double denominator = positive[index] - negative[index];
      if (denominator <= std::numeric_limits<double>::epsilon()) {
        gradient[index] = 0.0;
        continue;
      }
      gradient[index] =
        (minimum_singular_value(side, positive) -
        minimum_singular_value(side, negative)) /
        denominator;
    }
    return gradient;
  }

  ArmJointVector lower_limits(ArmSide side) const
  {
    ArmJointVector result;
    const auto & indices = q_indices(side);
    for (std::size_t index = 0; index < indices.size(); ++index) {
      result[static_cast<Eigen::Index>(index)] =
        model.lowerPositionLimit[indices[index]] +
        settings.joint_limit_margin_rad;
    }
    return result;
  }

  ArmJointVector upper_limits(ArmSide side) const
  {
    ArmJointVector result;
    const auto & indices = q_indices(side);
    for (std::size_t index = 0; index < indices.size(); ++index) {
      result[static_cast<Eigen::Index>(index)] =
        model.upperPositionLimit[indices[index]] -
        settings.joint_limit_margin_rad;
    }
    return result;
  }

  ArmJointVector clamp_to_limits(
    ArmSide side,
    const ArmJointVector & joints_rad) const
  {
    return joints_rad.cwiseMax(lower_limits(side)).cwiseMin(
      upper_limits(side));
  }

  double joint_center_cost(
    ArmSide side,
    const ArmJointVector & joints_rad) const
  {
    const ArmJointVector lower = lower_limits(side);
    const ArmJointVector upper = upper_limits(side);
    const ArmJointVector half_range = 0.5 * (upper - lower);
    const ArmJointVector center = 0.5 * (upper + lower);
    const ArmJointVector normalized =
      (joints_rad - center).cwiseQuotient(half_range);
    double cost = 0.0;
    for (Eigen::Index index = 0; index < normalized.size(); ++index) {
      cost -= std::log(std::max(
        1.0e-6,
        1.0 - normalized[index] * normalized[index]));
    }
    return cost / static_cast<double>(joints_rad.size());
  }

  ArmJointVector joint_center_cost_gradient(
    ArmSide side,
    const ArmJointVector & joints_rad) const
  {
    const ArmJointVector lower = lower_limits(side);
    const ArmJointVector upper = upper_limits(side);
    const ArmJointVector half_range = 0.5 * (upper - lower);
    const ArmJointVector center = 0.5 * (upper + lower);
    const ArmJointVector normalized =
      (joints_rad - center).cwiseQuotient(half_range);
    ArmJointVector gradient;
    for (Eigen::Index index = 0; index < normalized.size(); ++index) {
      const double denominator = std::max(
        1.0e-6,
        1.0 - normalized[index] * normalized[index]);
      gradient[index] =
        2.0 * normalized[index] /
        (denominator * half_range[index] *
        static_cast<double>(joints_rad.size()));
    }
    return gradient;
  }

  double adaptive_damping(double minimum_singular_value) const
  {
    if (minimum_singular_value >= settings.singular_value_threshold) {
      return settings.minimum_damping;
    }
    const double ratio = std::clamp(
      1.0 - minimum_singular_value /
      settings.singular_value_threshold,
      0.0,
      1.0);
    return settings.minimum_damping +
           (settings.maximum_damping - settings.minimum_damping) *
           ratio * ratio;
  }

  double minimum_limit_margin(
    ArmSide side,
    const ArmJointVector & joints_rad) const
  {
    const ArmJointVector lower = lower_limits(side);
    const ArmJointVector upper = upper_limits(side);
    return std::min(
      (joints_rad - lower).minCoeff(),
      (upper - joints_rad).minCoeff());
  }

  IkSettings settings;
  pin::Model model;
  std::unique_ptr<pin::Data> data;
  std::array<pin::JointIndex, 7> left_joint_q_indices{};
  std::array<pin::JointIndex, 7> right_joint_q_indices{};
  std::array<pin::JointIndex, 7> left_joint_v_indices{};
  std::array<pin::JointIndex, 7> right_joint_v_indices{};
  pin::FrameIndex left_base_frame{};
  pin::FrameIndex right_base_frame{};
  pin::FrameIndex left_shoulder_frame{};
  pin::FrameIndex right_shoulder_frame{};
  pin::FrameIndex left_elbow_frame{};
  pin::FrameIndex right_elbow_frame{};
  pin::FrameIndex left_tcp_frame{};
  pin::FrameIndex right_tcp_frame{};
};

PinocchioArmIk::PinocchioArmIk(
  const std::string & urdf_path,
  const IkSettings & settings)
: impl_(std::make_unique<Impl>(urdf_path, settings))
{
}

PinocchioArmIk::~PinocchioArmIk() = default;
PinocchioArmIk::PinocchioArmIk(PinocchioArmIk &&) noexcept = default;
PinocchioArmIk & PinocchioArmIk::operator=(PinocchioArmIk &&) noexcept =
  default;

Eigen::Isometry3d PinocchioArmIk::forward(
  ArmSide side,
  const ArmJointVector & joints_rad) const
{
  return to_eigen(
    impl_->evaluate(side, joints_rad, Eigen::Isometry3d::Identity()).current);
}

Eigen::Vector3d PinocchioArmIk::elbow_ik_direction(
  ArmSide side,
  const ArmJointVector & joints_rad) const
{
  return impl_->elbow_geometry(side, joints_rad).ik_direction;
}

IkResult PinocchioArmIk::solve(
  ArmSide side,
  const Eigen::Isometry3d & target_pose,
  const ArmJointVector & current_joints_rad,
  const Eigen::Vector3d & smpl_ik_direction) const
{
  const IkSettings & settings = impl_->settings;
  bool arm_angle_requested =
    settings.arm_angle_gain > 0.0 &&
    smpl_ik_direction.allFinite() &&
    smpl_ik_direction.norm() > 1.0e-8;
  if (arm_angle_requested) {
    const Eigen::Vector3d shoulder =
      impl_->shoulder_position(side, current_joints_rad);
    const Eigen::Vector3d target_axis =
      target_pose.translation() - shoulder;
    if (target_axis.norm() < 1.0e-8) {
      arm_angle_requested = false;
    } else {
      const Eigen::Vector3d normalized_axis = target_axis.normalized();
      const Eigen::Vector3d projected_direction =
        smpl_ik_direction -
        smpl_ik_direction.dot(normalized_axis) * normalized_axis;
      arm_angle_requested =
        projected_direction.norm() >
        1.0e-4 * smpl_ik_direction.norm();
    }
  }
  ArmJointVector working = impl_->clamp_to_limits(
    side, current_joints_rad);
  bool moved = false;
  bool stalled = false;
  double minimum_singular_value = 0.0;
  double damping = settings.maximum_damping;

  for (int iteration = 0; iteration < settings.max_iterations; ++iteration) {
    const PoseEvaluation evaluation =
      impl_->evaluate(side, working, target_pose);
    const std::optional<double> arm_angle_error =
      arm_angle_requested ?
      impl_->try_arm_angle_error(
      side, working, smpl_ik_direction) :
      std::nullopt;
    const bool arm_angle_active = arm_angle_error.has_value();
    const ArmJacobian jacobian = impl_->jacobian(side, working);
    const Eigen::JacobiSVD<ArmJacobian> svd(jacobian);
    minimum_singular_value = svd.singularValues().minCoeff();
    const bool singularity_objective_active =
      settings.singularity_avoidance_gain > 0.0 &&
      minimum_singular_value < settings.singular_value_threshold;
    const bool joint_center_objective_active =
      settings.joint_center_gain > 0.0 &&
      impl_->minimum_limit_margin(side, working) <
      settings.joint_center_activation_margin_rad;
    if (
      evaluation.position_error_m <= settings.position_tolerance_m &&
      evaluation.orientation_error_rad <=
      settings.orientation_tolerance_rad &&
      (!arm_angle_active ||
      std::abs(*arm_angle_error) <= settings.arm_angle_tolerance_rad) &&
      !singularity_objective_active &&
      !joint_center_objective_active)
    {
      break;
    }

    damping = impl_->adaptive_damping(minimum_singular_value);

    Eigen::Matrix<double, 6, 6> normal =
      jacobian * jacobian.transpose();
    normal.diagonal().array() += damping * damping;
    const Eigen::Matrix<double, 7, 6> damped_pseudoinverse =
      jacobian.transpose() *
      normal.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
    ArmJointVector increment =
      damped_pseudoinverse * evaluation.error;
    const Eigen::Matrix<double, 7, 7> nullspace =
      Eigen::Matrix<double, 7, 7>::Identity() -
      damped_pseudoinverse * jacobian;

    if (arm_angle_active) {
      const std::optional<ArmJointVector> arm_gradient =
        impl_->try_arm_angle_error_gradient(
        side, working, smpl_ik_direction);
      if (arm_gradient.has_value()) {
        const ArmJointVector nullspace_gradient =
          nullspace * *arm_gradient;
        const double denominator =
          arm_gradient->dot(nullspace_gradient) +
          settings.nullspace_damping * settings.nullspace_damping;
        if (denominator > 1.0e-12) {
          increment -=
            settings.arm_angle_gain * *arm_angle_error /
            denominator * nullspace_gradient;
        }
      }
    }

    if (singularity_objective_active) {
      const ArmJointVector singularity_gradient =
        impl_->singularity_gradient(side, working);
      const ArmJointVector nullspace_gradient =
        nullspace * singularity_gradient;
      const double denominator =
        singularity_gradient.dot(nullspace_gradient) +
        settings.nullspace_damping * settings.nullspace_damping;
      if (denominator > 1.0e-12) {
        const double singularity_gap =
          settings.singular_value_threshold -
          minimum_singular_value;
        increment +=
          settings.singularity_avoidance_gain * singularity_gap /
          denominator * nullspace_gradient;
      }
    }

    if (joint_center_objective_active) {
      const ArmJointVector center_gradient =
        impl_->joint_center_cost_gradient(side, working);
      increment -=
        settings.joint_center_gain * nullspace * center_gradient;
    }

    const double maximum_increment = increment.cwiseAbs().maxCoeff();
    if (maximum_increment > settings.maximum_iteration_step_rad) {
      increment *= settings.maximum_iteration_step_rad / maximum_increment;
    }

    bool accepted_iteration = false;
    double fraction = 1.0;
    const double current_cost =
      evaluation.error.squaredNorm() +
      settings.arm_angle_merit_weight *
      std::pow(arm_angle_error.value_or(0.0), 2) +
      (settings.joint_center_gain > 0.0 ?
      settings.joint_center_merit_weight : 0.0) *
      impl_->joint_center_cost(side, working) +
      (settings.singularity_avoidance_gain > 0.0 ?
      settings.singularity_merit_weight : 0.0) *
      std::pow(
      std::max(
        0.0,
        settings.singular_value_threshold -
        minimum_singular_value),
      2);
    for (int attempt = 0; attempt < 8; ++attempt) {
      const ArmJointVector candidate = impl_->clamp_to_limits(
        side, working + fraction * increment);
      const PoseEvaluation candidate_evaluation =
        impl_->evaluate(side, candidate, target_pose);
      const std::optional<double> candidate_arm_angle_error =
        arm_angle_requested ?
        impl_->try_arm_angle_error(
        side, candidate, smpl_ik_direction) :
        std::nullopt;
      const double candidate_minimum_singular_value =
        impl_->minimum_singular_value(side, candidate);
      const double candidate_cost =
        candidate_evaluation.error.squaredNorm() +
        settings.arm_angle_merit_weight *
        std::pow(candidate_arm_angle_error.value_or(0.0), 2) +
        (settings.joint_center_gain > 0.0 ?
        settings.joint_center_merit_weight : 0.0) *
        impl_->joint_center_cost(side, candidate) +
        (settings.singularity_avoidance_gain > 0.0 ?
        settings.singularity_merit_weight : 0.0) *
        std::pow(
        std::max(
          0.0,
          settings.singular_value_threshold -
          candidate_minimum_singular_value),
        2);
      if (
        candidate_cost < current_cost - 1.0e-14)
      {
        working = candidate;
        moved = true;
        accepted_iteration = true;
        break;
      }
      fraction *= 0.5;
    }
    if (!accepted_iteration) {
      stalled = true;
      break;
    }
  }

  ArmJointVector bounded = working;
  ArmJointVector requested_delta = working - current_joints_rad;
  const double requested_maximum_step =
    requested_delta.cwiseAbs().maxCoeff();
  bool joint_step_limited = false;
  if (requested_maximum_step > settings.maximum_joint_step_rad) {
    requested_delta *=
      settings.maximum_joint_step_rad / requested_maximum_step;
    bounded = current_joints_rad + requested_delta;
    joint_step_limited = true;
  }
  bounded = impl_->clamp_to_limits(side, bounded);

  const PoseEvaluation final_evaluation =
    impl_->evaluate(side, bounded, target_pose);
  const double final_minimum_singular_value =
    impl_->minimum_singular_value(side, bounded);
  damping = impl_->adaptive_damping(final_minimum_singular_value);
  const std::optional<double> final_arm_angle_error =
    arm_angle_requested ?
    impl_->try_arm_angle_error(
    side, bounded, smpl_ik_direction) :
    std::nullopt;
  const bool pose_converged =
    final_evaluation.position_error_m <= settings.position_tolerance_m &&
    final_evaluation.orientation_error_rad <=
    settings.orientation_tolerance_rad;
  const bool arm_angle_converged =
    !final_arm_angle_error.has_value() ||
    std::abs(*final_arm_angle_error) <=
    settings.arm_angle_tolerance_rad;
  IkResult result;
  result.joints_rad = bounded;
  result.achieved_pose = to_eigen(final_evaluation.current);
  // 臂角、关节居中和奇异性规避均为零空间软目标。软目标无法继续下降时，
  // 不能把已经满足的末端主任务标成“不可达”，否则上层会误报限位卡死。
  result.accepted = moved || pose_converged;
  result.converged = pose_converged;
  result.saturated = stalled && !pose_converged;
  result.joint_step_limited = joint_step_limited;
  result.requested_maximum_joint_step_rad = requested_maximum_step;
  result.singularity_active =
    final_minimum_singular_value < settings.singular_value_threshold;
  result.position_error_m = final_evaluation.position_error_m;
  result.orientation_error_rad = final_evaluation.orientation_error_rad;
  result.minimum_singular_value = final_minimum_singular_value;
  result.damping = damping;
  result.arm_angle_error_rad = final_arm_angle_error.value_or(0.0);
  result.minimum_limit_margin_rad =
    impl_->minimum_limit_margin(side, bounded);
  result.maximum_joint_step_rad =
    (bounded - current_joints_rad).cwiseAbs().maxCoeff();
  if (result.converged) {
    result.status = arm_angle_converged ?
      "converged" : "converged_secondary_residual";
  } else if (result.saturated) {
    result.status = "stalled";
  } else {
    result.status = "tracking";
  }
  return result;
}

}  // namespace pico_body_tianji
