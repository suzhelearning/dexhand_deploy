#include "pico_body_tianji/ik/pinocchio_qp/pinocchio_qp_arm_ik.hpp"

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
#include <vector>

namespace pico_body_tianji
{

namespace pin = pinocchio;

namespace
{

using ArmHessian = Eigen::Matrix<double, 7, 7>;
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

struct BoxQpResult
{
  ArmJointVector solution{ArmJointVector::Zero()};
  bool converged{false};
  int iterations{0};
  int active_constraints{0};
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

Eigen::Vector3d clamp_norm(
  const Eigen::Vector3d & value,
  double maximum_norm)
{
  const double norm = value.norm();
  if (norm <= maximum_norm || norm <= std::numeric_limits<double>::epsilon()) {
    return value;
  }
  return maximum_norm / norm * value;
}

BoxQpResult solve_box_qp(
  const ArmHessian & hessian,
  const ArmJointVector & linear,
  const ArmJointVector & lower,
  const ArmJointVector & upper,
  const ArmJointVector & warm_start,
  int maximum_iterations,
  double tolerance)
{
  if ((lower.array() > upper.array()).any()) {
    throw std::runtime_error("QP 关节速度上下界冲突");
  }

  BoxQpResult result;
  result.solution = warm_start.cwiseMax(lower).cwiseMin(upper);
  // -1/1 分别表示下/上界，2 表示上下界重合，0 表示自由变量。
  std::array<int, 7> active{};
  for (Eigen::Index index = 0; index < result.solution.size(); ++index) {
    if (upper[index] - lower[index] <= tolerance) {
      result.solution[index] = 0.5 * (lower[index] + upper[index]);
      active[static_cast<std::size_t>(index)] = 2;
    } else if (result.solution[index] <= lower[index] + tolerance) {
      result.solution[index] = lower[index];
      active[static_cast<std::size_t>(index)] = -1;
    } else if (result.solution[index] >= upper[index] - tolerance) {
      result.solution[index] = upper[index];
      active[static_cast<std::size_t>(index)] = 1;
    }
  }

  for (int iteration = 0; iteration < maximum_iterations; ++iteration) {
    result.iterations = iteration + 1;
    const ArmJointVector gradient = hessian * result.solution + linear;
    std::vector<Eigen::Index> free_indices;
    free_indices.reserve(7);
    for (Eigen::Index index = 0; index < result.solution.size(); ++index) {
      if (active[static_cast<std::size_t>(index)] == 0) {
        free_indices.push_back(index);
      }
    }

    ArmJointVector direction = ArmJointVector::Zero();
    if (!free_indices.empty()) {
      const Eigen::Index free_count =
        static_cast<Eigen::Index>(free_indices.size());
      Eigen::MatrixXd reduced_hessian(free_count, free_count);
      Eigen::VectorXd reduced_gradient(free_count);
      for (Eigen::Index row = 0; row < free_count; ++row) {
        reduced_gradient[row] = gradient[free_indices[row]];
        for (Eigen::Index column = 0; column < free_count; ++column) {
          reduced_hessian(row, column) =
            hessian(free_indices[row], free_indices[column]);
        }
      }
      const Eigen::LDLT<Eigen::MatrixXd> factorization(reduced_hessian);
      if (factorization.info() != Eigen::Success) {
        return result;
      }
      const Eigen::VectorXd reduced_direction =
        factorization.solve(-reduced_gradient);
      if (
        factorization.info() != Eigen::Success ||
        !reduced_direction.allFinite())
      {
        return result;
      }
      for (Eigen::Index index = 0; index < free_count; ++index) {
        direction[free_indices[index]] = reduced_direction[index];
      }
    }

    if (direction.cwiseAbs().maxCoeff() <= tolerance) {
      Eigen::Index release_index = -1;
      double maximum_violation = tolerance;
      for (Eigen::Index index = 0; index < result.solution.size(); ++index) {
        const int state = active[static_cast<std::size_t>(index)];
        double violation = 0.0;
        if (state == -1) {
          violation = -gradient[index];
        } else if (state == 1) {
          violation = gradient[index];
        }
        if (violation > maximum_violation) {
          maximum_violation = violation;
          release_index = index;
        }
      }
      if (release_index < 0) {
        result.converged = true;
        break;
      }
      active[static_cast<std::size_t>(release_index)] = 0;
      continue;
    }

    double step = 1.0;
    for (const Eigen::Index index : free_indices) {
      if (direction[index] > tolerance) {
        step = std::min(
          step,
          (upper[index] - result.solution[index]) / direction[index]);
      } else if (direction[index] < -tolerance) {
        step = std::min(
          step,
          (lower[index] - result.solution[index]) / direction[index]);
      }
    }
    step = std::clamp(step, 0.0, 1.0);
    result.solution += step * direction;
    result.solution = result.solution.cwiseMax(lower).cwiseMin(upper);

    if (step < 1.0 - tolerance) {
      for (const Eigen::Index index : free_indices) {
        if (result.solution[index] <= lower[index] + tolerance) {
          result.solution[index] = lower[index];
          active[static_cast<std::size_t>(index)] = -1;
        } else if (result.solution[index] >= upper[index] - tolerance) {
          result.solution[index] = upper[index];
          active[static_cast<std::size_t>(index)] = 1;
        }
      }
    }
  }

  result.active_constraints = static_cast<int>(std::count_if(
      active.begin(), active.end(), [](int state) {return state != 0;}));
  return result;
}

void add_vector_least_squares(
  ArmHessian & hessian,
  ArmJointVector & linear,
  const Eigen::Matrix<double, 3, 7> & matrix,
  const Eigen::Vector3d & target,
  double weight)
{
  hessian.noalias() += 2.0 * weight * matrix.transpose() * matrix;
  linear.noalias() -= 2.0 * weight * matrix.transpose() * target;
}

void add_diagonal_tracking_cost(
  ArmHessian & hessian,
  ArmJointVector & linear,
  const ArmJointVector & target,
  const ArmJointVector & scale,
  double weight)
{
  for (Eigen::Index index = 0; index < target.size(); ++index) {
    const double coefficient = weight / (scale[index] * scale[index]);
    hessian(index, index) += 2.0 * coefficient;
    linear[index] -= 2.0 * coefficient * target[index];
  }
}

}  // namespace

struct PinocchioQpArmIk::Impl
{
  explicit Impl(
    const std::string & urdf_path,
    const IkSettings & solver_settings)
  : settings(solver_settings)
  {
    validate_settings();
    pin::urdf::buildModel(urdf_path, model);
    data = std::make_unique<pin::Data>(model);
    configure_arm(
      "L", left_joint_q_indices, left_joint_v_indices, left_base_frame,
      left_shoulder_frame, left_elbow_frame, left_tcp_frame);
    configure_arm(
      "R", right_joint_q_indices, right_joint_v_indices, right_base_frame,
      right_shoulder_frame, right_elbow_frame, right_tcp_frame);
    settings.qp_left_nominal_rad = clamp_to_limits(
      ArmSide::kLeft, settings.qp_left_nominal_rad);
    settings.qp_right_nominal_rad = clamp_to_limits(
      ArmSide::kRight, settings.qp_right_nominal_rad);
  }

  void validate_settings() const
  {
    const bool invalid_positive_parameter =
      settings.control_period_s <= 0.0 ||
      settings.qp_position_time_constant_s <= 0.0 ||
      settings.qp_orientation_time_constant_s <= 0.0 ||
      settings.qp_max_linear_speed_m_s <= 0.0 ||
      settings.qp_max_angular_speed_rad_s <= 0.0 ||
      settings.qp_posture_time_constant_s <= 0.0 ||
      settings.maximum_joint_step_rad <= 0.0 ||
      settings.joint_limit_margin_rad < 0.0 ||
      settings.qp_joint_limit_activation_margin_rad <= 0.0 ||
      settings.qp_joint_limit_velocity_damper_gain <= 0.0 ||
      settings.singular_value_threshold <= 0.0 ||
      settings.qp_singularity_critical_threshold < 0.0 ||
      settings.qp_singularity_critical_threshold >=
      settings.singular_value_threshold ||
      settings.qp_singularity_escape_speed_rad_s < 0.0 ||
      settings.qp_max_active_set_iterations <= 0 ||
      settings.qp_active_set_tolerance <= 0.0;
    const bool invalid_weight =
      settings.qp_position_weight <= 0.0 ||
      settings.qp_orientation_weight < 0.0 ||
      settings.qp_velocity_regularization_weight <= 0.0 ||
      settings.qp_continuity_weight < 0.0 ||
      settings.qp_posture_weight < 0.0 ||
      settings.qp_singularity_orientation_scale < 0.0 ||
      settings.qp_singularity_orientation_scale > 1.0 ||
      settings.qp_singularity_posture_multiplier < 1.0 ||
      settings.qp_singularity_velocity_multiplier < 1.0 ||
      settings.qp_singularity_escape_weight < 0.0;
    if (
      invalid_positive_parameter || invalid_weight ||
      !settings.qp_joint_velocity_limits_rad_s.allFinite() ||
      (settings.qp_joint_velocity_limits_rad_s.array() <= 0.0).any() ||
      !settings.qp_left_nominal_rad.allFinite() ||
      !settings.qp_right_nominal_rad.allFinite())
    {
      throw std::invalid_argument("pinocchio_qp 参数无效");
    }
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

  std::size_t side_index(ArmSide side) const
  {
    return side == ArmSide::kLeft ? 0U : 1U;
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

  pin::FrameIndex shoulder_frame(ArmSide side) const
  {
    return side == ArmSide::kLeft ?
           left_shoulder_frame : right_shoulder_frame;
  }

  pin::FrameIndex elbow_frame(ArmSide side) const
  {
    return side == ArmSide::kLeft ? left_elbow_frame : right_elbow_frame;
  }

  pin::FrameIndex tcp_frame(ArmSide side) const
  {
    return side == ArmSide::kLeft ? left_tcp_frame : right_tcp_frame;
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
    const Eigen::Vector3d shoulder = world_from_base.actInv(
      data->oMf[shoulder_frame(side)]).translation();
    const Eigen::Vector3d elbow = world_from_base.actInv(
      data->oMf[elbow_frame(side)]).translation();
    const Eigen::Vector3d tcp = world_from_base.actInv(
      data->oMf[tcp_frame(side)]).translation();
    const Eigen::Vector3d shoulder_to_tcp = tcp - shoulder;
    if (shoulder_to_tcp.norm() < 1.0e-8) {
      throw std::runtime_error("肩—TCP 轴退化，无法计算臂角");
    }
    const Eigen::Vector3d axis = shoulder_to_tcp.normalized();
    const Eigen::Vector3d shoulder_to_elbow = elbow - shoulder;
    const Eigen::Vector3d offset =
      shoulder_to_elbow - shoulder_to_elbow.dot(axis) * axis;
    if (offset.norm() < 1.0e-8) {
      throw std::runtime_error("机械臂接近完全伸直，肘平面退化");
    }
    return ElbowGeometry{axis, -offset.normalized()};
  }

  std::optional<double> arm_angle_error(
    ArmSide side,
    const ArmJointVector & joints_rad,
    const Eigen::Vector3d & desired_ik_direction) const
  {
    try {
      const ElbowGeometry geometry = elbow_geometry(side, joints_rad);
      Eigen::Vector3d desired = desired_ik_direction -
        desired_ik_direction.dot(geometry.axis) * geometry.axis;
      if (desired.norm() < 1.0e-8) {
        return std::nullopt;
      }
      desired.normalize();
      const double sine = geometry.axis.dot(
        geometry.ik_direction.cross(desired));
      const double cosine = std::clamp(
        geometry.ik_direction.dot(desired), -1.0, 1.0);
      return std::atan2(sine, cosine);
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }

  std::optional<ArmJointVector> arm_angle_gradient(
    ArmSide side,
    const ArmJointVector & joints_rad,
    const Eigen::Vector3d & desired_ik_direction) const
  {
    ArmJointVector gradient;
    const ArmJointVector lower = lower_limits(side);
    const ArmJointVector upper = upper_limits(side);
    for (Eigen::Index index = 0; index < gradient.size(); ++index) {
      ArmJointVector positive = joints_rad;
      ArmJointVector negative = joints_rad;
      positive[index] = std::min(
        upper[index], positive[index] + settings.arm_angle_finite_difference_rad);
      negative[index] = std::max(
        lower[index], negative[index] - settings.arm_angle_finite_difference_rad);
      const double denominator = positive[index] - negative[index];
      if (denominator <= std::numeric_limits<double>::epsilon()) {
        gradient[index] = 0.0;
        continue;
      }
      const auto positive_error = arm_angle_error(
        side, positive, desired_ik_direction);
      const auto negative_error = arm_angle_error(
        side, negative, desired_ik_direction);
      if (!positive_error.has_value() || !negative_error.has_value()) {
        return std::nullopt;
      }
      gradient[index] = wrap_angle(*positive_error - *negative_error) /
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

  const ArmJointVector & nominal(ArmSide side) const
  {
    return side == ArmSide::kLeft ?
           settings.qp_left_nominal_rad : settings.qp_right_nominal_rad;
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

  double minimum_singular_value(
    ArmSide side,
    const ArmJointVector & joints_rad) const
  {
    return Eigen::JacobiSVD<ArmJacobian>(
      jacobian(side, joints_rad)).singularValues().minCoeff();
  }

  ArmJointVector singularity_gradient(
    ArmSide side,
    const ArmJointVector & joints_rad) const
  {
    ArmJointVector gradient;
    const ArmJointVector lower = lower_limits(side);
    const ArmJointVector upper = upper_limits(side);
    for (Eigen::Index index = 0; index < gradient.size(); ++index) {
      ArmJointVector positive = joints_rad;
      ArmJointVector negative = joints_rad;
      positive[index] = std::min(
        upper[index],
        positive[index] + settings.singularity_finite_difference_rad);
      negative[index] = std::max(
        lower[index],
        negative[index] - settings.singularity_finite_difference_rad);
      const double denominator = positive[index] - negative[index];
      if (denominator <= std::numeric_limits<double>::epsilon()) {
        gradient[index] = 0.0;
      } else {
        gradient[index] =
          (minimum_singular_value(side, positive) -
          minimum_singular_value(side, negative)) / denominator;
      }
    }
    return gradient;
  }

  void velocity_bounds(
    ArmSide side,
    const ArmJointVector & joints_rad,
    ArmJointVector & lower_velocity,
    ArmJointVector & upper_velocity) const
  {
    const ArmJointVector lower = lower_limits(side);
    const ArmJointVector upper = upper_limits(side);
    const double dt = settings.control_period_s;
    const double public_step_speed = settings.maximum_joint_step_rad / dt;
    const ArmJointVector limits =
      settings.qp_joint_velocity_limits_rad_s.cwiseMin(
      ArmJointVector::Constant(public_step_speed));
    lower_velocity = -limits;
    upper_velocity = limits;
    for (Eigen::Index index = 0; index < joints_rad.size(); ++index) {
      if (joints_rad[index] < lower[index]) {
        lower_velocity[index] = 0.0;
        continue;
      }
      if (joints_rad[index] > upper[index]) {
        upper_velocity[index] = 0.0;
        continue;
      }
      lower_velocity[index] = std::max(
        lower_velocity[index],
        (lower[index] - joints_rad[index]) / dt);
      upper_velocity[index] = std::min(
        upper_velocity[index],
        (upper[index] - joints_rad[index]) / dt);
      const double lower_distance = joints_rad[index] - lower[index];
      const double upper_distance = upper[index] - joints_rad[index];
      if (lower_distance < settings.qp_joint_limit_activation_margin_rad) {
        lower_velocity[index] = std::max(
          lower_velocity[index],
          -settings.qp_joint_limit_velocity_damper_gain * lower_distance);
      }
      if (upper_distance < settings.qp_joint_limit_activation_margin_rad) {
        upper_velocity[index] = std::min(
          upper_velocity[index],
          settings.qp_joint_limit_velocity_damper_gain * upper_distance);
      }
    }
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
  mutable std::array<ArmJointVector, 2> previous_velocity{
    ArmJointVector::Zero(), ArmJointVector::Zero()};
  mutable std::array<ArmJointVector, 2> expected_joints{
    ArmJointVector::Zero(), ArmJointVector::Zero()};
  mutable std::array<bool, 2> warm_start_valid{false, false};
};

PinocchioQpArmIk::PinocchioQpArmIk(
  const std::string & urdf_path,
  const IkSettings & settings)
: impl_(std::make_unique<Impl>(urdf_path, settings))
{
}

PinocchioQpArmIk::~PinocchioQpArmIk() = default;
PinocchioQpArmIk::PinocchioQpArmIk(PinocchioQpArmIk &&) noexcept = default;
PinocchioQpArmIk & PinocchioQpArmIk::operator=(
  PinocchioQpArmIk &&) noexcept = default;

Eigen::Isometry3d PinocchioQpArmIk::forward(
  ArmSide side,
  const ArmJointVector & joints_rad) const
{
  return to_eigen(
    impl_->evaluate(side, joints_rad, Eigen::Isometry3d::Identity()).current);
}

IkResult PinocchioQpArmIk::solve(
  ArmSide side,
  const Eigen::Isometry3d & target_pose,
  const ArmJointVector & current_joints_rad,
  const Eigen::Vector3d & elbow_ik_direction) const
{
  if (!target_pose.matrix().allFinite()) {
    throw std::invalid_argument("QP IK 目标位姿含有非有限值");
  }
  if (!current_joints_rad.allFinite()) {
    throw std::invalid_argument("QP IK 当前关节含有非有限值");
  }

  const IkSettings & settings = impl_->settings;
  const std::size_t state_index = impl_->side_index(side);
  if (
    impl_->warm_start_valid[state_index] &&
    (current_joints_rad - impl_->expected_joints[state_index])
    .cwiseAbs().maxCoeff() > 3.0 * settings.maximum_joint_step_rad)
  {
    impl_->previous_velocity[state_index].setZero();
    impl_->warm_start_valid[state_index] = false;
  }

  const PoseEvaluation evaluation =
    impl_->evaluate(side, current_joints_rad, target_pose);
  const ArmJacobian jacobian = impl_->jacobian(side, current_joints_rad);
  const double minimum_singular_value = Eigen::JacobiSVD<ArmJacobian>(
    jacobian).singularValues().minCoeff();
  const double singularity_activation = std::clamp(
    (settings.singular_value_threshold - minimum_singular_value) /
    (settings.singular_value_threshold -
    settings.qp_singularity_critical_threshold),
    0.0,
    1.0);

  const Eigen::Vector3d desired_linear = clamp_norm(
    evaluation.error.template head<3>() /
    settings.qp_position_time_constant_s,
    settings.qp_max_linear_speed_m_s);
  const Eigen::Vector3d desired_angular = clamp_norm(
    evaluation.error.template tail<3>() /
    settings.qp_orientation_time_constant_s,
    settings.qp_max_angular_speed_rad_s);

  const double orientation_scale =
    (1.0 - singularity_activation) +
    singularity_activation * settings.qp_singularity_orientation_scale;
  const double posture_multiplier =
    (1.0 - singularity_activation) + singularity_activation *
    settings.qp_singularity_posture_multiplier;
  const double velocity_multiplier =
    (1.0 - singularity_activation) + singularity_activation *
    settings.qp_singularity_velocity_multiplier;

  ArmHessian hessian = ArmHessian::Zero();
  ArmJointVector linear = ArmJointVector::Zero();
  add_vector_least_squares(
    hessian,
    linear,
    jacobian.template topRows<3>(),
    desired_linear,
    settings.qp_position_weight /
    std::pow(settings.qp_max_linear_speed_m_s, 2));
  add_vector_least_squares(
    hessian,
    linear,
    jacobian.template bottomRows<3>(),
    desired_angular,
    settings.qp_orientation_weight * orientation_scale /
    std::pow(settings.qp_max_angular_speed_rad_s, 2));

  const ArmJointVector velocity_scale =
    settings.qp_joint_velocity_limits_rad_s;
  add_diagonal_tracking_cost(
    hessian,
    linear,
    ArmJointVector::Zero(),
    velocity_scale,
    settings.qp_velocity_regularization_weight * velocity_multiplier);

  const ArmJointVector previous_velocity =
    impl_->warm_start_valid[state_index] ?
    impl_->previous_velocity[state_index] : ArmJointVector::Zero();
  add_diagonal_tracking_cost(
    hessian,
    linear,
    previous_velocity,
    velocity_scale,
    settings.qp_continuity_weight);

  ArmJointVector posture_velocity =
    (impl_->nominal(side) - current_joints_rad) /
    settings.qp_posture_time_constant_s;
  posture_velocity = posture_velocity.cwiseMax(-velocity_scale).cwiseMin(
    velocity_scale);
  add_diagonal_tracking_cost(
    hessian,
    linear,
    posture_velocity,
    velocity_scale,
    settings.qp_posture_weight * posture_multiplier);

  if (
    singularity_activation > 0.0 &&
    settings.qp_singularity_escape_weight > 0.0)
  {
    const ArmJointVector gradient = impl_->singularity_gradient(
      side, current_joints_rad);
    if (gradient.allFinite() && gradient.norm() > 1.0e-10) {
      const ArmJointVector escape_velocity =
        singularity_activation * settings.qp_singularity_escape_speed_rad_s *
        gradient.normalized();
      add_diagonal_tracking_cost(
        hessian,
        linear,
        escape_velocity,
        velocity_scale,
        settings.qp_singularity_escape_weight * singularity_activation);
    }
  }

  std::optional<double> arm_angle_error;
  const bool arm_angle_requested =
    settings.arm_angle_gain > 0.0 &&
    elbow_ik_direction.allFinite() && elbow_ik_direction.norm() > 1.0e-8;
  if (arm_angle_requested) {
    const Eigen::Vector3d normalized_direction = elbow_ik_direction.normalized();
    arm_angle_error = impl_->arm_angle_error(
      side, current_joints_rad, normalized_direction);
    const auto gradient = impl_->arm_angle_gradient(
      side, current_joints_rad, normalized_direction);
    if (arm_angle_error.has_value() && gradient.has_value()) {
      const double desired_rate = std::clamp(
        -settings.arm_angle_gain * *arm_angle_error,
        -settings.qp_max_angular_speed_rad_s,
        settings.qp_max_angular_speed_rad_s);
      const double weight = settings.arm_angle_merit_weight /
        std::pow(settings.qp_max_angular_speed_rad_s, 2);
      hessian.noalias() +=
        2.0 * weight * *gradient * gradient->transpose();
      linear.noalias() -= 2.0 * weight * desired_rate * *gradient;
    }
  }

  // 保证 Hessian 严格正定，避免极端权重组合导致约化系统数值退化。
  hessian.diagonal().array() += 1.0e-12;
  hessian = 0.5 * (hessian + hessian.transpose());

  ArmJointVector lower_velocity;
  ArmJointVector upper_velocity;
  impl_->velocity_bounds(
    side, current_joints_rad, lower_velocity, upper_velocity);
  const BoxQpResult qp_result = solve_box_qp(
    hessian,
    linear,
    lower_velocity,
    upper_velocity,
    previous_velocity,
    settings.qp_max_active_set_iterations,
    settings.qp_active_set_tolerance);

  IkResult result;
  result.joints_rad = current_joints_rad;
  result.achieved_pose = to_eigen(evaluation.current);
  result.minimum_singular_value = minimum_singular_value;
  result.singularity_active =
    minimum_singular_value < settings.singular_value_threshold;
  result.solver_iterations = qp_result.iterations;
  result.active_joint_constraints = qp_result.active_constraints;
  result.arm_angle_error_rad = arm_angle_error.value_or(0.0);
  if (!qp_result.converged || !qp_result.solution.allFinite()) {
    result.saturated = true;
    result.position_error_m = evaluation.position_error_m;
    result.orientation_error_rad = evaluation.orientation_error_rad;
    result.minimum_limit_margin_rad = impl_->minimum_limit_margin(
      side, current_joints_rad);
    result.status = "qp_failed";
    impl_->previous_velocity[state_index].setZero();
    impl_->expected_joints[state_index] = current_joints_rad;
    impl_->warm_start_valid[state_index] = false;
    return result;
  }

  ArmJointVector candidate = current_joints_rad +
    settings.control_period_s * qp_result.solution;
  const ArmJointVector actual_delta = candidate - current_joints_rad;
  const double maximum_step = actual_delta.cwiseAbs().maxCoeff();
  if (maximum_step > settings.maximum_joint_step_rad + 1.0e-10) {
    result.saturated = true;
    result.status = "qp_contract_violation";
    return result;
  }

  const PoseEvaluation achieved = impl_->evaluate(side, candidate, target_pose);
  const double achieved_minimum_singular_value =
    impl_->minimum_singular_value(side, candidate);
  std::optional<double> achieved_arm_angle_error;
  if (arm_angle_requested) {
    achieved_arm_angle_error = impl_->arm_angle_error(
      side, candidate, elbow_ik_direction.normalized());
  }
  const Eigen::Vector3d linear_residual =
    jacobian.template topRows<3>() * qp_result.solution - desired_linear;
  const Eigen::Vector3d angular_residual =
    jacobian.template bottomRows<3>() * qp_result.solution - desired_angular;
  result.joints_rad = candidate;
  result.achieved_pose = to_eigen(achieved.current);
  result.position_error_m = achieved.position_error_m;
  result.orientation_error_rad = achieved.orientation_error_rad;
  result.position_velocity_residual_m_s = linear_residual.norm();
  result.orientation_velocity_residual_rad_s = angular_residual.norm();
  result.minimum_singular_value = achieved_minimum_singular_value;
  result.singularity_active =
    achieved_minimum_singular_value < settings.singular_value_threshold;
  result.arm_angle_error_rad = achieved_arm_angle_error.value_or(0.0);
  result.minimum_limit_margin_rad = impl_->minimum_limit_margin(side, candidate);
  result.maximum_joint_step_rad = maximum_step;
  result.requested_maximum_joint_step_rad = maximum_step;
  result.joint_step_limited = maximum_step >=
    settings.maximum_joint_step_rad - 1.0e-8;
  result.accepted = true;
  result.converged =
    achieved.position_error_m <= settings.position_tolerance_m &&
    achieved.orientation_error_rad <= settings.orientation_tolerance_rad &&
    (!achieved_arm_angle_error.has_value() ||
    std::abs(*achieved_arm_angle_error) <= settings.arm_angle_tolerance_rad);
  const bool critical_singularity =
    achieved_minimum_singular_value <=
    settings.qp_singularity_critical_threshold;
  const bool materially_constrained =
    qp_result.active_constraints > 0 &&
    (linear_residual.norm() > 0.25 * settings.qp_max_linear_speed_m_s ||
    angular_residual.norm() > 0.25 * settings.qp_max_angular_speed_rad_s);
  result.saturated = !result.converged &&
    (critical_singularity || materially_constrained);
  if (result.converged) {
    result.status = "converged";
  } else if (critical_singularity) {
    result.status = "singularity_recovery";
  } else if (materially_constrained) {
    result.status = "constrained_tracking";
  } else {
    result.status = "tracking";
  }

  impl_->previous_velocity[state_index] = qp_result.solution;
  impl_->expected_joints[state_index] = candidate;
  impl_->warm_start_valid[state_index] = true;
  return result;
}

}  // namespace pico_body_tianji
