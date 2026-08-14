#include "pico_body_tianji/ik/arm_ik_factory.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pico_body_tianji
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kLeftIndex = 0;
constexpr std::size_t kRightIndex = 1;

double radians(double degrees)
{
  return degrees * kPi / 180.0;
}

double degrees(double radians_value)
{
  return radians_value * 180.0 / kPi;
}

ArmJointVector joint_vector_from_degrees(
  const std::vector<double> & values,
  const std::string & parameter_name)
{
  if (values.size() != 7) {
    throw std::invalid_argument(
            parameter_name + " 必须恰好包含 7 个关节角");
  }
  ArmJointVector result;
  for (Eigen::Index index = 0; index < result.size(); ++index) {
    const double value = values[static_cast<std::size_t>(index)];
    if (!std::isfinite(value)) {
      throw std::invalid_argument(parameter_name + " 含有非有限值");
    }
    result[index] = radians(value);
  }
  return result;
}

std::string json_quote(const std::string & value)
{
  std::ostringstream stream;
  stream << '"';
  for (const char character : value) {
    switch (character) {
      case '"':
        stream << "\\\"";
        break;
      case '\\':
        stream << "\\\\";
        break;
      case '\n':
        stream << "\\n";
        break;
      case '\r':
        stream << "\\r";
        break;
      case '\t':
        stream << "\\t";
        break;
      default:
        stream << character;
        break;
    }
  }
  stream << '"';
  return stream.str();
}

std::string json_optional_string(
  const std::optional<std::string> & value)
{
  return value.has_value() ? json_quote(*value) : "null";
}

std::string json_optional_number(const std::optional<double> & value)
{
  if (!value.has_value() || !std::isfinite(*value)) {
    return "null";
  }
  std::ostringstream stream;
  stream << std::setprecision(10) << *value;
  return stream.str();
}

const char * json_bool(bool value)
{
  return value ? "true" : "false";
}

Eigen::Isometry3d pose_from_message(
  const geometry_msgs::msg::PoseStamped & message)
{
  const Eigen::Quaterniond quaternion(
    message.pose.orientation.w,
    message.pose.orientation.x,
    message.pose.orientation.y,
    message.pose.orientation.z);
  if (
    !quaternion.coeffs().allFinite() ||
    quaternion.norm() < 1.0e-8)
  {
    throw std::invalid_argument("末端目标四元数无效");
  }
  const Eigen::Vector3d translation(
    message.pose.position.x,
    message.pose.position.y,
    message.pose.position.z);
  if (!translation.allFinite()) {
    throw std::invalid_argument("末端目标位置含有非有限值");
  }

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = quaternion.normalized().toRotationMatrix();
  pose.translation() = translation;
  return pose;
}

geometry_msgs::msg::Pose pose_message(const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::Pose message;
  message.position.x = pose.translation().x();
  message.position.y = pose.translation().y();
  message.position.z = pose.translation().z();
  const Eigen::Quaterniond quaternion(pose.rotation());
  message.orientation.x = quaternion.x();
  message.orientation.y = quaternion.y();
  message.orientation.z = quaternion.z();
  message.orientation.w = quaternion.w();
  return message;
}

}  // namespace

class TianjiKinematicSimNode : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TianjiKinematicSimNode()
  : Node("tianji_kinematic_sim")
  {
    declare_parameter("ik_backend", "pinocchio_cpp");
    declare_parameter("official_ik_library", "");
    declare_parameter("official_ik_config", "");
    declare_parameter("rate", 30.0);
    declare_parameter("home_minimum_duration", 2.0);
    declare_parameter("home_max_speed_deg_s", 25.0);
    declare_parameter("joint_limit_margin_deg", 5.0);
    declare_parameter("max_joint_step_deg", 3.0);
    declare_parameter("max_iteration_step_deg", 4.5);
    declare_parameter("max_iterations", 24);
    declare_parameter("position_tolerance_m", 1.0e-3);
    declare_parameter("orientation_tolerance_deg", 0.6);
    declare_parameter("minimum_damping", 1.0e-3);
    declare_parameter("maximum_damping", 0.15);
    declare_parameter("singular_value_threshold", 0.05);
    declare_parameter("arm_angle_gain", 0.8);
    declare_parameter("arm_angle_tolerance_deg", 2.0);
    declare_parameter("arm_angle_merit_weight", 1.0e-3);
    declare_parameter("nullspace_damping", 1.0e-3);
    declare_parameter("joint_center_gain", 0.3);
    declare_parameter("joint_center_activation_margin_deg", 15.0);
    declare_parameter("joint_center_merit_weight", 1.0e-3);
    declare_parameter("singularity_avoidance_gain", 0.2);
    declare_parameter("singularity_merit_weight", 1.0e-2);
    declare_parameter("qp_position_time_constant_s", 0.30);
    declare_parameter("qp_orientation_time_constant_s", 0.40);
    declare_parameter("qp_max_linear_speed_m_s", 0.25);
    declare_parameter("qp_max_angular_speed_rad_s", 1.00);
    declare_parameter<std::vector<double>>(
      "qp_joint_velocity_limits_deg_s",
      {55.0, 55.0, 55.0, 55.0, 55.0, 55.0, 55.0});
    declare_parameter("qp_position_weight", 1.0);
    declare_parameter("qp_orientation_weight", 0.45);
    declare_parameter("qp_velocity_regularization_weight", 2.0e-2);
    declare_parameter("qp_continuity_weight", 6.0e-2);
    declare_parameter("qp_posture_weight", 8.0e-3);
    declare_parameter("qp_posture_time_constant_s", 2.5);
    declare_parameter("qp_joint_limit_activation_margin_deg", 15.0);
    declare_parameter("qp_joint_limit_velocity_damper_gain", 4.0);
    declare_parameter("qp_singularity_critical_threshold", 1.5e-2);
    declare_parameter("qp_singularity_orientation_scale", 0.15);
    declare_parameter("qp_singularity_posture_multiplier", 8.0);
    declare_parameter("qp_singularity_velocity_multiplier", 4.0);
    declare_parameter("qp_singularity_escape_weight", 3.0e-2);
    declare_parameter("qp_singularity_escape_speed_deg_s", 8.6);
    declare_parameter("qp_max_active_set_iterations", 48);
    declare_parameter("qp_active_set_tolerance", 1.0e-9);
    declare_parameter<std::vector<double>>(
      "left_home_deg",
      {55.0, -65.0, -70.0, -60.0, 60.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>(
      "right_home_deg",
      {-55.0, -65.0, 70.0, -60.0, -60.0, 0.0, 0.0});

    const std::string backend = get_parameter("ik_backend").as_string();
    ik_backend_ = backend;

    rate_hz_ = get_parameter("rate").as_double();
    home_minimum_duration_s_ =
      get_parameter("home_minimum_duration").as_double();
    home_max_speed_deg_s_ =
      get_parameter("home_max_speed_deg_s").as_double();
    if (
      rate_hz_ <= 0.0 ||
      home_minimum_duration_s_ <= 0.0 ||
      home_max_speed_deg_s_ <= 0.0)
    {
      throw std::invalid_argument("频率与回零轨迹参数必须为正数");
    }

    IkSettings settings;
    settings.max_iterations =
      static_cast<int>(get_parameter("max_iterations").as_int());
    settings.position_tolerance_m =
      get_parameter("position_tolerance_m").as_double();
    settings.orientation_tolerance_rad =
      radians(get_parameter("orientation_tolerance_deg").as_double());
    settings.minimum_damping =
      get_parameter("minimum_damping").as_double();
    settings.maximum_damping =
      get_parameter("maximum_damping").as_double();
    settings.singular_value_threshold =
      get_parameter("singular_value_threshold").as_double();
    settings.maximum_iteration_step_rad =
      radians(get_parameter("max_iteration_step_deg").as_double());
    settings.maximum_joint_step_rad =
      radians(get_parameter("max_joint_step_deg").as_double());
    maximum_joint_step_rad_ = settings.maximum_joint_step_rad;
    settings.joint_limit_margin_rad =
      radians(get_parameter("joint_limit_margin_deg").as_double());
    settings.arm_angle_gain =
      get_parameter("arm_angle_gain").as_double();
    settings.arm_angle_tolerance_rad =
      radians(get_parameter("arm_angle_tolerance_deg").as_double());
    settings.arm_angle_merit_weight =
      get_parameter("arm_angle_merit_weight").as_double();
    settings.nullspace_damping =
      get_parameter("nullspace_damping").as_double();
    settings.joint_center_gain =
      get_parameter("joint_center_gain").as_double();
    settings.joint_center_activation_margin_rad = radians(
      get_parameter("joint_center_activation_margin_deg").as_double());
    settings.joint_center_merit_weight =
      get_parameter("joint_center_merit_weight").as_double();
    settings.singularity_avoidance_gain =
      get_parameter("singularity_avoidance_gain").as_double();
    settings.singularity_merit_weight =
      get_parameter("singularity_merit_weight").as_double();
    settings.control_period_s = 1.0 / rate_hz_;
    settings.qp_position_time_constant_s =
      get_parameter("qp_position_time_constant_s").as_double();
    settings.qp_orientation_time_constant_s =
      get_parameter("qp_orientation_time_constant_s").as_double();
    settings.qp_max_linear_speed_m_s =
      get_parameter("qp_max_linear_speed_m_s").as_double();
    settings.qp_max_angular_speed_rad_s =
      get_parameter("qp_max_angular_speed_rad_s").as_double();
    settings.qp_joint_velocity_limits_rad_s = joint_vector_from_degrees(
      get_parameter("qp_joint_velocity_limits_deg_s").as_double_array(),
      "qp_joint_velocity_limits_deg_s");
    settings.qp_position_weight =
      get_parameter("qp_position_weight").as_double();
    settings.qp_orientation_weight =
      get_parameter("qp_orientation_weight").as_double();
    settings.qp_velocity_regularization_weight =
      get_parameter("qp_velocity_regularization_weight").as_double();
    settings.qp_continuity_weight =
      get_parameter("qp_continuity_weight").as_double();
    settings.qp_posture_weight =
      get_parameter("qp_posture_weight").as_double();
    settings.qp_posture_time_constant_s =
      get_parameter("qp_posture_time_constant_s").as_double();
    settings.qp_joint_limit_activation_margin_rad = radians(
      get_parameter("qp_joint_limit_activation_margin_deg").as_double());
    settings.qp_joint_limit_velocity_damper_gain =
      get_parameter("qp_joint_limit_velocity_damper_gain").as_double();
    settings.qp_singularity_critical_threshold =
      get_parameter("qp_singularity_critical_threshold").as_double();
    settings.qp_singularity_orientation_scale =
      get_parameter("qp_singularity_orientation_scale").as_double();
    settings.qp_singularity_posture_multiplier =
      get_parameter("qp_singularity_posture_multiplier").as_double();
    settings.qp_singularity_velocity_multiplier =
      get_parameter("qp_singularity_velocity_multiplier").as_double();
    settings.qp_singularity_escape_weight =
      get_parameter("qp_singularity_escape_weight").as_double();
    settings.qp_singularity_escape_speed_rad_s = radians(
      get_parameter("qp_singularity_escape_speed_deg_s").as_double());
    settings.qp_max_active_set_iterations = static_cast<int>(
      get_parameter("qp_max_active_set_iterations").as_int());
    settings.qp_active_set_tolerance =
      get_parameter("qp_active_set_tolerance").as_double();
    settings.qp_left_nominal_rad = joint_vector_from_degrees(
      get_parameter("left_home_deg").as_double_array(), "left_home_deg");
    settings.qp_right_nominal_rad = joint_vector_from_degrees(
      get_parameter("right_home_deg").as_double_array(), "right_home_deg");
    arm_angle_required_ = settings.arm_angle_gain > 0.0;

    const std::string package_share =
      ament_index_cpp::get_package_share_directory("pico_body_tianji");
    const std::string urdf_path =
      package_share +
      "/assets/marvin_m6_ccs/urdf/"
      "marvin_m6_s_ccs_696_v4.urdf";
    ArmIkBackendOptions backend_options;
    backend_options.urdf_path = urdf_path;
    backend_options.official_library_path =
      get_parameter("official_ik_library").as_string();
    backend_options.official_config_path =
      get_parameter("official_ik_config").as_string();
    solver_ = create_arm_ik_solver(backend, backend_options, settings);
    RCLCPP_INFO(
      get_logger(), "已选择 IK 后端：%s", backend.c_str());

    arms_[kLeftIndex].side = ArmSide::kLeft;
    arms_[kLeftIndex].side_name = "left";
    arms_[kLeftIndex].suffix = "L";
    arms_[kLeftIndex].home = joint_vector_from_degrees(
      get_parameter("left_home_deg").as_double_array(),
      "left_home_deg");
    arms_[kRightIndex].side = ArmSide::kRight;
    arms_[kRightIndex].side_name = "right";
    arms_[kRightIndex].suffix = "R";
    arms_[kRightIndex].home = joint_vector_from_degrees(
      get_parameter("right_home_deg").as_double_array(),
      "right_home_deg");
    for (ArmState & arm : arms_) {
      arm.current = arm.home;
      arm.achieved = solver_->forward(arm.side, arm.current);
    }

    create_publishers_and_subscriptions();
    broadcast_model_frames();

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz_),
      std::bind(&TianjiKinematicSimNode::tick, this));
    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&TianjiKinematicSimNode::publish_status, this));

    publish_at_home(true);
    RCLCPP_INFO(
      get_logger(),
      "双臂纯运动学节点已启动，IK 后端=%s；未连接实体机械臂",
      ik_backend_.c_str());
  }

private:
  struct ReturnTrajectory
  {
    ArmJointVector start{ArmJointVector::Zero()};
    std::chrono::steady_clock::time_point start_time{};
    double duration_s{0.0};
    bool active{false};
  };

  struct ArmState
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ArmSide side{ArmSide::kLeft};
    std::string side_name;
    std::string suffix;
    ArmJointVector home{ArmJointVector::Zero()};
    ArmJointVector current{ArmJointVector::Zero()};
    std::optional<Eigen::Isometry3d> target;
    std::optional<Eigen::Vector3d> elbow_direction;
    std::optional<Eigen::Isometry3d> achieved;
    std::optional<IkResult> result;
    std::optional<std::string> error;
    ReturnTrajectory return_trajectory;
  };

  void create_publishers_and_subscriptions()
  {
    const rclcpp::QoS latched_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    for (std::size_t index = 0; index < arms_.size(); ++index) {
      const std::string & side = arms_[index].side_name;
      joint_publishers_[index] =
        create_publisher<sensor_msgs::msg::JointState>(
        "/pico_body_sim/" + side + "_arm/joint_commands", 10);
      solved_pose_publishers_[index] =
        create_publisher<geometry_msgs::msg::PoseStamped>(
        "/pico_body_sim/" + side + "_arm/solved_pose", 10);
      pose_subscriptions_[index] =
        create_subscription<geometry_msgs::msg::PoseStamped>(
        "/pico_body/" + side + "_arm_target_pose",
        10,
        [this, index](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          on_pose(index, *message);
        });
      direction_subscriptions_[index] =
        create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/pico_body/" + side + "_arm_elbow_direction",
        10,
        [this, index](
          geometry_msgs::msg::Vector3Stamped::ConstSharedPtr message)
        {
          on_direction(index, *message);
        });
    }

    model_joint_publisher_ =
      create_publisher<sensor_msgs::msg::JointState>(
      "/pico_body_sim/model_joint_states", 10);
    marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      "/pico_body_sim/markers", 10);
    at_home_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
      "/pico_body_sim/at_home", latched_qos);
    return_complete_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
      "/pico_body_sim/return_complete", 10);
    status_publisher_ =
      create_publisher<std_msgs::msg::String>(
      "/pico_body_sim/status", 10);
    teleop_state_subscription_ =
      create_subscription<std_msgs::msg::String>(
      "/pico_body/teleop_state",
      10,
      std::bind(
        &TianjiKinematicSimNode::on_teleop_state,
        this,
        std::placeholders::_1));
  }

  void on_pose(
    std::size_t arm_index,
    const geometry_msgs::msg::PoseStamped & message)
  {
    try {
      arms_[arm_index].target = pose_from_message(message);
      arms_[arm_index].error.reset();
    } catch (const std::exception & exception) {
      arms_[arm_index].error = exception.what();
    }
  }

  void on_direction(
    std::size_t arm_index,
    const geometry_msgs::msg::Vector3Stamped & message)
  {
    const Eigen::Vector3d direction(
      message.vector.x,
      message.vector.y,
      message.vector.z);
    if (!direction.allFinite() || direction.norm() < 1.0e-8) {
      arms_[arm_index].error = "SMPL 臂角方向无效";
      return;
    }
    arms_[arm_index].elbow_direction = direction.normalized();
  }

  void on_teleop_state(std_msgs::msg::String::ConstSharedPtr message)
  {
    if (message->data == "teleop") {
      mode_ = "teleop";
      for (ArmState & arm : arms_) {
        arm.return_trajectory.active = false;
        arm.error.reset();
      }
      publish_at_home(false);
      return;
    }
    if (message->data == "returning" && mode_ != "returning") {
      begin_return();
      return;
    }
    if (message->data == "idle" && mode_ != "returning") {
      mode_ = "idle";
    }
  }

  void begin_return()
  {
    const auto start_time = std::chrono::steady_clock::now();
    for (ArmState & arm : arms_) {
      const double maximum_delta_deg =
        degrees((arm.home - arm.current).cwiseAbs().maxCoeff());
      const double speed_limited_duration =
        1.5 * maximum_delta_deg / home_max_speed_deg_s_;
      arm.return_trajectory.start = arm.current;
      arm.return_trajectory.start_time = start_time;
      arm.return_trajectory.duration_s =
        std::max(home_minimum_duration_s_, speed_limited_duration);
      arm.return_trajectory.active = true;
      arm.target.reset();
      arm.elbow_direction.reset();
      arm.result.reset();
      arm.error.reset();
    }
    mode_ = "returning";
    publish_at_home(false);
    RCLCPP_WARN(get_logger(), "开始按零端速 smoothstep 缓慢回安全初始位");
  }

  void tick()
  {
    if (mode_ == "teleop") {
      solve_targets();
    } else if (mode_ == "returning") {
      sample_return();
    }
    publish_joint_states();
    publish_markers();
  }

  void solve_targets()
  {
    for (std::size_t index = 0; index < arms_.size(); ++index) {
      ArmState & arm = arms_[index];
      if (
        !arm.target.has_value() ||
        (arm_angle_required_ && !arm.elbow_direction.has_value()))
      {
        continue;
      }
      try {
        const Eigen::Vector3d elbow_direction =
          arm.elbow_direction.value_or(Eigen::Vector3d::Zero());
        IkResult result = solver_->solve(
          arm.side,
          *arm.target,
          arm.current,
          elbow_direction);
        if (!result.joints_rad.allFinite()) {
          throw std::runtime_error("IK 后端返回非有限关节角");
        }
        const double actual_maximum_step =
          (result.joints_rad - arm.current).cwiseAbs().maxCoeff();
        result.maximum_joint_step_rad = actual_maximum_step;
        if (
          result.accepted &&
          actual_maximum_step > maximum_joint_step_rad_ + 1.0e-10)
        {
          throw std::runtime_error("IK 后端返回值超过公共关节步长安全限制");
        }
        if (result.accepted) {
          arm.current = result.joints_rad;
        }
        arm.achieved = solver_->forward(arm.side, arm.current);
        if (!arm.achieved->matrix().allFinite()) {
          throw std::runtime_error("IK 后端 FK 返回非有限位姿");
        }
        arm.result = result;
        arm.error = result.saturated ?
          std::optional<std::string>(
          "目标暂不可达，保持在连续安全边界并等待恢复") :
          std::nullopt;
        publish_solved_pose(index);
      } catch (const std::exception & exception) {
        arm.error = exception.what();
        arm.achieved = solver_->forward(arm.side, arm.current);
      }
    }
  }

  void sample_return()
  {
    const auto now = std::chrono::steady_clock::now();
    bool complete = true;
    for (ArmState & arm : arms_) {
      ReturnTrajectory & trajectory = arm.return_trajectory;
      if (!trajectory.active) {
        continue;
      }
      const double elapsed_s =
        std::chrono::duration<double>(now - trajectory.start_time).count();
      const double progress = std::clamp(
        elapsed_s / trajectory.duration_s, 0.0, 1.0);
      const double blend =
        progress * progress * (3.0 - 2.0 * progress);
      arm.current =
        trajectory.start + blend * (arm.home - trajectory.start);
      arm.achieved = solver_->forward(arm.side, arm.current);
      trajectory.active = progress < 1.0;
      complete = complete && !trajectory.active;
    }

    if (complete) {
      mode_ = "idle";
      publish_at_home(true);
      std_msgs::msg::Bool message;
      message.data = true;
      return_complete_publisher_->publish(message);
      RCLCPP_INFO(get_logger(), "IK 双臂已回到安全初始位");
    }
  }

  void publish_solved_pose(std::size_t arm_index)
  {
    const ArmState & arm = arms_[arm_index];
    if (!arm.achieved.has_value()) {
      return;
    }
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now();
    message.header.frame_id = arm.side_name + "_chest";
    message.pose = pose_message(*arm.achieved);
    solved_pose_publishers_[arm_index]->publish(message);
  }

  void publish_joint_states()
  {
    const rclcpp::Time stamp = now();
    sensor_msgs::msg::JointState model_message;
    model_message.header.stamp = stamp;
    model_message.name.reserve(14);
    model_message.position.reserve(14);

    for (std::size_t arm_index = 0; arm_index < arms_.size(); ++arm_index) {
      const ArmState & arm = arms_[arm_index];
      sensor_msgs::msg::JointState command_message;
      command_message.header.stamp = stamp;
      command_message.header.frame_id =
        arm.side_name + "_base_marvin_degrees";
      for (Eigen::Index joint_index = 0; joint_index < 7; ++joint_index) {
        command_message.name.push_back(
          arm.side_name + "_joint_" +
          std::to_string(joint_index + 1));
        command_message.position.push_back(
          degrees(arm.current[joint_index]));
        model_message.name.push_back(
          "Joint" + std::to_string(joint_index + 1) + "_" + arm.suffix);
        model_message.position.push_back(arm.current[joint_index]);
      }
      joint_publishers_[arm_index]->publish(command_message);
    }
    model_joint_publisher_->publish(model_message);
  }

  visualization_msgs::msg::Marker make_target_marker(
    const ArmState & arm,
    int marker_id,
    const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = arm.side_name + "_chest";
    marker.ns = "target";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose_message(*arm.target);
    const Eigen::Quaterniond target_rotation(arm.target->rotation());
    const Eigen::Quaterniond arrow_x_to_tcp_z(
      Eigen::AngleAxisd(-0.5 * kPi, Eigen::Vector3d::UnitY()));
    const Eigen::Quaterniond display_rotation =
      target_rotation * arrow_x_to_tcp_z;
    marker.pose.orientation.x = display_rotation.x();
    marker.pose.orientation.y = display_rotation.y();
    marker.pose.orientation.z = display_rotation.z();
    marker.pose.orientation.w = display_rotation.w();
    marker.scale.x = 0.16;
    marker.scale.y = 0.025;
    marker.scale.z = 0.025;
    marker.color.r = 1.0;
    marker.color.g = 0.65;
    marker.color.a = 0.9;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    return marker;
  }

  visualization_msgs::msg::Marker make_achieved_marker(
    const ArmState & arm,
    int marker_id,
    const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = arm.side_name + "_chest";
    marker.ns = "ik_tcp";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose_message(*arm.achieved);
    marker.scale.x = 0.06;
    marker.scale.y = 0.06;
    marker.scale.z = 0.06;
    marker.color.g = 0.9;
    marker.color.b = 1.0;
    marker.color.a = 0.95;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    return marker;
  }

  std::vector<visualization_msgs::msg::Marker> robot_reference_markers(
    const ArmState & arm,
    int marker_id,
    const rclcpp::Time & stamp) const
  {
    const std::array<std::pair<std::string, std::string>, 3> frames{{
      {"shoulder", "Link1_" + arm.suffix},
      {"elbow", "Link4_" + arm.suffix},
      {"tcp", "TCP_Link_" + arm.suffix},
    }};
    std::vector<visualization_msgs::msg::Marker> markers;
    markers.reserve(4);
    for (const auto & [name, frame] : frames) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = frame;
      marker.ns = "robot_" + name;
      marker.id = marker_id;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.044;
      marker.scale.y = 0.044;
      marker.scale.z = 0.044;
      marker.color.r = 0.2;
      marker.color.g = 1.0;
      marker.color.b = 0.25;
      marker.color.a = 0.95;
      marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      markers.push_back(marker);
    }
    visualization_msgs::msg::Marker label;
    label.header.stamp = stamp;
    label.header.frame_id = "Link4_" + arm.suffix;
    label.ns = "robot_elbow_label";
    label.id = marker_id;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position.z = 0.065;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.042;
    label.text = "Robot " + arm.side_name + " elbow";
    label.color.r = 0.65;
    label.color.g = 1.0;
    label.color.b = 0.65;
    label.color.a = 1.0;
    label.lifetime = rclcpp::Duration::from_seconds(0.2);
    markers.push_back(label);
    return markers;
  }

  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray array;
    const rclcpp::Time stamp = now();
    for (std::size_t index = 0; index < arms_.size(); ++index) {
      const ArmState & arm = arms_[index];
      const int marker_id = static_cast<int>(index);
      auto references = robot_reference_markers(
        arm, marker_id, stamp);
      array.markers.insert(
        array.markers.end(), references.begin(), references.end());
      if (arm.target.has_value()) {
        array.markers.push_back(
          make_target_marker(arm, marker_id, stamp));
      }
      if (arm.achieved.has_value()) {
        array.markers.push_back(
          make_achieved_marker(arm, marker_id, stamp));
      }
    }
    marker_publisher_->publish(array);
  }

  bool at_safe_home() const
  {
    return std::all_of(
      arms_.begin(),
      arms_.end(),
      [](const ArmState & arm) {
        return (arm.current - arm.home).cwiseAbs().maxCoeff() <=
               radians(0.1);
      });
  }

  void publish_at_home(bool value)
  {
    std_msgs::msg::Bool message;
    message.data = value;
    at_home_publisher_->publish(message);
  }

  std::string arm_status_json(const ArmState & arm) const
  {
    const std::optional<double> position_error_mm =
      arm.result.has_value() ?
      std::optional<double>(1000.0 * arm.result->position_error_m) :
      std::nullopt;
    const std::optional<double> orientation_error_deg =
      arm.result.has_value() ?
      std::optional<double>(
      degrees(arm.result->orientation_error_rad)) :
      std::nullopt;
    const std::optional<double> limit_margin_deg =
      arm.result.has_value() ?
      std::optional<double>(
      degrees(arm.result->minimum_limit_margin_rad)) :
      std::nullopt;
    const std::optional<double> maximum_step_deg =
      arm.result.has_value() ?
      std::optional<double>(
      degrees(arm.result->maximum_joint_step_rad)) :
      std::nullopt;
    const std::optional<double> minimum_singular_value =
      arm.result.has_value() ?
      std::optional<double>(arm.result->minimum_singular_value) :
      std::nullopt;
    const std::optional<double> damping =
      arm.result.has_value() ?
      std::optional<double>(arm.result->damping) :
      std::nullopt;
    const std::optional<double> arm_angle_error_deg =
      arm.result.has_value() ?
      std::optional<double>(
      degrees(arm.result->arm_angle_error_rad)) :
      std::nullopt;
    const std::optional<double> position_velocity_residual =
      arm.result.has_value() ?
      std::optional<double>(arm.result->position_velocity_residual_m_s) :
      std::nullopt;
    const std::optional<double> orientation_velocity_residual =
      arm.result.has_value() ?
      std::optional<double>(arm.result->orientation_velocity_residual_rad_s) :
      std::nullopt;
    const int solver_iterations =
      arm.result.has_value() ? arm.result->solver_iterations : 0;
    const int active_joint_constraints =
      arm.result.has_value() ? arm.result->active_joint_constraints : 0;
    const bool step_limited =
      arm.result.has_value() && arm.result->joint_step_limited;
    const bool saturated =
      arm.result.has_value() && arm.result->saturated;
    const bool singularity_active =
      arm.result.has_value() && arm.result->singularity_active;
    const std::optional<std::string> elbow_source =
      arm.elbow_direction.has_value() ?
      std::optional<std::string>("smpl_nullspace") :
      std::nullopt;
    const std::string prefix = arm.side_name;

    std::ostringstream stream;
    stream
      << json_quote(prefix + "_ik_error") << ':'
      << json_optional_string(arm.error) << ','
      << json_quote(prefix + "_position_error_mm") << ':'
      << json_optional_number(position_error_mm) << ','
      << json_quote(prefix + "_orientation_error_deg") << ':'
      << json_optional_number(orientation_error_deg) << ','
      << json_quote(prefix + "_min_limit_margin_deg") << ':'
      << json_optional_number(limit_margin_deg) << ','
      << json_quote(prefix + "_max_joint_step_deg") << ':'
      << json_optional_number(maximum_step_deg) << ','
      << json_quote(prefix + "_requested_max_joint_step_deg") << ':'
      << json_optional_number(maximum_step_deg) << ','
      << json_quote(prefix + "_joint_step_limited") << ':'
      << json_bool(step_limited) << ','
      << json_quote(prefix + "_elbow_constraint_source") << ':'
      << json_optional_string(elbow_source) << ','
      << json_quote(prefix + "_target_saturated") << ':'
      << json_bool(saturated) << ','
      << json_quote(prefix + "_requested_target_error") << ':'
      << (saturated ? json_optional_string(arm.error) : "null") << ','
      << json_quote(prefix + "_min_singular_value") << ':'
      << json_optional_number(minimum_singular_value) << ','
      << json_quote(prefix + "_damping") << ':'
      << json_optional_number(damping) << ','
      << json_quote(prefix + "_arm_angle_error_deg") << ':'
      << json_optional_number(arm_angle_error_deg) << ','
      << json_quote(prefix + "_position_velocity_residual_m_s") << ':'
      << json_optional_number(position_velocity_residual) << ','
      << json_quote(prefix + "_orientation_velocity_residual_rad_s") << ':'
      << json_optional_number(orientation_velocity_residual) << ','
      << json_quote(prefix + "_solver_iterations") << ':'
      << solver_iterations << ','
      << json_quote(prefix + "_active_joint_constraints") << ':'
      << active_joint_constraints << ','
      << json_quote(prefix + "_singularity_active") << ':'
      << json_bool(singularity_active);
    return stream.str();
  }

  void publish_status()
  {
    std_msgs::msg::String message;
    std::ostringstream stream;
    stream
      << '{'
      << "\"mode\":" << json_quote(mode_) << ','
      << "\"at_safe_home\":" << json_bool(at_safe_home()) << ','
      << arm_status_json(arms_[kLeftIndex]) << ','
      << arm_status_json(arms_[kRightIndex]) << ','
      << "\"sdk\":" << json_quote(ik_backend_) << ','
      << "\"ik_interface\":\"arm_ik_solver_v1\","
      << "\"ik_backend\":" << json_quote(ik_backend_) << ','
      << "\"robot_connected\":false,"
      << "\"scope\":\"preview_only\""
      << '}';
    message.data = stream.str();
    status_publisher_->publish(message);
  }

  void broadcast_model_frames()
  {
    static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(3);
    for (const auto & [parent, child] :
      std::array<std::pair<std::string, std::string>, 3>{{
        {"world", "Link_Base"},
        {"Base_L", "left_chest"},
        {"Base_R", "right_chest"},
      }})
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = now();
      transform.header.frame_id = parent;
      transform.child_frame_id = child;
      transform.transform.rotation.w = 1.0;
      transforms.push_back(transform);
    }
    static_broadcaster_->sendTransform(transforms);
  }

  std::unique_ptr<ArmIkSolver> solver_;
  std::array<ArmState, 2> arms_;
  std::string ik_backend_;
  std::string mode_{"idle"};
  double rate_hz_{30.0};
  double home_minimum_duration_s_{2.0};
  double home_max_speed_deg_s_{25.0};
  double maximum_joint_step_rad_{radians(3.0)};
  bool arm_angle_required_{false};

  std::array<
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr,
    2> joint_publishers_;
  std::array<
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr,
    2> solved_pose_publishers_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    model_joint_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr at_home_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    return_complete_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;

  std::array<
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr,
    2> pose_subscriptions_;
  std::array<
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr,
    2> direction_subscriptions_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    teleop_state_subscription_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster>
    static_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace pico_body_tianji

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<pico_body_tianji::TianjiKinematicSimNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("tianji_kinematic_sim"),
      "节点启动或运行失败：%s",
      exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
