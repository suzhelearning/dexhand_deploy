from __future__ import annotations

import json
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .hardware_safety import (
    HardwareSafetyController,
    HardwareSafetySettings,
)
from .host_readiness import HostReadinessGate
from .marvin_hardware import (
    MarvinFeedback,
    MarvinHardwareError,
    MarvinHardwareSession,
)
from .marvin_state import command_states_compatible


SIDES = ("left", "right")
CONFLICTING_CONTROLLER_NAMES = {
    "tianji_world_output_node",
    "tianji_arm_node",
}
OUTPUT_STEP_REFERENCE_VELOCITY_RATIO = 10


def _source_stamp_is_fresh(
    stamp_ns: int, now_ns: int, timeout_s: float
) -> bool:
    if stamp_ns <= 0 or timeout_s <= 0.0:
        return False
    age_s = (int(now_ns) - int(stamp_ns)) * 1.0e-9
    return -0.05 <= age_s <= float(timeout_s)


def _describe_feedback_failure(
    reason: str, feedback: MarvinFeedback
) -> str:
    if reason != "arm_error":
        return reason
    prefix = (
        "controller_emergency_stop"
        if feedback.error_codes == (13, 13)
        else "arm_error"
    )
    return (
        f"{prefix}:errors={feedback.error_codes},"
        f"states={feedback.arm_states},"
        f"commands={feedback.command_states}"
    )


def _scaled_output_step_deg(
    *,
    configured_step_deg: float,
    velocity_ratio: int,
    reference_velocity_ratio: int,
    rate_hz: float = 30.0,
    maximum_teleop_speed_deg_s: float = 40.0,
) -> float:
    configured = float(configured_step_deg)
    velocity = int(velocity_ratio)
    reference = int(reference_velocity_ratio)
    rate = float(rate_hz)
    speed_limit = float(maximum_teleop_speed_deg_s)
    if configured <= 0.0:
        raise ValueError("configured_step_deg 必须为正数")
    if not 1 <= velocity <= 100:
        raise ValueError("velocity_ratio 必须在 1..100")
    if not 1 <= reference <= 100:
        raise ValueError("reference_velocity_ratio 必须在 1..100")
    if rate <= 0.0:
        raise ValueError("rate_hz 必须为正数")
    if speed_limit <= 0.0:
        raise ValueError("maximum_teleop_speed_deg_s 必须为正数")
    ratio_scaled_step = configured * velocity / reference
    speed_limited_step = speed_limit / rate
    return min(ratio_scaled_step, speed_limited_step)


def create_official_marvin_session() -> MarvinHardwareSession:
    """延迟加载真机已验证的 Marvin Python SDK；不实例化官方 IK。"""
    from marvin_sdk.fx_robot import DCSS, Marvin_Robot

    session = MarvinHardwareSession(
        robot=Marvin_Robot(),
        dcss_factory=DCSS,
    )
    try:
        with open("/proc/self/maps", encoding="utf-8") as maps_file:
            maps = maps_file.read()
    except OSError:
        maps = ""
    if "libKine" in maps:
        raise MarvinHardwareError(
            "libKine unexpectedly loaded in joint-control process"
        )
    return session


class MarvinHardwareBridge(Node):
    """主机 IK 关节流到 Marvin 双臂的真机安全桥。"""

    def __init__(self):
        super().__init__("marvin_hardware_bridge")
        self._declare_parameters()
        self._rate_hz = float(self.get_parameter("rate").value)
        if self._rate_hz <= 0.0:
            raise ValueError("rate 必须为正数")
        self._command_timeout_s = float(
            self.get_parameter("command_timeout_s").value
        )
        self._left_home = self._parameter_vector("left_home_deg")
        self._right_home = self._parameter_vector("right_home_deg")
        self._lower_limits = self._parameter_vector(
            "lower_limits_deg"
        )
        self._upper_limits = self._parameter_vector(
            "upper_limits_deg"
        )
        self._robot_ip = str(self.get_parameter("robot_ip").value)
        if not self._robot_ip:
            raise ValueError("robot_ip 必须配置为 Marvin 控制器地址")
        velocity_ratio = int(self.get_parameter("velocity_ratio").value)
        self._maximum_output_step_deg = _scaled_output_step_deg(
            configured_step_deg=float(
                self.get_parameter("maximum_output_step_deg").value
            ),
            velocity_ratio=velocity_ratio,
            reference_velocity_ratio=(
                OUTPUT_STEP_REFERENCE_VELOCITY_RATIO
            ),
            rate_hz=self._rate_hz,
            maximum_teleop_speed_deg_s=float(
                self.get_parameter(
                    "maximum_teleop_speed_deg_s"
                ).value
            ),
        )

        self._readiness = HostReadinessGate(
            left_home_deg=self._left_home,
            right_home_deg=self._right_home,
            freshness_timeout_s=float(
                self.get_parameter("host_status_timeout_s").value
            ),
            command_timeout_s=float(
                self.get_parameter("command_timeout_s").value
            ),
            maximum_pair_skew_s=float(
                self.get_parameter("maximum_pair_skew_s").value
            ),
            home_tolerance_deg=float(
                self.get_parameter("home_tolerance_deg").value
            ),
            input_mode=str(
                self.get_parameter("host_input_mode").value
            ),
        )
        settings = HardwareSafetySettings(
            command_timeout_s=float(
                self.get_parameter("command_timeout_s").value
            ),
            state_timeout_s=float(
                self.get_parameter("state_timeout_s").value
            ),
            feedback_timeout_s=float(
                self.get_parameter("feedback_timeout_s").value
            ),
            maximum_pair_skew_s=float(
                self.get_parameter("maximum_pair_skew_s").value
            ),
            maximum_output_step_deg=self._maximum_output_step_deg,
            maximum_tracking_error_deg=float(
                self.get_parameter(
                    "maximum_tracking_error_deg"
                ).value
            ),
            return_minimum_duration_s=float(
                self.get_parameter(
                    "return_minimum_duration_s"
                ).value
            ),
            return_max_speed_deg_s=float(
                self.get_parameter("return_max_speed_deg_s").value
            ),
            home_tolerance_deg=float(
                self.get_parameter("home_tolerance_deg").value
            ),
        )
        self._safety = HardwareSafetyController(
            left_home_deg=self._left_home,
            right_home_deg=self._right_home,
            lower_limits_deg=self._lower_limits,
            upper_limits_deg=self._upper_limits,
            settings=settings,
        )

        self._session: MarvinHardwareSession | None = None
        self._phase = "waiting_for_safe_host"
        self._readiness_reason = "not_evaluated"
        self._last_action = "none"
        self._last_error = None
        self._tracking_error_detail = None
        self._command_count = 0
        self._latest_feedback: MarvinFeedback | None = None
        self._create_ros_interfaces()
        self.get_logger().warning(
            "真机桥已确认启动，但尚未连接 Marvin；"
            "等待同机 IK 链路处于 idle 安全零位。"
            "连接时会自动清除已释放的历史锁存错误；"
            "仍然生效的实体急停或安全链不会被绕过。"
            f"指令斜坡上限={self._maximum_output_step_deg:.3f}°/帧"
            f"（{self._maximum_output_step_deg * self._rate_hz:.2f}°/s）。"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "robot_ip": "",
            "rate": 30.0,
            "velocity_ratio": 10,
            "acceleration_ratio": 10,
            "host_status_timeout_s": 1.0,
            "command_timeout_s": 0.15,
            "state_timeout_s": 1.0,
            "feedback_timeout_s": 0.15,
            "maximum_pair_skew_s": 0.03,
            "maximum_output_step_deg": 0.5,
            "maximum_teleop_speed_deg_s": 40.0,
            "maximum_tracking_error_deg": 8.0,
            "return_minimum_duration_s": 2.0,
            "return_max_speed_deg_s": 10.0,
            "home_tolerance_deg": 1.0,
            "host_input_mode": "smpl",
            "feedback_hard_limit_padding_deg": 5.0,
            "left_home_deg": [
                55.0,
                -65.0,
                -70.0,
                -60.0,
                60.0,
                0.0,
                0.0,
            ],
            "right_home_deg": [
                -55.0,
                -65.0,
                70.0,
                -60.0,
                -60.0,
                0.0,
                0.0,
            ],
            "lower_limits_deg": [
                -165.0,
                -115.0,
                -165.0,
                -140.0,
                -165.0,
                -55.0,
                -85.0,
            ],
            "upper_limits_deg": [
                165.0,
                115.0,
                165.0,
                55.0,
                165.0,
                55.0,
                85.0,
            ],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _parameter_vector(self, name: str) -> np.ndarray:
        values = np.asarray(
            self.get_parameter(name).value, dtype=np.float64
        )
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError(f"{name} 必须包含 7 个有限数值")
        return values

    def _create_ros_interfaces(self) -> None:
        for side in SIDES:
            self.create_subscription(
                JointState,
                f"/pico_body_sim/{side}_arm/joint_commands",
                lambda message, side=side: self._on_command(
                    side, message
                ),
                1,
            )
        self.create_subscription(
            String,
            "/pico_body/teleop_state",
            self._on_teleop_state,
            1,
        )
        self.create_subscription(
            String,
            "/pico_body/status",
            self._on_input_status,
            1,
        )
        self.create_subscription(
            String,
            "/pico_body_sim/status",
            self._on_sim_status,
            1,
        )
        self._feedback_publishers = {
            side: self.create_publisher(
                JointState, f"/{side}_arm/joint_states", 10
            )
            for side in SIDES
        }
        self._status_publisher = self.create_publisher(
            String, "/pico_body_real/status", 10
        )
        self.create_timer(1.0 / self._rate_hz, self._tick)
        self.create_timer(0.5, self._publish_status)

    def _on_command(self, side: str, message: JointState) -> None:
        now = time.monotonic()
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if not _source_stamp_is_fresh(
            stamp_ns,
            self.get_clock().now().nanoseconds,
            self._command_timeout_s,
        ):
            self._last_error = f"stale_{side}_command_stamp"
            return
        try:
            self._readiness.observe_command(
                side,
                message.position,
                frame_id=message.header.frame_id,
                received_at=now,
            )
            self._safety.observe_command(
                side,
                message.position,
                frame_id=message.header.frame_id,
                received_at=now,
            )
        except ValueError as exc:
            self._last_error = f"invalid_{side}_command: {exc}"

    def _on_teleop_state(self, message: String) -> None:
        self._observe_state(message.data, time.monotonic())

    def _observe_state(self, state: str, received_at: float) -> None:
        try:
            self._readiness.observe_teleop_state(
                state, received_at=received_at
            )
            self._safety.observe_teleop_state(
                state, received_at=received_at
            )
            if self._phase.startswith("armed_"):
                self._phase = f"armed_{state}"
        except ValueError as exc:
            self._last_error = f"invalid_teleop_state: {exc}"

    def _on_input_status(self, message: String) -> None:
        now = time.monotonic()
        try:
            self._readiness.observe_input_status(
                message.data, received_at=now
            )
            payload = json.loads(message.data)
            state = payload.get("state")
            if isinstance(state, str):
                self._observe_state(state, now)
        except (ValueError, json.JSONDecodeError) as exc:
            self._last_error = f"invalid_input_status: {exc}"

    def _on_sim_status(self, message: String) -> None:
        try:
            self._readiness.observe_sim_status(
                message.data, received_at=time.monotonic()
            )
        except ValueError as exc:
            self._last_error = f"invalid_sim_status: {exc}"

    def _tick(self) -> None:
        if self._phase == "waiting_for_safe_host":
            self._try_start_hardware()
            return
        if self._session is None:
            return
        try:
            feedback = self._session.read_feedback()
            self._latest_feedback = feedback
            self._publish_feedback(feedback)
            now = time.monotonic()
            self._safety.observe_feedback(
                left_joints_deg=feedback.left_joints_deg,
                right_joints_deg=feedback.right_joints_deg,
                arm_states=feedback.arm_states,
                command_states=feedback.command_states,
                error_codes=feedback.error_codes,
                servo_error_reports=feedback.servo_error_reports,
                frame_serials=feedback.frame_serials,
                received_at=now,
            )
            feedback_reason = self._safety.feedback_unsafe_reason(now=now)
            if feedback_reason is not None:
                self._trip_soft_stop(
                    _describe_feedback_failure(
                        feedback_reason, feedback
                    )
                )
                return
            if self._phase == "waiting_for_post_home_snapshot":
                self._hold_home_until_host_is_fresh(now, feedback)
                return
            decision = self._safety.decide(now=now)
            self._last_action = f"{decision.action}:{decision.reason}"
            if decision.action == "soft_stop":
                if decision.reason == "tracking_error":
                    detail = self._safety.tracking_error_detail()
                    if detail is not None:
                        self._tracking_error_detail = {
                            "side": detail.side,
                            "joint_index": detail.joint_index,
                            "commanded_deg": detail.commanded_deg,
                            "measured_deg": detail.measured_deg,
                            "signed_error_deg": detail.signed_error_deg,
                            "absolute_error_deg": detail.absolute_error_deg,
                        }
                        self.get_logger().error(
                            "跟踪误差保护触发："
                            f"{detail.side} J{detail.joint_index}，"
                            f"commanded={detail.commanded_deg:.3f}°, "
                            f"measured={detail.measured_deg:.3f}°, "
                            f"error={detail.signed_error_deg:+.3f}°"
                        )
                self._trip_soft_stop(decision.reason)
                return
            self._session.send_joint_targets(
                decision.left_joints_deg,
                decision.right_joints_deg,
            )
            self._command_count += 1
        except BaseException as exc:
            self._trip_soft_stop(f"runtime_error: {exc}")

    def _try_start_hardware(self) -> None:
        conflicts = (
            set(self.get_node_names()) & CONFLICTING_CONTROLLER_NAMES
        )
        if conflicts:
            self._readiness_reason = (
                "conflicting_controller:" + ",".join(sorted(conflicts))
            )
            return
        readiness = self._readiness.evaluate(now=time.monotonic())
        self._readiness_reason = readiness.reason
        if not readiness.ready:
            return
        self._phase = "connecting"
        try:
            self._session = create_official_marvin_session()
            feedback = self._session.connect_and_prepare(
                self._robot_ip,
                velocity_ratio=int(
                    self.get_parameter("velocity_ratio").value
                ),
                acceleration_ratio=int(
                    self.get_parameter("acceleration_ratio").value
                ),
                lower_limits_deg=self._lower_limits,
                upper_limits_deg=self._upper_limits,
                hard_limit_padding_deg=float(
                    self.get_parameter(
                        "feedback_hard_limit_padding_deg"
                    ).value
                ),
            )
            self._latest_feedback = feedback
            feedback = self._session.move_to_home(
                self._left_home,
                self._right_home,
                rate_hz=self._rate_hz,
                minimum_duration_s=float(
                    self.get_parameter(
                        "return_minimum_duration_s"
                    ).value
                ),
                max_speed_deg_s=float(
                    self.get_parameter(
                        "return_max_speed_deg_s"
                    ).value
                ),
                maximum_tracking_error_deg=float(
                    self.get_parameter(
                        "maximum_tracking_error_deg"
                    ).value
                ),
                home_tolerance_deg=float(
                    self.get_parameter("home_tolerance_deg").value
                ),
                lower_limits_deg=self._lower_limits,
                upper_limits_deg=self._upper_limits,
                hard_limit_padding_deg=float(
                    self.get_parameter(
                        "feedback_hard_limit_padding_deg"
                    ).value
                ),
                required_state=1,
                feedback_timeout_s=float(
                    self.get_parameter("feedback_timeout_s").value
                ),
            )
            self._phase = "waiting_for_post_home_snapshot"
            self._last_error = None
            self.get_logger().warning(
                "Marvin 双臂已在低速位置模式缓慢到达安全零位；"
                "等待主机链路刷新后，可按右手柄 A 启动遥操作。"
            )
        except BaseException as exc:
            self._fail_startup(exc)

    def _hold_home_until_host_is_fresh(
        self, now: float, feedback: MarvinFeedback
    ) -> None:
        if (
            feedback.error_codes != (0, 0)
            or feedback.servo_error_reports != ("None", "None")
            or feedback.arm_states != (1, 1)
            or not command_states_compatible(
                feedback.command_states,
                1,
            )
        ):
            self._trip_soft_stop("unhealthy_feedback_after_home")
            return
        readiness = self._readiness.evaluate(now=now)
        self._readiness_reason = readiness.reason
        self._session.send_joint_targets(
            self._left_home, self._right_home
        )
        self._command_count += 1
        if readiness.ready:
            self._phase = "armed_idle"
            self.get_logger().warning(
                "真机链路已就绪：保持安全零位，按右手柄 A 开始。"
            )

    def _fail_startup(self, error: BaseException) -> None:
        self._last_error = f"startup_error: {error}"
        session = self._session
        if session is not None:
            try:
                session.soft_stop_once()
            except BaseException:
                pass
            try:
                session.shutdown()
            except BaseException:
                pass
        self._session = None
        self._phase = "failed"
        self.get_logger().error(self._last_error)

    def _trip_soft_stop(self, reason: str) -> None:
        self._last_action = f"soft_stop:{reason}"
        self._last_error = reason
        session = self._session
        if session is not None:
            try:
                session.soft_stop_once()
            except BaseException as exc:
                self._last_error = f"{reason}; soft_stop_failed: {exc}"
            try:
                session.shutdown()
            except BaseException as exc:
                self._last_error = (
                    f"{self._last_error}; shutdown_failed: {exc}"
                )
        self._session = None
        self._phase = "soft_stopped"
        self.get_logger().error(
            f"真机链路已锁存软急停并释放连接：{self._last_error}"
        )

    def _publish_feedback(self, feedback: MarvinFeedback) -> None:
        stamp = self.get_clock().now().to_msg()
        for side, joints in (
            ("left", feedback.left_joints_deg),
            ("right", feedback.right_joints_deg),
        ):
            message = JointState()
            message.header.stamp = stamp
            message.header.frame_id = (
                f"{side}_base_marvin_degrees_measured"
            )
            message.name = [
                f"{side}_joint_{index}" for index in range(1, 8)
            ]
            message.position = joints.tolist()
            self._feedback_publishers[side].publish(message)

    def _publish_status(self) -> None:
        feedback = self._latest_feedback
        payload = {
            "phase": self._phase,
            "readiness": self._readiness_reason,
            "last_action": self._last_action,
            "error": self._last_error,
            "robot_connected": self._session is not None,
            "arm_states": (
                None if feedback is None else feedback.arm_states
            ),
            "command_states": (
                None if feedback is None else feedback.command_states
            ),
            "error_codes": (
                None if feedback is None else feedback.error_codes
            ),
            "servo_error_reports": (
                None
                if feedback is None
                else feedback.servo_error_reports
            ),
            "frame_serials": (
                None if feedback is None else feedback.frame_serials
            ),
            "commands_sent": self._command_count,
            "tracking_error_detail": self._tracking_error_detail,
            "maximum_output_step_deg": self._maximum_output_step_deg,
            "maximum_output_speed_deg_s": (
                self._maximum_output_step_deg * self._rate_hz
            ),
            "ik_source": "arm_ik_solver_v1_host_topics",
            "hardware_sdk": "verified_marvin_python_sdk",
            "control_mode": "joint_position_state_1",
            "libkine_used": False,
            "velocity_ratios": (
                None if feedback is None else feedback.velocity_ratios
            ),
            "acceleration_ratios": (
                None
                if feedback is None
                else feedback.acceleration_ratios
            ),
        }
        self._status_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def close_hardware(self) -> None:
        session = self._session
        self._session = None
        if session is not None:
            session.shutdown()


def _remove_confirmation(arguments) -> tuple[bool, list[str]]:
    confirmed = False
    remaining = []
    for argument in arguments:
        if argument == "--confirm-real":
            confirmed = True
        else:
            remaining.append(argument)
    return confirmed, remaining


def main(args=None) -> int:
    arguments = list(sys.argv[1:] if args is None else args)
    confirmed, ros_arguments = _remove_confirmation(arguments)
    if not confirmed:
        print(
            "拒绝启动：marvin_hardware_bridge 必须提供 --confirm-real",
            file=sys.stderr,
        )
        return 2

    rclpy.init(args=ros_arguments)
    node = None
    try:
        node = MarvinHardwareBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.close_hardware()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
