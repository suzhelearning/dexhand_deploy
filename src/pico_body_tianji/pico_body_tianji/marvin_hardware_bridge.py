from __future__ import annotations

import json
import logging
import sys
import threading
import time

import numpy as np

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
from .zenoh_util import (
    ZenohJsonSub,
    ZenohPub,
    ZenohTextSub,
    key,
    load_node_config,
    open_session,
    parse_cli_args,
    parse_param_override,
    stamp_now,
)

_LOG = logging.getLogger("marvin_hardware_bridge")

SIDES = ("left", "right")
CONFLICTING_CONTROLLER_NAMES = {
    "tianji_world_output_node",
    "tianji_arm_node",
}
OUTPUT_STEP_REFERENCE_VELOCITY_RATIO = 10

DEFAULT_PARAMETERS = {
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
    "left_home_deg": [55.0, -65.0, -70.0, -60.0, 60.0, 0.0, 0.0],
    "right_home_deg": [-55.0, -65.0, 70.0, -60.0, -60.0, 0.0, 0.0],
    "lower_limits_deg": [
        -165.0, -115.0, -165.0, -140.0, -165.0, -55.0, -85.0,
    ],
    "upper_limits_deg": [165.0, 115.0, 165.0, 55.0, 165.0, 55.0, 85.0],
}


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


class MarvinHardwareBridge:
    """主机 IK 关节流到 Marvin 双臂的真机安全桥。"""

    def __init__(self, session, params: dict):
        self._session = session
        self._log = _LOG
        self._params = params

        self._rate_hz = float(params["rate"])
        if self._rate_hz <= 0.0:
            raise ValueError("rate 必须为正数")
        self._command_timeout_s = float(params["command_timeout_s"])
        self._left_home = self._parameter_vector("left_home_deg")
        self._right_home = self._parameter_vector("right_home_deg")
        self._lower_limits = self._parameter_vector("lower_limits_deg")
        self._upper_limits = self._parameter_vector("upper_limits_deg")
        self._robot_ip = str(params["robot_ip"])
        if not self._robot_ip:
            raise ValueError("robot_ip 必须配置为 Marvin 控制器地址")
        velocity_ratio = int(params["velocity_ratio"])
        self._maximum_output_step_deg = _scaled_output_step_deg(
            configured_step_deg=float(params["maximum_output_step_deg"]),
            velocity_ratio=velocity_ratio,
            reference_velocity_ratio=OUTPUT_STEP_REFERENCE_VELOCITY_RATIO,
            rate_hz=self._rate_hz,
            maximum_teleop_speed_deg_s=float(
                params["maximum_teleop_speed_deg_s"]
            ),
        )

        self._readiness = HostReadinessGate(
            left_home_deg=self._left_home,
            right_home_deg=self._right_home,
            freshness_timeout_s=float(params["host_status_timeout_s"]),
            command_timeout_s=float(params["command_timeout_s"]),
            maximum_pair_skew_s=float(params["maximum_pair_skew_s"]),
            home_tolerance_deg=float(params["home_tolerance_deg"]),
            input_mode=str(params["host_input_mode"]),
        )
        settings = HardwareSafetySettings(
            command_timeout_s=float(params["command_timeout_s"]),
            state_timeout_s=float(params["state_timeout_s"]),
            feedback_timeout_s=float(params["feedback_timeout_s"]),
            maximum_pair_skew_s=float(params["maximum_pair_skew_s"]),
            maximum_output_step_deg=self._maximum_output_step_deg,
            maximum_tracking_error_deg=float(
                params["maximum_tracking_error_deg"]
            ),
            return_minimum_duration_s=float(
                params["return_minimum_duration_s"]
            ),
            return_max_speed_deg_s=float(
                params["return_max_speed_deg_s"]
            ),
            home_tolerance_deg=float(params["home_tolerance_deg"]),
        )
        self._safety = HardwareSafetyController(
            left_home_deg=self._left_home,
            right_home_deg=self._right_home,
            lower_limits_deg=self._lower_limits,
            upper_limits_deg=self._upper_limits,
            settings=settings,
        )

        self._marvin: MarvinHardwareSession | None = None
        self._phase = "waiting_for_safe_host"
        self._readiness_reason = "not_evaluated"
        self._last_action = "none"
        self._last_error = None
        self._tracking_error_detail = None
        self._command_count = 0
        self._latest_feedback: MarvinFeedback | None = None
        self._live_cache: set | None = None
        self._live_cache_at = 0.0
        self._reset_runtime_diagnostics()
        self._create_zenoh_interfaces()
        self._log.warning(
            "真机桥已确认启动，但尚未连接 Marvin；"
            "等待同机 IK 链路处于 idle 安全零位。"
            "连接时会自动清除已释放的历史锁存错误；"
            "仍然生效的实体急停或安全链不会被绕过。"
            f"指令斜坡上限={self._maximum_output_step_deg:.3f}°/帧"
            f"（{self._maximum_output_step_deg * self._rate_hz:.2f}°/s）。"
        )

    def _parameter_vector(self, name: str) -> np.ndarray:
        values = np.asarray(self._params[name], dtype=np.float64)
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError(f"{name} 必须包含 7 个有限数值")
        return values

    def _reset_runtime_diagnostics(self) -> None:
        self._decision_counts: dict[str, int] = {}
        self._last_sent_output: np.ndarray | None = None
        self._last_output_step_deg = 0.0
        self._maximum_observed_output_step_deg = 0.0
        self._last_tracking_error_abs_deg = np.zeros(14, dtype=np.float64)
        self._maximum_tracking_error_abs_deg = np.zeros(
            14, dtype=np.float64
        )
        self._tick_count = 0
        self._first_tick_started_at: float | None = None
        self._last_tick_started_at: float | None = None
        self._last_tick_interval_ms = 0.0
        self._maximum_tick_interval_ms = 0.0
        self._deadline_miss_count = 0

    def _observe_tick_timing(self, started_at: float) -> None:
        if self._first_tick_started_at is None:
            self._first_tick_started_at = started_at
        if self._last_tick_started_at is not None:
            interval_s = started_at - self._last_tick_started_at
            self._last_tick_interval_ms = 1000.0 * interval_s
            self._maximum_tick_interval_ms = max(
                self._maximum_tick_interval_ms,
                self._last_tick_interval_ms,
            )
            if interval_s > 1.5 / self._rate_hz:
                self._deadline_miss_count += 1
        self._last_tick_started_at = started_at
        self._tick_count += 1

    def _observe_decision(self, decision, feedback: MarvinFeedback) -> None:
        key = f"{decision.action}:{decision.reason}"
        self._decision_counts[key] = self._decision_counts.get(key, 0) + 1
        if (
            decision.left_joints_deg is None
            or decision.right_joints_deg is None
        ):
            return
        output = np.concatenate(
            [decision.left_joints_deg, decision.right_joints_deg]
        )
        if self._last_sent_output is not None:
            self._last_output_step_deg = float(
                np.max(np.abs(output - self._last_sent_output), initial=0.0)
            )
            self._maximum_observed_output_step_deg = max(
                self._maximum_observed_output_step_deg,
                self._last_output_step_deg,
            )
        self._last_sent_output = output.copy()
        measured = np.concatenate(
            [feedback.left_joints_deg, feedback.right_joints_deg]
        )
        self._last_tracking_error_abs_deg = np.abs(output - measured)
        self._maximum_tracking_error_abs_deg = np.maximum(
            self._maximum_tracking_error_abs_deg,
            self._last_tracking_error_abs_deg,
        )

    def _create_zenoh_interfaces(self) -> None:
        for side in SIDES:
            ZenohJsonSub(
                self._session,
                key(f"/pico_body_sim/{side}_arm/joint_commands"),
                lambda data, side=side: self._on_command(side, data),
            )
        ZenohTextSub(
            self._session,
            key("/pico_body/teleop_state"),
            self._on_teleop_state,
        )
        ZenohTextSub(
            self._session,
            key("/pico_body/status"),
            self._on_input_status,
        )
        ZenohTextSub(
            self._session,
            key("/pico_body_sim/status"),
            self._on_sim_status,
        )
        self._feedback_publishers = {
            side: ZenohPub(self._session, key(f"/{side}_arm/joint_states"))
            for side in SIDES
        }
        self._status_publisher = ZenohPub(
            self._session, key("/pico_body_real/status")
        )

    def _on_command(self, side: str, data: dict) -> None:
        now = time.monotonic()
        stamp = data.get("stamp") or {}
        stamp_ns = (
            int(stamp.get("sec", 0)) * 1_000_000_000
            + int(stamp.get("nanosec", 0))
        )
        if not _source_stamp_is_fresh(
            stamp_ns,
            time.time_ns(),
            self._command_timeout_s,
        ):
            self._last_error = f"stale_{side}_command_stamp"
            return
        try:
            self._readiness.observe_command(
                side,
                data["position"],
                frame_id=data.get("frame_id"),
                received_at=now,
            )
            self._safety.observe_command(
                side,
                data["position"],
                frame_id=data.get("frame_id"),
                received_at=now,
            )
        except ValueError as exc:
            self._last_error = f"invalid_{side}_command: {exc}"

    def _on_teleop_state(self, state: str) -> None:
        self._observe_state(state, time.monotonic())

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

    def _on_input_status(self, text: str) -> None:
        now = time.monotonic()
        try:
            self._readiness.observe_input_status(text, received_at=now)
            payload = json.loads(text)
            state = payload.get("state")
            if isinstance(state, str):
                self._observe_state(state, now)
        except (ValueError, json.JSONDecodeError) as exc:
            self._last_error = f"invalid_input_status: {exc}"

    def _on_sim_status(self, text: str) -> None:
        try:
            self._readiness.observe_sim_status(
                text, received_at=time.monotonic()
            )
        except ValueError as exc:
            self._last_error = f"invalid_sim_status: {exc}"

    def _live_controller_names(self) -> set:
        """liveliness 查询 tj/live/*，2 秒缓存（替代 ros2 node list）。"""
        now = time.monotonic()
        if self._live_cache is not None and now - self._live_cache_at < 2.0:
            return self._live_cache
        names: set = set()
        done = threading.Event()

        def handler(reply) -> None:
            if reply.ok:
                names.add(
                    str(reply.result.key_expr).rsplit("/", 1)[-1]
                )
            done.set()

        try:
            self._session.liveliness().get(
                "tj/live/*", handler, timeout=1.0
            )
            done.wait(1.2)
        except Exception:
            pass
        self._live_cache = names
        self._live_cache_at = now
        return names

    def _tick(self) -> None:
        if self._phase == "waiting_for_safe_host":
            self._try_start_hardware()
            return
        if self._marvin is None:
            return
        self._observe_tick_timing(time.monotonic())
        try:
            feedback = self._marvin.read_feedback()
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
            self._observe_decision(decision, feedback)
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
                        self._log.error(
                            "跟踪误差保护触发："
                            f"{detail.side} J{detail.joint_index}，"
                            f"commanded={detail.commanded_deg:.3f}°, "
                            f"measured={detail.measured_deg:.3f}°, "
                            f"error={detail.signed_error_deg:+.3f}°"
                        )
                self._trip_soft_stop(decision.reason)
                return
            self._marvin.send_joint_targets(
                decision.left_joints_deg,
                decision.right_joints_deg,
            )
            self._command_count += 1
        except BaseException as exc:
            self._trip_soft_stop(f"runtime_error: {exc}")

    def _try_start_hardware(self) -> None:
        conflicts = (
            self._live_controller_names() & CONFLICTING_CONTROLLER_NAMES
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
            self._marvin = create_official_marvin_session()
            feedback = self._marvin.connect_and_prepare(
                self._robot_ip,
                velocity_ratio=int(self._params["velocity_ratio"]),
                acceleration_ratio=int(
                    self._params["acceleration_ratio"]
                ),
                lower_limits_deg=self._lower_limits,
                upper_limits_deg=self._upper_limits,
                hard_limit_padding_deg=float(
                    self._params["feedback_hard_limit_padding_deg"]
                ),
            )
            self._latest_feedback = feedback
            feedback = self._marvin.move_to_home(
                self._left_home,
                self._right_home,
                rate_hz=self._rate_hz,
                minimum_duration_s=float(
                    self._params["return_minimum_duration_s"]
                ),
                max_speed_deg_s=float(
                    self._params["return_max_speed_deg_s"]
                ),
                maximum_tracking_error_deg=float(
                    self._params["maximum_tracking_error_deg"]
                ),
                home_tolerance_deg=float(
                    self._params["home_tolerance_deg"]
                ),
                lower_limits_deg=self._lower_limits,
                upper_limits_deg=self._upper_limits,
                hard_limit_padding_deg=float(
                    self._params["feedback_hard_limit_padding_deg"]
                ),
                required_state=1,
                feedback_timeout_s=float(
                    self._params["feedback_timeout_s"]
                ),
            )
            self._phase = "waiting_for_post_home_snapshot"
            self._reset_runtime_diagnostics()
            self._last_error = None
            self._log.warning(
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
        self._marvin.send_joint_targets(
            self._left_home, self._right_home
        )
        self._command_count += 1
        if readiness.ready:
            self._phase = "armed_idle"
            self._log.warning(
                "真机链路已就绪：保持安全零位，主机开始遥操作后跟随"
                "（PICO 按右手柄 A / mocap 键盘按 s）。"
            )

    def _fail_startup(self, error: BaseException) -> None:
        self._last_error = f"startup_error: {error}"
        session = self._marvin
        if session is not None:
            try:
                session.soft_stop_once()
            except BaseException:
                pass
            try:
                session.shutdown()
            except BaseException:
                pass
        self._marvin = None
        self._phase = "failed"
        self._log.error(self._last_error)

    def _trip_soft_stop(self, reason: str) -> None:
        self._last_action = f"soft_stop:{reason}"
        self._last_error = reason
        session = self._marvin
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
        self._marvin = None
        self._phase = "soft_stopped"
        self._log.error(
            f"真机链路已锁存软急停并释放连接：{self._last_error}"
        )

    def _publish_feedback(self, feedback: MarvinFeedback) -> None:
        stamp = stamp_now()
        for side, joints in (
            ("left", feedback.left_joints_deg),
            ("right", feedback.right_joints_deg),
        ):
            message = {
                "stamp": stamp,
                "frame_id": f"{side}_base_marvin_degrees_measured",
                "name": [f"{side}_joint_{index}" for index in range(1, 8)],
                "position": joints.tolist(),
            }
            self._feedback_publishers[side].put_json(message)

    def _publish_status(self) -> None:
        feedback = self._latest_feedback
        observed_duration_s = (
            0.0
            if self._first_tick_started_at is None
            or self._last_tick_started_at is None
            else self._last_tick_started_at - self._first_tick_started_at
        )
        payload = {
            "phase": self._phase,
            "readiness": self._readiness_reason,
            "last_action": self._last_action,
            "error": self._last_error,
            "robot_connected": self._marvin is not None,
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
            "decision_counts": dict(self._decision_counts),
            "last_output_step_deg": self._last_output_step_deg,
            "maximum_observed_output_step_deg": (
                self._maximum_observed_output_step_deg
            ),
            "last_tracking_error_abs_deg": (
                self._last_tracking_error_abs_deg.tolist()
            ),
            "maximum_tracking_error_abs_deg": (
                self._maximum_tracking_error_abs_deg.tolist()
            ),
            "observed_tick_rate_hz": (
                (self._tick_count - 1) / observed_duration_s
                if self._tick_count >= 2 and observed_duration_s > 0.0
                else None
            ),
            "last_tick_interval_ms": self._last_tick_interval_ms,
            "maximum_tick_interval_ms": self._maximum_tick_interval_ms,
            "deadline_miss_count": self._deadline_miss_count,
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
        self._status_publisher.put_text(
            json.dumps(payload, ensure_ascii=False)
        )

    def run(self) -> None:
        """主循环：rate Hz 控制 tick + 0.5 s 状态。"""
        tick_interval = 1.0 / self._rate_hz
        status_interval = 0.5
        next_tick = time.monotonic() + tick_interval
        next_status = next_tick + status_interval
        while True:
            now = time.monotonic()
            if now >= next_tick:
                self._tick()
                next_tick += tick_interval
            if now >= next_status:
                self._publish_status()
                next_status += status_interval
            time.sleep(
                max(0.001, min(next_tick, next_status) - time.monotonic())
            )

    def close_hardware(self) -> None:
        session = self._marvin
        self._marvin = None
        if session is not None:
            session.shutdown()

    def close(self) -> None:
        self.close_hardware()
        self._session.close()


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
    confirmed, remaining = _remove_confirmation(arguments)
    if not confirmed:
        print(
            "拒绝启动：marvin_hardware_bridge 必须提供 --confirm-real",
            file=sys.stderr,
        )
        return 2

    parsed = parse_cli_args(argv=remaining)
    overrides = {}
    for spec in parsed.param:
        k, v = parse_param_override(spec)
        overrides[k] = v
    params = load_node_config(
        parsed.config,
        "marvin_hardware_bridge",
        DEFAULT_PARAMETERS,
        overrides,
    )
    session = open_session()
    node = None
    try:
        node = MarvinHardwareBridge(session, params)
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.close_hardware()
            finally:
                node.close()
    return 0


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    raise SystemExit(main())
