from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable

import numpy as np

from .home_trajectory import HomeTrajectory
from .marvin_state import command_states_compatible


class MarvinHardwareError(RuntimeError):
    """Marvin 控制会话中的可恢复边界错误。"""


@dataclass(frozen=True)
class MarvinFeedback:
    left_joints_deg: np.ndarray
    right_joints_deg: np.ndarray
    arm_states: tuple[int, int]
    command_states: tuple[int, int]
    error_codes: tuple[int, int]
    frame_serials: tuple[int, int]
    velocity_ratios: tuple[int, int]
    acceleration_ratios: tuple[int, int]
    servo_error_reports: tuple[str, str]


class MarvinHardwareSession:
    """只包装 Marvin 关节控制，不加载官方 IK/libKine。"""

    _CLEAR_SET_ATTEMPTS = 3
    _CLEAR_SET_RETRY_DELAY_S = 0.002

    def __init__(
        self,
        *,
        robot,
        dcss_factory: Callable[[], object],
        sleep: Callable[[float], None] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
    ):
        self._robot = robot
        self._dcss = dcss_factory()
        self._sleep = sleep
        self._monotonic = monotonic
        self._connected = False
        self._soft_stopped = False

    @staticmethod
    def _joints(values, label: str) -> list[float]:
        joints = np.asarray(values, dtype=np.float64)
        if joints.shape != (7,) or not np.isfinite(joints).all():
            raise ValueError(f"{label} must contain seven finite values")
        return joints.tolist()

    @staticmethod
    def _require_success(result, operation: str) -> None:
        # 真机已验证 SDK 的 setter/send 接口约定为 1=成功、0=失败；
        # 只接受明确的 1，避免 None 或其他异常返回被当成成功。
        if result != 1:
            raise MarvinHardwareError(f"Marvin SDK call failed: {operation}")

    def _begin_command_batch(self) -> None:
        # SDK 的 1 kHz 指令缓冲在上一批尚未消费时会让 OnClearSet
        # 瞬时返回 0。clear_set 之前还没有写入本批目标，因此只在这个
        # 无副作用边界做短暂、有限重试；持续失败仍由上层锁存软急停。
        last_result = None
        for attempt in range(self._CLEAR_SET_ATTEMPTS):
            last_result = self._robot.clear_set()
            if last_result == 1:
                return
            if attempt + 1 < self._CLEAR_SET_ATTEMPTS:
                self._sleep(self._CLEAR_SET_RETRY_DELAY_S)
        raise MarvinHardwareError(
            "Marvin SDK call failed: clear_set after "
            f"{self._CLEAR_SET_ATTEMPTS} attempts "
            f"(last_result={last_result!r})"
        )

    def read_feedback(
        self, *, include_servo_errors: bool = False
    ) -> MarvinFeedback:
        try:
            payload = self._robot.subscribe(self._dcss)
            states = payload["states"]
            outputs = payload["outputs"]
            inputs = payload["inputs"]
            left = self._joints(
                outputs[0]["fb_joint_pos"], "left feedback"
            )
            right = self._joints(
                outputs[1]["fb_joint_pos"], "right feedback"
            )
            arm_states = (
                int(states[0]["cur_state"]),
                int(states[1]["cur_state"]),
            )
            command_states = (
                int(states[0]["cmd_state"]),
                int(states[1]["cmd_state"]),
            )
            error_codes = (
                int(states[0]["err_code"]),
                int(states[1]["err_code"]),
            )
            frame_serials = (
                int(outputs[0]["frame_serial"]),
                int(outputs[1]["frame_serial"]),
            )
            velocity_ratios = (
                int(inputs[0]["joint_vel_ratio"]),
                int(inputs[1]["joint_vel_ratio"]),
            )
            acceleration_ratios = (
                int(inputs[0]["joint_acc_ratio"]),
                int(inputs[1]["joint_acc_ratio"]),
            )
            servo_error_reports = tuple(
                self._servo_error_report(arm)
                if include_servo_errors or error_codes[index] == 2
                else "None"
                for index, arm in enumerate(("A", "B"))
            )
        except (IndexError, KeyError, TypeError, ValueError) as exc:
            raise MarvinHardwareError(
                f"invalid Marvin feedback: {exc}"
            ) from exc
        return MarvinFeedback(
            left_joints_deg=np.asarray(left, dtype=np.float64),
            right_joints_deg=np.asarray(right, dtype=np.float64),
            arm_states=arm_states,
            command_states=command_states,
            error_codes=error_codes,
            frame_serials=frame_serials,
            velocity_ratios=velocity_ratios,
            acceleration_ratios=acceleration_ratios,
            servo_error_reports=servo_error_reports,
        )

    def _servo_error_report(self, arm: str) -> str:
        report = self._robot.get_servo_error_code(arm, lang="EN")
        if not isinstance(report, str) or not report.strip():
            raise MarvinHardwareError(
                f"invalid Marvin servo error report for arm {arm}"
            )
        return report.strip()

    def send_joint_targets(self, left_joints_deg, right_joints_deg) -> None:
        if self._soft_stopped:
            raise MarvinHardwareError(
                "Marvin session is soft-stopped; restart is required"
            )
        left = self._joints(left_joints_deg, "left_joints_deg")
        right = self._joints(right_joints_deg, "right_joints_deg")
        self._begin_command_batch()
        self._require_success(
            self._robot.set_joint_cmd_pose("A", left),
            "set_joint_cmd_pose(A)",
        )
        self._require_success(
            self._robot.set_joint_cmd_pose("B", right),
            "set_joint_cmd_pose(B)",
        )
        self._require_success(self._robot.send_cmd(), "send_cmd")

    def connect_and_prepare(
        self,
        robot_ip: str,
        *,
        velocity_ratio: int = 10,
        acceleration_ratio: int = 10,
        verification_timeout_s: float = 1.0,
        transition_timeout_s: float = 5.0,
        poll_interval_s: float = 0.05,
        lower_limits_deg=None,
        upper_limits_deg=None,
        hard_limit_padding_deg: float = 5.0,
    ) -> MarvinFeedback:
        self._validate_ratio(velocity_ratio, "velocity_ratio")
        self._validate_ratio(acceleration_ratio, "acceleration_ratio")
        hard_limits = self._hard_limit_bounds(
            lower_limits_deg,
            upper_limits_deg,
            hard_limit_padding_deg,
        )
        self._require_success(
            self._robot.connect(robot_ip), "connect"
        )
        self._connected = True
        self._soft_stopped = False
        try:
            # 官方控制器在 connect 后留出 0.5 秒，再开始清错和使能。
            self._sleep(0.5)
            # 厂家 check_error_and_clear helper 内部使用 if/elif：
            # 如果 A 清错后因外部安全输入立即重锁，下一次仍会只选 A，
            # B 将永远得不到清错。启动时直接调用官方的单臂清错接口，
            # A/B 各执行一次；返回值语义不可靠，后面以新鲜反馈为准。
            for arm in ("A", "B"):
                self._robot.clear_error(arm)
                self._sleep(0.5)
            self._verify_feedback_advances(
                timeout_s=verification_timeout_s,
                poll_interval_s=poll_interval_s,
            )
            measured = self.read_feedback(include_servo_errors=True)
            self._require_error_free_feedback(
                measured, "before position-mode enable"
            )
            self._require_feedback_within_hard_limits(
                measured,
                hard_limits,
            )
            self._configure_velocity_limits(
                velocity_ratio=velocity_ratio,
                acceleration_ratio=acceleration_ratio,
            )
            measured = self._wait_for_feedback_after(
                measured.frame_serials,
                timeout_s=verification_timeout_s,
                poll_interval_s=poll_interval_s,
                include_servo_errors=True,
            )
            self._require_error_free_feedback(
                measured, "immediately before position-mode enable"
            )
            self._require_feedback_within_hard_limits(
                measured, hard_limits
            )
            self._enter_position_control_at_measured_pose(
                measured,
            )
            prepared = self._wait_for_state(
                target_state=1,
                timeout_s=transition_timeout_s,
                poll_interval_s=poll_interval_s,
                expected_velocity_ratio=velocity_ratio,
                expected_acceleration_ratio=acceleration_ratio,
                hard_limits=hard_limits,
            )
            return prepared
        except BaseException:
            self._release_after_prepare_failure()
            raise

    @staticmethod
    def _validate_ratio(value: int, label: str) -> None:
        if isinstance(value, bool) or int(value) != value:
            raise ValueError(f"{label} must be an integer")
        if not 1 <= int(value) <= 100:
            raise ValueError(f"{label} must be between 1 and 100")

    @staticmethod
    def _ratio_feedback_matches(
        measured: tuple[int, int],
        expected: int | None,
    ) -> bool:
        if expected is None:
            return True
        # 控制器内部百分比会经过整数编码，30% 等值可能向下回读为
        # 29%。仅容许最多低 1%，绝不接受高于请求值的反馈。
        return all(
            int(expected) - 1 <= value <= int(expected)
            for value in measured
        )

    def _configure_velocity_limits(
        self,
        *,
        velocity_ratio: int,
        acceleration_ratio: int,
    ) -> None:
        self._begin_command_batch()
        for arm in ("A", "B"):
            self._require_success(
                self._robot.set_vel_acc(
                    arm, velocity_ratio, acceleration_ratio
                ),
                f"set_vel_acc({arm})",
            )
        self._require_success(self._robot.send_cmd(), "send_cmd")
        self._sleep(0.1)

    def _verify_feedback_advances(
        self, *, timeout_s: float, poll_interval_s: float
    ) -> None:
        if timeout_s <= 0.0 or poll_interval_s < 0.0:
            raise ValueError("feedback verification timings are invalid")
        first = self.read_feedback().frame_serials
        self._wait_for_feedback_after(
            first,
            timeout_s=timeout_s,
            poll_interval_s=poll_interval_s,
        )

    def _wait_for_feedback_after(
        self,
        previous_serials: tuple[int, int],
        *,
        timeout_s: float,
        poll_interval_s: float,
        include_servo_errors: bool = False,
    ) -> MarvinFeedback:
        deadline = self._monotonic() + float(timeout_s)
        while True:
            current = self.read_feedback(
                include_servo_errors=include_servo_errors
            )
            if all(
                serial != 0 and serial != previous_serials[index]
                for index, serial in enumerate(current.frame_serials)
            ):
                return current
            if self._monotonic() >= deadline:
                break
            self._sleep(float(poll_interval_s))
        raise MarvinHardwareError(
            "Marvin feedback frame serial did not advance"
        )

    def _enter_position_control_at_measured_pose(
        self,
        feedback: MarvinFeedback,
    ) -> None:
        self._begin_command_batch()
        for arm, joints in (
            ("A", feedback.left_joints_deg.tolist()),
            ("B", feedback.right_joints_deg.tolist()),
        ):
            self._require_success(
                self._robot.set_joint_cmd_pose(arm, joints),
                f"set_joint_cmd_pose({arm})",
            )
        for arm in ("A", "B"):
            self._require_success(
                self._robot.set_state(arm, 1),
                f"set_state({arm}, 1)",
            )
        self._require_success(self._robot.send_cmd(), "send_cmd")

    def _wait_for_state(
        self,
        *,
        target_state: int,
        timeout_s: float,
        poll_interval_s: float,
        expected_velocity_ratio: int | None = None,
        expected_acceleration_ratio: int | None = None,
        hard_limits: tuple[np.ndarray, np.ndarray] | None = None,
    ) -> MarvinFeedback:
        if timeout_s <= 0.0 or poll_interval_s < 0.0:
            raise ValueError("state transition timings are invalid")
        deadline = self._monotonic() + float(timeout_s)
        last_feedback = None
        while True:
            last_feedback = self.read_feedback()
            if last_feedback.servo_error_reports != ("None", "None"):
                raise MarvinHardwareError(
                    "servo error during state transition: "
                    f"{last_feedback.servo_error_reports}"
                )
            self._require_feedback_within_hard_limits(
                last_feedback, hard_limits
            )
            if (
                last_feedback.arm_states
                == (target_state, target_state)
                and command_states_compatible(
                    last_feedback.command_states,
                    target_state,
                )
                and last_feedback.error_codes == (0, 0)
                and self._ratio_feedback_matches(
                    last_feedback.velocity_ratios,
                    expected_velocity_ratio,
                )
                and self._ratio_feedback_matches(
                    last_feedback.acceleration_ratios,
                    expected_acceleration_ratio,
                )
            ):
                return last_feedback
            transient = all(
                self._state_can_still_reach_target(
                    state=state,
                    error=error,
                )
                for state, error in zip(
                    last_feedback.arm_states,
                    last_feedback.error_codes,
                )
            )
            if not transient:
                raise MarvinHardwareError(
                    "unexpected Marvin state transition feedback: "
                    f"states={last_feedback.arm_states}, "
                    f"commands={last_feedback.command_states}, "
                    f"errors={last_feedback.error_codes}"
                )
            if self._monotonic() >= deadline:
                break
            self._sleep(float(poll_interval_s))
        states = (
            None
            if last_feedback is None
            else last_feedback.arm_states
        )
        errors = (
            None
            if last_feedback is None
            else last_feedback.error_codes
        )
        commands = (
            None
            if last_feedback is None
            else last_feedback.command_states
        )
        velocity_ratios = (
            None
            if last_feedback is None
            else last_feedback.velocity_ratios
        )
        acceleration_ratios = (
            None
            if last_feedback is None
            else last_feedback.acceleration_ratios
        )
        raise MarvinHardwareError(
            f"Marvin arms did not reach state {target_state}: "
            f"states={states}, commands={commands}, errors={errors}, "
            "velocity/acceleration="
            f"{velocity_ratios}/{acceleration_ratios}"
        )

    @staticmethod
    def _state_can_still_reach_target(*, state: int, error: int) -> bool:
        if error == 0 and state in (0, 1, 2, 3, 101, 102, 103):
            return True
        if state in (101, 102, 103) and error in (4, 6, 8):
            return True
        # 控制器切换状态时可能短暂报告 100/101/102/103。
        return state == 100 and error == 6

    def soft_stop_once(self) -> None:
        if self._soft_stopped or not self._connected:
            return
        # 已验证 Python SDK 将 soft_stop 作为无返回值安全命令；
        # 释放连接后绝不能再次进入该函数。
        self._robot.soft_stop("AB")
        self._soft_stopped = True

    def move_to_home(
        self,
        left_home_deg,
        right_home_deg,
        *,
        rate_hz: float = 30.0,
        minimum_duration_s: float = 2.0,
        max_speed_deg_s: float = 10.0,
        maximum_tracking_error_deg: float = 8.0,
        home_tolerance_deg: float = 1.0,
        lower_limits_deg=None,
        upper_limits_deg=None,
        hard_limit_padding_deg: float = 5.0,
        required_state: int = 1,
        feedback_timeout_s: float = 0.15,
    ) -> MarvinFeedback:
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        if maximum_tracking_error_deg <= 0.0:
            raise ValueError(
                "maximum_tracking_error_deg must be positive"
            )
        if home_tolerance_deg <= 0.0:
            raise ValueError("home_tolerance_deg must be positive")
        if feedback_timeout_s <= 0.0:
            raise ValueError("feedback_timeout_s must be positive")
        hard_limits = self._hard_limit_bounds(
            lower_limits_deg,
            upper_limits_deg,
            hard_limit_padding_deg,
        )
        home = np.concatenate(
            [
                np.asarray(
                    self._joints(left_home_deg, "left_home_deg")
                ),
                np.asarray(
                    self._joints(right_home_deg, "right_home_deg")
                ),
            ]
        )
        feedback = self.read_feedback()
        self._require_healthy_state_feedback(
            feedback, required_state, "before return-home"
        )
        self._require_feedback_within_hard_limits(
            feedback, hard_limits
        )
        start = np.concatenate(
            [
                feedback.left_joints_deg,
                feedback.right_joints_deg,
            ]
        )
        trajectory = HomeTrajectory(
            start_joints=start,
            home_joints=home,
            start_time=self._monotonic(),
            minimum_duration=float(minimum_duration_s),
            max_speed_deg_s=float(max_speed_deg_s),
        )
        period = 1.0 / float(rate_hz)
        last_frame_serials = feedback.frame_serials
        frame_advanced_at = [self._monotonic(), self._monotonic()]
        while True:
            sample = trajectory.sample(self._monotonic())
            self.send_joint_targets(
                sample.joints[:7], sample.joints[7:]
            )
            self._sleep(period)
            feedback = self.read_feedback()
            feedback_now = self._monotonic()
            for index, serial in enumerate(feedback.frame_serials):
                if serial != last_frame_serials[index]:
                    frame_advanced_at[index] = feedback_now
            last_frame_serials = feedback.frame_serials
            if any(
                feedback_now - advanced_at > feedback_timeout_s
                for advanced_at in frame_advanced_at
            ):
                self.soft_stop_once()
                raise MarvinHardwareError(
                    "feedback frame stalled during return-home"
                )
            self._require_healthy_state_feedback(
                feedback, required_state, "during return-home"
            )
            self._require_feedback_within_hard_limits(
                feedback, hard_limits
            )
            measured = np.concatenate(
                [
                    feedback.left_joints_deg,
                    feedback.right_joints_deg,
                ]
            )
            tracking_error = float(
                np.max(np.abs(sample.joints - measured), initial=0.0)
            )
            if tracking_error > maximum_tracking_error_deg:
                self.soft_stop_once()
                raise MarvinHardwareError(
                    "return-home tracking error exceeded safety limit"
                )
            if sample.complete:
                break

        home_error = float(
            np.max(np.abs(home - measured), initial=0.0)
        )
        if home_error > home_tolerance_deg:
            self.soft_stop_once()
            raise MarvinHardwareError(
                "robot did not reach safe home tolerance"
            )
        return feedback

    @staticmethod
    def _hard_limit_bounds(
        lower_limits_deg,
        upper_limits_deg,
        hard_limit_padding_deg: float,
    ) -> tuple[np.ndarray, np.ndarray] | None:
        if (lower_limits_deg is None) != (upper_limits_deg is None):
            raise ValueError("both lower and upper limits are required")
        if lower_limits_deg is None:
            return None
        if hard_limit_padding_deg < 0.0:
            raise ValueError("hard_limit_padding_deg must be nonnegative")
        lower_arm = np.asarray(lower_limits_deg, dtype=np.float64)
        upper_arm = np.asarray(upper_limits_deg, dtype=np.float64)
        if (
            lower_arm.shape not in {(7,), (14,)}
            or upper_arm.shape != lower_arm.shape
            or not np.isfinite(lower_arm).all()
            or not np.isfinite(upper_arm).all()
        ):
            raise ValueError(
                "joint limits must contain seven shared or fourteen "
                "side-specific finite values"
            )
        lower = (
            np.tile(lower_arm, 2)
            if lower_arm.shape == (7,)
            else lower_arm.copy()
        )
        upper = (
            np.tile(upper_arm, 2)
            if upper_arm.shape == (7,)
            else upper_arm.copy()
        )
        if np.any(lower >= upper):
            raise ValueError("lower joint limits must be below upper limits")
        padding = float(hard_limit_padding_deg)
        return lower - padding, upper + padding

    @staticmethod
    def _require_feedback_within_hard_limits(
        feedback: MarvinFeedback,
        hard_limits: tuple[np.ndarray, np.ndarray] | None,
    ) -> None:
        if hard_limits is None:
            return
        measured = np.concatenate(
            [feedback.left_joints_deg, feedback.right_joints_deg]
        )
        lower, upper = hard_limits
        outside = (measured < lower) | (measured > upper)
        if np.any(outside):
            details = []
            for flat_index in np.flatnonzero(outside):
                side, joint = divmod(int(flat_index), 7)
                details.append(
                    f"{'left' if side == 0 else 'right'} J{joint + 1}="
                    f"{measured[flat_index]:.3f} deg outside "
                    f"[{lower[flat_index]:.3f}, {upper[flat_index]:.3f}]"
                )
            raise MarvinHardwareError(
                "measured joints exceed physical hard joint limits: "
                + "; ".join(details)
            )

    @staticmethod
    def _require_error_free_feedback(
        feedback: MarvinFeedback,
        context: str,
    ) -> None:
        if feedback.error_codes != (0, 0):
            raise MarvinHardwareError(
                f"arm error {context}: {feedback.error_codes}"
            )
        if 100 in feedback.arm_states:
            raise MarvinHardwareError(
                f"fault state {context}: {feedback.arm_states}"
            )
        if feedback.servo_error_reports != ("None", "None"):
            raise MarvinHardwareError(
                f"servo error {context}: "
                f"{feedback.servo_error_reports}"
            )

    @staticmethod
    def _require_healthy_state_feedback(
        feedback: MarvinFeedback,
        required_state: int,
        context: str,
    ) -> None:
        MarvinHardwareSession._require_error_free_feedback(
            feedback, context
        )
        if feedback.arm_states != (required_state, required_state):
            raise MarvinHardwareError(
                f"invalid arm state {context}: {feedback.arm_states}; "
                f"required=({required_state}, {required_state})"
            )
        if not command_states_compatible(
            feedback.command_states,
            required_state,
        ):
            raise MarvinHardwareError(
                f"invalid command state {context}: "
                f"{feedback.command_states}; "
                f"required=({required_state}, {required_state}) "
                "or consumed=(-1, -1)"
            )

    def shutdown(
        self,
        *,
        poll_timeout_s: float = 3.0,
        poll_interval_s: float = 0.05,
    ) -> None:
        if not self._connected:
            return
        try:
            try:
                self._require_success(
                    self._robot.clear_set(), "clear_set"
                )
                for arm in ("A", "B"):
                    self._require_success(
                        self._robot.set_state(arm, 0),
                        f"set_state({arm}, 0)",
                    )
                self._require_success(
                    self._robot.send_cmd(), "send_cmd"
                )
            except BaseException:
                pass
            try:
                self._wait_for_state(
                    target_state=0,
                    timeout_s=max(0.5, float(poll_timeout_s)),
                    poll_interval_s=float(poll_interval_s),
                )
            except BaseException:
                pass
        finally:
            try:
                self._robot.release_robot()
            finally:
                self._connected = False

    def _release_after_prepare_failure(self) -> None:
        if not self._connected:
            return
        try:
            self.soft_stop_once()
        except BaseException:
            pass
        try:
            self._require_success(
                self._robot.clear_set(), "clear_set"
            )
            self._require_success(
                self._robot.set_state("A", 0), "set_state(A, 0)"
            )
            self._require_success(
                self._robot.set_state("B", 0), "set_state(B, 0)"
            )
            self._require_success(
                self._robot.send_cmd(), "send_cmd"
            )
        except BaseException:
            pass
        try:
            self._wait_for_state(
                target_state=0,
                timeout_s=0.5,
                poll_interval_s=0.05,
            )
        except BaseException:
            pass
        try:
            self._robot.release_robot()
        except BaseException:
            pass
        self._connected = False
