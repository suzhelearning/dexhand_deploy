from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .home_trajectory import HomeTrajectory
from .marvin_state import command_states_compatible


SIDES = ("left", "right")


@dataclass(frozen=True)
class HardwareSafetySettings:
    command_timeout_s: float = 0.1
    state_timeout_s: float = 0.1
    feedback_timeout_s: float = 0.1
    maximum_pair_skew_s: float = 0.03
    maximum_output_step_deg: float = 0.5
    maximum_tracking_error_deg: float = 8.0
    return_minimum_duration_s: float = 2.0
    return_max_speed_deg_s: float = 10.0
    home_tolerance_deg: float = 1.0
    feedback_hard_limit_padding_deg: float = 5.0


@dataclass(frozen=True)
class HardwareDecision:
    action: str
    reason: str
    left_joints_deg: np.ndarray | None = None
    right_joints_deg: np.ndarray | None = None


@dataclass(frozen=True)
class TrackingErrorDetail:
    side: str
    joint_index: int
    commanded_deg: float
    measured_deg: float
    signed_error_deg: float
    absolute_error_deg: float


@dataclass(frozen=True)
class _TimedJoints:
    joints_deg: np.ndarray
    received_at: float


@dataclass(frozen=True)
class _Feedback:
    joints_deg: np.ndarray
    arm_states: tuple[int, int]
    command_states: tuple[int, int]
    error_codes: tuple[int, int]
    servo_error_reports: tuple[str, str]
    received_at: float


class HardwareSafetyController:
    """真机输出前的纯安全决策；不持有或调用 Marvin SDK。"""

    def __init__(
        self,
        *,
        left_home_deg,
        right_home_deg,
        lower_limits_deg,
        upper_limits_deg,
        settings: HardwareSafetySettings | None = None,
    ):
        self.settings = settings or HardwareSafetySettings()
        positive_settings = (
            self.settings.command_timeout_s,
            self.settings.state_timeout_s,
            self.settings.feedback_timeout_s,
            self.settings.maximum_output_step_deg,
            self.settings.maximum_tracking_error_deg,
            self.settings.return_minimum_duration_s,
            self.settings.return_max_speed_deg_s,
            self.settings.home_tolerance_deg,
        )
        if any(value <= 0.0 for value in positive_settings):
            raise ValueError("hardware safety settings must be positive")
        if (
            self.settings.maximum_tracking_error_deg
            <= self.settings.maximum_output_step_deg
        ):
            raise ValueError(
                "maximum_tracking_error_deg must exceed "
                "maximum_output_step_deg"
            )
        if (
            self.settings.maximum_pair_skew_s < 0.0
            or self.settings.feedback_hard_limit_padding_deg < 0.0
        ):
            raise ValueError("hardware safety tolerances are invalid")
        self._home = np.concatenate(
            [
                self._arm_vector(left_home_deg, "left_home_deg"),
                self._arm_vector(right_home_deg, "right_home_deg"),
            ]
        )
        self._lower = np.tile(
            self._arm_vector(lower_limits_deg, "lower_limits_deg"), 2
        )
        self._upper = np.tile(
            self._arm_vector(upper_limits_deg, "upper_limits_deg"), 2
        )
        if np.any(self._lower >= self._upper):
            raise ValueError("lower joint limits must be below upper limits")
        if np.any(self._home < self._lower) or np.any(
            self._home > self._upper
        ):
            raise ValueError("safe home must be inside joint limits")
        self._commands: dict[str, _TimedJoints | None] = {
            side: None for side in SIDES
        }
        self._feedback: _Feedback | None = None
        self._teleop_state: tuple[str, float] | None = None
        self._last_output = self._home.copy()
        self._return_trajectory: HomeTrajectory | None = None
        self._soft_stop_latched = False
        self._feedback_frame_serials: tuple[int, int] | None = None
        self._feedback_frame_advanced_at: list[float | None] = [
            None,
            None,
        ]

    @staticmethod
    def _arm_vector(values, label: str) -> np.ndarray:
        vector = np.asarray(values, dtype=np.float64)
        if vector.shape != (7,) or not np.isfinite(vector).all():
            raise ValueError(f"{label} must contain seven finite values")
        return vector.copy()

    def observe_command(
        self,
        side: str,
        joints_deg,
        *,
        received_at: float,
        frame_id: str,
    ) -> None:
        if side not in SIDES:
            raise ValueError(f"unsupported side: {side!r}")
        expected_frame = f"{side}_base_marvin_degrees"
        if frame_id != expected_frame:
            raise ValueError(
                f"{side} command frame must be {expected_frame!r}"
            )
        self._commands[side] = _TimedJoints(
            self._arm_vector(joints_deg, f"{side} command"),
            float(received_at),
        )

    def observe_teleop_state(self, state: str, *, received_at: float) -> None:
        if state not in {"idle", "teleop", "returning"}:
            raise ValueError(f"unsupported teleop state: {state!r}")
        self._teleop_state = (state, float(received_at))

    def observe_feedback(
        self,
        *,
        left_joints_deg,
        right_joints_deg,
        arm_states,
        error_codes,
        command_states=None,
        servo_error_reports=("None", "None"),
        frame_serials=None,
        received_at: float,
    ) -> None:
        states = tuple(int(value) for value in arm_states)
        commands = (
            states
            if command_states is None
            else tuple(int(value) for value in command_states)
        )
        errors = tuple(int(value) for value in error_codes)
        servo_errors = tuple(str(value) for value in servo_error_reports)
        if (
            len(states) != 2
            or len(commands) != 2
            or len(errors) != 2
            or len(servo_errors) != 2
        ):
            raise ValueError("feedback must contain two arm states and errors")
        self._feedback = _Feedback(
            joints_deg=np.concatenate(
                [
                    self._arm_vector(left_joints_deg, "left feedback"),
                    self._arm_vector(right_joints_deg, "right feedback"),
                ]
            ),
            arm_states=states,
            command_states=commands,
            error_codes=errors,
            servo_error_reports=servo_errors,
            received_at=float(received_at),
        )
        if frame_serials is not None:
            serials = tuple(int(value) for value in frame_serials)
            if len(serials) != 2:
                raise ValueError(
                    "feedback must contain two frame serials"
                )
            if self._feedback_frame_serials is None:
                self._feedback_frame_advanced_at = [
                    float(received_at),
                    float(received_at),
                ]
            else:
                for index, serial in enumerate(serials):
                    if serial != self._feedback_frame_serials[index]:
                        self._feedback_frame_advanced_at[index] = float(
                            received_at
                        )
            self._feedback_frame_serials = serials

    def decide(self, *, now: float) -> HardwareDecision:
        now = float(now)
        if self._soft_stop_latched:
            return HardwareDecision("soft_stop", "soft_stop_latched")
        unsafe_reason = self._unsafe_reason(now)
        if unsafe_reason is not None:
            return self._latch_soft_stop(unsafe_reason)

        tracking_error = float(
            np.max(
                np.abs(self._last_output - self._feedback.joints_deg),
                initial=0.0,
            )
        )
        if tracking_error > self.settings.maximum_tracking_error_deg:
            return self._latch_soft_stop("tracking_error")
        command_reason = self._command_unavailable_reason(now)
        if (
            command_reason is None
            and self._return_trajectory is not None
            and self._can_rearm_at_home(now)
        ):
            self._return_trajectory = None
            self._last_output = self._home.copy()
            return self._split_decision(
                "hold_home", "idle_rearmed", self._home
            )
        if command_reason is None and self._return_trajectory is None:
            target = np.concatenate(
                [
                    self._commands["left"].joints_deg,
                    self._commands["right"].joints_deg,
                ]
            )
            clipped_target = np.clip(target, self._lower, self._upper)
            target_clipped = not np.array_equal(
                clipped_target, target
            )
            target = clipped_target
            delta = target - self._last_output
            max_delta = float(np.max(np.abs(delta), initial=0.0))
            output_limited = (
                max_delta > self.settings.maximum_output_step_deg
            )
            if output_limited:
                delta *= (
                    self.settings.maximum_output_step_deg / max_delta
                )
            output = self._last_output + delta
            command_lead_deg = (
                self.settings.maximum_tracking_error_deg
                - self.settings.maximum_output_step_deg
            )
            lead_limited_output = np.clip(
                output,
                self._feedback.joints_deg - command_lead_deg,
                self._feedback.joints_deg + command_lead_deg,
            )
            tracking_lead_limited = not np.array_equal(
                lead_limited_output, output
            )
            self._last_output = lead_limited_output
            return self._split_decision(
                "send",
                (
                    "tracking_lead_limited"
                    if tracking_lead_limited
                    else (
                        "command_clipped_and_step_limited"
                        if target_clipped and output_limited
                        else (
                            "command_clipped_to_safe_limits"
                            if target_clipped
                            else (
                                "output_step_limited"
                                if output_limited
                                else "fresh_command"
                            )
                        )
                    )
                ),
                self._last_output,
            )
        return self._return_home(now, command_reason or "return_latched")

    def tracking_error_detail(self) -> TrackingErrorDetail | None:
        """返回上一条已下发关节目标相对最新反馈的最大逐轴误差。"""
        if self._feedback is None:
            return None
        signed_errors = self._last_output - self._feedback.joints_deg
        flat_index = int(np.argmax(np.abs(signed_errors)))
        side_index, joint_offset = divmod(flat_index, 7)
        signed_error = float(signed_errors[flat_index])
        return TrackingErrorDetail(
            side=SIDES[side_index],
            joint_index=joint_offset + 1,
            commanded_deg=float(self._last_output[flat_index]),
            measured_deg=float(self._feedback.joints_deg[flat_index]),
            signed_error_deg=signed_error,
            absolute_error_deg=abs(signed_error),
        )

    def feedback_unsafe_reason(self, *, now: float) -> str | None:
        """检查反馈链健康度，供回零后等待主机快照阶段复用。"""
        if self._soft_stop_latched:
            return "soft_stop_latched"
        return self._unsafe_reason(float(now))

    def _latch_soft_stop(self, reason: str) -> HardwareDecision:
        self._soft_stop_latched = True
        return HardwareDecision("soft_stop", reason)

    def _unsafe_reason(self, now: float) -> str | None:
        if self._feedback is None:
            return "feedback_missing"
        if (
            now - self._feedback.received_at
            > self.settings.feedback_timeout_s
        ):
            return "feedback_stale"
        padding = self.settings.feedback_hard_limit_padding_deg
        if np.any(self._feedback.joints_deg < self._lower - padding) or np.any(
            self._feedback.joints_deg > self._upper + padding
        ):
            return "feedback_out_of_hard_limits"
        if (
            self._feedback_frame_serials is not None
            and any(
                advanced_at is None
                or now - advanced_at
                > self.settings.feedback_timeout_s
                for advanced_at in self._feedback_frame_advanced_at
            )
        ):
            return "feedback_frame_stale"
        if self._feedback.error_codes != (0, 0):
            return "arm_error"
        if self._feedback.servo_error_reports != ("None", "None"):
            return "servo_error"
        if self._feedback.arm_states != (1, 1):
            return "arm_state_invalid"
        if not command_states_compatible(
            self._feedback.command_states,
            1,
        ):
            return "command_state_invalid"
        return None

    def _command_unavailable_reason(self, now: float) -> str | None:
        if (
            self._teleop_state is None
            or now - self._teleop_state[1] > self.settings.state_timeout_s
        ):
            return "teleop_state_stale"
        if any(self._commands[side] is None for side in SIDES):
            return "command_missing"
        timestamps = [
            self._commands[side].received_at for side in SIDES
        ]
        if any(
            now - timestamp > self.settings.command_timeout_s
            for timestamp in timestamps
        ):
            return "command_stale"
        pair_skew = abs(timestamps[0] - timestamps[1])
        if pair_skew > self.settings.maximum_pair_skew_s:
            return "command_pair_unsynchronized"
        if self._teleop_state[0] == "idle":
            target = np.concatenate(
                [
                    self._commands["left"].joints_deg,
                    self._commands["right"].joints_deg,
                ]
            )
            if (
                np.max(np.abs(target - self._home), initial=0.0)
                > self.settings.home_tolerance_deg
            ):
                return "idle_command_not_at_home"
        return None

    def _can_rearm_at_home(self, now: float) -> bool:
        state = self._teleop_state[0]
        target = np.concatenate(
            [
                self._commands["left"].joints_deg,
                self._commands["right"].joints_deg,
            ]
        )
        tolerance = self.settings.home_tolerance_deg
        return (
            state == "idle"
            and self._return_trajectory.sample(now).complete
            and np.max(np.abs(target - self._home), initial=0.0)
            <= tolerance
            and np.max(
                np.abs(self._feedback.joints_deg - self._home), initial=0.0
            )
            <= tolerance
        )

    def _return_home(self, now: float, reason: str) -> HardwareDecision:
        if self._return_trajectory is None:
            self._last_output = self._feedback.joints_deg.copy()
            self._return_trajectory = HomeTrajectory(
                start_joints=self._last_output,
                home_joints=self._home,
                start_time=now,
                minimum_duration=self.settings.return_minimum_duration_s,
                max_speed_deg_s=self.settings.return_max_speed_deg_s,
            )
        sample = self._return_trajectory.sample(now)
        self._last_output = sample.joints
        if sample.complete:
            return self._split_decision(
                "hold_home", "local_return_complete", sample.joints
            )
        return self._split_decision("return_home", reason, sample.joints)

    @staticmethod
    def _split_decision(
        action: str, reason: str, joints_deg: np.ndarray
    ) -> HardwareDecision:
        return HardwareDecision(
            action=action,
            reason=reason,
            left_joints_deg=joints_deg[:7].copy(),
            right_joints_deg=joints_deg[7:].copy(),
        )
