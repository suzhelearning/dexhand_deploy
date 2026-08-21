from __future__ import annotations

from dataclasses import dataclass
import json

import numpy as np


SIDES = ("left", "right")


@dataclass(frozen=True)
class HostReadiness:
    ready: bool
    reason: str
    left_joints_deg: np.ndarray | None = None
    right_joints_deg: np.ndarray | None = None


@dataclass(frozen=True)
class _TimedCommand:
    joints_deg: np.ndarray
    received_at: float


@dataclass(frozen=True)
class _TimedStatus:
    payload: dict
    received_at: float


class HostReadinessGate:
    """真机连接前验证同机 IK 主机链路的纯逻辑门。"""

    def __init__(
        self,
        *,
        left_home_deg,
        right_home_deg,
        freshness_timeout_s: float = 1.0,
        command_timeout_s: float = 0.2,
        maximum_pair_skew_s: float = 0.03,
        home_tolerance_deg: float = 1.0,
        input_mode: str = "smpl",
    ):
        if freshness_timeout_s <= 0.0 or command_timeout_s <= 0.0:
            raise ValueError("readiness timeouts must be positive")
        if maximum_pair_skew_s < 0.0 or home_tolerance_deg <= 0.0:
            raise ValueError("readiness tolerances are invalid")
        if input_mode not in {"smpl", "controller_only"}:
            raise ValueError("unsupported host input mode")
        self._home = {
            "left": self._joints(left_home_deg, "left_home_deg"),
            "right": self._joints(right_home_deg, "right_home_deg"),
        }
        self._freshness_timeout_s = float(freshness_timeout_s)
        self._command_timeout_s = float(command_timeout_s)
        self._maximum_pair_skew_s = float(maximum_pair_skew_s)
        self._home_tolerance_deg = float(home_tolerance_deg)
        self._input_mode = input_mode
        self._commands: dict[str, _TimedCommand | None] = {
            side: None for side in SIDES
        }
        self._teleop_state: tuple[str, float] | None = None
        self._input_status: _TimedStatus | None = None
        self._sim_status: _TimedStatus | None = None

    @staticmethod
    def _joints(values, label: str) -> np.ndarray:
        joints = np.asarray(values, dtype=np.float64)
        if joints.shape != (7,) or not np.isfinite(joints).all():
            raise ValueError(f"{label} must contain seven finite values")
        return joints.copy()

    @staticmethod
    def _status(payload, label: str) -> dict:
        try:
            parsed = (
                json.loads(payload)
                if isinstance(payload, str)
                else payload
            )
        except json.JSONDecodeError as exc:
            raise ValueError(f"{label} is not valid JSON") from exc
        if not isinstance(parsed, dict):
            raise ValueError(f"{label} must be a JSON object")
        return dict(parsed)

    def observe_command(
        self,
        side: str,
        joints_deg,
        *,
        frame_id: str,
        received_at: float,
    ) -> None:
        if side not in SIDES:
            raise ValueError(f"unsupported side: {side!r}")
        expected_frame = f"{side}_base_marvin_degrees"
        if frame_id != expected_frame:
            raise ValueError(
                f"{side} command frame must be {expected_frame!r}"
            )
        self._commands[side] = _TimedCommand(
            self._joints(joints_deg, f"{side} command"),
            float(received_at),
        )

    def observe_teleop_state(self, state: str, *, received_at: float) -> None:
        if state not in {"idle", "teleop", "returning"}:
            raise ValueError(f"unsupported teleop state: {state!r}")
        self._teleop_state = (state, float(received_at))

    def observe_input_status(self, payload, *, received_at: float) -> None:
        self._input_status = _TimedStatus(
            self._status(payload, "input status"),
            float(received_at),
        )

    def observe_sim_status(self, payload, *, received_at: float) -> None:
        self._sim_status = _TimedStatus(
            self._status(payload, "simulation status"),
            float(received_at),
        )

    def evaluate(self, *, now: float) -> HostReadiness:
        now = float(now)
        if self._input_status is None or self._sim_status is None:
            return HostReadiness(False, "host_status_missing")
        if (
            now - self._input_status.received_at
            > self._freshness_timeout_s
            or now - self._sim_status.received_at
            > self._freshness_timeout_s
        ):
            return HostReadiness(False, "host_status_stale")
        if self._teleop_state is None:
            return HostReadiness(False, "teleop_state_missing")
        if (
            now - self._teleop_state[1] > self._freshness_timeout_s
        ):
            return HostReadiness(False, "teleop_state_stale")

        sim = self._sim_status.payload
        if not (
            sim.get("ik_interface") == "arm_ik_solver_v1"
            and isinstance(sim.get("ik_backend"), str)
            and bool(sim.get("ik_backend"))
            and sim.get("robot_connected") is False
            and sim.get("scope") == "preview_only"
        ):
            return HostReadiness(
                False, "sim_not_isolated_expected_ik"
            )
        if (
            sim.get("mode") != "idle"
            or sim.get("at_safe_home") is not True
        ):
            return HostReadiness(False, "sim_not_idle_at_home")

        source = self._input_status.payload
        if (
            source.get("state") != "idle"
            or self._teleop_state[0] != "idle"
        ):
            return HostReadiness(False, "host_not_idle")
        if self._input_mode == "smpl":
            if not (
                source.get("source") == "live"
                and source.get("smpl_source")
                in {"live", "live_signature_fallback"}
                and source.get("smpl_used") is True
                and source.get("at_safe_home") is True
                and source.get("error") is None
            ):
                return HostReadiness(False, "pico_smpl_not_live")
        elif source.get("input") == "mocap_live":
            # Motive 刚体仅用于按 s 定零，随后由键盘累计虚拟目标；
            # 要求新控制模式身份及动捕跟踪器可用于初始参考。
            if not (
                source.get("source") == "live"
                and source.get("mapping")
                == "controller_relative_end_pose_conditioned_v1"
                and source.get("control_mode")
                == "motive_reference_keyboard_step"
                and source.get("body_tracking") == "disabled"
                and source.get("motion_trackers_required") is True
                and source.get("elbow_constraint")
                == "published_default_zsp_backend_selected"
                and source.get("smpl_used") is False
                and source.get("scope") == "mocap_live"
                and source.get("at_safe_home") is True
                and source.get("error") is None
            ):
                return HostReadiness(False, "mocap_live_not_ready")
        elif source.get("input") == "mocap_keyboard_step":
            # mocap 键盘步进主机：确定性逐点验收/标定输入，
            # 接受离线来源（source=offline_replay），要求
            # preview-only 身份与就绪字段。
            if not (
                source.get("source") == "offline_replay"
                and source.get("mapping")
                == "controller_relative_end_pose_conditioned_v1"
                and source.get("body_tracking") == "disabled"
                and source.get("motion_trackers_required") is False
                and source.get("elbow_constraint")
                == "published_default_zsp_backend_selected"
                and source.get("smpl_used") is False
                and source.get("scope") == "mocap_keyboard_step"
                and source.get("at_safe_home") is True
                and source.get("error") is None
            ):
                return HostReadiness(
                    False, "mocap_keyboard_step_not_ready"
                )
        elif source.get("input") == "mocap_h5_replay":
            # H5 真机回放仅接受尚未开始、已确认 Home 的低速 wrist
            # frame0 对齐主机。Motive marker 在连接前必须新鲜有效；
            # Enter 必须可用且松开，避免桥刚 armed 就开始运动。
            motive = source.get("motive_right_arm")
            recording = source.get("recording")
            right_summary = (
                recording.get("hands", {}).get("right", {})
                if isinstance(recording, dict)
                else {}
            )
            speed = source.get("speed")
            yaw_deg = source.get("yaw_deg")
            if not (
                source.get("source") == "offline_replay"
                and source.get("mapping")
                == "motive_rigid_offset_absolute_wrist_tcp_v5"
                and source.get("control_mode")
                == "h5_right_wrist_to_right_arm_hold_to_run"
                and source.get("body_tracking") == "disabled"
                and source.get("motion_trackers_required") is True
                and source.get("elbow_constraint")
                == "published_default_zsp_backend_selected"
                and source.get("smpl_used") is False
                and source.get("scope") == "mocap_replay"
                and source.get("endpoint") == "wuji2_r_wrist"
                and source.get("side") == "right"
                and source.get("phase") == "armed"
                and source.get("at_safe_home") is True
                and source.get("deadman_available") is True
                and source.get("deadman_pressed") is False
                and source.get("deadman_error") is None
                and source.get("source_complete") is False
                and source.get("error") is None
                and isinstance(motive, dict)
                and motive.get("tracking_valid") is True
                and isinstance(motive.get("resolved_id"), int)
                and not isinstance(motive.get("resolved_id"), bool)
                and motive.get("resolved_id") > 0
                and isinstance(right_summary.get("valid_frames"), int)
                and not isinstance(right_summary.get("valid_frames"), bool)
                and right_summary.get("valid_frames") > 0
                and isinstance(speed, (int, float))
                and not isinstance(speed, bool)
                and 0.0 < float(speed) <= 0.25
                and isinstance(yaw_deg, (int, float))
                and not isinstance(yaw_deg, bool)
                and float(yaw_deg) == 0.0
            ):
                return HostReadiness(False, "mocap_h5_not_ready")
        elif not (
            source.get("source") == "live"
            and source.get("input") == "pico_controllers_only"
            and source.get("mapping")
            == "controller_relative_end_pose_conditioned_v1"
            and source.get("body_tracking") == "disabled"
            and source.get("motion_trackers_required") is False
            and source.get("elbow_constraint")
            == "published_default_zsp_backend_selected"
            and source.get("smpl_used") is False
            and source.get("scope") == "controller_only_ik"
            and source.get("at_safe_home") is True
            and source.get("error") is None
        ):
            return HostReadiness(
                False, "pico_controller_only_not_live"
            )

        if any(self._commands[side] is None for side in SIDES):
            return HostReadiness(False, "command_missing")
        timestamps = [
            self._commands[side].received_at for side in SIDES
        ]
        if any(
            now - timestamp > self._command_timeout_s
            for timestamp in timestamps
        ):
            return HostReadiness(False, "command_stale")
        if (
            abs(timestamps[0] - timestamps[1])
            > self._maximum_pair_skew_s
        ):
            return HostReadiness(False, "command_pair_unsynchronized")
        if any(
            np.max(
                np.abs(
                    self._commands[side].joints_deg - self._home[side]
                ),
                initial=0.0,
            )
            > self._home_tolerance_deg
            for side in SIDES
        ):
            return HostReadiness(False, "command_not_at_home")
        return HostReadiness(
            True,
            "ready",
            left_joints_deg=self._commands["left"].joints_deg.copy(),
            right_joints_deg=self._commands["right"].joints_deg.copy(),
        )
