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
    """真机连接前验证同机 Pinocchio 主机链路的纯逻辑门。"""

    def __init__(
        self,
        *,
        left_home_deg,
        right_home_deg,
        freshness_timeout_s: float = 1.0,
        command_timeout_s: float = 0.2,
        maximum_pair_skew_s: float = 0.03,
        home_tolerance_deg: float = 1.0,
    ):
        if freshness_timeout_s <= 0.0 or command_timeout_s <= 0.0:
            raise ValueError("readiness timeouts must be positive")
        if maximum_pair_skew_s < 0.0 or home_tolerance_deg <= 0.0:
            raise ValueError("readiness tolerances are invalid")
        self._home = {
            "left": self._joints(left_home_deg, "left_home_deg"),
            "right": self._joints(right_home_deg, "right_home_deg"),
        }
        self._freshness_timeout_s = float(freshness_timeout_s)
        self._command_timeout_s = float(command_timeout_s)
        self._maximum_pair_skew_s = float(maximum_pair_skew_s)
        self._home_tolerance_deg = float(home_tolerance_deg)
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
            sim.get("sdk") == "pinocchio_cpp"
            and sim.get("robot_connected") is False
            and sim.get("scope") == "preview_only"
        ):
            return HostReadiness(
                False, "sim_not_isolated_pinocchio"
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
        if not (
            source.get("source") == "live"
            and source.get("smpl_source")
            in {"live", "live_signature_fallback"}
            and source.get("smpl_used") is True
            and source.get("at_safe_home") is True
            and source.get("error") is None
        ):
            return HostReadiness(False, "pico_smpl_not_live")

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
