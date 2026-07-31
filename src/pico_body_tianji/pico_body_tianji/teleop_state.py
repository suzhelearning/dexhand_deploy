from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TeleopTransition:
    state: str
    action: str
    reason: str | None = None


class _RisingEdge:
    def __init__(self, min_interval: float):
        self._min_interval = float(min_interval)
        self._previous: bool | None = None
        self._last_rising_at = float("-inf")

    def update(self, pressed: bool, now: float) -> bool:
        pressed = bool(pressed)
        rising = self._previous is False and pressed
        self._previous = pressed
        if not rising:
            return False
        if float(now) - self._last_rising_at < self._min_interval:
            return False
        self._last_rising_at = float(now)
        return True


class TeleopStateMachine:
    """右手柄 A 键启停、断流回位的纯状态机。"""

    def __init__(self, min_press_interval: float = 0.25):
        self.state = "idle"
        self._right_a = _RisingEdge(min_press_interval)

    def update(
        self,
        *,
        right_a_pressed: bool,
        signal_live: bool,
        at_home: bool,
        return_complete: bool,
        now: float,
    ) -> TeleopTransition:
        right_a_rising = self._right_a.update(right_a_pressed, now)

        if self.state == "returning":
            if return_complete and at_home:
                self.state = "idle"
                return TeleopTransition(
                    self.state, "return_complete", "at_home"
                )
            return TeleopTransition(self.state, "none")

        if self.state == "teleop":
            if not signal_live:
                self.state = "returning"
                return TeleopTransition(
                    self.state, "start_return", "signal_lost"
                )
            if right_a_rising:
                self.state = "returning"
                return TeleopTransition(
                    self.state, "start_return", "right_a"
                )
            return TeleopTransition(self.state, "none")

        if right_a_rising:
            if not signal_live:
                return TeleopTransition(
                    self.state, "reject_start", "signal_unavailable"
                )
            if not at_home:
                return TeleopTransition(
                    self.state, "reject_start", "not_at_home"
                )
            self.state = "teleop"
            return TeleopTransition(
                self.state, "start_teleop", "right_a"
            )

        return TeleopTransition(self.state, "none")
