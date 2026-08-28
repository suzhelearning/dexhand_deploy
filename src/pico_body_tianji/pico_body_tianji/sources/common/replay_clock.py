"""Replay clocks that pause recorded time without stopping wire freshness."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class HoldToRunClock:
    """Accumulate only deadman-pressed intervals with bounded tick jumps."""

    elapsed_s: float = 0.0
    running: bool = False
    maximum_step_s: float | None = None
    _last_update_s: float | None = None

    def __post_init__(self) -> None:
        if not np.isfinite(self.elapsed_s) or self.elapsed_s < 0.0:
            raise ValueError("elapsed_s must be a non-negative finite value")
        if self.maximum_step_s is not None and (
            not np.isfinite(self.maximum_step_s) or self.maximum_step_s <= 0.0
        ):
            raise ValueError("maximum_step_s must be a positive finite value")

    def update(self, now_s: float, pressed: bool) -> float:
        now = float(now_s)
        if not np.isfinite(now):
            raise ValueError("now_s must be finite")
        if self._last_update_s is None:
            self._last_update_s = now
            self.running = bool(pressed)
            return self.elapsed_s
        if now < self._last_update_s:
            raise ValueError("now_s cannot move backwards")
        interval_s = now - self._last_update_s
        if self.running:
            if self.maximum_step_s is not None:
                interval_s = min(interval_s, self.maximum_step_s)
            self.elapsed_s += interval_s
        self._last_update_s = now
        self.running = bool(pressed)
        return self.elapsed_s


__all__ = ["HoldToRunClock"]
