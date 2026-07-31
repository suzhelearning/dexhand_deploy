from __future__ import annotations

from dataclasses import dataclass
from typing import Hashable


@dataclass(frozen=True)
class FreshnessStatus:
    state: str
    allow_publish: bool
    reliable_clock: bool
    age_seconds: float | None


class FreshnessGate:
    """根据上游时间戳或降级帧签名判断输入是否仍在刷新。"""

    def __init__(self, timeout_seconds: float, allow_unstamped: bool):
        self.timeout_seconds = float(timeout_seconds)
        self.allow_unstamped = bool(allow_unstamped)
        self._last_token: Hashable | None = None
        self._last_new_frame_at: float | None = None

    def observe(
        self,
        *,
        source_timestamp_ns: int,
        frame_signature: Hashable,
        now: float,
    ) -> FreshnessStatus:
        if source_timestamp_ns > 0:
            token = ("source", int(source_timestamp_ns))
            state = "live"
            reliable = True
        elif self.allow_unstamped:
            token = ("signature", frame_signature)
            state = "live_degraded"
            reliable = False
        else:
            return FreshnessStatus("unreliable", False, False, None)

        if token != self._last_token:
            self._last_token = token
            self._last_new_frame_at = float(now)

        age = float(now) - self._last_new_frame_at
        if age > self.timeout_seconds:
            return FreshnessStatus("stale", False, reliable, age)
        return FreshnessStatus(state, True, reliable, age)
