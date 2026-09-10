"""Selectable Wuji Hand 2 retarget backends for simulation.

The legacy geometry backend remains the default for existing profiles.  The
XR+Manus simulation profile selects the pinned official Hand 2 worker, which
keeps the vendor retargeter in its isolated Python environment and leaves the
executor responsible only for authorization, watchdogs, and command output.
"""
from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any, Protocol, Sequence

import numpy as np

from ...producers.hand_retarget import OfficialHandClient
from .config import WujiHandConfig


RETARGET_BACKENDS = ("geometry", "official_wuji_hand2")


def retarget_keypoints(points: Sequence[Sequence[float]], config: WujiHandConfig) -> list[float]:
    """Legacy finite geometry retargeter kept for existing profiles."""
    points = np.asarray(points, dtype=np.float64)
    if points.shape != (21, 3) or not np.isfinite(points).all():
        raise ValueError("keypoints must be a finite [21,3] array")
    points = points - points[0]
    values: list[float] = []
    # Thumb has four independent joints. The remaining fingers are four-joint
    # chains; flexion follows segment direction and abduction follows x spread.
    for base in (1, 5, 9, 13, 17):
        chain = points[base : base + 4]
        if chain.shape != (4, 3):
            raise ValueError("keypoint chain is incomplete")
        if base == 1:
            bend = []
            for index in range(3):
                vector = chain[index + 1] - chain[index]
                bend.append(float(np.arctan2(np.linalg.norm(vector[:2]), max(abs(float(vector[2])), 1e-6))))
            joint = [bend[0], float(np.arctan2(chain[1, 1], max(abs(float(chain[1, 0])), 1e-6))), bend[1], bend[2]]
        else:
            segments = np.diff(np.vstack((np.zeros((1, 3)), chain)), axis=0)
            bend = [float(np.arctan2(np.linalg.norm(segment[:2]), max(abs(float(segment[2])), 1e-6))) for segment in segments]
            spread = float(np.arctan2(float(chain[0, 0]), max(abs(float(chain[0, 1])), 1e-6)))
            joint = [bend[0], spread, bend[1], bend[2]]
        values.extend(joint)
    return config.validate_positions(values)


def _validate_points(points: Sequence[Sequence[float]]) -> list[float]:
    array = np.asarray(points, dtype=np.float64)
    if array.shape != (21, 3) or not np.isfinite(array).all():
        raise ValueError("keypoints must be a finite [21,3] array")
    return array.reshape(-1).tolist()


class HandRetargetBackend(Protocol):
    def retarget(self, points: Sequence[Sequence[float]], *, sequence: int, timestamp_ns: int) -> list[float]: ...

    def close(self) -> None: ...


class GeometryHandRetargetBackend:
    def __init__(self, config: WujiHandConfig) -> None:
        self.config = config

    def retarget(self, points: Sequence[Sequence[float]], *, sequence: int, timestamp_ns: int) -> list[float]:
        del sequence, timestamp_ns
        return retarget_keypoints(points, self.config)

    def close(self) -> None:
        return None


class OfficialWujiHand2RetargetBackend:
    """Adapt one side of a target stream to the official Hand 2 worker."""

    def __init__(
        self,
        *,
        side: str,
        config: WujiHandConfig,
        client: Any | None = None,
        python: str | os.PathLike[str] | None = None,
        script: str | os.PathLike[str] | None = None,
    ) -> None:
        if side not in {"left", "right"}:
            raise ValueError("official Hand 2 retarget side must be left or right")
        self.side = side
        self.config = config
        self._owns_client = client is None
        if client is not None:
            self.client = client
            return
        root = Path(
            os.environ.get(
                "TIANJI_TELEOP_BUNDLE_ROOT",
                Path(__file__).resolve().parents[5],
            )
        ).resolve()
        worker_python = Path(
            python
            or os.environ.get(
                "TIANJI_WUJI_RETARGET_PYTHON",
                root / "tools/wuji_hand_native/.pixi/envs/default/bin/python",
            )
        )
        worker_script = Path(
            script
            or os.environ.get(
                "TIANJI_WUJI_RETARGET_WORKER",
                root / "scripts/wuji_hand_worker.py",
            )
        )
        self.client = OfficialHandClient(
            python=worker_python,
            script=worker_script,
            startup_handshake=True,
            single_hand_side=side,
        )

    def retarget(self, points: Sequence[Sequence[float]], *, sequence: int, timestamp_ns: int) -> list[float]:
        if isinstance(sequence, bool) or not isinstance(sequence, int) or not 0 < sequence < 2**63:
            raise ValueError("official Hand 2 target sequence must be a positive int64")
        if isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, int) or not 0 < timestamp_ns < 2**63:
            raise ValueError("official Hand 2 target timestamp must be a positive int64")
        row = self.client.retarget(
            _validate_points(points),
            sequence=sequence,
            timestamp_ns=timestamp_ns,
        )
        hand = row.get(self.side) if isinstance(row, dict) else None
        if not isinstance(hand, dict) or hand.get("valid") is not True:
            raise ValueError(f"official Hand 2 backend returned no valid {self.side} result")
        values = hand.get("position_rad")
        if not isinstance(values, list) or len(values) != 20:
            raise ValueError(f"official Hand 2 backend returned invalid {self.side} joint vector")
        if any(type(value) not in (int, float) or not math.isfinite(float(value)) for value in values):
            raise ValueError(f"official Hand 2 backend returned non-finite {self.side} joints")
        return self.config.validate_positions(values, field="official position_rad")

    def close(self) -> None:
        close = getattr(self.client, "close", None)
        if callable(close):
            close()


def create_hand_retarget_backend(
    name: str,
    *,
    side: str,
    config: WujiHandConfig,
    client: Any | None = None,
) -> HandRetargetBackend:
    if name == "geometry":
        if client is not None:
            raise ValueError("geometry backend does not accept an official client")
        return GeometryHandRetargetBackend(config)
    if name == "official_wuji_hand2":
        return OfficialWujiHand2RetargetBackend(side=side, config=config, client=client)
    raise ValueError(f"unknown Wuji Hand 2 retarget backend: {name}")


__all__ = [
    "RETARGET_BACKENDS",
    "HandRetargetBackend",
    "OfficialWujiHand2RetargetBackend",
    "create_hand_retarget_backend",
    "retarget_keypoints",
]
