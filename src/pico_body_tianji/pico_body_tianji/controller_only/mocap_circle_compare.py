#!/usr/bin/env python3
"""同时录制右臂圆轨迹命令、IK 解算末端与 Motive right_arm 实测。

原始数据保留各自坐标系和接收时间；比较数据将各轨迹减去录制零点，
再把 Motive 相对位移映射到 right_chest 坐标系。输出原始 CSV、对齐 CSV、
误差摘要 JSON 和可直接打开的 SVG 轨迹图。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import threading
import time
from typing import Any, Sequence

import numpy as np

from ..zenoh_util import ZenohJsonSub, key, load_tianji_config, open_session

FRAME_KEY = "mocap/hands/frame"
RIGID_BODY_NAMES_KEY = "mocap/rigid_body_names"
STATUS_KEY = "pico_body/status"
TARGET_KEY = "pico_body/right_arm_target_pose"
SOLVED_KEY = "pico_body_sim/right_arm/solved_pose"


@dataclass(frozen=True)
class PositionSample:
    received_ns: int
    position_m: np.ndarray
    metadata: dict[str, Any]

    def __post_init__(self) -> None:
        position = np.asarray(self.position_m, dtype=np.float64)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError("position_m 必须是有限三维向量")
        object.__setattr__(self, "position_m", position)


@dataclass(frozen=True)
class CaptureSnapshot:
    active_ns: int
    end_ns: int
    complete: bool
    stop_reason: str
    target: tuple[PositionSample, ...]
    solved: tuple[PositionSample, ...]
    motive: tuple[PositionSample, ...]
    statuses: tuple[tuple[int, dict[str, Any]], ...]


class CircleTrajectoryCapture:
    """线程安全的 Zenoh 回调采集器。"""

    def __init__(self, right_rigid_id: int | str) -> None:
        self._right_rigid_id = right_rigid_id
        self._lock = threading.Lock()
        self._rigid_body_names: dict[int, str] = {}
        self._target: list[PositionSample] = []
        self._solved: list[PositionSample] = []
        self._motive: list[PositionSample] = []
        self._statuses: list[tuple[int, dict[str, Any]]] = []
        self._active_ns: int | None = None
        self._observed_inactive_before_active = False
        self._started_late = False
        self._complete_ns: int | None = None
        self._aborted_ns: int | None = None
        self._aborted_reason = ""
        self._invalid_counts = {"target": 0, "solved": 0, "motive": 0}

    @staticmethod
    def _position(payload: dict[str, Any]) -> np.ndarray | None:
        value = payload.get("position")
        if isinstance(value, dict):
            values = [value.get(axis) for axis in ("x", "y", "z")]
        elif isinstance(value, (list, tuple)) and len(value) == 3:
            values = list(value)
        else:
            return None
        try:
            position = np.asarray(values, dtype=np.float64)
        except (TypeError, ValueError):
            return None
        if position.shape != (3,) or not np.isfinite(position).all():
            return None
        return position

    def _pose_sample(self, payload: dict[str, Any], stream: str) -> None:
        received_ns = time.monotonic_ns()
        position = self._position(payload)
        if position is None:
            with self._lock:
                self._invalid_counts[stream] += 1
            return
        metadata = {
            "frame_id": payload.get("header", {}).get("frame_id", ""),
            "stamp": payload.get("header", {}).get("stamp"),
        }
        sample = PositionSample(received_ns, position, metadata)
        with self._lock:
            getattr(self, f"_{stream}").append(sample)

    def on_target(self, payload: dict[str, Any]) -> None:
        self._pose_sample(payload, "target")

    def on_solved(self, payload: dict[str, Any]) -> None:
        self._pose_sample(payload, "solved")

    def on_names(self, payload: dict[str, Any]) -> None:
        mapping = payload.get("names", payload)
        if not isinstance(mapping, dict):
            return
        parsed: dict[int, str] = {}
        for rigid_id, name in mapping.items():
            try:
                parsed[int(rigid_id)] = str(name)
            except (TypeError, ValueError):
                continue
        with self._lock:
            self._rigid_body_names = parsed

    def _resolved_rigid_id_locked(self) -> int | None:
        if isinstance(self._right_rigid_id, int):
            return self._right_rigid_id
        for rigid_id, name in self._rigid_body_names.items():
            if name == self._right_rigid_id:
                return rigid_id
        return None

    def on_motive(self, payload: dict[str, Any]) -> None:
        received_ns = time.monotonic_ns()
        with self._lock:
            rigid_id = self._resolved_rigid_id_locked()
        if rigid_id is None:
            return
        for body in payload.get("rigid_bodies", []):
            if body.get("id") != rigid_id:
                continue
            if not body.get("tracking_valid", False):
                with self._lock:
                    self._invalid_counts["motive"] += 1
                return
            position = self._position(body)
            if position is None:
                with self._lock:
                    self._invalid_counts["motive"] += 1
                return
            metadata = {
                "rigid_id": rigid_id,
                "frame_number": payload.get("frame_number"),
                "source_timestamp": payload.get("timestamp"),
                "mean_error": body.get("mean_error"),
            }
            sample = PositionSample(received_ns, position, metadata)
            with self._lock:
                self._motive.append(sample)
            return
        with self._lock:
            self._invalid_counts["motive"] += 1

    def on_status(self, payload: dict[str, Any]) -> None:
        received_ns = time.monotonic_ns()
        circle = payload.get("circle_trajectory")
        with self._lock:
            self._statuses.append((received_ns, payload))
            if not isinstance(circle, dict):
                return
            active = bool(circle.get("active", False))
            complete = bool(circle.get("complete", False))
            if not active and self._active_ns is None:
                self._observed_inactive_before_active = True
            if active and self._active_ns is None:
                self._started_late = (
                    not self._observed_inactive_before_active
                )
                self._active_ns = received_ns
            if self._active_ns is None:
                return
            if complete and self._complete_ns is None:
                self._complete_ns = received_ns
            elif (
                not active
                and self._complete_ns is None
                and self._aborted_ns is None
                and payload.get("phase") in ("idle", "returning", "armed")
            ):
                self._aborted_ns = received_ns
                self._aborted_reason = (
                    f"轨迹完成前进入 {payload.get('phase')} 状态"
                )

    def state(self) -> dict[str, Any]:
        with self._lock:
            status = self._statuses[-1][1] if self._statuses else {}
            return {
                "active_ns": self._active_ns,
                "complete_ns": self._complete_ns,
                "aborted_ns": self._aborted_ns,
                "started_late": self._started_late,
                "aborted_reason": self._aborted_reason,
                "sample_counts": {
                    "target": len(self._target),
                    "solved": len(self._solved),
                    "motive": len(self._motive),
                },
                "invalid_counts": dict(self._invalid_counts),
                "status": status,
                "rigid_body_names": dict(self._rigid_body_names),
            }

    def snapshot(
        self,
        *,
        end_ns: int,
        preroll_s: float,
        stop_reason: str,
        complete: bool,
    ) -> CaptureSnapshot:
        with self._lock:
            if self._active_ns is None:
                raise RuntimeError("尚未检测到圆轨迹开始")
            start_ns = self._active_ns - round(preroll_s * 1.0e9)

            def selected(samples: Sequence[PositionSample]):
                return tuple(
                    sample
                    for sample in samples
                    if start_ns <= sample.received_ns <= end_ns
                )

            return CaptureSnapshot(
                active_ns=self._active_ns,
                end_ns=end_ns,
                complete=complete,
                stop_reason=stop_reason,
                target=selected(self._target),
                solved=selected(self._solved),
                motive=selected(self._motive),
                statuses=tuple(
                    (stamp, payload)
                    for stamp, payload in self._statuses
                    if start_ns <= stamp <= end_ns
                ),
            )


def right_motive_to_chest_matrix() -> np.ndarray:
    """返回与在线控制器完全相同的 Motive → right_chest 旋转。

    Motive 系(+X 左, +Z 前)与 PICO 系(+X 右, +Z 后)水平轴相差 180°，
    必须使用独立的 mocap_to_robot 同向映射。
    """
    config = load_tianji_config()
    matrix = (
        np.asarray(
            config.get_world_to_chest_rotation("right"), dtype=np.float64
        )
        @ np.asarray(config.mocap_to_robot, dtype=np.float64)
    )
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise RuntimeError("right Motive→chest 坐标矩阵无效")
    if not np.allclose(matrix @ matrix.T, np.eye(3), atol=1.0e-6):
        raise RuntimeError("right Motive→chest 坐标矩阵不是正交矩阵")
    return matrix


def _series(
    samples: Sequence[PositionSample], active_ns: int
) -> tuple[np.ndarray, np.ndarray]:
    times = np.asarray(
        [(sample.received_ns - active_ns) / 1.0e9 for sample in samples],
        dtype=np.float64,
    )
    positions = np.asarray(
        [sample.position_m for sample in samples], dtype=np.float64
    )
    if times.size == 0:
        return times, np.empty((0, 3), dtype=np.float64)
    order = np.argsort(times, kind="stable")
    return times[order], positions[order]


def _baseline(
    times: np.ndarray, positions: np.ndarray, baseline_s: float
) -> np.ndarray:
    before = np.flatnonzero(times <= 0.0)
    if before.size:
        window_start = times[before[0]]
        initial_before = before[
            times[before] <= window_start + baseline_s
        ]
        return np.median(positions[initial_before], axis=0)
    initial = np.flatnonzero(times <= times[0] + baseline_s)
    return np.median(positions[initial], axis=0)


def _interpolate(
    source_times: np.ndarray,
    source_values: np.ndarray,
    query_times: np.ndarray,
) -> np.ndarray:
    return np.column_stack(
        [
            np.interp(query_times, source_times, source_values[:, axis])
            for axis in range(3)
        ]
    )


def _error_metrics(error_mm: np.ndarray) -> dict[str, Any]:
    norms = np.linalg.norm(error_mm, axis=1)
    return {
        "rmse_3d_mm": float(np.sqrt(np.mean(norms * norms))),
        "p95_3d_mm": float(np.percentile(norms, 95.0)),
        "maximum_3d_mm": float(np.max(norms)),
        "mean_3d_mm": float(np.mean(norms)),
        "rmse_axis_mm": {
            axis: float(np.sqrt(np.mean(error_mm[:, index] ** 2)))
            for index, axis in enumerate(("x", "y", "z"))
        },
    }


def _lag_compensated(
    reference_times: np.ndarray,
    reference_mm: np.ndarray,
    observed_times: np.ndarray,
    observed_mm: np.ndarray,
    maximum_lag_s: float,
) -> tuple[float, np.ndarray, np.ndarray, dict[str, Any]]:
    if maximum_lag_s <= 0.0:
        lag_candidates = np.asarray([0.0])
    else:
        lag_candidates = np.linspace(
            -maximum_lag_s,
            maximum_lag_s,
            max(3, round(2.0 * maximum_lag_s / 0.005) + 1),
        )
    minimum_samples = min(20, max(4, observed_times.size // 3))
    best: tuple[float, float, np.ndarray, np.ndarray] | None = None
    for lag_s in lag_candidates:
        query = observed_times - lag_s
        mask = (query >= reference_times[0]) & (query <= reference_times[-1])
        if np.count_nonzero(mask) < minimum_samples:
            continue
        shifted = _interpolate(reference_times, reference_mm, query[mask])
        error = shifted - observed_mm[mask]
        score = float(np.sqrt(np.mean(np.sum(error * error, axis=1))))
        if best is None or score < best[0]:
            best = (score, float(lag_s), mask, shifted)
    if best is None:
        raise RuntimeError("没有足够的共同时间样本估计轨迹延迟")
    _, lag_s, mask, shifted = best
    error = shifted - observed_mm[mask]
    return lag_s, mask, shifted, _error_metrics(error)


def compare_trajectory_samples(
    snapshot: CaptureSnapshot,
    motive_to_chest: np.ndarray,
    *,
    baseline_s: float = 0.5,
    maximum_lag_s: float = 1.0,
) -> dict[str, Any]:
    """对齐三路轨迹；返回可写 CSV/SVG 的数组和误差摘要。"""
    if baseline_s <= 0.0 or not math.isfinite(baseline_s):
        raise ValueError("baseline_s 必须为正有限数值")
    matrix = np.asarray(motive_to_chest, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("motive_to_chest 必须是有限 3x3 矩阵")
    streams = {
        "target": _series(snapshot.target, snapshot.active_ns),
        "solved": _series(snapshot.solved, snapshot.active_ns),
        "motive": _series(snapshot.motive, snapshot.active_ns),
    }
    missing = [name for name, (times, _) in streams.items() if times.size < 4]
    if missing:
        raise RuntimeError(
            "轨迹样本不足（每路至少 4 个）：" + ", ".join(missing)
        )

    relative: dict[str, np.ndarray] = {}
    baselines: dict[str, np.ndarray] = {}
    for name, (times, positions) in streams.items():
        baselines[name] = _baseline(times, positions, baseline_s)
        relative[name] = positions - baselines[name]
    relative["motive"] = relative["motive"] @ matrix.T

    target_times, _ = streams["target"]
    solved_times, _ = streams["solved"]
    motive_times, _ = streams["motive"]
    common_start = max(target_times[0], solved_times[0], motive_times[0])
    common_end = min(target_times[-1], solved_times[-1], motive_times[-1])
    motive_mask = (motive_times >= common_start) & (motive_times <= common_end)
    if np.count_nonzero(motive_mask) < 4:
        raise RuntimeError("三路轨迹没有足够的共同时间窗口")
    times = motive_times[motive_mask]
    motive_mm = relative["motive"][motive_mask] * 1000.0
    target_mm = _interpolate(
        target_times, relative["target"] * 1000.0, times
    )
    solved_mm = _interpolate(
        solved_times, relative["solved"] * 1000.0, times
    )

    target_error = target_mm - motive_mm
    solved_error = solved_mm - motive_mm
    target_lag_s, target_lag_mask, target_shifted, target_lag_metrics = (
        _lag_compensated(
            target_times,
            relative["target"] * 1000.0,
            times,
            motive_mm,
            maximum_lag_s,
        )
    )
    solved_lag_s, solved_lag_mask, solved_shifted, solved_lag_metrics = (
        _lag_compensated(
            solved_times,
            relative["solved"] * 1000.0,
            times,
            motive_mm,
            maximum_lag_s,
        )
    )
    return {
        "times_s": times,
        "target_mm": target_mm,
        "solved_mm": solved_mm,
        "motive_chest_mm": motive_mm,
        "motive_samples": tuple(
            sample
            for sample, keep in zip(snapshot.motive, motive_mask)
            if keep
        ),
        "target_error_mm": target_error,
        "solved_error_mm": solved_error,
        "target_lag": {
            "seconds": target_lag_s,
            "mask": target_lag_mask,
            "reference_mm": target_shifted,
            "metrics": target_lag_metrics,
        },
        "solved_lag": {
            "seconds": solved_lag_s,
            "mask": solved_lag_mask,
            "reference_mm": solved_shifted,
            "metrics": solved_lag_metrics,
        },
        "summary": {
            "complete": snapshot.complete,
            "stop_reason": snapshot.stop_reason,
            "common_duration_s": float(times[-1] - times[0]),
            "samples": {
                "target_raw": len(snapshot.target),
                "solved_raw": len(snapshot.solved),
                "motive_raw": len(snapshot.motive),
                "aligned": int(times.size),
            },
            "baselines_m": {
                name: value.tolist() for name, value in baselines.items()
            },
            "motive_to_right_chest": matrix.tolist(),
            "target_vs_motive_direct": _error_metrics(target_error),
            "solved_vs_motive_direct": _error_metrics(solved_error),
            "target_vs_motive_lag_compensated": {
                "estimated_lag_s": target_lag_s,
                **target_lag_metrics,
            },
            "solved_vs_motive_lag_compensated": {
                "estimated_lag_s": solved_lag_s,
                **solved_lag_metrics,
            },
        },
    }


def _stamp_fields(metadata: dict[str, Any]) -> tuple[Any, Any]:
    stamp = metadata.get("stamp")
    if not isinstance(stamp, dict):
        return "", ""
    return stamp.get("sec", ""), stamp.get("nanosec", "")


def _write_raw_pose_csv(
    path: Path,
    samples: Sequence[PositionSample],
    active_ns: int,
) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time_s",
                "received_monotonic_ns",
                "x_m",
                "y_m",
                "z_m",
                "frame_id",
                "source_stamp_sec",
                "source_stamp_nanosec",
            ]
        )
        for sample in samples:
            sec, nanosec = _stamp_fields(sample.metadata)
            writer.writerow(
                [
                    (sample.received_ns - active_ns) / 1.0e9,
                    sample.received_ns,
                    *sample.position_m.tolist(),
                    sample.metadata.get("frame_id", ""),
                    sec,
                    nanosec,
                ]
            )


def _write_raw_motive_csv(
    path: Path,
    samples: Sequence[PositionSample],
    active_ns: int,
) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time_s",
                "received_monotonic_ns",
                "frame_number",
                "source_timestamp",
                "rigid_id",
                "tracking_valid",
                "x_m",
                "y_m",
                "z_m",
                "mean_error",
            ]
        )
        for sample in samples:
            writer.writerow(
                [
                    (sample.received_ns - active_ns) / 1.0e9,
                    sample.received_ns,
                    sample.metadata.get("frame_number", ""),
                    sample.metadata.get("source_timestamp", ""),
                    sample.metadata.get("rigid_id", ""),
                    True,
                    *sample.position_m.tolist(),
                    sample.metadata.get("mean_error", ""),
                ]
            )


def _write_comparison_csv(path: Path, result: dict[str, Any]) -> None:
    fields = [
        "time_s",
        *[
            f"{stream}_{axis}_mm"
            for stream in ("target", "solved", "motive_chest")
            for axis in ("x", "y", "z")
        ],
        "target_error_3d_mm",
        "solved_error_3d_mm",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        target_norm = np.linalg.norm(result["target_error_mm"], axis=1)
        solved_norm = np.linalg.norm(result["solved_error_mm"], axis=1)
        for index, time_s in enumerate(result["times_s"]):
            writer.writerow(
                [
                    time_s,
                    *result["target_mm"][index].tolist(),
                    *result["solved_mm"][index].tolist(),
                    *result["motive_chest_mm"][index].tolist(),
                    target_norm[index],
                    solved_norm[index],
                ]
            )


def _polyline(
    points: np.ndarray,
    x_limits: tuple[float, float],
    y_limits: tuple[float, float],
    box: tuple[float, float, float, float],
) -> str:
    left, top, width, height = box
    x_min, x_max = x_limits
    y_min, y_max = y_limits
    if points.shape[0] > 2500:
        points = points[np.linspace(0, points.shape[0] - 1, 2500).astype(int)]
    x = left + (points[:, 0] - x_min) / (x_max - x_min) * width
    y = top + height - (points[:, 1] - y_min) / (y_max - y_min) * height
    return " ".join(f"{px:.2f},{py:.2f}" for px, py in zip(x, y))


def _limits(values: np.ndarray, minimum_span: float = 10.0) -> tuple[float, float]:
    low = float(np.min(values))
    high = float(np.max(values))
    span = max(high - low, minimum_span)
    padding = 0.08 * span
    center = 0.5 * (low + high)
    return center - 0.5 * span - padding, center + 0.5 * span + padding


def write_svg(path: Path, result: dict[str, Any]) -> None:
    colors = {
        "target": "#2563eb",
        "solved": "#ea580c",
        "motive": "#16a34a",
    }
    target = result["target_mm"]
    solved = result["solved_mm"]
    motive = result["motive_chest_mm"]
    times = result["times_s"]
    spans = np.ptp(target, axis=0)
    plane_axes = np.argsort(spans)[-2:]
    plane_axes.sort()
    axis_names = ("X", "Y", "Z")
    plane_values = np.concatenate(
        [target[:, plane_axes], solved[:, plane_axes], motive[:, plane_axes]],
        axis=0,
    )
    x_limits = _limits(plane_values[:, 0])
    y_limits = _limits(plane_values[:, 1])
    x_span = x_limits[1] - x_limits[0]
    y_span = y_limits[1] - y_limits[0]
    equal_span = max(x_span, y_span)
    x_center = sum(x_limits) / 2.0
    y_center = sum(y_limits) / 2.0
    x_limits = (x_center - equal_span / 2.0, x_center + equal_span / 2.0)
    y_limits = (y_center - equal_span / 2.0, y_center + equal_span / 2.0)

    width, height = 1440, 920
    plane_box = (80.0, 150.0, 650.0, 650.0)
    time_boxes = [
        (810.0, 150.0 + index * 220.0, 550.0, 160.0)
        for index in range(3)
    ]
    direct = result["summary"]["solved_vs_motive_direct"]
    compensated = result["summary"]["solved_vs_motive_lag_compensated"]
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#fafafa"/>',
        '<style>text{font-family:DejaVu Sans,Arial,sans-serif;fill:#172033}'
        '.title{font-size:28px;font-weight:700}.sub{font-size:15px;fill:#526071}'
        '.axis{font-size:14px}.panel{fill:#fff;stroke:#cbd5e1;stroke-width:1}'
        '.grid{stroke:#e2e8f0;stroke-width:1}.curve{fill:none;stroke-width:2.2}'
        '.legend{font-size:15px;font-weight:600}</style>',
        '<text x="80" y="55" class="title">Mocap circle trajectory comparison</text>',
        (
            '<text x="80" y="85" class="sub">right_chest relative displacement; '
            f'solved vs Motive direct RMSE {direct["rmse_3d_mm"]:.2f} mm; '
            f'lag {compensated["estimated_lag_s"] * 1000.0:.1f} ms; '
            f'compensated RMSE {compensated["rmse_3d_mm"]:.2f} mm</text>'
        ),
        f'<rect x="{plane_box[0]}" y="{plane_box[1]}" width="{plane_box[2]}" height="{plane_box[3]}" class="panel"/>',
        (
            f'<text x="{plane_box[0]}" y="125" class="legend">Trajectory plane: '
            f'{axis_names[plane_axes[0]]}-{axis_names[plane_axes[1]]} (mm)</text>'
        ),
    ]
    for fraction in (0.25, 0.5, 0.75):
        x = plane_box[0] + plane_box[2] * fraction
        y = plane_box[1] + plane_box[3] * fraction
        lines.append(
            f'<line x1="{x}" y1="{plane_box[1]}" x2="{x}" y2="{plane_box[1] + plane_box[3]}" class="grid"/>'
        )
        lines.append(
            f'<line x1="{plane_box[0]}" y1="{y}" x2="{plane_box[0] + plane_box[2]}" y2="{y}" class="grid"/>'
        )
    for name, values in (
        ("target", target),
        ("solved", solved),
        ("motive", motive),
    ):
        points = values[:, plane_axes]
        lines.append(
            f'<polyline points="{_polyline(points, x_limits, y_limits, plane_box)}" '
            f'class="curve" stroke="{colors[name]}"/>'
        )
    lines.extend(
        [
            f'<text x="{plane_box[0]}" y="825" class="axis">{x_limits[0]:.1f}</text>',
            f'<text x="{plane_box[0] + plane_box[2] - 42}" y="825" class="axis">{x_limits[1]:.1f}</text>',
            f'<text x="20" y="{plane_box[1] + 10}" class="axis">{y_limits[1]:.1f}</text>',
            f'<text x="20" y="{plane_box[1] + plane_box[3]}" class="axis">{y_limits[0]:.1f}</text>',
        ]
    )

    time_limits = (float(times[0]), float(times[-1]))
    for axis, box in enumerate(time_boxes):
        values = np.concatenate(
            [target[:, axis], solved[:, axis], motive[:, axis]]
        )
        value_limits = _limits(values)
        lines.extend(
            [
                f'<rect x="{box[0]}" y="{box[1]}" width="{box[2]}" height="{box[3]}" class="panel"/>',
                f'<text x="{box[0]}" y="{box[1] - 12}" class="legend">{axis_names[axis]} displacement (mm)</text>',
            ]
        )
        for name, data in (
            ("target", target),
            ("solved", solved),
            ("motive", motive),
        ):
            points = np.column_stack((times, data[:, axis]))
            lines.append(
                f'<polyline points="{_polyline(points, time_limits, value_limits, box)}" '
                f'class="curve" stroke="{colors[name]}"/>'
            )

    legend_y = 875
    legend_x = 80
    for name, label in (
        ("target", "command target"),
        ("solved", "IK solved pose"),
        ("motive", "Motive right_arm"),
    ):
        lines.append(
            f'<line x1="{legend_x}" y1="{legend_y}" x2="{legend_x + 32}" y2="{legend_y}" stroke="{colors[name]}" stroke-width="4"/>'
        )
        lines.append(
            f'<text x="{legend_x + 42}" y="{legend_y + 5}" class="legend">{label}</text>'
        )
        legend_x += 220
    lines.append('</svg>')
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _circle_metadata(snapshot: CaptureSnapshot) -> dict[str, Any]:
    circles = [
        payload.get("circle_trajectory")
        for _, payload in snapshot.statuses
        if isinstance(payload.get("circle_trajectory"), dict)
    ]
    return circles[-1] if circles else {}


def write_capture_outputs(
    output_dir: Path,
    snapshot: CaptureSnapshot,
    result: dict[str, Any],
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=False)
    paths = {
        "target": output_dir / "raw_target_right_chest.csv",
        "solved": output_dir / "raw_solved_right_chest.csv",
        "motive": output_dir / "raw_motive_right_arm.csv",
        "status": output_dir / "raw_status.jsonl",
        "comparison": output_dir / "comparison_right_chest.csv",
        "summary": output_dir / "summary.json",
        "figure": output_dir / "trajectory_comparison.svg",
    }
    _write_raw_pose_csv(paths["target"], snapshot.target, snapshot.active_ns)
    _write_raw_pose_csv(paths["solved"], snapshot.solved, snapshot.active_ns)
    _write_raw_motive_csv(paths["motive"], snapshot.motive, snapshot.active_ns)
    with paths["status"].open("w", encoding="utf-8") as stream:
        for received_ns, payload in snapshot.statuses:
            stream.write(
                json.dumps(
                    {
                        "time_s": (received_ns - snapshot.active_ns) / 1.0e9,
                        "received_monotonic_ns": received_ns,
                        "payload": payload,
                    },
                    ensure_ascii=False,
                    separators=(",", ":"),
                )
                + "\n"
            )
    _write_comparison_csv(paths["comparison"], result)
    summary = {
        **result["summary"],
        "circle_trajectory": _circle_metadata(snapshot),
        "files": {name: path.name for name, path in paths.items()},
    }
    paths["summary"].write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    write_svg(paths["figure"], result)
    return paths


def _parse_rigid_spec(value: str) -> int | str:
    try:
        rigid_id = int(value)
    except ValueError:
        if not value.strip():
            raise argparse.ArgumentTypeError("刚体名不能为空")
        return value.strip()
    if rigid_id <= 0:
        raise argparse.ArgumentTypeError("刚体 id 必须为正整数")
    return rigid_id


def _positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("必须是正有限数值")
    return parsed


def _nonnegative_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError("必须是非负有限数值")
    return parsed


def _open_zenoh_session(endpoint: str):
    if not endpoint:
        return open_session()
    import zenoh

    config = zenoh.Config.from_json5(
        json.dumps(
            {"mode": "client", "connect": {"endpoints": [endpoint]}}
        )
    )
    return zenoh.open(config)


def _default_output_dir() -> Path:
    root = Path(
        os.environ.get(
            "PICO_BODY_TIANJI_BUNDLE_ROOT",
            Path(__file__).resolve().parents[4],
        )
    )
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return root / "log" / "mocap_circle_compare" / timestamp


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "同时录制 right_chest 目标/IK 解算轨迹和 Motive right_arm "
            "刚体轨迹，自动对齐并输出 CSV、JSON、SVG。请先启动本脚本，"
            "再到 sim_mocap_live 终端按 s、c、按住 Enter。"
        )
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="输出目录（默认 log/mocap_circle_compare/时间戳）",
    )
    parser.add_argument(
        "--right-rigid-id",
        type=_parse_rigid_spec,
        default="right_arm",
        help="Motive 右臂刚体 id 或名字（默认 right_arm）",
    )
    parser.add_argument(
        "--connect-endpoint",
        default="",
        help="zenohd Router 端点（默认空=本机 scouting）",
    )
    parser.add_argument(
        "--wait-timeout",
        type=_positive_float,
        default=300.0,
        metavar="SECONDS",
        help="等待 c 装载圆轨迹的超时（默认 300s）",
    )
    parser.add_argument(
        "--record-timeout",
        type=_positive_float,
        default=600.0,
        metavar="SECONDS",
        help="轨迹开始后的录制超时（默认 600s）",
    )
    parser.add_argument(
        "--preroll-seconds",
        type=_nonnegative_float,
        default=1.0,
        help="保留圆轨迹激活前数据（默认 1s）",
    )
    parser.add_argument(
        "--settle-seconds",
        type=_nonnegative_float,
        default=0.5,
        help="轨迹完成后继续录制时间（默认 0.5s）",
    )
    parser.add_argument(
        "--baseline-seconds",
        type=_positive_float,
        default=0.5,
        help="激活前零点中位数窗口（默认 0.5s）",
    )
    parser.add_argument(
        "--maximum-lag-seconds",
        type=_nonnegative_float,
        default=1.0,
        help="自动估计实测滞后的搜索范围（默认 ±1s）",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    output_dir = args.output_dir or _default_output_dir()
    if output_dir.exists():
        raise SystemExit(f"拒绝覆盖已有输出目录：{output_dir}")

    capture = CircleTrajectoryCapture(args.right_rigid_id)
    session = _open_zenoh_session(args.connect_endpoint)
    subscribers = [
        ZenohJsonSub(session, key(TARGET_KEY), capture.on_target),
        ZenohJsonSub(session, key(SOLVED_KEY), capture.on_solved),
        ZenohJsonSub(session, key(FRAME_KEY), capture.on_motive),
        ZenohJsonSub(session, key(RIGID_BODY_NAMES_KEY), capture.on_names),
        ZenohJsonSub(session, key(STATUS_KEY), capture.on_status),
    ]
    complete = False
    full_capture = True
    stop_reason = ""
    end_ns = time.monotonic_ns()
    try:
        print(
            "轨迹对比录制器已就绪。请在 sim_mocap_live 终端依次执行："
            "s 定零，c 装载，按住 Enter 运行；脚本将在整圆完成后自动保存。",
            flush=True,
        )
        wait_deadline = time.monotonic() + args.wait_timeout
        last_report = 0.0
        while True:
            state = capture.state()
            if state["active_ns"] is not None:
                circle = state["status"].get("circle_trajectory", {})
                started_elapsed_s = float(
                    circle.get("elapsed_hold_s") or 0.0
                )
                if state["started_late"]:
                    full_capture = False
                    print(
                        "警告：录制器在轨迹已推进 "
                        f"{started_elapsed_s:.2f}s 后才接入；将保存部分轨迹，"
                        "但不会标记为完整录制。",
                        flush=True,
                    )
                else:
                    print(
                        "检测到圆轨迹已装载，开始录制（包含预录窗口）。",
                        flush=True,
                    )
                break
            if time.monotonic() >= wait_deadline:
                raise RuntimeError(
                    "等待圆轨迹超时；确认 sim_mocap_live 已启动并按 s、c"
                )
            now = time.monotonic()
            if now - last_report >= 2.0:
                counts = state["sample_counts"]
                print(
                    "等待 c：target={target} solved={solved} motive={motive}".format(
                        **counts
                    ),
                    flush=True,
                )
                last_report = now
            time.sleep(0.05)

        record_deadline = time.monotonic() + args.record_timeout
        while True:
            state = capture.state()
            if state["complete_ns"] is not None:
                complete = True
                stop_reason = "圆轨迹完成"
                time.sleep(args.settle_seconds)
                end_ns = time.monotonic_ns()
                break
            if state["aborted_ns"] is not None:
                stop_reason = state["aborted_reason"]
                end_ns = time.monotonic_ns()
                break
            if time.monotonic() >= record_deadline:
                stop_reason = "录制超时"
                end_ns = time.monotonic_ns()
                break
            now = time.monotonic()
            if now - last_report >= 1.0:
                circle = state["status"].get("circle_trajectory", {})
                print(
                    "录制中：elapsed={:.2f}s segment={} Enter={} samples={}".format(
                        float(circle.get("elapsed_hold_s") or 0.0),
                        circle.get("segment") or "waiting",
                        "按住" if circle.get("deadman_pressed") else "松开",
                        state["sample_counts"],
                    ),
                    flush=True,
                )
                last_report = now
            time.sleep(0.05)
    except KeyboardInterrupt:
        state = capture.state()
        if state["active_ns"] is None:
            print("尚未开始轨迹，未生成空记录。", flush=True)
            return 130
        stop_reason = "用户中断"
        end_ns = time.monotonic_ns()
    finally:
        for subscriber in subscribers:
            subscriber.close()
        session.close()

    if complete and not full_capture:
        stop_reason = "圆轨迹完成，但录制器中途接入"
    snapshot = capture.snapshot(
        end_ns=end_ns,
        preroll_s=args.preroll_seconds,
        stop_reason=stop_reason,
        complete=complete and full_capture,
    )
    result = compare_trajectory_samples(
        snapshot,
        right_motive_to_chest_matrix(),
        baseline_s=args.baseline_seconds,
        maximum_lag_s=args.maximum_lag_seconds,
    )
    paths = write_capture_outputs(output_dir, snapshot, result)
    summary = result["summary"]
    solved_direct = summary["solved_vs_motive_direct"]
    solved_lag = summary["solved_vs_motive_lag_compensated"]
    print(
        "\n录制结果：\n"
        f"  输出目录：{output_dir}\n"
        f"  图表：{paths['figure']}\n"
        f"  对齐 CSV：{paths['comparison']}\n"
        f"  IK solved vs Motive 直接 RMSE：{solved_direct['rmse_3d_mm']:.2f} mm\n"
        f"  估计滞后：{solved_lag['estimated_lag_s'] * 1000.0:.1f} ms\n"
        f"  滞后补偿 RMSE：{solved_lag['rmse_3d_mm']:.2f} mm",
        flush=True,
    )
    return 0 if snapshot.complete else 1


if __name__ == "__main__":
    raise SystemExit(main())
