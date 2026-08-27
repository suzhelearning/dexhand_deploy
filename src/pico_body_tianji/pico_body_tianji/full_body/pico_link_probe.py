from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass

import numpy as np

from .controller_source import ControllerSample, XRoboControllerSource


@dataclass
class PicoLinkProbeStats:
    """Accumulate read-only evidence for the PICO-to-host input link."""

    read_attempts: int = 0
    valid_controller_samples: int = 0
    invalid_controller_samples: int = 0
    controller_timestamp_updates: int = 0
    left_pose_updates: int = 0
    right_pose_updates: int = 0
    body_samples: int = 0
    read_errors: int = 0
    last_error: str | None = None
    last_tracker_count: int | None = None
    last_sample: ControllerSample | None = None

    def observe(
        self,
        sample: ControllerSample | None,
        tracker_count: int | None,
    ) -> None:
        self.read_attempts += 1
        self.last_tracker_count = tracker_count
        if sample is None:
            self.invalid_controller_samples += 1
            return

        previous = self.last_sample
        self.valid_controller_samples += 1
        if sample.body_frame is not None:
            self.body_samples += 1
        if previous is not None:
            if sample.source_timestamp_ns > previous.source_timestamp_ns:
                self.controller_timestamp_updates += 1
            if not np.array_equal(
                sample.frame.left_pose,
                previous.frame.left_pose,
            ):
                self.left_pose_updates += 1
            if not np.array_equal(
                sample.frame.right_pose,
                previous.frame.right_pose,
            ):
                self.right_pose_updates += 1
        self.last_sample = sample

    def observe_error(self, error: Exception) -> None:
        self.read_attempts += 1
        self.read_errors += 1
        self.last_error = str(error)

    @property
    def controller_timestamp_live(self) -> bool:
        return (
            self.valid_controller_samples >= 2
            and self.controller_timestamp_updates >= 1
        )

    @property
    def left_controller_updated(self) -> bool:
        return self.left_pose_updates >= 1

    @property
    def right_controller_updated(self) -> bool:
        return self.right_pose_updates >= 1

    @property
    def controller_link_live(self) -> bool:
        return (
            self.controller_timestamp_live
            and self.left_controller_updated
            and self.right_controller_updated
        )

    def summary(self) -> dict[str, object]:
        sample = self.last_sample
        return {
            "controller_link_live": self.controller_link_live,
            "controller_timestamp_live": self.controller_timestamp_live,
            "left_controller_updated": self.left_controller_updated,
            "right_controller_updated": self.right_controller_updated,
            "read_attempts": self.read_attempts,
            "valid_controller_samples": self.valid_controller_samples,
            "invalid_controller_samples": self.invalid_controller_samples,
            "controller_timestamp_updates": (
                self.controller_timestamp_updates
            ),
            "left_pose_updates": self.left_pose_updates,
            "right_pose_updates": self.right_pose_updates,
            "last_controller_timestamp_ns": (
                None if sample is None else sample.source_timestamp_ns
            ),
            "right_a_pressed": (
                None if sample is None else sample.right_a_pressed
            ),
            "body_data_received": self.body_samples > 0,
            "motion_tracker_count": self.last_tracker_count,
            "read_errors": self.read_errors,
            "last_error": self.last_error,
        }


def _pose_text(values: np.ndarray) -> str:
    return np.array2string(
        np.asarray(values, dtype=np.float64),
        precision=3,
        suppress_small=True,
        separator=",",
    )


def _print_progress(stats: PicoLinkProbeStats, elapsed: float) -> None:
    sample = stats.last_sample
    if sample is None:
        print(
            f"[{elapsed:5.1f}s] controllers=NO_VALID_SAMPLE "
            f"trackers={stats.last_tracker_count!r} "
            f"error={stats.last_error!r}",
            flush=True,
        )
        return
    print(
        f"[{elapsed:5.1f}s] controllers=RECEIVED "
        f"timestamp_ns={sample.source_timestamp_ns} "
        f"A={int(sample.right_a_pressed)} "
        f"body={'yes' if sample.body_frame is not None else 'no'} "
        f"trackers={stats.last_tracker_count!r}\n"
        f"         left={_pose_text(sample.frame.left_pose)}\n"
        f"         right={_pose_text(sample.frame.right_pose)}",
        flush=True,
    )


def run_probe(
    *,
    duration_s: float,
    rate_hz: float,
    source: XRoboControllerSource | None = None,
) -> PicoLinkProbeStats:
    if duration_s <= 0.0:
        raise ValueError("duration_s must be positive")
    if rate_hz <= 0.0:
        raise ValueError("rate_hz must be positive")

    source = XRoboControllerSource() if source is None else source
    stats = PicoLinkProbeStats()
    period_s = 1.0 / rate_hz
    source.open()
    started_at = time.monotonic()
    next_report_at = started_at
    try:
        while True:
            iteration_started = time.monotonic()
            if iteration_started - started_at >= duration_s:
                break
            try:
                sample = source.read()
                stats.observe(sample, source.motion_tracker_count())
            except Exception as exc:
                stats.observe_error(exc)
            report_time = time.monotonic()
            if report_time >= next_report_at:
                _print_progress(stats, report_time - started_at)
                next_report_at = report_time + 1.0
            remaining = period_s - (
                time.monotonic() - iteration_started
            )
            if remaining > 0.0:
                time.sleep(remaining)
    finally:
        source.close()
    return stats


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "只读检测 PICO 到 MiniPC 的双手柄链路；"
            "不发布 IK 或控制命令"
        )
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="检测时长（秒，默认 10）",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=30.0,
        help="读取频率（Hz，默认 30）",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    print(
        "开始只读检测。请保持腿部 Motion Tracker 关闭，"
        "并分别移动左右手柄。",
        flush=True,
    )
    try:
        stats = run_probe(duration_s=args.duration, rate_hz=args.rate)
    except KeyboardInterrupt:
        print("检测已取消。", flush=True)
        return 130
    except Exception as exc:
        print(f"PICO SDK 初始化失败：{exc}", flush=True)
        return 1

    summary = stats.summary()
    print("最终统计：", json.dumps(summary, ensure_ascii=False), flush=True)
    if stats.controller_link_live:
        print(
            "结论：双手柄数据和时间戳持续到达 MiniPC。"
            "Body/Tracker 是否缺失不影响本探针通过。",
            flush=True,
        )
        return 0
    if stats.valid_controller_samples > 0:
        print(
            "结论：收到过双手柄位姿，但没有同时确认时间戳、"
            "左手柄和"
            "右手柄都持续变化。请分别移动两只手柄后重试。",
            flush=True,
        )
        return 2
    print("结论：没有收到有效的左右手柄位姿。", flush=True)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
