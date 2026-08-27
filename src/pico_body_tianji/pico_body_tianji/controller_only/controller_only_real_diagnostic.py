from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time
from typing import Any

from ..zenoh_util import (
    LiveToken,
    ZenohJsonSub,
    ZenohTextSub,
    key,
    load_node_config,
    open_session,
    parse_param_override,
)


SIDES = ("left", "right")
SCHEMA = "pico_body_tianji.controller_only_real_diagnostic.v1"
# 工具参数以 CLI 为主；--config/--param 是统一覆盖通道（缺省时回落 CLI 默认）。
DEFAULT_PARAMETERS: dict[str, Any] = {}
INPUT_FLAGS = (
    "workspace_soft_limited",
    "linear_speed_limited",
    "linear_acceleration_limited",
    "angular_speed_limited",
    "angular_acceleration_limited",
)
IK_FLAGS = (
    "joint_step_limited",
    "target_saturated",
    "singularity_active",
    "soft_limit_active",
    "workspace_backoff_active",
    "orientation_relaxed",
)


def _json_object(text: str) -> dict[str, Any] | None:
    try:
        value = json.loads(text)
    except json.JSONDecodeError:
        return None
    return value if isinstance(value, dict) else None


def _finite(value: Any) -> float | None:
    if isinstance(value, (int, float)) and math.isfinite(value):
        return float(value)
    return None


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _summary(values: list[float]) -> dict[str, float | None]:
    return {
        "mean": statistics.fmean(values) if values else None,
        "p95": _percentile(values, 0.95),
        "max": max(values) if values else None,
    }


def _ratio(count: int, total: int) -> float:
    return count / total if total > 0 else 0.0


def _positions(message: dict[str, Any]) -> list[float] | None:
    values = [float(value) for value in message["position"]]
    if len(values) != 7 or not all(math.isfinite(value) for value in values):
        return None
    return values


class RealDiagnosticCollector:
    """只读采集纯手柄 IK、真机桥和 Marvin 反馈的限制状态。"""

    def __init__(self, session, output: Path, sample_rate_hz: float):
        self.session = session
        self.output = output
        self.sample_rate = sample_rate_hz
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.output.open("x", encoding="utf-8")
        self.started = time.monotonic()
        self.latest: dict[str, Any] = {
            "input_status": None,
            "ik_status": None,
            "real_status": None,
            "teleop_state": None,
            "commands_deg": {},
            "feedback_deg": {},
        }
        self.message_counts: Counter[str] = Counter()
        self.input_samples = {side: 0 for side in SIDES}
        self.input_flags = {side: Counter() for side in SIDES}
        self.input_values = {
            side: {
                key: []
                for key in (
                    "requested_workspace_utilization",
                    "workspace_utilization",
                    "requested_linear_speed_m_s",
                    "applied_linear_speed_m_s",
                    "requested_angular_speed_rad_s",
                    "applied_angular_speed_rad_s",
                )
            }
            for side in SIDES
        }
        self.ik_samples = {side: 0 for side in SIDES}
        self.ik_flags = {side: Counter() for side in SIDES}
        self.ik_statuses = {side: Counter() for side in SIDES}
        self.ik_errors = {side: Counter() for side in SIDES}
        self.ik_values = {
            side: {
                key: []
                for key in (
                    "requested_max_joint_step_deg",
                    "max_joint_step_deg",
                    "position_error_mm",
                    "orientation_error_deg",
                    "min_limit_margin_deg",
                    "workspace_backoff_fraction",
                )
            }
            for side in SIDES
        }
        self.peak_rejections = {side: 0 for side in SIDES}
        self.previous_commands: dict[str, list[float]] = {}
        self.command_steps = {side: [] for side in SIDES}
        self.feedback_steps = {side: [] for side in SIDES}
        self.previous_feedback: dict[str, list[float]] = {}
        self.host_tracking_errors = {side: [] for side in SIDES}
        self.host_tracking_joint_max = {
            side: [0.0] * 7 for side in SIDES
        }
        self.hardware_actions: Counter[str] = Counter()
        self._previous_hardware_actions: dict[str, int] | None = None
        self.hardware_deadline_misses = 0
        self._previous_deadline_misses: int | None = None
        self.real_errors: Counter[str] = Counter()

        self.stream.write(
            json.dumps(
                {
                    "schema": SCHEMA,
                    "type": "metadata",
                    "created_at": datetime.now(timezone.utc).isoformat(),
                    "sample_rate_hz": sample_rate_hz,
                    "read_only": True,
                },
                ensure_ascii=False,
            )
            + "\n"
        )

        ZenohTextSub(session, key("/pico_body/status"), self._on_input_status)
        ZenohTextSub(
            session, key("/pico_body_sim/status"), self._on_ik_status
        )
        ZenohTextSub(
            session, key("/pico_body_real/status"), self._on_real_status
        )
        ZenohTextSub(
            session,
            key("/pico_body/teleop_state"),
            self._on_teleop_state,
        )
        for side in SIDES:
            ZenohJsonSub(
                session,
                key(f"/pico_body_sim/{side}_arm/joint_commands"),
                lambda message, side=side: self._on_command(side, message),
            )
            ZenohJsonSub(
                session,
                key(f"/{side}_arm/joint_states"),
                lambda message, side=side: self._on_feedback(side, message),
            )

    def _on_teleop_state(self, text: str) -> None:
        self.message_counts["teleop_state"] += 1
        self.latest["teleop_state"] = text

    def _on_input_status(self, text: str) -> None:
        payload = _json_object(text)
        if payload is None:
            return
        self.message_counts["input_status"] += 1
        self.latest["input_status"] = payload
        if payload.get("state") != "teleop":
            return
        conditioning = payload.get("target_conditioning")
        if not isinstance(conditioning, dict):
            return
        for side in SIDES:
            values = conditioning.get(side)
            if not isinstance(values, dict):
                continue
            self.input_samples[side] += 1
            for flag in INPUT_FLAGS:
                self.input_flags[side][flag] += bool(values.get(flag))
            for key, target in self.input_values[side].items():
                value = _finite(values.get(key))
                if value is not None:
                    target.append(value)

    def _on_ik_status(self, text: str) -> None:
        payload = _json_object(text)
        if payload is None:
            return
        self.message_counts["ik_status"] += 1
        self.latest["ik_status"] = payload
        if payload.get("mode") != "teleop":
            return
        for side in SIDES:
            self.ik_samples[side] += 1
            for flag in IK_FLAGS:
                self.ik_flags[side][flag] += bool(
                    payload.get(f"{side}_{flag}")
                )
            status = payload.get(f"{side}_ik_status")
            if isinstance(status, str) and status:
                self.ik_statuses[side][status] += 1
            error = payload.get(f"{side}_ik_error")
            if isinstance(error, str) and error:
                self.ik_errors[side][error] += 1
            rejection = payload.get(f"{side}_consecutive_rejections")
            if isinstance(rejection, int):
                self.peak_rejections[side] = max(
                    self.peak_rejections[side], rejection
                )
            for suffix, target in self.ik_values[side].items():
                value = _finite(payload.get(f"{side}_{suffix}"))
                if value is not None:
                    target.append(value)

    def _accumulate_counter_delta(
        self,
        current: dict[str, int],
        previous: dict[str, int] | None,
        total: Counter[str],
    ) -> None:
        if previous is None:
            return
        for key, value in current.items():
            old = int(previous.get(key, 0))
            total[key] += value - old if value >= old else value

    def _on_real_status(self, text: str) -> None:
        payload = _json_object(text)
        if payload is None:
            return
        self.message_counts["real_status"] += 1
        self.latest["real_status"] = payload
        raw_actions = payload.get("decision_counts")
        if isinstance(raw_actions, dict):
            actions = {
                str(key): int(value)
                for key, value in raw_actions.items()
                if isinstance(value, int) and value >= 0
            }
            self._accumulate_counter_delta(
                actions,
                self._previous_hardware_actions,
                self.hardware_actions,
            )
            self._previous_hardware_actions = actions
        deadline_misses = payload.get("deadline_miss_count")
        if isinstance(deadline_misses, int) and deadline_misses >= 0:
            if self._previous_deadline_misses is not None:
                self.hardware_deadline_misses += (
                    deadline_misses - self._previous_deadline_misses
                    if deadline_misses >= self._previous_deadline_misses
                    else deadline_misses
                )
            self._previous_deadline_misses = deadline_misses
        error = payload.get("error")
        if isinstance(error, str) and error:
            self.real_errors[error] += 1

    def _on_command(self, side: str, message) -> None:
        values = _positions(message)
        if values is None:
            return
        self.message_counts[f"{side}_commands"] += 1
        previous = self.previous_commands.get(side)
        if previous is not None:
            self.command_steps[side].append(
                max(abs(value - previous[index]) for index, value in enumerate(values))
            )
        self.previous_commands[side] = values
        self.latest["commands_deg"][side] = values

    def _on_feedback(self, side: str, message) -> None:
        values = _positions(message)
        if values is None:
            return
        self.message_counts[f"{side}_feedback"] += 1
        previous = self.previous_feedback.get(side)
        if previous is not None:
            self.feedback_steps[side].append(
                max(abs(value - previous[index]) for index, value in enumerate(values))
            )
        self.previous_feedback[side] = values
        self.latest["feedback_deg"][side] = values
        command = self.latest["commands_deg"].get(side)
        if command is None:
            return
        errors = [
            abs(command[index] - value) for index, value in enumerate(values)
        ]
        self.host_tracking_errors[side].append(max(errors))
        self.host_tracking_joint_max[side] = [
            max(self.host_tracking_joint_max[side][index], error)
            for index, error in enumerate(errors)
        ]

    def _sample(self) -> None:
        record = {
            "type": "sample",
            "elapsed_s": time.monotonic() - self.started,
            **self.latest,
        }
        self.stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        self.stream.flush()

    def run(self, duration: float | None) -> None:
        """主循环：sample_rate_hz 采样落盘；duration 为 None 时直到 Ctrl+C。"""
        interval = 1.0 / self.sample_rate
        next_sample = time.monotonic() + interval
        deadline = None if duration is None else self.started + duration
        while True:
            now = time.monotonic()
            if now >= next_sample:
                self._sample()
                next_sample += interval
            if deadline is not None and now >= deadline:
                return
            target = (
                next_sample
                if deadline is None
                else min(next_sample, deadline)
            )
            time.sleep(max(0.001, target - time.monotonic()))

    def _side_report(self, side: str) -> dict[str, Any]:
        input_total = self.input_samples[side]
        ik_total = self.ik_samples[side]
        joint_max = self.host_tracking_joint_max[side]
        worst_joint = max(range(7), key=joint_max.__getitem__)
        return {
            "input_samples": input_total,
            "input_limit_ratios": {
                flag: _ratio(self.input_flags[side][flag], input_total)
                for flag in INPUT_FLAGS
            },
            "input_values": {
                key: _summary(values)
                for key, values in self.input_values[side].items()
            },
            "ik_samples": ik_total,
            "ik_limit_ratios": {
                flag: _ratio(self.ik_flags[side][flag], ik_total)
                for flag in IK_FLAGS
            },
            "ik_status_counts": dict(self.ik_statuses[side]),
            "ik_errors": dict(self.ik_errors[side]),
            "peak_consecutive_rejections": self.peak_rejections[side],
            "ik_values": {
                key: _summary(values)
                for key, values in self.ik_values[side].items()
            },
            "host_command_step_deg": _summary(self.command_steps[side]),
            "feedback_step_deg": _summary(self.feedback_steps[side]),
            "host_command_to_feedback_error_deg": _summary(
                self.host_tracking_errors[side]
            ),
            "worst_host_tracking_joint": worst_joint + 1,
            "worst_host_tracking_error_deg": joint_max[worst_joint],
        }

    def report(self) -> dict[str, Any]:
        latest_real = self.latest.get("real_status")
        if not isinstance(latest_real, dict):
            latest_real = {}
        maximum_actual_tracking = latest_real.get(
            "maximum_tracking_error_abs_deg"
        )
        if not (
            isinstance(maximum_actual_tracking, list)
            and len(maximum_actual_tracking) == 14
        ):
            maximum_actual_tracking = [0.0] * 14
        report = {
            "schema": SCHEMA,
            "duration_s": time.monotonic() - self.started,
            "message_counts": dict(self.message_counts),
            "hardware": {
                "phase": latest_real.get("phase"),
                "error": latest_real.get("error"),
                "decision_counts_during_capture": dict(
                    self.hardware_actions
                ),
                "deadline_misses_during_capture": (
                    self.hardware_deadline_misses
                ),
                "observed_tick_rate_hz": latest_real.get(
                    "observed_tick_rate_hz"
                ),
                "maximum_tick_interval_ms": latest_real.get(
                    "maximum_tick_interval_ms"
                ),
                "maximum_output_step_deg": latest_real.get(
                    "maximum_output_step_deg"
                ),
                "maximum_observed_output_step_deg": latest_real.get(
                    "maximum_observed_output_step_deg"
                ),
                "maximum_output_speed_deg_s": latest_real.get(
                    "maximum_output_speed_deg_s"
                ),
                "maximum_tracking_error_abs_deg": latest_real.get(
                    "maximum_tracking_error_abs_deg"
                ),
                "velocity_ratios": latest_real.get("velocity_ratios"),
                "acceleration_ratios": latest_real.get(
                    "acceleration_ratios"
                ),
                "errors_seen": dict(self.real_errors),
            },
            "left": self._side_report("left"),
            "right": self._side_report("right"),
        }
        for side_index, side in enumerate(SIDES):
            values = [
                float(value)
                for value in maximum_actual_tracking[
                    side_index * 7 : (side_index + 1) * 7
                ]
                if isinstance(value, (int, float)) and math.isfinite(value)
            ]
            if len(values) == 7:
                worst_joint = max(range(7), key=values.__getitem__)
                report[side]["safe_output_tracking_worst_joint"] = (
                    worst_joint + 1
                )
                report[side]["safe_output_tracking_max_error_deg"] = values[
                    worst_joint
                ]
        report["diagnosis"] = self._diagnose(report)
        return report

    @staticmethod
    def _diagnose(report: dict[str, Any]) -> list[str]:
        findings: list[str] = []
        hardware = report["hardware"]
        actions = hardware["decision_counts_during_capture"]
        decision_total = sum(actions.values())
        output_limited = sum(
            count
            for action, count in actions.items()
            if "output_step_limited" in action
        )
        lead_limited = sum(
            count
            for action, count in actions.items()
            if "tracking_lead_limited" in action
        )
        if _ratio(output_limited, decision_total) >= 0.05:
            findings.append(
                f"真机输出角度斜坡是主要瓶颈：{_ratio(output_limited, decision_total):.1%} 决策触发限幅。"
            )
        if _ratio(lead_limited, decision_total) >= 0.02:
            findings.append(
                f"真机跟踪领先保护在工作：{_ratio(lead_limited, decision_total):.1%} 决策被压回反馈附近。"
            )
        tick_rate = _finite(hardware.get("observed_tick_rate_hz"))
        if tick_rate is not None and tick_rate < 85.0:
            findings.append(
                f"真机桥实际循环仅 {tick_rate:.1f} Hz，低于配置的 90 Hz。"
            )
        if hardware.get("deadline_misses_during_capture", 0) > 0:
            findings.append(
                f"采集期间真机桥出现 {hardware['deadline_misses_during_capture']} 次周期漏期。"
            )

        for side in SIDES:
            label = "左臂" if side == "left" else "右臂"
            data = report[side]
            input_ratios = data["input_limit_ratios"]
            ik_ratios = data["ik_limit_ratios"]
            if input_ratios["workspace_soft_limited"] >= 0.05:
                findings.append(
                    f"{label}受到椭球工作空间压缩：{input_ratios['workspace_soft_limited']:.1%} 输入样本触发。"
                )
            speed_ratio = max(
                input_ratios["linear_speed_limited"],
                input_ratios["angular_speed_limited"],
            )
            if speed_ratio >= 0.10:
                findings.append(
                    f"{label}输入速度上限频繁生效：最高触发比例 {speed_ratio:.1%}。"
                )
            acceleration_ratio = max(
                input_ratios["linear_acceleration_limited"],
                input_ratios["angular_acceleration_limited"],
            )
            if acceleration_ratio >= 0.10:
                findings.append(
                    f"{label}输入加速度整形产生明显缓起感：最高触发比例 {acceleration_ratio:.1%}。"
                )
            if ik_ratios["joint_step_limited"] >= 0.05:
                findings.append(
                    f"{label} IK 关节角单步限幅生效：{ik_ratios['joint_step_limited']:.1%} IK 状态样本触发。"
                )
            if ik_ratios["singularity_active"] > 0.0:
                findings.append(
                    f"{label}进入厂商判定的奇异区域：{ik_ratios['singularity_active']:.1%}。"
                )
            if ik_ratios["workspace_backoff_active"] > 0.0:
                findings.append(
                    f"{label}官方 IK 工作空间回退生效：{ik_ratios['workspace_backoff_active']:.1%}。"
                )
            if ik_ratios["orientation_relaxed"] > 0.0:
                findings.append(
                    f"{label}为保持可解而放松末端姿态：{ik_ratios['orientation_relaxed']:.1%}。"
                )
            if ik_ratios["soft_limit_active"] > 0.0:
                findings.append(
                    f"{label}接近 IK 软关节限位：{ik_ratios['soft_limit_active']:.1%}。"
                )
            if data["peak_consecutive_rejections"] > 0:
                findings.append(
                    f"{label} IK 连续拒绝峰值为 {data['peak_consecutive_rejections']} 帧。"
                )
            if data["ik_errors"]:
                common = max(data["ik_errors"], key=data["ik_errors"].get)
                findings.append(f"{label}最常见 IK 错误：{common}。")
            actual_error = _finite(
                data.get("safe_output_tracking_max_error_deg")
            )
            if actual_error is not None and actual_error >= 2.0:
                findings.append(
                    f"{label}真机安全输出相对反馈最大误差达到 {actual_error:.2f}°，控制器或机械负载可能跟不上。"
                )
        if not findings:
            findings.append(
                "采集期间没有发现持续的软件限幅；优先检查动作是否覆盖问题姿态、控制器内部跟随和机械负载。"
            )
        return findings

    def close(self) -> dict[str, Any]:
        report = self.report()
        self.stream.write(
            json.dumps(
                {"type": "summary", "report": report},
                ensure_ascii=False,
            )
            + "\n"
        )
        self.stream.close()
        return report


def _print_report(report: dict[str, Any], output: Path) -> None:
    print("\n========== 纯手柄真机跟随诊断 ==========")
    print(f"采集时长：{report['duration_s']:.1f} s")
    print(f"原始记录：{output}")
    hardware = report["hardware"]
    print(
        "真机桥："
        f"phase={hardware.get('phase')}，"
        f"实际频率={hardware.get('observed_tick_rate_hz')} Hz，"
        f"周期漏期={hardware.get('deadline_misses_during_capture')}"
    )
    print(
        "真机输出："
        f"配置单步={hardware.get('maximum_output_step_deg')}°，"
        f"观测最大单步={hardware.get('maximum_observed_output_step_deg')}°，"
        f"速度上限={hardware.get('maximum_output_speed_deg_s')}°/s"
    )
    for side in SIDES:
        label = "左臂" if side == "left" else "右臂"
        data = report[side]
        input_ratios = data["input_limit_ratios"]
        ik_ratios = data["ik_limit_ratios"]
        error = data["host_command_to_feedback_error_deg"]
        print(f"\n{label}：")
        print(
            "  输入限制："
            f"工作空间 {input_ratios['workspace_soft_limited']:.1%}，"
            f"线速度 {input_ratios['linear_speed_limited']:.1%}，"
            f"线加速度 {input_ratios['linear_acceleration_limited']:.1%}，"
            f"角速度 {input_ratios['angular_speed_limited']:.1%}，"
            f"角加速度 {input_ratios['angular_acceleration_limited']:.1%}"
        )
        print(
            "  IK 限制："
            f"关节单步 {ik_ratios['joint_step_limited']:.1%}，"
            f"奇异 {ik_ratios['singularity_active']:.1%}，"
            f"回退 {ik_ratios['workspace_backoff_active']:.1%}，"
            f"姿态放松 {ik_ratios['orientation_relaxed']:.1%}，"
            f"软关节限位 {ik_ratios['soft_limit_active']:.1%}"
        )
        print(
            "  主机目标-反馈误差："
            f"P95={error.get('p95')}°，max={error.get('max')}°，"
            f"最差 J{data['worst_host_tracking_joint']}="
            f"{data['worst_host_tracking_error_deg']:.3f}°"
        )
        print(
            "  真机实际下发-反馈最大误差："
            f"{data.get('safe_output_tracking_max_error_deg')}°，"
            f"最差 J{data.get('safe_output_tracking_worst_joint')}"
        )
        print(
            "  IK 连续拒绝峰值："
            f"{data['peak_consecutive_rejections']}"
        )
    print("\n判断：")
    for finding in report["diagnosis"]:
        print(f"  - {finding}")
    print("========================================\n")


def _default_output() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("diagnostics") / f"controller_only_real_{stamp}.jsonl"


def _unified_args_parent() -> argparse.ArgumentParser:
    """parse_cli_args 的统一参数（--config/--param），叠加到本工具 CLI。"""
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--config", default="", help="节点参数 YAML 文件")
    parser.add_argument(
        "--param", action="append", default=[], metavar="key:=value"
    )
    return parser


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="运行期间只读采集纯手柄真机跟随限制并自动诊断",
        parents=[_unified_args_parent()],
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=60.0,
        help="采集秒数；0 表示直到 Ctrl+C（默认 60）",
    )
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args(argv)
    if args.duration < 0.0:
        parser.error("--duration 不能为负数")
    if args.rate <= 0.0:
        parser.error("--rate 必须为正数")

    overrides = {}
    for spec in args.param:
        param_key, value = parse_param_override(spec)
        overrides[param_key] = value
    params = load_node_config(
        args.config,
        "controller_only_real_diagnostic",
        DEFAULT_PARAMETERS,
        overrides,
    )

    output = (args.output or _default_output()).resolve()
    session = open_session()
    collector = None
    try:
        collector = RealDiagnosticCollector(
            session,
            output,
            float(params.get("rate", args.rate)),
        )
        print(
            "只读诊断已开始：请在采集期间复现左右手同时运动、阻尼感和空间边界；"
            "该节点不发布控制命令。"
        )
        duration = None if args.duration == 0.0 else args.duration
        with LiveToken(session, "controller_only_real_diagnostic"):
            collector.run(duration)
    except KeyboardInterrupt:
        pass
    finally:
        if collector is not None:
            report = collector.close()
        session.close()
    _print_report(report, output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
