from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time
from typing import Any, Iterable

from ..zenoh_util import (
    LiveToken,
    ZenohJsonSub,
    ZenohPub,
    ZenohTextSub,
    key,
    load_node_config,
    open_session,
    parse_param_override,
    stamp_ns,
    stamp_now,
)


TRACE_SCHEMA = "pico_body_tianji.controller_only_trace.v1"
SIDES = ("left", "right")
# 工具参数以 CLI 为主；--config/--param 是统一覆盖通道（缺省时回落 CLI 默认）。
DEFAULT_PARAMETERS: dict[str, Any] = {}


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


def load_trace(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    f"trace line {line_number} is not valid JSON"
                ) from exc
            if not isinstance(record, dict):
                raise ValueError(
                    f"trace line {line_number} must be a JSON object"
                )
            records.append(record)
    if not records or records[0].get("schema") != TRACE_SCHEMA:
        raise ValueError(f"trace schema must be {TRACE_SCHEMA}")
    return [record for record in records if record.get("type") == "frame"]


def calculate_metrics(frames: Iterable[dict[str, Any]]) -> dict[str, Any]:
    frames = list(frames)
    times = [
        float(frame["elapsed_s"])
        for frame in frames
        if isinstance(frame.get("elapsed_s"), (int, float))
    ]
    duration = max(times) - min(times) if len(times) >= 2 else 0.0
    result: dict[str, Any] = {
        "frame_count": len(frames),
        "duration_s": duration,
        "effective_rate_hz": (
            (len(times) - 1) / duration if duration > 0.0 else None
        ),
    }
    for side in SIDES:
        solve_times: list[float] = []
        transport_times: list[float] = []
        position_errors: list[float] = []
        orientation_errors: list[float] = []
        minimum_margins: list[float] = []
        requested_steps: list[float] = []
        applied_steps: list[float] = []
        saturation_count = 0
        backoff_count = 0
        soft_limit_count = 0
        restart_count = 0
        rejection_peak = 0
        for frame in frames:
            status = frame.get("ik_status")
            if not isinstance(status, dict):
                continue
            for key, target in (
                (f"{side}_solve_time_ms", solve_times),
                (f"{side}_transport_time_ms", transport_times),
                (f"{side}_position_error_mm", position_errors),
                (f"{side}_orientation_error_deg", orientation_errors),
                (f"{side}_min_limit_margin_deg", minimum_margins),
                (f"{side}_requested_max_joint_step_deg", requested_steps),
                (f"{side}_max_joint_step_deg", applied_steps),
            ):
                value = status.get(key)
                if isinstance(value, (int, float)) and math.isfinite(value):
                    target.append(float(value))
            saturation_count += bool(status.get(f"{side}_target_saturated"))
            backoff_count += bool(
                status.get(f"{side}_workspace_backoff_active")
            )
            soft_limit_count += bool(
                status.get(f"{side}_soft_limit_active")
            )
            restart = status.get(f"{side}_transport_restart_count")
            if isinstance(restart, int):
                restart_count += restart
            rejection = status.get(f"{side}_consecutive_rejections")
            if isinstance(rejection, int):
                rejection_peak = max(rejection_peak, rejection)

        def summary(values: list[float]) -> dict[str, float | None]:
            return {
                "mean": statistics.fmean(values) if values else None,
                "p95": _percentile(values, 0.95),
                "max": max(values) if values else None,
            }

        result[side] = {
            "solve_time_ms": summary(solve_times),
            "transport_time_ms": summary(transport_times),
            "position_error_mm": summary(position_errors),
            "orientation_error_deg": summary(orientation_errors),
            "minimum_limit_margin_deg": (
                min(minimum_margins) if minimum_margins else None
            ),
            "requested_joint_step_deg": summary(requested_steps),
            "applied_joint_step_deg": summary(applied_steps),
            "saturation_frame_ratio": (
                saturation_count / len(frames) if frames else None
            ),
            "workspace_backoff_frames": backoff_count,
            "soft_limit_frames": soft_limit_count,
            "transport_restart_count": restart_count,
            "peak_consecutive_rejections": rejection_peak,
        }
    return result


def _stamp_ns(message: dict[str, Any]) -> int:
    return stamp_ns(message.get("stamp"))


def _pose_payload(message: dict[str, Any]) -> dict[str, Any]:
    return {
        "stamp_ns": _stamp_ns(message),
        "frame_id": message["frame_id"],
        "position": [
            message["position"]["x"],
            message["position"]["y"],
            message["position"]["z"],
        ],
        "quaternion": [
            message["orientation"]["x"],
            message["orientation"]["y"],
            message["orientation"]["z"],
            message["orientation"]["w"],
        ],
    }


def _vector_payload(message: dict[str, Any]) -> dict[str, Any]:
    return {
        "stamp_ns": _stamp_ns(message),
        "frame_id": message["frame_id"],
        "vector": [
            message["vector"]["x"],
            message["vector"]["y"],
            message["vector"]["z"],
        ],
    }


def _json_object(text: str) -> dict[str, Any] | None:
    try:
        value = json.loads(text)
    except json.JSONDecodeError:
        return None
    return value if isinstance(value, dict) else None


class TraceRecorder:
    def __init__(self, session, output: Path, rate_hz: float):
        self.session = session
        self.output = output
        self.rate = rate_hz
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.output.open("x", encoding="utf-8")
        self.started = time.monotonic()
        self.latest: dict[str, Any] = {
            "target": {},
            "elbow": {},
            "solved": {},
            "joints_deg": {},
            "input_status": None,
            "ik_status": None,
            "teleop_state": None,
        }
        self._last_target_stamps: tuple[int | None, int | None] = (None, None)
        self.stream.write(
            json.dumps(
                {
                    "schema": TRACE_SCHEMA,
                    "type": "metadata",
                    "created_at": datetime.now(timezone.utc).isoformat(),
                    "rate_hz": rate_hz,
                },
                ensure_ascii=False,
            )
            + "\n"
        )
        for side in SIDES:
            ZenohJsonSub(
                session,
                key(f"/pico_body/{side}_arm_target_pose"),
                lambda message, side=side: self.latest["target"].__setitem__(
                    side, _pose_payload(message)
                ),
            )
            ZenohJsonSub(
                session,
                key(f"/pico_body/{side}_arm_elbow_direction"),
                lambda message, side=side: self.latest["elbow"].__setitem__(
                    side, _vector_payload(message)
                ),
            )
            ZenohJsonSub(
                session,
                key(f"/pico_body_sim/{side}_arm/solved_pose"),
                lambda message, side=side: self.latest["solved"].__setitem__(
                    side, _pose_payload(message)
                ),
            )
            ZenohJsonSub(
                session,
                key(f"/pico_body_sim/{side}_arm/joint_commands"),
                lambda message, side=side: self.latest[
                    "joints_deg"
                ].__setitem__(side, list(message["position"])),
            )
        ZenohTextSub(
            session,
            key("/pico_body/status"),
            lambda text: self.latest.__setitem__(
                "input_status", _json_object(text)
            ),
        )
        ZenohTextSub(
            session,
            key("/pico_body_sim/status"),
            lambda text: self.latest.__setitem__(
                "ik_status", _json_object(text)
            ),
        )
        ZenohTextSub(
            session,
            key("/pico_body/teleop_state"),
            lambda text: self.latest.__setitem__("teleop_state", text),
        )

    def run(self) -> None:
        """主循环：rate Hz 采样落盘（替代 ROS Timer 节流）。"""
        interval = 1.0 / self.rate
        next_sample = time.monotonic() + interval
        while True:
            now = time.monotonic()
            if now >= next_sample:
                self.sample()
                next_sample += interval
            time.sleep(max(0.001, next_sample - time.monotonic()))

    def sample(self) -> None:
        stamps = tuple(
            self.latest["target"].get(side, {}).get("stamp_ns")
            for side in SIDES
        )
        if stamps == self._last_target_stamps or None in stamps:
            return
        self._last_target_stamps = stamps
        frame = {
            "type": "frame",
            "elapsed_s": time.monotonic() - self.started,
            **self.latest,
        }
        self.stream.write(json.dumps(frame, ensure_ascii=False) + "\n")
        self.stream.flush()

    def close(self) -> None:
        self.stream.close()


@dataclass
class ReplayFrame:
    elapsed_s: float
    payload: dict[str, Any]


class TraceReplay:
    def __init__(self, session, frames: list[dict[str, Any]], speed: float):
        if speed <= 0.0:
            raise ValueError("replay speed must be positive")
        self.session = session
        self.frames = [
            ReplayFrame(float(frame["elapsed_s"]), frame) for frame in frames
        ]
        if not self.frames:
            raise ValueError("trace contains no frames")
        self.speed = speed
        self.index = 0
        self.phase = "arming"
        self.started = time.monotonic()
        self.pose_publishers = {
            side: ZenohPub(session, key(f"/pico_body/{side}_arm_target_pose"))
            for side in SIDES
        }
        self.elbow_publishers = {
            side: ZenohPub(
                session, key(f"/pico_body/{side}_arm_elbow_direction")
            )
            for side in SIDES
        }
        self.state_publisher = ZenohPub(
            session, key("/pico_body/teleop_state")
        )
        self.status_publisher = ZenohPub(session, key("/pico_body/status"))

    def _publish_state(self, state: str) -> None:
        self.state_publisher.put_text(state)
        self.status_publisher.put_text(
            json.dumps(
                {
                    "state": state,
                    "source": "offline_replay",
                    "input": "controller_only_trace",
                    "scope": "controller_only_replay",
                    "error": None,
                }
            )
        )

    def _publish_frame(self, frame: ReplayFrame) -> None:
        stamp = stamp_now()
        for side in SIDES:
            pose_data = frame.payload.get("target", {}).get(side)
            if isinstance(pose_data, dict):
                self.pose_publishers[side].put_json(
                    {
                        "stamp": stamp,
                        "frame_id": str(
                            pose_data.get("frame_id", f"{side}_chest")
                        ),
                        "position": {
                            "x": float(pose_data["position"][0]),
                            "y": float(pose_data["position"][1]),
                            "z": float(pose_data["position"][2]),
                        },
                        "orientation": {
                            "x": float(pose_data["quaternion"][0]),
                            "y": float(pose_data["quaternion"][1]),
                            "z": float(pose_data["quaternion"][2]),
                            "w": float(pose_data["quaternion"][3]),
                        },
                    }
                )
            elbow_data = frame.payload.get("elbow", {}).get(side)
            if isinstance(elbow_data, dict):
                self.elbow_publishers[side].put_json(
                    {
                        "stamp": stamp,
                        "frame_id": str(
                            elbow_data.get("frame_id", f"{side}_chest")
                        ),
                        "vector": {
                            "x": float(elbow_data["vector"][0]),
                            "y": float(elbow_data["vector"][1]),
                            "z": float(elbow_data["vector"][2]),
                        },
                    }
                )

    def tick(self) -> bool:
        """推进一个 0.005 s 节拍；返回 True 表示回放结束。"""
        now = time.monotonic()
        if self.phase == "arming":
            self._publish_state("idle")
            if now - self.started >= 1.0:
                self.phase = "replaying"
                self.started = now
                self._publish_state("teleop")
            return False
        if self.phase == "returning":
            self._publish_state("returning")
            if now - self.started >= 3.0:
                print("离线 replay 完成并已请求回 Home")
                return True
            return False
        elapsed = (now - self.started) * self.speed
        base = self.frames[0].elapsed_s
        while (
            self.index < len(self.frames)
            and self.frames[self.index].elapsed_s - base <= elapsed
        ):
            self._publish_frame(self.frames[self.index])
            self.index += 1
        self._publish_state("teleop")
        if self.index >= len(self.frames):
            self.phase = "returning"
            self.started = now
        return False

    def run(self) -> None:
        """主循环：0.005 s 节拍驱动 tick（替代 ROS Timer）。"""
        while True:
            if self.tick():
                return
            time.sleep(0.005)

    def close(self) -> None:
        for publisher in self.elbow_publishers.values():
            publisher.close()
        for publisher in self.pose_publishers.values():
            publisher.close()
        self.state_publisher.close()
        self.status_publisher.close()


def _assert_replay_graph_is_safe(session) -> None:
    """回放前确认系统图中没有真机桥/实时输入节点。

    基于 zenoh liveliness 注册（tj/live/<node-name>）；对没有 liveliness
    接口的旧式图查询对象（单元测试夹具）回退到 get_node_names_and_namespaces。
    """
    deadline = time.monotonic() + 1.0
    names: list[str] = []
    if hasattr(session, "liveliness"):
        while time.monotonic() < deadline:
            replies = session.liveliness().get("tj/live/**", timeout=0.5)
            names = [
                str(reply.result.key_expr).split("/")[-1]
                for reply in replies
                if reply.ok
            ]
            if names:
                break
            time.sleep(0.05)
    else:
        names = [
            name for name, _namespace in session.get_node_names_and_namespaces()
        ]
    forbidden = {
        "marvin_hardware_bridge",
        "tianji_world_output_node",
        "tianji_arm_node",
        "pico_controller_only_input",
        "pico_controller_input",
    }
    conflicts = sorted(forbidden.intersection(names))
    if conflicts:
        raise RuntimeError(
            "replay 拒绝启动，系统图中存在真机桥或实时输入节点："
            + ", ".join(conflicts)
        )


def _load_params(args) -> dict[str, Any]:
    """统一参数通道：--config yaml + --param key:=value（缺省回落 CLI 默认）。"""
    overrides = {}
    for spec in args.param:
        param_key, value = parse_param_override(spec)
        overrides[param_key] = value
    return load_node_config(
        args.config,
        "controller_only_trace",
        DEFAULT_PARAMETERS,
        overrides,
    )


def _run_record(args) -> int:
    params = _load_params(args)
    session = open_session()
    recorder = TraceRecorder(
        session, args.output.resolve(), float(params.get("rate", args.rate))
    )
    print(f"开始只读记录：{recorder.output}")
    try:
        with LiveToken(session, "controller_only_trace_recorder"):
            recorder.run()
    except KeyboardInterrupt:
        pass
    finally:
        recorder.close()
        session.close()
    return 0


def _run_replay(args) -> int:
    params = _load_params(args)
    frames = load_trace(args.trace.resolve())
    session = open_session()
    replay = None
    try:
        _assert_replay_graph_is_safe(session)
        replay = TraceReplay(
            session, frames, float(params.get("speed", args.speed))
        )
        print("开始 preview-only 离线 replay；该身份不能通过真机 readiness")
        with LiveToken(session, "controller_only_trace_replay"):
            replay.run()
    except KeyboardInterrupt:
        pass
    finally:
        if replay is not None:
            replay.close()
        session.close()
    return 0


def _run_metrics(args) -> int:
    metrics = calculate_metrics(load_trace(args.trace.resolve()))
    print(json.dumps(metrics, indent=2, ensure_ascii=False))
    return 0


def _unified_args_parent() -> argparse.ArgumentParser:
    """parse_cli_args 的统一参数（--config/--param），叠加到各子命令。"""
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--config", default="", help="节点参数 YAML 文件")
    parser.add_argument(
        "--param", action="append", default=[], metavar="key:=value"
    )
    return parser


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="controller-only IK JSONL 记录、离线回放与指标统计"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    record = subparsers.add_parser(
        "record", help="只读记录现有仿真话题", parents=[_unified_args_parent()]
    )
    record.add_argument("--output", required=True, type=Path)
    record.add_argument("--rate", type=float, default=90.0)
    record.set_defaults(handler=_run_record)
    replay = subparsers.add_parser(
        "replay", help="发布 preview-only 目标轨迹", parents=[_unified_args_parent()]
    )
    replay.add_argument("trace", type=Path)
    replay.add_argument("--speed", type=float, default=1.0)
    replay.set_defaults(handler=_run_replay)
    metrics = subparsers.add_parser(
        "metrics", help="输出轨迹质量指标", parents=[_unified_args_parent()]
    )
    metrics.add_argument("trace", type=Path)
    metrics.set_defaults(handler=_run_metrics)
    args = parser.parse_args(argv)
    return int(args.handler(args))


if __name__ == "__main__":
    raise SystemExit(main())
