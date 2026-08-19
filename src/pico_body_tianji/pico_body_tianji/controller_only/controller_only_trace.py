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


TRACE_SCHEMA = "pico_body_tianji.controller_only_trace.v1"
SIDES = ("left", "right")


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


def _stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def _pose_payload(message) -> dict[str, Any]:
    return {
        "stamp_ns": _stamp_ns(message),
        "frame_id": message.header.frame_id,
        "position": [
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        ],
        "quaternion": [
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ],
    }


def _vector_payload(message) -> dict[str, Any]:
    return {
        "stamp_ns": _stamp_ns(message),
        "frame_id": message.header.frame_id,
        "vector": [message.vector.x, message.vector.y, message.vector.z],
    }


def _json_object(text: str) -> dict[str, Any] | None:
    try:
        value = json.loads(text)
    except json.JSONDecodeError:
        return None
    return value if isinstance(value, dict) else None


class TraceRecorder:
    def __init__(self, node, output: Path, rate_hz: float):
        from geometry_msgs.msg import PoseStamped, Vector3Stamped
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String

        self.node = node
        self.output = output
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
            node.create_subscription(
                PoseStamped,
                f"/pico_body/{side}_arm_target_pose",
                lambda message, side=side: self.latest["target"].__setitem__(
                    side, _pose_payload(message)
                ),
                10,
            )
            node.create_subscription(
                Vector3Stamped,
                f"/pico_body/{side}_arm_elbow_direction",
                lambda message, side=side: self.latest["elbow"].__setitem__(
                    side, _vector_payload(message)
                ),
                10,
            )
            node.create_subscription(
                PoseStamped,
                f"/pico_body_sim/{side}_arm/solved_pose",
                lambda message, side=side: self.latest["solved"].__setitem__(
                    side, _pose_payload(message)
                ),
                10,
            )
            node.create_subscription(
                JointState,
                f"/pico_body_sim/{side}_arm/joint_commands",
                lambda message, side=side: self.latest[
                    "joints_deg"
                ].__setitem__(side, list(message.position)),
                10,
            )
        node.create_subscription(
            String,
            "/pico_body/status",
            lambda message: self.latest.__setitem__(
                "input_status", _json_object(message.data)
            ),
            10,
        )
        node.create_subscription(
            String,
            "/pico_body_sim/status",
            lambda message: self.latest.__setitem__(
                "ik_status", _json_object(message.data)
            ),
            10,
        )
        node.create_subscription(
            String,
            "/pico_body/teleop_state",
            lambda message: self.latest.__setitem__(
                "teleop_state", message.data
            ),
            10,
        )
        node.create_timer(1.0 / rate_hz, self.sample)

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
    def __init__(self, node, frames: list[dict[str, Any]], speed: float):
        from geometry_msgs.msg import PoseStamped, Vector3Stamped
        from std_msgs.msg import String

        if speed <= 0.0:
            raise ValueError("replay speed must be positive")
        self.node = node
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
            side: node.create_publisher(
                PoseStamped, f"/pico_body/{side}_arm_target_pose", 10
            )
            for side in SIDES
        }
        self.elbow_publishers = {
            side: node.create_publisher(
                Vector3Stamped,
                f"/pico_body/{side}_arm_elbow_direction",
                10,
            )
            for side in SIDES
        }
        self.state_publisher = node.create_publisher(
            String, "/pico_body/teleop_state", 10
        )
        self.status_publisher = node.create_publisher(
            String, "/pico_body/status", 10
        )
        node.create_timer(0.005, self.tick)

    def _publish_state(self, state: str) -> None:
        from std_msgs.msg import String

        self.state_publisher.publish(String(data=state))
        self.status_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "state": state,
                        "source": "offline_replay",
                        "input": "controller_only_trace",
                        "scope": "controller_only_replay",
                        "error": None,
                    }
                )
            )
        )

    def _publish_frame(self, frame: ReplayFrame) -> None:
        from geometry_msgs.msg import PoseStamped, Vector3Stamped

        stamp = self.node.get_clock().now().to_msg()
        for side in SIDES:
            pose_data = frame.payload.get("target", {}).get(side)
            if isinstance(pose_data, dict):
                message = PoseStamped()
                message.header.stamp = stamp
                message.header.frame_id = str(
                    pose_data.get("frame_id", f"{side}_chest")
                )
                position = pose_data["position"]
                quaternion = pose_data["quaternion"]
                (
                    message.pose.position.x,
                    message.pose.position.y,
                    message.pose.position.z,
                ) = map(float, position)
                (
                    message.pose.orientation.x,
                    message.pose.orientation.y,
                    message.pose.orientation.z,
                    message.pose.orientation.w,
                ) = map(float, quaternion)
                self.pose_publishers[side].publish(message)
            elbow_data = frame.payload.get("elbow", {}).get(side)
            if isinstance(elbow_data, dict):
                message = Vector3Stamped()
                message.header.stamp = stamp
                message.header.frame_id = str(
                    elbow_data.get("frame_id", f"{side}_chest")
                )
                (
                    message.vector.x,
                    message.vector.y,
                    message.vector.z,
                ) = map(float, elbow_data["vector"])
                self.elbow_publishers[side].publish(message)

    def tick(self) -> None:
        now = time.monotonic()
        if self.phase == "arming":
            self._publish_state("idle")
            if now - self.started >= 1.0:
                self.phase = "replaying"
                self.started = now
                self._publish_state("teleop")
            return
        if self.phase == "returning":
            self._publish_state("returning")
            if now - self.started >= 3.0:
                self.node.get_logger().info("离线 replay 完成并已请求回 Home")
                # context.try_shutdown() 在本机 rclpy(Humble) 的 executor
                # 回调内会死锁，导致回放结束不退出；改用 SystemExit 从
                # spin 的 while 循环中直接抛出（main 中捕获并正常清理）。
                raise SystemExit(0)
            return
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


def _assert_replay_graph_is_safe(node) -> None:
    deadline = time.monotonic() + 1.0
    names: list[str] = []
    while time.monotonic() < deadline:
        names = [
            name for name, _namespace in node.get_node_names_and_namespaces()
        ]
        if names:
            break
        time.sleep(0.05)
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
            "replay 拒绝启动，ROS 图中存在真机桥或实时输入节点："
            + ", ".join(conflicts)
        )


def _run_record(args) -> int:
    import rclpy

    rclpy.init()
    node = rclpy.create_node("controller_only_trace_recorder")
    recorder = TraceRecorder(node, args.output.resolve(), args.rate)
    node.get_logger().info(f"开始只读记录：{recorder.output}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def _run_replay(args) -> int:
    import rclpy

    frames = load_trace(args.trace.resolve())
    rclpy.init()
    node = rclpy.create_node("controller_only_trace_replay")
    try:
        _assert_replay_graph_is_safe(node)
        TraceReplay(node, frames, args.speed)
        node.get_logger().warning(
            "开始 preview-only 离线 replay；该身份不能通过真机 readiness"
        )
        try:
            rclpy.spin(node)
        except SystemExit:
            return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def _run_metrics(args) -> int:
    metrics = calculate_metrics(load_trace(args.trace.resolve()))
    print(json.dumps(metrics, indent=2, ensure_ascii=False))
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="controller-only IK JSONL 记录、离线回放与指标统计"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    record = subparsers.add_parser("record", help="只读记录现有仿真话题")
    record.add_argument("--output", required=True, type=Path)
    record.add_argument("--rate", type=float, default=90.0)
    record.set_defaults(handler=_run_record)
    replay = subparsers.add_parser("replay", help="发布 preview-only 目标轨迹")
    replay.add_argument("trace", type=Path)
    replay.add_argument("--speed", type=float, default=1.0)
    replay.set_defaults(handler=_run_replay)
    metrics = subparsers.add_parser("metrics", help="输出轨迹质量指标")
    metrics.add_argument("trace", type=Path)
    metrics.set_defaults(handler=_run_metrics)
    args = parser.parse_args(argv)
    return int(args.handler(args))


if __name__ == "__main__":
    raise SystemExit(main())
