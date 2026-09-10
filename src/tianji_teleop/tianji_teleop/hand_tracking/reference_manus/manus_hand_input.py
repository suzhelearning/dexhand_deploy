#!/usr/bin/env python3
"""Publish semantic Manus rawviz frames as the ROS2 ``/hand_input`` stream."""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from typing import Callable

import numpy as np

from .wuji2_hand_input import (
    Wuji2MappingError,
    parse_rawviz_line,
    resolve_wuji2_keypoints,
)


@dataclass(frozen=True)
class HandInputFrame:
    """One complete ROS2 hand-input payload and its source metadata."""

    values: np.ndarray
    sequences: dict[str, int]
    source_timestamps_ns: dict[str, int]


class HandInputAssembler:
    """Hold the latest complete Manus sides in the established ROS order."""

    def __init__(self, include_right_hand: bool = True, include_left_hand: bool = True):
        if not (include_right_hand or include_left_hand):
            raise ValueError("at least one hand must be enabled")
        self.include_right_hand = bool(include_right_hand)
        self.include_left_hand = bool(include_left_hand)
        self._latest: dict[str, tuple[np.ndarray, int, int]] = {}
        self._new_data_received = False

    @property
    def enabled_sides(self) -> tuple[str, ...]:
        return tuple(
            side
            for side, enabled in (
                ("right", self.include_right_hand),
                ("left", self.include_left_hand),
            )
            if enabled
        )

    def update(
        self,
        side: str,
        landmarks: np.ndarray,
        sequence: int,
        source_timestamp_ns: int,
    ) -> None:
        side = str(side).strip().lower()
        if side not in {"left", "right"}:
            raise ValueError(f"side must be left or right, got {side!r}")
        if side not in self.enabled_sides:
            return
        values = np.asarray(landmarks, dtype=np.float32)
        if values.shape != (21, 3):
            raise ValueError(f"landmarks must have shape (21, 3), got {values.shape}")
        if not np.isfinite(values).all():
            raise ValueError("landmarks contain NaN or infinity")

        sequence = int(sequence)
        previous = self._latest.get(side)
        if previous is not None and sequence <= previous[1]:
            return
        self._latest[side] = (values.copy(), sequence, int(source_timestamp_ns))
        self._new_data_received = True

    def take_latest(self) -> HandInputFrame | None:
        if not self._new_data_received:
            return None
        if any(side not in self._latest for side in self.enabled_sides):
            return None

        values = np.concatenate(
            [self._latest[side][0].reshape(-1) for side in self.enabled_sides]
        ).astype(np.float32, copy=False)
        sequences = {side: self._latest[side][1] for side in self.enabled_sides}
        timestamps = {
            side: self._latest[side][2] for side in self.enabled_sides
        }
        self._new_data_received = False
        return HandInputFrame(values=values.copy(), sequences=sequences, source_timestamps_ns=timestamps)

    def invalidate(self, side: str) -> None:
        """Drop the last valid frame for a side until a new frame arrives."""

        side = str(side).strip().lower()
        if side not in {"left", "right"}:
            raise ValueError(f"side must be left or right, got {side!r}")
        self._latest.pop(side, None)
        self._new_data_received = False


class RawvizHandInputProcessor:
    """Convert rawviz stdin records and invoke a callback for each payload."""

    def __init__(
        self,
        assembler: HandInputAssembler,
        publish: Callable[[HandInputFrame], None],
        right_glove: str | None = None,
        left_glove: str | None = None,
    ):
        self.assembler = assembler
        self.publish = publish
        self.glove_sides = {
            glove: side
            for glove, side in (
                (right_glove, "right"),
                (left_glove, "left"),
            )
            if glove
        }
        self._streams: dict[str, dict[str, object]] = {}

    def process_line(self, line: str) -> None:
        try:
            event = parse_rawviz_line(line)
        except ValueError:
            return
        if event is None:
            return

        kind = event["kind"]
        if kind == "hand":
            glove_id = str(event["glove_id"])
            side = self.glove_sides.get(glove_id, str(event["side"]).lower())
            if side not in {"left", "right"}:
                return
            self._streams[glove_id] = {
                "side": side,
                "node_count": int(event["node_count"]),
                "nodes": [],
            }
            return

        if kind == "node":
            glove_id = str(event["glove_id"])
            stream = self._streams.get(glove_id)
            if stream is None:
                return
            nodes = stream["nodes"]
            assert isinstance(nodes, list)
            nodes.append({
                "array_index": int(event["array_index"]),
                "node_id": int(event["node_id"]),
                "parent_id": int(event["parent_id"]),
                "chain_type": int(event["chain_type"]),
                "side": int(event["side"]),
                "finger_joint_type": int(event["finger_joint_type"]),
            })
            return

        if kind != "pose":
            return
        glove_id = str(event["glove_id"])
        stream = self._streams.get(glove_id)
        if stream is None:
            return
        node_count = int(stream["node_count"])
        positions = np.asarray(event["positions"], dtype=np.float32)
        if positions.shape != (node_count, 3):
            self.assembler.invalidate(str(stream["side"]))
            return
        try:
            landmarks = resolve_wuji2_keypoints(stream["nodes"], positions)
            self.assembler.update(
                str(stream["side"]),
                landmarks,
                int(event["sequence"]),
                int(event["source_monotonic_ns"]),
            )
        except (ValueError, Wuji2MappingError):
            self.assembler.invalidate(str(stream["side"]))
            return
        frame = self.assembler.take_latest()
        if frame is not None:
            self.publish(frame)


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish semantic Manus rawviz frames on ROS2 /hand_input"
    )
    parser.add_argument("--right-glove", help="rawviz glove ID for the right hand")
    parser.add_argument("--left-glove", help="rawviz glove ID for the left hand")
    parser.add_argument("--topic", default="/hand_input")
    parser.add_argument("--publish-rate-hz", type=float, default=120.0)
    parser.add_argument("--right-only", action="store_true")
    parser.add_argument("--left-only", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    raw_argv = sys.argv if argv is None else [sys.argv[0], *argv]
    try:
        import rclpy
        from rclpy.utilities import remove_ros_args
    except ImportError as exc:
        raise RuntimeError("ROS2 rclpy is required") from exc

    args = _parse_args(remove_ros_args(raw_argv)[1:])
    if args.right_only and args.left_only:
        raise ValueError("--right-only and --left-only cannot be combined")
    include_right = not args.left_only
    include_left = not args.right_only
    assembler = HandInputAssembler(include_right, include_left)

    try:
        from rclpy.node import Node
        from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
        from std_msgs.msg import Float32MultiArray
    except ImportError as exc:
        raise RuntimeError("ROS2 rclpy and std_msgs are required") from exc

    rclpy.init(args=raw_argv)

    class PublisherNode(Node):
        def __init__(self) -> None:
            super().__init__("manus_wuji2_hand_input")
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.publisher = self.create_publisher(Float32MultiArray, args.topic, qos)

        def publish_frame(self, frame: HandInputFrame) -> None:
            message = Float32MultiArray()
            message.data = frame.values.tolist()
            self.publisher.publish(message)

    node = PublisherNode()
    processor = RawvizHandInputProcessor(
        assembler,
        node.publish_frame,
        right_glove=args.right_glove,
        left_glove=args.left_glove,
    )
    try:
        for line in sys.stdin:
            processor.process_line(line)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
