#!/usr/bin/env python3
"""Check that one stage of the embedded legacy PICO ROS graph is alive."""

from __future__ import annotations

import argparse
import os
import sys
import time
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any


DRIVER_TOPICS = ("/pico/smpl_raw",)
RAW_TOPICS = DRIVER_TOPICS
M0_TOPICS = DRIVER_TOPICS + (
    "/pico/palm_left",
    "/pico/palm_right",
    "/pico/smpl_palm_corrected",
    "/pico/smpl_palm_corrected_ik",
    "/pico/smpl_palm_corrected/status",
    "/pico/tracking_epoch",
    "/pico/tracking_epoch/status",
)
BRIDGE_TOPICS = ("/pico/tianji_mujoco_teleop/status",)


@dataclass(frozen=True)
class StreamRequirement:
    topic: str
    message_type: type
    qos: Any


class ReadinessTracker:
    def __init__(self, required_topics: Iterable[str]) -> None:
        self._required = tuple(required_topics)
        self._received: set[str] = set()

    def mark_received(self, topic: str) -> None:
        if topic in self._required:
            self._received.add(topic)

    @property
    def missing(self) -> tuple[str, ...]:
        return tuple(topic for topic in self._required if topic not in self._received)

    @property
    def ready(self) -> bool:
        return not self.missing


def _requirements(mode: str) -> tuple[StreamRequirement, ...]:
    # Keep ROS imports inside the runtime function. Invalid CLI arguments must
    # fail before a ROS context is initialized or an external graph is touched.
    from geometry_msgs.msg import PoseArray, PoseStamped
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from std_msgs.msg import String, UInt64

    reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
    transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
    driver = (StreamRequirement(DRIVER_TOPICS[0], PoseArray, qos_profile_sensor_data),)
    palm = (
        StreamRequirement(M0_TOPICS[1], PoseStamped, qos_profile_sensor_data),
        StreamRequirement(M0_TOPICS[2], PoseStamped, qos_profile_sensor_data),
    )
    if mode == "raw":
        return driver
    if mode == "m0":
        return driver + palm + (
            StreamRequirement(M0_TOPICS[3], PoseArray, qos_profile_sensor_data),
            StreamRequirement(M0_TOPICS[4], PoseArray, qos_profile_sensor_data),
            StreamRequirement(M0_TOPICS[5], String, reliable),
            StreamRequirement(M0_TOPICS[6], UInt64, transient),
            StreamRequirement(M0_TOPICS[7], String, transient),
        )
    if mode == "bridge":
        return (StreamRequirement(BRIDGE_TOPICS[0], String, reliable),)
    raise ValueError(f"unknown preflight mode: {mode}")


def wait_for_streams(mode: str, timeout_s: float, domain: int) -> tuple[float, tuple[str, ...]]:
    if not 0 <= domain <= 232:
        raise ValueError("ROS domain must satisfy 0 <= domain <= 232")
    if timeout_s < 0.0:
        raise ValueError("--timeout-s must be non-negative")

    # Import rclpy only after all non-I/O validation above has succeeded.
    import rclpy
    from rclpy.node import Node

    os.environ["ROS_DOMAIN_ID"] = str(domain)
    requirements = _requirements(mode)
    tracker = ReadinessTracker(requirement.topic for requirement in requirements)
    node = None
    started = time.monotonic()
    try:
        rclpy.init()
        node = Node(f"embedded_pico_{mode}_preflight_{os.getpid()}")
        subscriptions = []
        for requirement in requirements:
            subscriptions.append(
                node.create_subscription(
                    requirement.message_type,
                    requirement.topic,
                    lambda _message, stream=requirement.topic: tracker.mark_received(
                        stream
                    ),
                    requirement.qos,
                )
            )
        deadline = started + timeout_s
        while not tracker.ready:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(node, timeout_sec=min(0.05, remaining))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return time.monotonic() - started, tracker.missing


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("raw", "m0", "bridge"), required=True)
    parser.add_argument("--timeout-s", type=float, default=20.0)
    parser.add_argument("--domain", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "120")))
    arguments = parser.parse_args(argv)

    try:
        if not 0 <= arguments.domain <= 232:
            raise ValueError("ROS domain must satisfy 0 <= domain <= 232")
        if arguments.timeout_s < 0.0:
            raise ValueError("--timeout-s must be non-negative")
        elapsed_s, missing = wait_for_streams(
            arguments.mode, arguments.timeout_s, arguments.domain
        )
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    if missing:
        print(
            f"PICO {arguments.mode} readiness failed after {elapsed_s:.2f}s; "
            f"missing messages: {', '.join(missing)}",
            file=os.sys.stderr,
        )
        return 2
    print(f"PICO {arguments.mode} streams ready ({elapsed_s:.2f}s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
