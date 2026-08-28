"""Runnable CLI for session-v1 target/joint replay profiles."""
from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

from ..zenoh_util import open_session, require_single_router
from .replay import JointReplayNode, TargetReplaySource


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="session-v1 replay source")
    parser.add_argument("mode", choices=("target", "joint"))
    parser.add_argument("recording", type=Path)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--rate", type=float, default=60.0)
    parser.add_argument("--active-sides", default="left,right")
    parser.add_argument("--inactive-sides", default="")
    parser.add_argument("--record", default=None, help="rejected: replay profiles cannot be recorded")
    parser.add_argument("--inactive-hand-sides", default="left,right")
    return parser


def _sides(raw: str) -> tuple[str, ...]:
    return tuple(item for item in (value.strip() for value in raw.split(",")) if item)


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.record is not None:
        print("replay profile cannot be recorded", file=__import__("sys").stderr)
        return 2
    if not args.headless:
        raise SystemExit("replay profile requires --headless")
    instance = os.environ.get("TIANJI_COMPONENT_INSTANCE_ID")
    coordinator = os.environ.get("TIANJI_COORDINATOR_INSTANCE_ID")
    if not instance or not coordinator:
        raise SystemExit("TIANJI_COMPONENT_INSTANCE_ID and TIANJI_COORDINATOR_INSTANCE_ID are required")
    session = open_session()
    node = None
    try:
        router = require_single_router(session, os.environ.get("TIANJI_ROUTER_ZID"))
        common = dict(
            session=session,
            router_zid=router,
            active_sides=_sides(args.active_sides),
            inactive_sides=_sides(args.inactive_sides),
            active_hand_sides=_sides(args.active_hand_sides),
            inactive_hand_sides=_sides(args.inactive_hand_sides),
            rate_hz=args.rate,
            expected_coordinator_instance_id=coordinator,
        )
        if args.mode == "target":
            node = TargetReplaySource(args.recording, publisher_instance_id=instance, **common)
        else:
            node = JointReplayNode(
                args.recording,
                source_publisher_instance_id=instance,
                producer_publisher_instance_id=os.environ.get("TIANJI_PRODUCER_INSTANCE_ID", instance),
                **common,
            )
        node.start()
        while True:
            node.tick()
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        return 0
    finally:
        if node is not None:
            node.close()
        session.close()


__all__ = ["main"]

if __name__ == "__main__":
    raise SystemExit(main())
