"""Runnable CLI for session-v1 target/joint replay profiles."""
from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

from ..zenoh_util import open_session, require_single_router
from .replay import JointReplayNode, TargetReplaySource, validate_direct_real_recording



def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="session-v1 replay source")
    parser.add_argument("mode", choices=("target", "joint"))
    parser.add_argument("recording", type=Path)
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--rate", type=float, default=None)
    parser.add_argument("--active-sides", default="left,right")
    parser.add_argument("--inactive-sides", default="")
    parser.add_argument("--active-hand-sides", default="")
    parser.add_argument("--inactive-hand-sides", default="left,right")
    parser.add_argument("--record", default=None, help="rejected: replay profiles cannot be recorded")
    parser.add_argument(
        "--auto-start",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="headless mode requests coordinator authorization before replay (default: true)",
    )
    parser.add_argument("--pause-after", type=float, default=None, help="pause after this many seconds")
    parser.add_argument("--resume-after", type=float, default=None, help="resume after this many seconds")
    parser.add_argument("--return-after", type=float, default=None, help="request return after this many seconds")
    return parser

def _sides(raw: str) -> tuple[str, ...]:
    return tuple(item for item in (value.strip() for value in raw.split(",")) if item)


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.record is not None:
        print("replay profile cannot be recorded", file=__import__("sys").stderr)
        return 2
    import yaml
    replay_config = {}
    if args.config is not None:
        replay_config = yaml.safe_load(args.config.read_text(encoding="utf-8")) or {}
    capability = str(os.environ.get("TIANJI_REQUIRED_CAPABILITY", replay_config.get("required_capability", "simulation")))
    if capability not in {"simulation", "real"}:
        raise SystemExit(f"unsupported replay capability: {capability}")
    active_sides = _sides(args.active_sides)
    inactive_sides = _sides(args.inactive_sides)
    active_hand_sides = _sides(args.active_hand_sides)
    if args.mode == "joint" and capability == "real" and not active_hand_sides:
        active_hand_sides = ("left", "right")
    if args.mode == "joint" and capability == "real":
        if replay_config.get("real_preflight") is not True:
            raise SystemExit("direct real replay requires explicit real_preflight=true")
        try:
            validate_direct_real_recording(args.recording, active_sides=active_sides, active_hand_sides=active_hand_sides)
        except ValueError as exc:
            raise SystemExit(f"direct real replay preflight failed: {exc}") from exc
    capabilities = ("simulation",) if capability == "simulation" else ("real", "simulation")
    rate = float(args.rate or replay_config.get("rate_hz", 60.0))
    if rate <= 0:
        raise SystemExit("replay rate must be positive")
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
            active_sides=active_sides,
            inactive_sides=inactive_sides,
            active_hand_sides=active_hand_sides,
            inactive_hand_sides=_sides(args.inactive_hand_sides),
            rate_hz=rate,
            expected_coordinator_instance_id=coordinator,
        )
        if args.mode == "target":
            node = TargetReplaySource(args.recording, publisher_instance_id=instance, **common)
        else:
            node = JointReplayNode(
                args.recording,
                source_publisher_instance_id=instance,
                producer_publisher_instance_id=os.environ.get("TIANJI_PRODUCER_INSTANCE_ID", instance),
                capabilities=capabilities,
                **common,
            )
        node.start()
        started_at = time.monotonic()
        paused = False
        returned = False
        if args.auto_start:
            try:
                node.request_start()
            except (TimeoutError, RuntimeError) as exc:
                raise SystemExit(f"replay start authorization failed: {exc}") from exc
        while True:
            elapsed = time.monotonic() - started_at
            if args.pause_after is not None and not paused and elapsed >= args.pause_after:
                node.pause()
                paused = True
            if args.resume_after is not None and paused and elapsed >= args.resume_after:
                node.resume()
                paused = False
            if args.return_after is not None and not returned and elapsed >= args.return_after:
                node.request_return()
                returned = True
            node.tick()
            if node.phase in {"armed", "fault"} and (returned or getattr(node, "_return_requested", False)):
                break
            time.sleep(1.0 / rate)
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        if node is not None:
            node.close()
        session.close()


__all__ = ["main"]

if __name__ == "__main__":
    raise SystemExit(main())
