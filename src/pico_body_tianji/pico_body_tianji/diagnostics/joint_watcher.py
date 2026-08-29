"""Read-only watcher for canonical arm/hand state and final commands."""
from __future__ import annotations

import argparse
import json
import os
import time

from ..protocol import topics
from ..protocol.messages import strict_loads
from ..zenoh_util import open_session, require_single_router


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="watch canonical joint streams")
    parser.add_argument("--duration", type=float, default=0.0)
    args = parser.parse_args(argv)
    session = open_session()
    resources = []
    end = time.monotonic() + args.duration if args.duration else None
    try:
        router = require_single_router(session, os.environ.get("TIANJI_ROUTER_ZID"))
        keys = [topics.ARM_STATE, topics.arm_command("left"), topics.arm_command("right")]
        for side in ("left", "right"):
            keys.extend((topics.hand_state(side), topics.hand_command(side)))
        def on_sample(sample, key):
            try:
                value = strict_loads(bytes(sample.payload))
                if value.get("router_zid") == router:
                    print(json.dumps({"topic": key, "message": value}, separators=(",", ":"), ensure_ascii=False), flush=True)
            except Exception:
                return
        for key in keys:
            resources.append(session.declare_subscriber(key, lambda sample, key=key: on_sample(sample, key)))
        while end is None or time.monotonic() < end:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for resource in resources:
            try: resource.undeclare()
            except Exception: pass
        session.close()
    return 0


__all__ = ["main"]
