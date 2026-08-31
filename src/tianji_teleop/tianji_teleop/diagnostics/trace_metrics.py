"""只读 trace metrics 诊断，不声明任何 target/command/state publisher。"""
from __future__ import annotations

import argparse
import json
import time
from collections import Counter
from pathlib import Path
from typing import Any, Mapping

from ..protocol import topics
from ..protocol.messages import SessionState, strict_loads
from ..zenoh_util import open_session, require_single_router


class TraceMetrics:
    def __init__(self, output: str | Path | None = None) -> None:
        self.output = Path(output) if output is not None else None
        self.started_ns = time.monotonic_ns()
        self.counts: Counter[str] = Counter()
        self.state_counts: Counter[str] = Counter()
        self.errors: list[str] = []
        self.last_timestamp_ns: dict[str, int] = {}

    def receive(self, topic: str, payload: Mapping[str, Any] | bytes | bytearray) -> None:
        try:
            value = strict_loads(bytes(payload)) if isinstance(payload, (bytes, bytearray)) else dict(payload)
            self.counts[topic] += 1
            if value.get("timestamp_ns") is not None:
                self.last_timestamp_ns[topic] = int(value["timestamp_ns"])
            if topic == topics.SESSION_STATE:
                self.state_counts[SessionState.from_dict(value).state] += 1
        except Exception as exc:
            self.errors.append(f"{topic}: {exc}")

    def snapshot(self) -> dict[str, Any]:
        result = {"started_ns": self.started_ns, "counts": dict(self.counts), "state_counts": dict(self.state_counts), "last_timestamp_ns": dict(self.last_timestamp_ns), "errors": list(self.errors)}
        if self.output is not None:
            self.output.parent.mkdir(parents=True, exist_ok=True)
            self.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="read-only canonical trace metrics")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; zero runs until Ctrl+C")
    args = parser.parse_args(argv)
    if args.duration < 0:
        raise SystemExit("duration must be non-negative")
    session = open_session()
    metrics = TraceMetrics(args.output)
    resources = []
    try:
        require_single_router(session, __import__("os").environ.get("TIANJI_ROUTER_ZID"))
        for topic in (topics.SESSION_STATE, topics.SOURCE_STATUS, topics.PRODUCER_STATUS, topics.COORDINATOR_STATUS, topics.EXECUTOR_STATUS):
            resources.append(session.declare_subscriber(topic, lambda sample, topic=topic: metrics.receive(topic, bytes(sample.payload))))
        deadline = time.monotonic() + args.duration if args.duration else None
        while deadline is None or time.monotonic() < deadline:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for resource in resources:
            try: resource.undeclare()
            except Exception: pass
        session.close()
        metrics.snapshot()
    return 0


__all__ = ["TraceMetrics", "main"]
