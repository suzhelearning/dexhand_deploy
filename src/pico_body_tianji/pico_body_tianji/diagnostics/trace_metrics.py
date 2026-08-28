"""只读遥操作 trace metrics 诊断。

该模块不声明任何 target/command/state publisher；它只观察协议消息并把统计
写到本地 JSON，避免诊断进程成为第二 authority。
"""
from __future__ import annotations

import json
import time
from collections import Counter
from pathlib import Path
from typing import Any, Mapping

from ..protocol.messages import SessionState, strict_loads


class TraceMetrics:
    """按 topic 汇总接收率、状态和错误的本地观察器。"""

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
            timestamp = value.get("timestamp_ns")
            if timestamp is not None:
                self.last_timestamp_ns[topic] = int(timestamp)
            if topic.endswith("session/state"):
                state = SessionState.from_dict(value)
                self.state_counts[state.state] += 1
        except Exception as exc:
            self.errors.append(f"{topic}: {exc}")

    def snapshot(self) -> dict[str, Any]:
        result = {
            "started_ns": self.started_ns,
            "counts": dict(self.counts),
            "state_counts": dict(self.state_counts),
            "last_timestamp_ns": dict(self.last_timestamp_ns),
            "errors": list(self.errors),
        }
        if self.output is not None:
            self.output.parent.mkdir(parents=True, exist_ok=True)
            self.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        return result


def main(argv: list[str] | None = None) -> int:
    del argv
    TraceMetrics().snapshot()
    return 0


__all__ = ["TraceMetrics", "main"]
