"""H5 v4 wrist diagnostic overlay preparation。

该工具只读外部 acquisition v4 文件并输出 frame/hand skeleton 摘要；真实的
MuJoCo 执行状态仍由唯一 ``MujocoExecutor`` 提供，因此本进程不会发布
SessionState、JointState 或 final command。
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from ..sources.mocap.h5 import load_mocap_h5


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="read-only H5 wrist diagnostic")
    parser.add_argument("h5", type=Path)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(argv)
    recording = load_mocap_h5(args.h5)
    summary = recording.summary()
    summary.update({"overlay": "frame0_hand_skeleton", "executor_authority": "MujocoExecutor"})
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


__all__ = ["main"]

if __name__ == "__main__":
    raise SystemExit(main())
