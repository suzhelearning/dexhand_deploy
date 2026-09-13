#!/usr/bin/env python3
"""Run a deterministic receive-only XR + Manus target-layer smoke."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/tianji_teleop"))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--arm-input",
        choices=("xr_tracker", "xr_controller"),
        default="xr_controller",
        help="XR pose binding to exercise (default: xr_controller)",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=100,
        help="number of deterministic frames (default: 100)",
    )
    args = parser.parse_args(argv)
    try:
        from tianji_teleop.hand_tracking.xr_manus_smoke import run_offline_smoke

        report = run_offline_smoke(arm_input=args.arm_input, frame_count=args.frames)
    except (RuntimeError, ValueError) as exc:
        print(f"XR + Manus receive-only smoke failed: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(report, allow_nan=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
