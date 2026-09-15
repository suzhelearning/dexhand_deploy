#!/usr/bin/env python3
"""Cold-path launcher for the native fixed-rate Hand2 scheduler.

The pinned official bridge is used only to resolve the existing Hand2 YAML
configuration and model/joint order into the binary manifest. The child is
then replaced by C++; no Python retarget callback remains after exec.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import tempfile

ROOT = Path(__file__).resolve().parents[1]
OFFICIAL = ROOT / "third_party/wuji_hand_retargeting"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native-scheduler", type=Path, required=True)
    parser.add_argument("--period-ns", type=int, default=5_000_000)
    parser.add_argument("--freshness-ns", type=int, default=200_000_000)
    parser.add_argument("--filter-continuity-ns", type=int, default=0)
    parser.add_argument("--output-capacity", type=int, default=256)
    parser.add_argument("--startup-handshake", action="store_true")
    parser.add_argument("--left-config", type=Path,
                        default=OFFICIAL / "example/config/adaptive_analytical_manus_wuji_hand_2_left.yaml")
    parser.add_argument("--right-config", type=Path,
                        default=OFFICIAL / "example/config/adaptive_analytical_manus_wuji_hand_2_right.yaml")
    args = parser.parse_args()
    if args.filter_continuity_ns < 0 or args.filter_continuity_ns >= 2**63:
        parser.error('--filter-continuity-ns must be zero or positive int64')
    scheduler = args.native_scheduler.resolve(strict=True)
    if not os.access(scheduler, os.X_OK):
        raise RuntimeError("native Hand2 scheduler is not executable")
    left_config = args.left_config.resolve(strict=True)
    right_config = args.right_config.resolve(strict=True)
    if left_config == right_config:
        raise ValueError("left and right Hand2 configurations must be distinct")

    # Import the existing cold-path manifest builder. It is deliberately not
    # moved into the C++ hot path, and it does not change the Python reference.
    import sys
    sys.path.insert(0, str(ROOT / "scripts"))
    from wuji_hand_native_launcher import _manifest

    runtime_dir = Path(os.environ.get("XDG_RUNTIME_DIR", tempfile.gettempdir())) / "tianji-teleop-hand"
    runtime_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd, manifest_name = tempfile.mkstemp(prefix="wuji-hand-scheduler-", suffix=".manifest", dir=runtime_dir)
    os.close(fd)
    manifest = Path(manifest_name)
    try:
        # selected_side is retained in the manifest for compatibility; the
        # scheduler intentionally owns both independent side pipelines.
        manifest.write_bytes(_manifest(left_config, right_config, "right"))
        os.chmod(manifest, 0o600)
        command = [str(scheduler), "--manifest", str(manifest),
                   "--period-ns", str(args.period_ns),
                   "--freshness-ns", str(args.freshness_ns),
                   "--output-capacity", str(args.output_capacity)]
        if args.filter_continuity_ns:
            command.extend(["--filter-continuity-ns", str(args.filter_continuity_ns)])
        if args.startup_handshake:
            command.append("--startup-handshake")
        os.execv(str(scheduler), command)
    except BaseException:
        manifest.unlink(missing_ok=True)
        raise


if __name__ == "__main__":
    main()
