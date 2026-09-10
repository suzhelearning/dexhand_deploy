#!/usr/bin/env python3
"""Check the XRoboToolkit Pybind surface without connecting to a device.

The caller may provide ``TIANJI_XR_SDK_PYTHONPATH``.  This process only
imports the binding and inspects callable names; it never calls ``init`` and
therefore does not open PC-Service or claim a live device.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/tianji_teleop"))


def main() -> int:
    from tianji_teleop.hand_tracking.xr_input import (
        XRoboToolkitClient,
        validate_xr_sdk_module,
    )

    try:
        client = XRoboToolkitClient()
        sdk = client._load_sdk()
    except RuntimeError as exc:
        print(f"XR SDK preflight failed: {exc}", file=sys.stderr)
        return 1

    missing = validate_xr_sdk_module(sdk)
    if missing:
        print(
            "XR SDK preflight failed: missing callable symbols: "
            + ", ".join(missing),
            file=sys.stderr,
        )
        return 1
    print(json.dumps({"module": "xrobotoolkit_sdk", "missing": []}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
