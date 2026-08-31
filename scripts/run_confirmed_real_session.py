#!/usr/bin/env python3
"""Issue a sealed, run-bound capability for an explicitly confirmed real session."""
from __future__ import annotations

import argparse
import ctypes
import fcntl
import json
import math
import os
import uuid


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", required=True)
    parser.add_argument("--speed", required=True, type=float)
    parser.add_argument("--yaw-deg", required=True, type=float)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("missing session command")
    if not math.isfinite(args.speed) or not 0.0 < args.speed <= 1.0:
        parser.error("real session speed must be in (0, 1]")
    if not math.isfinite(args.yaw_deg) or args.yaw_deg != 0.0:
        parser.error("real session yaw must be 0")

    environment = os.environ.copy()
    run_id = environment.get("TIANJI_RUN_ID") or str(uuid.uuid4())
    nonce = uuid.uuid4().hex
    payload = json.dumps(
        {
            "schema_version": 1,
            "issuer": "confirmed_real_session",
            "run_id": run_id,
            "profile": args.profile,
            "nonce": nonce,
            "capability": {
                "speed": args.speed,
                "yaw_deg": args.yaw_deg,
                "deadman_available": True,
                "preflight_passed": True,
            },
        },
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    libc = ctypes.CDLL(None, use_errno=True)
    libc.memfd_create.argtypes = [ctypes.c_char_p, ctypes.c_uint]
    libc.memfd_create.restype = ctypes.c_int
    fd = libc.memfd_create(b"tianji-confirmed-real", 0x0002)
    if fd < 0:
        raise OSError(ctypes.get_errno(), "memfd_create failed")
    os.write(fd, payload)
    fcntl.fcntl(
        fd,
        getattr(fcntl, "F_ADD_SEALS", 1033),
        getattr(fcntl, "F_SEAL_SEAL", 1)
        | getattr(fcntl, "F_SEAL_SHRINK", 2)
        | getattr(fcntl, "F_SEAL_GROW", 4)
        | getattr(fcntl, "F_SEAL_WRITE", 8),
    )
    os.set_inheritable(fd, True)
    environment.update(
        {
            "TIANJI_RUN_ID": run_id,
            "TIANJI_REAL_PROFILE": args.profile,
            "TIANJI_CONFIRMED_REAL_PREFLIGHT_FD": str(fd),
            "TIANJI_CONFIRMED_REAL_PREFLIGHT_NONCE": nonce,
        }
    )
    os.execvpe(command[0], command, environment)
    raise AssertionError("exec returned")


if __name__ == "__main__":
    raise SystemExit(main())
