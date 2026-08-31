from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

from ...zenoh_util import open_session, require_single_router
from .node import WujiHandExecutor


def _native_bridge() -> Path | None:
    configured = os.environ.get("TIANJI_WUJI_NATIVE_BRIDGE")
    candidates = [Path(configured)] if configured else []
    root = Path(os.environ.get("TIANJI_TELEOP_BUNDLE_ROOT", Path(__file__).resolve().parents[5]))
    candidates.extend((
        root / "staging/ik/lib/tianji_teleop/wuji_hand2_bridge",
        root / "runtime/tianji_teleop/lib/tianji_teleop/wuji_hand2_bridge.bin",
        root / "build/ik/wuji_hand2_bridge",
    ))
    return next((path for path in candidates if path.is_file() and os.access(path, os.X_OK)), None)


def _positive_finite_rate(value: str) -> float:
    rate = float(value)
    if not math.isfinite(rate) or rate <= 0.0:
        raise argparse.ArgumentTypeError("--rate must be a positive finite float")
    return rate


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="canonical Wuji Hand 2 executor")
    parser.add_argument("--mode", choices=("direct", "retarget"), required=True)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--config", default=None)
    parser.add_argument("--rate", type=_positive_finite_rate, default=100.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    if not args.dry_run:
        native = _native_bridge()
        if native is None:
            print(
                "real Wuji executor requires native wuji_hand2_bridge and a connected SDK device; "
                "refusing Python no-op fallback",
                file=__import__("sys").stderr,
            )
            return 1
        if args.config:
            os.environ["TIANJI_WUJI_CONFIG"] = str(Path(args.config).resolve())
        os.execv(str(native), [str(native), "--mode", args.mode, "--side", args.side])
        raise AssertionError("os.execv returned unexpectedly")
    session = open_session()
    executor = None
    try:
        router = require_single_router(session, os.environ.get("TIANJI_ROUTER_ZID"))
        real_capability = None
        if not args.dry_run:
            from ..marvin.preflight import trusted_real_capability
            real_capability = trusted_real_capability
        executor = WujiHandExecutor(
            config=args.config,
            mode=args.mode,
            side=args.side,
            publisher_instance_id=os.environ.get("TIANJI_COMPONENT_INSTANCE_ID", ""),
            producer_publisher_instance_id=os.environ.get("TIANJI_HAND_PRODUCER_INSTANCE_ID"),
            router_zid=router,
            authorized_producer=os.environ.get("TIANJI_HAND_PRODUCER_ID", ""),
            authorized_publisher_instance_id=os.environ.get(
                "TIANJI_HAND_INPUT_INSTANCE_ID",
                os.environ.get("TIANJI_HAND_PRODUCER_INSTANCE_ID", ""),
            ),
            coordinator_instance_id=os.environ.get("TIANJI_COORDINATOR_INSTANCE_ID"),
            session=session,
            dry_run=args.dry_run,
            real_capability=real_capability,
            run_id=os.environ.get("TIANJI_RUN_ID"),
            safety_supervisor_instance_id=os.environ.get("TIANJI_SAFETY_SUPERVISOR_INSTANCE_ID"),
        )
        executor.run(rate_hz=args.rate)
    except KeyboardInterrupt:
        return 0
    finally:
        if executor is not None:
            executor.close()
        session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
