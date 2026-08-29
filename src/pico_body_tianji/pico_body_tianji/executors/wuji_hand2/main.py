from __future__ import annotations

import argparse
import os

from ...zenoh_util import open_session, require_single_router
from .node import WujiHandExecutor


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="canonical Wuji Hand 2 executor")
    parser.add_argument("--mode", choices=("direct", "retarget"), required=True)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--config", default=None)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
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
            router_zid=router,
            authorized_producer=os.environ.get("TIANJI_HAND_PRODUCER_ID", ""),
            authorized_publisher_instance_id=os.environ.get("TIANJI_HAND_PRODUCER_INSTANCE_ID", ""),
            coordinator_instance_id=os.environ.get("TIANJI_COORDINATOR_INSTANCE_ID"),
            session=session,
            dry_run=args.dry_run,
            real_capability=real_capability,
            run_id=os.environ.get("TIANJI_RUN_ID"),
            safety_supervisor_instance_id=os.environ.get("TIANJI_SAFETY_SUPERVISOR_INSTANCE_ID"),
        )
        executor.run()
    except KeyboardInterrupt:
        return 0
    finally:
        if executor is not None:
            executor.close()
        session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
