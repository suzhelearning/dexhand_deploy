#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

activate_bundle_runtime
exec python - <<'PY'
import json
import threading

import zenoh


def main() -> None:
    session = zenoh.open(zenoh.Config())
    try:
        def on_model_joint_states(sample) -> None:
            payload = bytes(sample.payload)
            if not payload:
                return
            message = json.loads(payload.decode("utf-8"))
            for name, position in zip(
                message.get("name", []), message.get("position", [])
            ):
                print(f"{name}: {position}", flush=True)

        session.declare_subscriber(
            "pico_body_sim/model_joint_states", on_model_joint_states
        )
        print("等待 pico_body_sim/model_joint_states（Ctrl+C 退出）", flush=True)
        threading.Event().wait()
    except KeyboardInterrupt:
        pass
    finally:
        session.close()


if __name__ == "__main__":
    main()
PY
