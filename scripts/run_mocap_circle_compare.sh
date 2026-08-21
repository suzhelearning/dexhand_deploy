#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# 直接 bash 运行时自动进入 Pixi，保证 zenoh/numpy 可用。
if ! python -c 'import zenoh, numpy' 2>/dev/null; then
  if command -v pixi >/dev/null 2>&1; then
    exec pixi run bash "${BASH_SOURCE[0]}" "$@"
  fi
fi

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
activate_bundle_runtime

exec python -m \
  pico_body_tianji.controller_only.mocap_circle_compare "$@"
