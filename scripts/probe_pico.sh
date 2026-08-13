#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

acquire_teleop_guard pico-probe
install_teleop_cleanup_traps
activate_bundle_runtime
assert_no_conflicting_teleop_nodes

printf '%s\n' \
  '只读检测 PICO 输入；不会启动 IK、仿真或 Marvin 真机。' \
  '检测期间请勿同时运行 sim、real 或其他 XRoboToolkit SDK 客户端。'

python -m pico_body_tianji.pico_link_probe "$@"
