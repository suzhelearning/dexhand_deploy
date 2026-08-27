#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

# 直接 bash 运行时自动进入 pixi default 环境（zenoh）。
if ! python -c 'import zenoh' 2>/dev/null; then
  if command -v pixi >/dev/null 2>&1; then
    exec pixi run bash "${BASH_SOURCE[0]}" "$@"
  fi
fi

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
activate_bundle_runtime

# 优先 staging 调试版；未构建时退回 runtime 部署的 .bin。
BRIDGE="${BUNDLE_ROOT}/staging/ik/lib/pico_body_tianji/wuji_hand2_bridge"
if [[ ! -x "${BRIDGE}" ]]; then
  BRIDGE="${PROJECT_PREFIX}/lib/pico_body_tianji/wuji_hand2_bridge.bin"
fi
if [[ ! -x "${BRIDGE}" ]]; then
  printf '错误：wuji_hand2_bridge 二进制未生成（先 pixi run -e ik-build build-ik 并 deploy-ik）。\n' >&2
  exit 1
fi
if [[ ! -r "${BUNDLE_ROOT}/vendor/wuji-sdk/lib/libwuji_sdk_c.so" ]]; then
  printf '错误：缺少 vendor/wuji-sdk/lib/libwuji_sdk_c.so。\n' >&2
  exit 1
fi

export LD_LIBRARY_PATH="${BUNDLE_ROOT}/vendor/wuji-sdk/lib:${BUNDLE_ROOT}/vendor/zenoh/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

# dry-run 模式：不连接手部硬件，只做 “键点 → retarget → 发布命令”，
# 配合 sim_mocap_h5（H5 键点）与 wrist 回放 --hand-commands 做仿真验收。
exec stdbuf -oL -eL "${BRIDGE}" --dry-run "$@"
