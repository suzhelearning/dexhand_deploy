#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/../../../.." && pwd)"
ABI_LIBRARY_ROOT="${BUNDLE_ROOT}/runtime/abi/lib"
ZENOH_LIBRARY_ROOT="${BUNDLE_ROOT}/vendor/zenoh/lib"
WUJI_LIBRARY_ROOT="${BUNDLE_ROOT}/vendor/wuji-sdk/lib"
BRIDGE_BINARY="${SCRIPT_DIR}/wuji_hand2_bridge.bin"

# Wuji SDK 依赖系统的 libudev.so.1 / libcap.so.2（Ubuntu 22.04 默认安装），
# 经便携 glibc 2.35 加载器的默认搜索路径按需解析。
exec "${ABI_LIBRARY_ROOT}/ld-linux-x86-64.so.2" \
  --library-path \
  "${ABI_LIBRARY_ROOT}:${WUJI_LIBRARY_ROOT}:${ZENOH_LIBRARY_ROOT}:${SCRIPT_DIR}" \
  "${BRIDGE_BINARY}" "$@"
