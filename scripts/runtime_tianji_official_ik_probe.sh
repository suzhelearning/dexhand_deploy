#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/../../../.." && pwd)"
ABI_LIBRARY_ROOT="${BUNDLE_ROOT}/runtime/abi/lib"
PIN_LIBRARY_ROOT="${BUNDLE_ROOT}/runtime/pin/lib"
PROBE_BINARY="${SCRIPT_DIR}/tianji_official_ik_probe.bin"

if [[ "$#" -eq 0 ]]; then
  set -- \
    "${BUNDLE_ROOT}/runtime/tianji_official/kinematicsSDK/libKine.so" \
    "${BUNDLE_ROOT}/runtime/tianji_official/CommonConfig/ccs_m6_40.MvKDCfg"
fi

exec "${ABI_LIBRARY_ROOT}/ld-linux-x86-64.so.2" \
  --library-path \
  "${PIN_LIBRARY_ROOT}:${SCRIPT_DIR}:${ABI_LIBRARY_ROOT}" \
  "${PROBE_BINARY}" \
  "$@"
