#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/../../../.." && pwd)"
ABI_LIBRARY_ROOT="${BUNDLE_ROOT}/runtime/abi/lib"
PIN_LIBRARY_ROOT="${BUNDLE_ROOT}/runtime/pin/lib"
ZENOH_LIBRARY_ROOT="${BUNDLE_ROOT}/vendor/zenoh/lib"
IK_BINARY="${SCRIPT_DIR}/arm_ik_producer.bin"

export TIANJI_OFFICIAL_IK_WORKER="${SCRIPT_DIR}/tianji_official_ik_worker"
export TIANJI_OFFICIAL_IK_LIBRARY="${BUNDLE_ROOT}/runtime/tianji_official/kinematicsSDK/libKine.so"
export TIANJI_OFFICIAL_IK_CONFIG="${BUNDLE_ROOT}/runtime/tianji_official/CommonConfig/ccs_m6_40.MvKDCfg"
exec "${ABI_LIBRARY_ROOT}/ld-linux-x86-64.so.2" \
  --library-path \
  "${PIN_LIBRARY_ROOT}:${ZENOH_LIBRARY_ROOT}:${SCRIPT_DIR}:${ABI_LIBRARY_ROOT}" \
  "${IK_BINARY}" "$@"
