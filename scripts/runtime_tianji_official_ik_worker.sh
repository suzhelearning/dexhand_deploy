#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/../../../.." && pwd)"
ABI_LIBRARY_ROOT="${BUNDLE_ROOT}/runtime/abi/lib"

exec "${ABI_LIBRARY_ROOT}/ld-linux-x86-64.so.2" \
  --library-path "${ABI_LIBRARY_ROOT}" \
  "${SCRIPT_DIR}/tianji_official_ik_worker.bin" "$@"
