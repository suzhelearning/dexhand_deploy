#!/usr/bin/env bash
# 快捷入口: bash adb.sh [up|list|down] —— 等价于 scripts/adb_reverse.sh
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${SCRIPT_DIR}/scripts/adb_reverse.sh" "$@"
