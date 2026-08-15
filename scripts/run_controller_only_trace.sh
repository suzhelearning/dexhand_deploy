#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
activate_bundle_runtime
exec "${PROJECT_PREFIX}/lib/pico_body_tianji/controller_only_trace" "$@"
