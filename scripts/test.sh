#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

router_pid=""
router_endpoint="${TIANJI_TEST_ROUTER_ENDPOINT:-tcp/127.0.0.1:7448}"
cleanup() {
  if [[ -n "${router_pid}" ]] && kill -0 "${router_pid}" 2>/dev/null; then
    kill "${router_pid}" 2>/dev/null || true
    wait "${router_pid}" 2>/dev/null || true
  fi
  release_teleop_guard || true
}
trap cleanup EXIT INT TERM

if ! command -v zenohd >/dev/null 2>&1; then
  printf '%s\n' '错误：test 需要可执行 zenohd 以启动受管临时 router。' >&2
  exit 1
fi
acl_config="${TIANJI_ACL_CONFIG:-/home/current/syz/mocap/acquisition/config/zenohd_acl.yaml}"
[[ -f "${acl_config}" ]] || { printf '错误：ACL 配置不存在：%s\n' "${acl_config}" >&2; exit 1; }
export TIANJI_ROUTER_ENDPOINT="${router_endpoint}"
mkdir -p -- "${TELEOP_RUNTIME_DIR}"
chmod 700 -- "${TELEOP_RUNTIME_DIR}"
setsid zenohd -e "${router_endpoint}" -c "${acl_config}" >"${TELEOP_RUNTIME_DIR}/test-router.log" 2>&1 &
router_pid=$!
for _ in {1..50}; do
  if TIANJI_ROUTER_ENDPOINT="${router_endpoint}" read_router_zid >/dev/null 2>&1; then break; fi
  sleep 0.1
done
if ! router_zid="$(read_router_zid 2>/dev/null)"; then
  router_unavailable_message
  exit 1
fi
export TIANJI_ROUTER_ZID="${router_zid}"
printf '%s\n' "managed test router: endpoint=${router_endpoint} zid=${router_zid}"

# doctor consumes this endpoint; it must not start a second router.
"${SCRIPT_DIR}/doctor.sh"
PYTHONPATH="${BUNDLE_ROOT}/src/pico_body_tianji:${PYTHONPATH:-}" python -m unittest \
  tests.test_task8_config_launcher \
  tests.test_protocol \
  tests.test_session_h5 \
  tests.test_session_recorder \
  tests.test_session_replay \
  tests.test_policy_producer
