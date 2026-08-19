#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

acquire_teleop_guard test
preview_log=""
cleanup_preview() {
  local cleanup_failed=0
  trap - EXIT INT TERM
  teleop_cleanup_and_release || cleanup_failed=1
  if [[ -n "${preview_log}" ]]; then
    rm -f -- "${preview_log}"
  fi
  return "${cleanup_failed}"
}
stop_on_signal() {
  cleanup_preview
  exit 130
}
trap cleanup_preview EXIT
trap stop_on_signal INT TERM

"${SCRIPT_DIR}/doctor.sh"
activate_bundle_runtime
assert_no_conflicting_teleop_nodes

python -m unittest \
  tests.test_pico_link_probe \
  tests.test_controller_only_mapper \
  tests.test_controller_only_trace \
  tests.test_controller_only_host_readiness \
  tests.test_controller_only_real_profile \
  tests.test_mocap_h5

IK_NODE="${PROJECT_PREFIX}/lib/pico_body_tianji/tianji_kinematic_sim"
IK_NODE_BIN="${IK_NODE}.bin"
if [[ ! -x "${IK_NODE}" ]]; then
  printf '错误：可配置 IK 节点未生成：%s\n' "${IK_NODE}" >&2
  exit 1
fi
CONTROLLER_ONLY_CONFIG="${PROJECT_PREFIX}/share/pico_body_tianji/config/mode/controller_only/controller_only_ik.yaml"
HANGING_WORKER="${BUNDLE_ROOT}/tests/fake_hanging_official_ik_worker.sh"
worker_timeout_log="$(mktemp)"
set +e
TIANJI_OFFICIAL_IK_WORKER="${HANGING_WORKER}" \
TIANJI_OFFICIAL_IK_LIBRARY="${BUNDLE_ROOT}/runtime/tianji_official/kinematicsSDK/libKine.so" \
TIANJI_OFFICIAL_IK_CONFIG="${BUNDLE_ROOT}/runtime/tianji_official/CommonConfig/ccs_m6_40.MvKDCfg" \
  timeout 2 "${IK_NODE_BIN}" --ros-args \
  --params-file "${CONTROLLER_ONLY_CONFIG}" \
  -p ik_backend:=tianji_official \
  >"${worker_timeout_log}" 2>&1
worker_timeout_exit=$?
set -e
if [[ "${worker_timeout_exit}" -eq 124 ]] ||
   ! grep -Fq '官方 IK worker 恢复失败' "${worker_timeout_log}"
then
  cat "${worker_timeout_log}" >&2
  rm -f -- "${worker_timeout_log}"
  printf '%s\n' '错误：官方 IK worker deadline/重启保护未按时生效。' >&2
  exit 1
fi
rm -f -- "${worker_timeout_log}"

PREVIEW_CONFIG="${PROJECT_PREFIX}/share/pico_body_tianji/config/mode/full_body/preview.yaml"
IK_BACKEND="$(
  awk '$1 == "ik_backend:" {print $2; exit}' "${PREVIEW_CONFIG}"
)"
case "${IK_BACKEND}" in
  pinocchio_cpp|pinocchio_qp|tianji_official) ;;
  *)
    printf '错误：%s 中的 ik_backend=%s 无效\n' \
      "${PREVIEW_CONFIG}" "${IK_BACKEND:-<missing>}" >&2
    exit 2
    ;;
esac
python - <<'PY'
from ament_index_python.packages import get_package_share_directory

share = get_package_share_directory("pico_body_tianji")
assert "/runtime/pico_body_tianji/share/pico_body_tianji" in share
print("Ament 安装索引检查通过：", share)
PY

setsid timeout 3 "${IK_NODE}" --ros-args \
  --params-file "${PREVIEW_CONFIG}" &
ik_probe_pid=$!
register_teleop_process_group \
  "${ik_probe_pid}" portable-test-ik-probe 5
set +e
wait "${ik_probe_pid}"
ik_exit=$?
set -e
if [[ "${ik_exit}" -ne 124 && "${ik_exit}" -ne 0 ]]; then
  printf '错误：可配置 IK 纯运动学节点启动失败：%s\n' \
    "${ik_exit}" >&2
  exit "${ik_exit}"
fi

preview_log="$(mktemp)"
preview_pid=""

setsid python "${ROS_ROOT}/bin/ros2" launch \
  pico_body_tianji preview.launch.py with_rviz:=false \
  >"${preview_log}" 2>&1 &
preview_pid=$!
register_teleop_process_group "${preview_pid}" portable-test-preview 5
sleep 3

if ! kill -0 "${preview_pid}" 2>/dev/null; then
  cat "${preview_log}" >&2
  printf '%s\n' \
    '错误：RViz/MuJoCo 共用仿真话题链路提前退出。' >&2
  exit 1
fi
if grep -Eq \
  'process has died|Failed to load entry point|Traceback \(most recent call last\)' \
  "${preview_log}"
then
  cat "${preview_log}" >&2
  printf '%s\n' \
    '错误：RViz/MuJoCo 共用仿真话题链路中的子进程异常退出。' >&2
  exit 1
fi

node_list="$(
  timeout 4 python "${ROS_ROOT}/bin/ros2" node list --no-daemon
)"
for required_node in \
  /pico_body_sim/marvin_robot_state_publisher \
  /tianji_kinematic_sim \
  /pico_controller_input
do
  if ! grep -Fxq "${required_node}" <<<"${node_list}"; then
    cat "${preview_log}" >&2
    printf '错误：纯仿真节点未启动：%s\n' "${required_node}" >&2
    exit 1
  fi
done

cleanup_preview
trap - EXIT INT TERM

printf '%s\n' \
  '便携包 ROS + 可配置 IK + 可视化话题链路验证通过；未连接实体机械臂。'
