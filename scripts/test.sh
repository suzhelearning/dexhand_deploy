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
  tests.test_controller_only_host_readiness

IK_NODE="${PROJECT_PREFIX}/lib/pico_body_tianji/tianji_kinematic_sim"
if [[ ! -x "${IK_NODE}" ]]; then
  printf '错误：Pinocchio IK 节点未生成：%s\n' "${IK_NODE}" >&2
  exit 1
fi
python - <<'PY'
from ament_index_python.packages import get_package_share_directory

share = get_package_share_directory("pico_body_tianji")
assert "/runtime/pico_body_tianji/share/pico_body_tianji" in share
print("Ament 安装索引检查通过：", share)
PY

setsid timeout 3 "${IK_NODE}" --ros-args \
  --params-file \
  "${PROJECT_PREFIX}/share/pico_body_tianji/config/preview.yaml" &
ik_probe_pid=$!
register_teleop_process_group \
  "${ik_probe_pid}" portable-test-ik-probe 5
set +e
wait "${ik_probe_pid}"
ik_exit=$?
set -e
if [[ "${ik_exit}" -ne 124 && "${ik_exit}" -ne 0 ]]; then
  printf '错误：Pinocchio 纯运动学节点启动失败：%s\n' \
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
  '便携包 ROS + Pinocchio + 可视化话题链路验证通过；未连接实体机械臂。'
