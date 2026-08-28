#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
STAGING_BIN="${BUNDLE_ROOT}/staging/ik/lib/pico_body_tianji"
RUNTIME_BIN="${BUNDLE_ROOT}/runtime/pico_body_tianji/lib/pico_body_tianji"
NEW_IK="${STAGING_BIN}/arm_ik_producer"
NEW_PROBE="${STAGING_BIN}/tianji_official_ik_probe"
NEW_WORKER="${STAGING_BIN}/tianji_official_ik_worker"
NEW_BRIDGE="${STAGING_BIN}/wuji_hand2_bridge"
RUNTIME_PROGRAMS=(
  pico_controller_source
  mocap_live
  mocap_h5_replay
  mocap_calibration
  target_replay
  joint_replay
  session_recorder
  arm_command_coordinator
  policy_hold_producer
  mujoco_executor
  marvin_executor
  wuji_hand2_executor
  trace_metrics
  real_diagnostic
  h5_wrist_diagnostic
)
BACKUP_DIR="${BUNDLE_ROOT}/staging/runtime-backup"
SDK_SOURCE_ROOT="${TIANJI_OFFICIAL_SDK_ROOT:-/home/ice/TJ_FX_ROBOT_CONTRL_SDK}"
SDK_RUNTIME_ROOT="${BUNDLE_ROOT}/runtime/tianji_official"
SDK_LIBRARY="${SDK_SOURCE_ROOT}/kinematicsSDK/libKine.so"
SDK_CONFIG="${SDK_SOURCE_ROOT}/CommonConfig/ccs_m6_40.MvKDCfg"
RUNTIME_SHARE="${BUNDLE_ROOT}/runtime/pico_body_tianji/share/pico_body_tianji"
SOURCE_CONFIG="${BUNDLE_ROOT}/src/pico_body_tianji/config"
RUNTIME_CONFIG="${RUNTIME_SHARE}/config"
SOURCE_ASSETS="${BUNDLE_ROOT}/src/pico_body_tianji/assets"
RUNTIME_ASSETS="${RUNTIME_SHARE}/assets"
STAGING_PYTHON="${BUNDLE_ROOT}/staging/ik/lib/python3.10/site-packages/pico_body_tianji"
STRIP_TOOL="${IK_STRIP_TOOL:-/usr/bin/strip}"

for binary in "${NEW_IK}" "${NEW_PROBE}" "${NEW_WORKER}" "${NEW_BRIDGE}"; do
  if [[ ! -x "${binary}" ]]; then
    printf '错误：请先执行 pixi run -e ik-build build-ik；缺少 %s\n' \
      "${binary}" >&2
    exit 1
  fi
done
for program in "${RUNTIME_PROGRAMS[@]}"; do
  if [[ ! -x "${STAGING_BIN}/${program}" ]]; then
    printf '错误：staging 缺少 Python 入口：%s\n' "${program}" >&2
    exit 1
  fi
done
for sdk_file in "${SDK_LIBRARY}" "${SDK_CONFIG}"; do
  if [[ ! -f "${sdk_file}" ]]; then
    printf '错误：天机官方 SDK 文件不存在：%s\n' "${sdk_file}" >&2
    exit 1
  fi
done
if [[ ! -x "${STRIP_TOOL}" ]]; then
  printf '错误：找不到可执行的 strip 工具：%s\n' "${STRIP_TOOL}" >&2
  exit 1
fi
if [[ ! -x "${RUNTIME_BIN}/arm_ik_producer" ]]; then
  printf '%s\n' '错误：runtime arm_ik_producer Bash 包装器不存在，拒绝部署。' >&2
  exit 1
fi
if ! head -n 1 "${RUNTIME_BIN}/arm_ik_producer" | grep -Fxq '#!/usr/bin/env bash'; then
  printf '%s\n' '错误：runtime arm_ik_producer 入口不是预期的 Bash 包装器，拒绝部署。' >&2
  exit 1
fi
shopt -s nullglob
# Remove obsolete runtime entries explicitly, without keeping a compatibility
# alias in the shipped tree.
for stale in \
  "${RUNTIME_BIN}"/pico_controller_* \
  "${RUNTIME_BIN}"/marvin_hardware_* \
  "${RUNTIME_BIN}"/mocap_keyboard_* \
  "${RUNTIME_BIN}"/mujoco_joint_viewer* \
  "${RUNTIME_BIN}"/controller_* \
  "${RUNTIME_BIN}"/tianji_kinematic_*; do
  rm -f -- "${stale}"
done
shopt -u nullglob
mkdir -p "${BACKUP_DIR}"
for path in \
  "${RUNTIME_BIN}/arm_ik_producer" \
  "${RUNTIME_BIN}/tianji_official_ik_probe" \
  "${RUNTIME_BIN}/tianji_official_ik_probe.bin" \
  "${RUNTIME_BIN}/tianji_official_ik_worker" \
  "${RUNTIME_BIN}/tianji_official_ik_worker.bin" \
  "${RUNTIME_BIN}/wuji_hand2_bridge" \
  "${RUNTIME_BIN}/wuji_hand2_bridge.bin" \
  "${SDK_RUNTIME_ROOT}/kinematicsSDK/libKine.so" \
  "${SDK_RUNTIME_ROOT}/CommonConfig/ccs_m6_40.MvKDCfg"
do
  if [[ -f "${path}" ]]; then
    cp -a -- "${path}" "${BACKUP_DIR}/$(basename "${path}")"
  fi
done
if [[ -d "${RUNTIME_CONFIG}" ]]; then
  mkdir -p "${BACKUP_DIR}/config"
  rsync -a --delete \
    "${RUNTIME_CONFIG}/" \
    "${BACKUP_DIR}/config/"
fi
if [[ -d "${RUNTIME_ASSETS}" ]]; then
  mkdir -p "${BACKUP_DIR}/assets"
  rsync -a --delete \
    "${RUNTIME_ASSETS}/" \
    "${BACKUP_DIR}/assets/"
fi

mkdir -p \
  "${SDK_RUNTIME_ROOT}/kinematicsSDK" \
  "${SDK_RUNTIME_ROOT}/CommonConfig" \
  "${RUNTIME_CONFIG}" \
  "${RUNTIME_ASSETS}"
mkdir -p "${RUNTIME_PYTHON}" "${STAGING_PYTHON}"
for python_root in "${RUNTIME_PYTHON}" "${STAGING_PYTHON}"; do
  rsync -a --delete \
    "${BUNDLE_ROOT}/src/pico_body_tianji/pico_body_tianji/" \
    "${python_root}/"
done
# 允许 TIANJI_OFFICIAL_SDK_ROOT 指向 runtime 自身（自拷贝场景，
# 例如 SDK 源机器不可用时）；源与目标相同则跳过。
if [[ "$(realpath "${SDK_LIBRARY}")" != "$(realpath "${SDK_RUNTIME_ROOT}/kinematicsSDK/libKine.so")" ]]; then
  install -m 0755 \
    "${SDK_LIBRARY}" \
    "${SDK_RUNTIME_ROOT}/kinematicsSDK/libKine.so"
fi
if [[ "$(realpath "${SDK_CONFIG}")" != "$(realpath "${SDK_RUNTIME_ROOT}/CommonConfig/ccs_m6_40.MvKDCfg")" ]]; then
  install -m 0644 \
    "${SDK_CONFIG}" \
    "${SDK_RUNTIME_ROOT}/CommonConfig/ccs_m6_40.MvKDCfg"
fi

install -m 0755 "${NEW_IK}" "${RUNTIME_BIN}/arm_ik_producer.bin.new"
install -m 0755 "${NEW_PROBE}" "${RUNTIME_BIN}/tianji_official_ik_probe.bin.new"
install -m 0755 "${NEW_WORKER}" "${RUNTIME_BIN}/tianji_official_ik_worker.bin.new"
install -m 0755 "${NEW_BRIDGE}" "${RUNTIME_BIN}/wuji_hand2_bridge.bin.new"
# staging 保留 RelWithDebInfo 完整调试符号；runtime 只部署去除
# DWARF 调试段的运行版，避免将数十 MB 的调试信息提交到 Git。
"${STRIP_TOOL}" --strip-debug \
  "${RUNTIME_BIN}/arm_ik_producer.bin.new"
"${STRIP_TOOL}" --strip-debug \
  "${RUNTIME_BIN}/tianji_official_ik_probe.bin.new"
"${STRIP_TOOL}" --strip-debug \
  "${RUNTIME_BIN}/tianji_official_ik_worker.bin.new"
"${STRIP_TOOL}" --strip-debug \
  "${RUNTIME_BIN}/wuji_hand2_bridge.bin.new"
mv -f -- \
  "${RUNTIME_BIN}/arm_ik_producer.bin.new" \
  "${RUNTIME_BIN}/arm_ik_producer.bin"
mv -f -- \
  "${RUNTIME_BIN}/tianji_official_ik_probe.bin.new" \
  "${RUNTIME_BIN}/tianji_official_ik_probe.bin"
mv -f -- \
  "${RUNTIME_BIN}/tianji_official_ik_worker.bin.new" \
  "${RUNTIME_BIN}/tianji_official_ik_worker.bin"
mv -f -- \
  "${RUNTIME_BIN}/wuji_hand2_bridge.bin.new" \
  "${RUNTIME_BIN}/wuji_hand2_bridge.bin"
install -m 0755 \
  "${BUNDLE_ROOT}/scripts/runtime_arm_ik_producer.sh" \
  "${RUNTIME_BIN}/arm_ik_producer"
install -m 0755 \
  "${BUNDLE_ROOT}/scripts/runtime_tianji_official_ik_probe.sh" \
  "${RUNTIME_BIN}/tianji_official_ik_probe"
install -m 0755 \
  "${BUNDLE_ROOT}/scripts/runtime_tianji_official_ik_worker.sh" \
  "${RUNTIME_BIN}/tianji_official_ik_worker"
install -m 0755 \
  "${BUNDLE_ROOT}/scripts/runtime_wuji_hand2_bridge.sh" \
  "${RUNTIME_BIN}/wuji_hand2_bridge"
for program in "${RUNTIME_PROGRAMS[@]}"; do
  install -m 0755 "${STAGING_BIN}/${program}" "${RUNTIME_BIN}/${program}"
done
# config 是一个整体：每次部署都让 runtime 与 src 递归一致。
# --delete 会清理已从源码删除的旧 profile，避免便携包留下过期配置。
rsync -a --delete \
  "${SOURCE_CONFIG}/" \
  "${RUNTIME_CONFIG}/"
if ! diff -qr -- "${SOURCE_CONFIG}" "${RUNTIME_CONFIG}" >/dev/null; then
  printf '%s\n' '错误：src 与 runtime 的 config 目录部署后仍不一致。' >&2
  exit 1
fi
# assets 与 config 同样是运行时契约；组合 URDF/mesh 必须随部署同步，
# 否则 --wuji2 会静默加载旧模型。
rsync -a --delete \
  "${SOURCE_ASSETS}/" \
  "${RUNTIME_ASSETS}/"
if ! diff -qr -- "${SOURCE_ASSETS}" "${RUNTIME_ASSETS}" >/dev/null; then
  printf '%s\n' '错误：src 与 runtime 的 assets 目录部署后仍不一致。' >&2
  exit 1
fi

runtime_hash="$(
  cd "${BUNDLE_ROOT}"
  find runtime \
    -type f \
    ! -name '*.pyc' \
    ! -name '*.pyo' \
    ! -path '*/__pycache__/*' \
    -print0 |
    LC_ALL=C sort -z |
    xargs -0 sha256sum |
    sha256sum |
    awk '{print $1}'
)"
printf '%s  runtime\n' "${runtime_hash}" >"${BUNDLE_ROOT}/RUNTIME_TREE_SHA256"

printf '%s\n' \
  "IK runtime 部署完成；旧文件备份在 ${BACKUP_DIR}" \
  "保留入口：${RUNTIME_BIN}/arm_ik_producer" \
  "新二进制：${RUNTIME_BIN}/arm_ik_producer.bin" \
  "runtime ELF 已移除 DWARF 调试信息；staging 仍保留调试版" \
  "官方 probe：${RUNTIME_BIN}/tianji_official_ik_probe" \
  "官方 SDK：${SDK_RUNTIME_ROOT}" \
  "wuji2 手桥：${RUNTIME_BIN}/wuji_hand2_bridge" \
  "配置已同步：${SOURCE_CONFIG} -> ${RUNTIME_CONFIG}" \
  "资产已同步：${SOURCE_ASSETS} -> ${RUNTIME_ASSETS}"
