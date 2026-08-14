#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
STAGING_BIN="${BUNDLE_ROOT}/staging/ik/lib/pico_body_tianji"
RUNTIME_BIN="${BUNDLE_ROOT}/runtime/pico_body_tianji/lib/pico_body_tianji"
NEW_IK="${STAGING_BIN}/tianji_kinematic_sim"
NEW_PROBE="${STAGING_BIN}/tianji_official_ik_probe"
NEW_WORKER="${STAGING_BIN}/tianji_official_ik_worker"
BACKUP_DIR="${BUNDLE_ROOT}/staging/runtime-backup"
SDK_SOURCE_ROOT="${TIANJI_OFFICIAL_SDK_ROOT:-/home/ice/TJ_FX_ROBOT_CONTRL_SDK}"
SDK_RUNTIME_ROOT="${BUNDLE_ROOT}/runtime/tianji_official"
SDK_LIBRARY="${SDK_SOURCE_ROOT}/kinematicsSDK/libKine.so"
SDK_CONFIG="${SDK_SOURCE_ROOT}/CommonConfig/ccs_m6_40.MvKDCfg"
RUNTIME_SHARE="${BUNDLE_ROOT}/runtime/pico_body_tianji/share/pico_body_tianji"
SOURCE_CONFIG="${BUNDLE_ROOT}/src/pico_body_tianji/config"
RUNTIME_CONFIG="${RUNTIME_SHARE}/config"

for binary in "${NEW_IK}" "${NEW_PROBE}" "${NEW_WORKER}"; do
  if [[ ! -x "${binary}" ]]; then
    printf '错误：请先执行 pixi run -e ik-build build-ik；缺少 %s\n' \
      "${binary}" >&2
    exit 1
  fi
done
for sdk_file in "${SDK_LIBRARY}" "${SDK_CONFIG}"; do
  if [[ ! -f "${sdk_file}" ]]; then
    printf '错误：天机官方 SDK 文件不存在：%s\n' "${sdk_file}" >&2
    exit 1
  fi
done
if [[ ! -x "${RUNTIME_BIN}/tianji_kinematic_sim" ]]; then
  printf '%s\n' '错误：runtime IK Bash 包装器不存在，拒绝部署。' >&2
  exit 1
fi
if ! head -n 1 "${RUNTIME_BIN}/tianji_kinematic_sim" | grep -Fxq '#!/usr/bin/env bash'; then
  printf '%s\n' '错误：runtime IK 入口不是预期的 Bash 包装器，拒绝部署。' >&2
  exit 1
fi

mkdir -p "${BACKUP_DIR}"
for path in \
  "${RUNTIME_BIN}/tianji_kinematic_sim.bin" \
  "${RUNTIME_BIN}/tianji_kinematic_sim" \
  "${RUNTIME_BIN}/tianji_official_ik_probe" \
  "${RUNTIME_BIN}/tianji_official_ik_probe.bin" \
  "${RUNTIME_BIN}/tianji_official_ik_worker" \
  "${RUNTIME_BIN}/tianji_official_ik_worker.bin" \
  "${SDK_RUNTIME_ROOT}/kinematicsSDK/libKine.so" \
  "${SDK_RUNTIME_ROOT}/CommonConfig/ccs_m6_40.MvKDCfg" \
  "${RUNTIME_SHARE}/launch/controller_only_ik.launch.py"
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

mkdir -p \
  "${SDK_RUNTIME_ROOT}/kinematicsSDK" \
  "${SDK_RUNTIME_ROOT}/CommonConfig" \
  "${RUNTIME_CONFIG}" \
  "${RUNTIME_SHARE}/launch"
install -m 0755 \
  "${SDK_LIBRARY}" \
  "${SDK_RUNTIME_ROOT}/kinematicsSDK/libKine.so"
install -m 0644 \
  "${SDK_CONFIG}" \
  "${SDK_RUNTIME_ROOT}/CommonConfig/ccs_m6_40.MvKDCfg"

install -m 0755 "${NEW_IK}" "${RUNTIME_BIN}/tianji_kinematic_sim.bin.new"
install -m 0755 "${NEW_PROBE}" "${RUNTIME_BIN}/tianji_official_ik_probe.bin.new"
install -m 0755 "${NEW_WORKER}" "${RUNTIME_BIN}/tianji_official_ik_worker.bin.new"
mv -f -- \
  "${RUNTIME_BIN}/tianji_kinematic_sim.bin.new" \
  "${RUNTIME_BIN}/tianji_kinematic_sim.bin"
mv -f -- \
  "${RUNTIME_BIN}/tianji_official_ik_probe.bin.new" \
  "${RUNTIME_BIN}/tianji_official_ik_probe.bin"
mv -f -- \
  "${RUNTIME_BIN}/tianji_official_ik_worker.bin.new" \
  "${RUNTIME_BIN}/tianji_official_ik_worker.bin"
install -m 0755 \
  "${BUNDLE_ROOT}/scripts/runtime_tianji_kinematic_sim.sh" \
  "${RUNTIME_BIN}/tianji_kinematic_sim"
install -m 0755 \
  "${BUNDLE_ROOT}/scripts/runtime_tianji_official_ik_probe.sh" \
  "${RUNTIME_BIN}/tianji_official_ik_probe"
install -m 0755 \
  "${BUNDLE_ROOT}/scripts/runtime_tianji_official_ik_worker.sh" \
  "${RUNTIME_BIN}/tianji_official_ik_worker"
# config 是一个整体：每次部署都让 runtime 与 src 递归一致。
# --delete 会清理已从源码删除的旧 profile，避免便携包留下过期配置。
rsync -a --delete \
  "${SOURCE_CONFIG}/" \
  "${RUNTIME_CONFIG}/"
if ! diff -qr -- "${SOURCE_CONFIG}" "${RUNTIME_CONFIG}" >/dev/null; then
  printf '%s\n' '错误：src 与 runtime 的 config 目录部署后仍不一致。' >&2
  exit 1
fi
install -m 0644 \
  "${BUNDLE_ROOT}/src/pico_body_tianji/launch/controller_only_ik.launch.py" \
  "${RUNTIME_SHARE}/launch/controller_only_ik.launch.py"

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
  "保留入口：${RUNTIME_BIN}/tianji_kinematic_sim" \
  "新二进制：${RUNTIME_BIN}/tianji_kinematic_sim.bin" \
  "官方 probe：${RUNTIME_BIN}/tianji_official_ik_probe" \
  "官方 SDK：${SDK_RUNTIME_ROOT}" \
  "配置已同步：${SOURCE_CONFIG} -> ${RUNTIME_CONFIG}"
