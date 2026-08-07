#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
IK_WRAPPER="${BUNDLE_ROOT}/runtime/pico_body_tianji/lib/pico_body_tianji/tianji_kinematic_sim"
RUNTIME_HASH_FILE="${BUNDLE_ROOT}/RUNTIME_TREE_SHA256"
PATCH_BEGIN="# BEGIN tianji_teleop CycloneDDS multiarch path patch"
PATCH_END="# END tianji_teleop CycloneDDS multiarch path patch"
TEMP_FILE=""

cleanup() {
  if [[ -n "${TEMP_FILE}" ]]; then
    rm -f -- "${TEMP_FILE}"
  fi
}
trap cleanup EXIT

usage() {
  cat <<'EOF'
用法：
  bash scripts/fix_cyclonedds_path.sh              应用补丁（默认）
  bash scripts/fix_cyclonedds_path.sh --apply      应用补丁
  bash scripts/fix_cyclonedds_path.sh --rollback   回退补丁
  bash scripts/fix_cyclonedds_path.sh --check      检查补丁和运行时哈希
EOF
}

calculate_runtime_hash() {
  (
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
  )
}

recorded_runtime_hash() {
  awk 'NR == 1 {print $1}' "${RUNTIME_HASH_FILE}"
}

verify_runtime_hash() {
  local recorded=""
  local actual=""
  recorded="$(recorded_runtime_hash)"
  actual="$(calculate_runtime_hash)"
  if [[ -z "${recorded}" || "${recorded}" != "${actual}" ]]; then
    printf '%s\n' \
      '错误：运行时树在执行补丁前已经被其他内容修改，拒绝覆盖。' \
      "  记录哈希：${recorded:-<空>}" \
      "  实际哈希：${actual}" >&2
    return 1
  fi
}

write_runtime_hash() {
  local actual=""
  actual="$(calculate_runtime_hash)"
  TEMP_FILE="$(mktemp "${RUNTIME_HASH_FILE}.tmp.XXXXXX")"
  printf '%s  -\n' "${actual}" > "${TEMP_FILE}"
  chmod --reference="${RUNTIME_HASH_FILE}" "${TEMP_FILE}"
  mv -- "${TEMP_FILE}" "${RUNTIME_HASH_FILE}"
  TEMP_FILE=""
}

marker_state() {
  local begin_count=0
  local end_count=0
  begin_count="$(grep -Fxc -- "${PATCH_BEGIN}" "${IK_WRAPPER}" || true)"
  end_count="$(grep -Fxc -- "${PATCH_END}" "${IK_WRAPPER}" || true)"
  if [[ "${begin_count}" == 0 && "${end_count}" == 0 ]]; then
    printf '%s\n' absent
  elif [[ "${begin_count}" == 1 && "${end_count}" == 1 ]]; then
    printf '%s\n' applied
  else
    printf '错误：补丁标记不完整：begin=%s end=%s\n' \
      "${begin_count}" "${end_count}" >&2
    return 1
  fi
}

apply_patch_to_wrapper() {
  local anchor_count=0
  anchor_count="$(grep -Fxc -- 'ros_library_path=""' "${IK_WRAPPER}" || true)"
  if [[ "${anchor_count}" != 1 ]]; then
    printf '错误：无法唯一定位 ros_library_path，实际匹配数：%s\n' \
      "${anchor_count}" >&2
    return 1
  fi

  TEMP_FILE="$(mktemp "${IK_WRAPPER}.tmp.XXXXXX")"
  awk -v patch_begin="${PATCH_BEGIN}" -v patch_end="${PATCH_END}" '
    {
      print
    }
    $0 == "ros_library_path=\"\"" {
      print patch_begin
      print "if [[ -d \"${ROS_ROOT}/lib/x86_64-linux-gnu\" ]]; then"
      print "  ros_library_path=\"${ROS_ROOT}/lib/x86_64-linux-gnu\""
      print "fi"
      print patch_end
    }
  ' "${IK_WRAPPER}" > "${TEMP_FILE}"
  chmod --reference="${IK_WRAPPER}" "${TEMP_FILE}"
  mv -- "${TEMP_FILE}" "${IK_WRAPPER}"
  TEMP_FILE=""
}

rollback_patch_from_wrapper() {
  TEMP_FILE="$(mktemp "${IK_WRAPPER}.tmp.XXXXXX")"
  awk -v patch_begin="${PATCH_BEGIN}" -v patch_end="${PATCH_END}" '
    $0 == patch_begin {
      skipping = 1
      next
    }
    $0 == patch_end {
      skipping = 0
      next
    }
    !skipping {
      print
    }
  ' "${IK_WRAPPER}" > "${TEMP_FILE}"
  chmod --reference="${IK_WRAPPER}" "${TEMP_FILE}"
  mv -- "${TEMP_FILE}" "${IK_WRAPPER}"
  TEMP_FILE=""
}

if [[ ! -f "${IK_WRAPPER}" || ! -f "${RUNTIME_HASH_FILE}" ]]; then
  printf '%s\n' '错误：请在完整的 tianji_teleop 工程中运行此脚本。' >&2
  exit 1
fi

action="${1:---apply}"
if (($# > 1)); then
  usage >&2
  exit 2
fi

case "${action}" in
  --apply)
    verify_runtime_hash
    if [[ "$(marker_state)" == applied ]]; then
      printf '%s\n' 'CycloneDDS 路径补丁已经应用，无需重复修改。'
      exit 0
    fi
    apply_patch_to_wrapper
    write_runtime_hash
    printf '%s\n' \
      'CycloneDDS 路径补丁应用完成。' \
      '现在可以运行：pixi run test'
    ;;
  --rollback)
    verify_runtime_hash
    if [[ "$(marker_state)" == absent ]]; then
      printf '%s\n' 'CycloneDDS 路径补丁尚未应用，无需回退。'
      exit 0
    fi
    rollback_patch_from_wrapper
    write_runtime_hash
    printf '%s\n' 'CycloneDDS 路径补丁已回退。'
    ;;
  --check)
    verify_runtime_hash
    if [[ "$(marker_state)" == applied ]]; then
      printf '%s\n' '状态：补丁已应用，运行时哈希正确。'
    else
      printf '%s\n' '状态：补丁未应用，运行时哈希正确。'
    fi
    ;;
  -h|--help)
    usage
    ;;
  *)
    printf '错误：未知参数：%s\n' "${action}" >&2
    usage >&2
    exit 2
    ;;
esac
