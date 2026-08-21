#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if ! python -c 'import h5py, mujoco' 2>/dev/null; then
  if command -v pixi >/dev/null 2>&1; then
    exec pixi run bash "${BASH_SOURCE[0]}" "$@"
  fi
fi

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
activate_bundle_runtime

H5_PATH=""
SPEED=1.0
LOOP=false
PAUSED=false
VALIDATE_ONLY=false
HEADLESS=false

usage() {
  cat <<'EOF'
用法：
  pixi run sim_regrind_h5 -- TAKE_REGRIND.h5 [选项]
  bash scripts/run_regrind_h5_replay.sh TAKE_REGRIND.h5 [选项]

独立回放 H5 同目录 README 声明的：
  regrind_retargeting_root_*   wuji2 r_base 自由根，WXYZ
  regrind_retargeting_joints   20 个 wuji2 关节，弧度
  object_pos/object_quat       物体自由根，WXYZ

选项：
  --speed N         回放倍速，必须 >0（默认 1）
  --loop            循环回放
  --paused          启动后停在 frame0
  --validate-only   校验 H5、right.urdf、锤子资产和首末帧后退出
  --headless        无窗口依次应用全部帧后退出
  -h, --help

窗口控制：Space 暂停/继续；R 从 frame0 重播；关闭窗口退出。
该路径不启动 Motive、Zenoh、机械臂 IK，也不连接 Marvin 控制器。
EOF
}

while (($#)); do
  case "$1" in
    --speed)
      if (($# < 2)); then
        printf '%s\n' '错误：--speed 缺少数值。' >&2
        exit 2
      fi
      SPEED="$2"
      shift
      ;;
    --loop)
      LOOP=true
      ;;
    --paused)
      PAUSED=true
      ;;
    --validate-only)
      VALIDATE_ONLY=true
      ;;
    --headless)
      HEADLESS=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --*)
      printf '错误：未知参数 %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
    *)
      if [[ -n "${H5_PATH}" ]]; then
        printf '错误：只能提供一个 H5，重复值：%s\n' "$1" >&2
        exit 2
      fi
      H5_PATH="$1"
      ;;
  esac
  shift
done

if [[ -z "${H5_PATH}" ]]; then
  printf '%s\n' '错误：必须提供 Regrind H5 路径。' >&2
  usage >&2
  exit 2
fi
if [[ ! -f "${H5_PATH}" ]]; then
  printf '错误：H5 文件不存在：%s\n' "${H5_PATH}" >&2
  exit 2
fi
if [[ "${VALIDATE_ONLY}" != true && "${HEADLESS}" != true && -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
  printf '%s\n' '错误：无显示环境；请使用 --validate-only 或 --headless。' >&2
  exit 1
fi

VIEWER="${BUNDLE_ROOT}/src/pico_body_tianji/scripts/mujoco_regrind_replay.py"
if [[ ! -f "${VIEWER}" ]]; then
  printf '错误：Regrind viewer 不存在：%s\n' "${VIEWER}" >&2
  exit 1
fi

arguments=("${H5_PATH}" --speed "${SPEED}")
if [[ "${LOOP}" == true ]]; then
  arguments+=(--loop)
fi
if [[ "${PAUSED}" == true ]]; then
  arguments+=(--paused)
fi
if [[ "${VALIDATE_ONLY}" == true ]]; then
  arguments+=(--validate-only)
fi
if [[ "${HEADLESS}" == true ]]; then
  arguments+=(--headless)
fi

printf '%s\n' \
  '启动 Regrind wuji2 + 物体独立 MuJoCo 回放：' \
  "  H5=${H5_PATH}" \
  "  speed=${SPEED} loop=${LOOP} paused=${PAUSED}" \
  '  坐标系=桌面中心、z-up；root=r_base；quat=WXYZ；dataset=regrind_retargeting_*' \
  '  不启动 Motive/Zenoh/IK，不读取 wuji_retargeting_*。'

exec python "${VIEWER}" "${arguments[@]}"
