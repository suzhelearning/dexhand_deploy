#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if ! python -c 'import h5py, mujoco' 2>/dev/null; then
  printf '%s\n' '错误：请在 pixi 环境运行（h5py/mujoco 缺失）。' >&2
  exit 1
fi

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
activate_bundle_runtime

H5_PATH=""
SPEED=1.0
HOLD_S=2.0
LOOP=false
PAUSED=false
VALIDATE_ONLY=false
HEADLESS=false
YAW_DEG=0.0

usage() {
  cat <<'EOF'
用法：
  pixi run sim_mocap_h5_replay -- TAKE.h5 [选项]
  bash scripts/run_mocap_h5_wrist_replay.sh TAKE.h5 [选项]

说明：
  机械臂+wuji2 组合 URDF 场景的纯 H5 数据回放：机械臂摆到
  controller_only_ik.yaml 的 left/right_home_deg 关节角（与
  sim_mocap_h5 的 Home 一致），只把 H5 右手 21 点关键点经
  Manus→wuji2 外参转到局部坐标后叠在 Home r_wrist 上按时间播放。
  不启动 IK/Motive/Zenoh。

  Home 关节角（度）：
    left  [55, -65, -70, -60, 60, 0, 0]
    right [-55, -65, 70, -60, -60, 0, 0]

选项：
  --speed N       播放倍速（默认 1.0）
  --hold-s N      首帧停留秒数（默认 2.0）
  --yaw-deg N     绕 Motive +Y 旋转整条轨迹（默认 0）
  --loop          循环播放
  --paused        开始即暂停
  --validate-only 只校验并输出首末帧腕点，不打开窗口
  --headless      依次应用全部帧后退出
  -h, --help

按键：
  Space  暂停/继续
  R      从首帧重播
  Esc/关闭窗口  退出
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
    --hold-s)
      if (($# < 2)); then
        printf '%s\n' '错误：--hold-s 缺少数值。' >&2
        exit 2
      fi
      HOLD_S="$2"
      shift
      ;;
    --yaw-deg)
      if (($# < 2)); then
        printf '%s\n' '错误：--yaw-deg 缺少数值。' >&2
        exit 2
      fi
      YAW_DEG="$2"
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
  printf '%s\n' '错误：必须提供要回放的 TAKE.h5 路径。' >&2
  usage >&2
  exit 2
fi
if [[ ! -f "${H5_PATH}" ]]; then
  printf '错误：H5 文件不存在：%s\n' "${H5_PATH}" >&2
  exit 2
fi
if [[
  "${VALIDATE_ONLY}" != true &&
  "${HEADLESS}" != true &&
  -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}"
]]; then
  printf '%s\n' '错误：无显示环境，无法启动 MuJoCo 窗口。' \
    '可使用 --validate-only 或 --headless。' >&2
  exit 1
fi

VIEWER="${BUNDLE_ROOT}/src/pico_body_tianji/scripts/mujoco_h5_wrist_replay.py"
if [[ ! -f "${VIEWER}" ]]; then
  printf '错误：纯数据回放 viewer 不存在：%s\n' "${VIEWER}" >&2
  exit 1
fi

arguments=(
  "${H5_PATH}"
  --speed "${SPEED}"
  --hold-s "${HOLD_S}"
  --yaw-deg "${YAW_DEG}"
)
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
  '启动机械臂+wuji2 纯 H5 数据回放：' \
  "  H5=${H5_PATH}" \
  "  speed=${SPEED} hold_s=${HOLD_S} yaw_deg=${YAW_DEG}" \
  "  loop=${LOOP} paused=${PAUSED}" \
  '  机械臂保持 Home；只播放右手 21 点手部轨迹；不启动 IK/Motive/Zenoh。'

exec python "${VIEWER}" "${arguments[@]}"
