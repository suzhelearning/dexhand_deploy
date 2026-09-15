#!/usr/bin/env bash
# Shortcut for the tested mapped-palm arm-only MuJoCo route. No hardware actuators.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
record_dir="$ROOT/recordings/device_acceptance"
preview=false
while (($#)); do
  case "$1" in
    --dry-run) preview=true; shift ;;
    --record-dir)
      [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || { echo '错误：--record-dir 需要目录' >&2; exit 2; }
      record_dir="$2"; shift 2 ;;
    -h|--help)
      printf '%s\n' \
        '用法：bash scripts/run_pico_vr_arms_cpp.sh [--dry-run] [--record-dir PATH]' \
        'PICO＋VR 手柄 → C++ mapped-palm → MuJoCo 机械臂；不启用 Manus/手部控制。' \
        '先连接头显并启动 Zenoh router；使用默认 PICO 标定目录。' \
        '权重读取当前 bandwidth.yaml（当前位置惩罚 90000），脚本不改写配置。' \
        'c：双手水平前伸标定；s：接管；h：回 Home；r：手动重置；q：保存退出。' \
        '--dry-run 仅打印命令，不创建文件、不连接设备。相对录制目录相对于工程根目录。'
      exit 0 ;;
    *) echo "错误：未知选项 $1；此脚本只用于机械臂仿真，请用 --help" >&2; exit 2 ;;
  esac
done
cd -- "$ROOT"
[[ "$record_dir" == /* ]] || record_dir="$ROOT/$record_dir"
record="$record_dir/native_arms_mapped_$(date +%Y%m%d_%H%M%S_%N)_$$.h5"
export TIANJI_ROUTER_ENDPOINT="${TIANJI_ROUTER_ENDPOINT:-tcp/127.0.0.1:7447}"
args=(bash scripts/run_session.sh
  --profile pico_vr_manus_sim
  --viewer --viewer-backend cpp
  --disable-hands --scheduler-backend cpp
  --publication-backend cpp --recording-adapter cpp
  --ik-backend pico_ee_mapped_corrected_palm_velocity_qp
  --mapped-palm-xz-calibration
  --arm-target-processor passthrough --joint-trajectory passthrough
  --command-step-clipping false --joint-limit-source urdf
  --tjvr-bind 127.0.0.1 --tjvr-port 15000
  --pico-world-x-offset-m 0.20
  --spark-overlay --pico-startup-timeout-s 30
  --record "$record")
if "$preview"; then
  printf 'TIANJI_ROUTER_ENDPOINT=%q\n' "$TIANJI_ROUTER_ENDPOINT"
  printf '%q ' pixi run "${args[@]}"
  printf '\n'
  exit 0
fi
command -v pixi >/dev/null || { echo '错误：找不到 pixi' >&2; exit 2; }
mkdir -p -- "$record_dir"
printf 'PICO＋VR C++ 机械臂仿真；录制：%s\n' "$record"
# Keep the original session as process owner: signals, Home and recorder draining
# are handled by its existing lifecycle, not by an additional wrapper supervisor.
exec pixi run "${args[@]}"
