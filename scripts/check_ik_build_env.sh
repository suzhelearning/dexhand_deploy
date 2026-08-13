#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"

if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
  printf '%s\n' '错误：IK 只能在 Linux x86_64 构建。' >&2
  exit 1
fi
if [[ ! -r /etc/os-release ]] ||
   ! grep -Fxq 'VERSION_ID="22.04"' /etc/os-release; then
  printf '%s\n' '错误：IK 构建环境必须是 Ubuntu 22.04。' >&2
  exit 1
fi
for executable in /usr/bin/gcc-11 /usr/bin/g++-11 /usr/bin/cmake /usr/bin/colcon; do
  if [[ ! -x "${executable}" ]]; then
    printf '错误：缺少构建工具：%s\n' "${executable}" >&2
    exit 1
  fi
done
if [[ ! -f "${ROS_SETUP}" ]]; then
  printf '错误：缺少 ROS Humble 开发环境：%s\n' "${ROS_SETUP}" >&2
  exit 1
fi

# ROS 生成的 setup.bash 会读取若干可能未定义的跟踪变量。
set +u
# shellcheck disable=SC1091
source "${ROS_SETUP}"
set -u
for package in \
  ament_cmake \
  ament_cmake_python \
  ament_index_cpp \
  geometry_msgs \
  rclcpp \
  sensor_msgs \
  std_msgs \
  tf2_ros \
  visualization_msgs
do
  if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    printf '错误：缺少 ROS Humble 开发包：%s\n' "${package}" >&2
    exit 1
  fi
done

rclcpp_version="$({
  sed -n 's/.*Found rclcpp: \([^ ]*\).*/\1/p' \
    /opt/ros/humble/share/rclcpp/cmake/rclcppConfig.cmake
} | head -n 1)"
if [[ "${rclcpp_version}" != 16.0.19 ]]; then
  printf '错误：要求 rclcpp 16.0.19，当前为 %s。\n' \
    "${rclcpp_version:-unknown}" >&2
  exit 1
fi

python - <<'PY'
from importlib.metadata import distribution
from pathlib import Path

pin = distribution("pin")
if pin.version != "4.0.0":
    raise SystemExit(f"错误：要求 pin 4.0.0，当前为 {pin.version}")
prefix = pin.locate_file("cmeel.prefix").resolve()
required = (
    prefix / "include" / "pinocchio",
    prefix / "lib" / "cmake" / "pinocchio" / "pinocchioConfig.cmake",
    prefix / "lib" / "libpinocchio_default.so",
    prefix / "lib" / "libpinocchio_parsers.so",
)
missing = [str(path) for path in required if not path.exists()]
if missing:
    raise SystemExit("错误：pin wheel 缺少开发文件：" + ", ".join(missing))
print("Pinocchio 构建前缀：", prefix)
PY

pin_prefix="$(python - <<'PY'
from importlib.metadata import distribution
print(distribution("pin").locate_file("cmeel.prefix").resolve())
PY
)"
for library in libpinocchio_default.so libpinocchio_parsers.so; do
  if ! cmp -s \
    "${pin_prefix}/lib/${library}" \
    "${BUNDLE_ROOT}/runtime/pin/lib/${library}"
  then
    printf '错误：Pixi %s 与 runtime/pin 的 ABI 不一致。\n' \
      "${library}" >&2
    exit 1
  fi
done

runtime_loader="${BUNDLE_ROOT}/runtime/abi/lib/ld-linux-x86-64.so.2"
runtime_cpp="${BUNDLE_ROOT}/runtime/abi/lib/libstdc++.so.6"
if [[ ! -x "${runtime_loader}" || ! -f "${runtime_cpp}" ]]; then
  printf '%s\n' '错误：bundled ABI runtime 不完整。' >&2
  exit 1
fi

printf '%s\n' \
  'IK 构建环境检查通过：Ubuntu 22.04 + GCC 11 + ROS Humble 16.0.19 + Pinocchio 4.0.0。'
