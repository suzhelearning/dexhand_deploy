#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
  printf '%s\n' '错误：IK 只能在 Linux x86_64 构建。' >&2
  exit 1
fi
if [[ ! -r /etc/os-release ]] ||
   ! grep -Eq 'VERSION_ID="(22\.04|24\.04)"' /etc/os-release; then
  printf '%s\n' '错误：IK 构建环境必须是 Ubuntu 22.04 或 24.04。' >&2
  exit 1
fi

# 编译器：系统默认 GCC（22.04 为 GCC 11，24.04 为 GCC 13）。
# 新二进制经 runtime/abi 便携层部署，GLIBCXX 需求必须被
# runtime/abi/lib/libstdc++.so.6 覆盖（见下方 ABI 检查）。
for executable in /usr/bin/gcc /usr/bin/g++; do
  if [[ ! -x "${executable}" ]]; then
    printf '错误：缺少编译器：%s\n' "${executable}" >&2
    exit 1
  fi
done
cc_version="$(/usr/bin/gcc -dumpfullversion -dumpversion)"
cc_major="${cc_version%%.*}"
if (( cc_major < 11 )); then
  printf '错误：要求 GCC >= 11，当前为 %s。\n' "${cc_version}" >&2
  exit 1
fi
if (( cc_major > 13 )); then
  printf '错误：GCC %s 的 GLIBCXX 版本超过 runtime/abi 便携层（GCC 13）；\n' \
    "${cc_version}" >&2
  printf '  请改用 gcc-13（update-alternatives 或环境变量）。\n' >&2
  exit 1
fi

# 编译环境来自 pixi ik-build，必须通过 pixi run -e ik-build 执行本脚本。
CONDA_PREFIX="${CONDA_PREFIX:-}"
if [[ -z "${CONDA_PREFIX}" || ! -x "${CONDA_PREFIX}/bin/python" ]]; then
  printf '%s\n' '错误：请通过 pixi run -e ik-build 运行本脚本。' >&2
  exit 1
fi
for executable in "${CONDA_PREFIX}/bin/cmake"; do
  if [[ ! -x "${executable}" ]]; then
    printf '错误：缺少构建工具：%s\n' "${executable}" >&2
    exit 1
  fi
done

# Zenoh C/C++ 绑定（vendored，不入 git）：libzenohc.so + 头文件。
ZENOH_LIB="${BUNDLE_ROOT}/vendor/zenoh/lib/libzenohc.so"
ZENOH_C_HEADER="${BUNDLE_ROOT}/vendor/zenoh/include/zenoh.h"
ZENOH_CPP_HEADER="${BUNDLE_ROOT}/vendor/zenoh-cpp/include/zenoh.hxx"
for required in "${ZENOH_LIB}" "${ZENOH_C_HEADER}" "${ZENOH_CPP_HEADER}"; do
  if [[ ! -e "${required}" ]]; then
    printf '错误：缺少 Zenoh 绑定：%s\n' "${required}" >&2
    printf '%s\n' \
      '  请从 eclipse-zenoh/zenoh-c release 1.10.0 下载' \
      '  zenoh-c-1.10.0-x86_64-unknown-linux-gnu-standalone.zip 解压到' \
      '  vendor/zenoh/（include/ 与 lib/），并把 zenoh-cpp 1.10.0 的' \
      '  include/ 放到 vendor/zenoh-cpp/include/。' >&2
    exit 1
  fi
done
if ! "${CONDA_PREFIX}/bin/python" -c 'import zenoh' 2>/dev/null; then
  printf '%s\n' '错误：ik-build 环境缺少 eclipse-zenoh（pixi install -e ik-build）。' >&2
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

# GCC 编译的二进制经 runtime/abi 便携层部署；该层 libstdc++ 至少提供
# GLIBCXX_3.4.29（GCC 11）。精确的 GLIBC/GLIBCXX 需求约束由
# build_ik.sh 在编译后按产物实测检查。
runtime_cpp_max="$(
  objdump -T "${runtime_cpp}" |
    grep -v '\*UND\*' |
    grep -oE 'GLIBCXX_[0-9.]+' |
    sort -V |
    tail -n 1
)"
if [[ "$(printf '%s\n%s\n' "3.4.29" "${runtime_cpp_max}" | sort -V | tail -n 1)" != "${runtime_cpp_max}" ]]; then
  printf '错误：runtime/abi libstdc++（GLIBCXX %s）低于便携层最小需求 3.4.29。\n' \
    "${runtime_cpp_max}" >&2
  exit 1
fi

printf '%s\n' \
  'IK 构建环境检查通过：Ubuntu 24.04 + GCC 13 + Pinocchio 4.0.0 + Zenoh 1.10.0。'
