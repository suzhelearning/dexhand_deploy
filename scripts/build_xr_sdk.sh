#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

source_dir=""
library_dir=""
output_dir="${BUNDLE_ROOT}/vendor/xr_sdk"
build_dir="${BUNDLE_ROOT}/staging/xr_sdk-build"
python_bin="${PYTHON_BIN:-python}"
jobs=""

usage() {
  printf '%s\n' \
    '用法: build_xr_sdk.sh --source PATH [选项]' \
    '' \
    '从 XRoboToolkit-PC-Service-Pybind 源码构建当前工程使用的 xrobotoolkit_sdk。' \
    '源码会被复制到临时 staging 目录，参考工程不会被修改。' \
    '' \
    '必选参数:' \
    '  --source PATH       含 CMakeLists.txt、bindings/ 和 include/ 的 SDK 源码根目录' \
    '' \
    '可选参数:' \
    '  --library-dir PATH  含 libPXREARobotSDK.so 的目录；默认按当前架构从 source/lib 选择' \
    '  --output-dir PATH   输出目录；默认 vendor/xr_sdk（Git 忽略）' \
    '  --build-dir PATH    CMake 构建父目录；默认 staging/xr_sdk-build' \
    '  --python PATH       Python 解释器；默认 python（要求 Python 3.10）' \
    '  --jobs N            并行编译任务数；默认系统 CPU 数' \
    '  --help              显示帮助'
}

fail_usage() {
  printf '错误：%s\n' "$1" >&2
  exit 2
}

while (($#)); do
  case "$1" in
    --source)
      [[ $# -ge 2 ]] || fail_usage '--source 缺少路径'
      source_dir="$2"
      shift 2
      ;;
    --library-dir)
      [[ $# -ge 2 ]] || fail_usage '--library-dir 缺少路径'
      library_dir="$2"
      shift 2
      ;;
    --output-dir)
      [[ $# -ge 2 ]] || fail_usage '--output-dir 缺少路径'
      output_dir="$2"
      shift 2
      ;;
    --build-dir)
      [[ $# -ge 2 ]] || fail_usage '--build-dir 缺少路径'
      build_dir="$2"
      shift 2
      ;;
    --python)
      [[ $# -ge 2 ]] || fail_usage '--python 缺少解释器'
      python_bin="$2"
      shift 2
      ;;
    --jobs)
      [[ $# -ge 2 ]] || fail_usage '--jobs 缺少数量'
      jobs="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      fail_usage "未知参数: $1"
      ;;
  esac
done

[[ -n "${source_dir}" ]] || fail_usage '必须提供 --source'

if ! python_bin_path="$(command -v -- "${python_bin}")"; then
  fail_usage "找不到 Python 解释器: ${python_bin}"
fi
if ! python_bin_path="$(realpath -e -- "${python_bin_path}")"; then
  fail_usage "无法解析 Python 解释器: ${python_bin}"
fi

if ! python_version="$("${python_bin_path}" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"; then
  fail_usage "无法运行 Python 解释器: ${python_bin_path}"
fi
[[ "${python_version}" == 3.10 ]] ||
  fail_usage "XRoboToolkit Pybind 扩展要求 Python 3.10，当前为 ${python_version}"

if ! pybind11_dir="$("${python_bin_path}" -c 'import pybind11; print(pybind11.get_cmake_dir())')"; then
  fail_usage '当前 Python 缺少 pybind11；请在 pixi 环境中运行该脚本'
fi
[[ -d "${pybind11_dir}" ]] || fail_usage "pybind11 CMake 目录不存在: ${pybind11_dir}"

if ! source_dir="$(realpath -e -- "${source_dir}")"; then
  fail_usage "SDK source 不存在: ${source_dir}"
fi
[[ -f "${source_dir}/CMakeLists.txt" ]] ||
  fail_usage "SDK source 缺少 CMakeLists.txt: ${source_dir}"
[[ -f "${source_dir}/bindings/py_bindings.cpp" ]] ||
  fail_usage "SDK source 缺少 bindings/py_bindings.cpp: ${source_dir}"
[[ -d "${source_dir}/include" ]] ||
  fail_usage "SDK source 缺少 include 目录: ${source_dir}"

if [[ -z "${library_dir}" ]]; then
  case "$(uname -m)" in
    aarch64|arm64) library_dir="${source_dir}/lib/aarch64" ;;
    *) library_dir="${source_dir}/lib" ;;
  esac
fi
if ! library_dir="$(realpath -e -- "${library_dir}")"; then
  fail_usage "SDK library directory 不存在: ${library_dir}"
fi
library_path="${library_dir}/libPXREARobotSDK.so"
[[ -f "${library_path}" ]] ||
  fail_usage "SDK library directory 缺少 libPXREARobotSDK.so: ${library_dir}"

if [[ -z "${jobs}" ]]; then
  jobs="$(getconf _NPROCESSORS_ONLN 2>/dev/null || true)"
  [[ "${jobs}" =~ ^[0-9]+$ && "${jobs}" -gt 0 ]] || jobs=1
fi
[[ "${jobs}" =~ ^[1-9][0-9]*$ ]] || fail_usage "--jobs 必须是正整数: ${jobs}"

if ! mkdir -p -- "${output_dir}" "${build_dir}"; then
  printf '错误：无法创建输出或构建目录: %s / %s\n' "${output_dir}" "${build_dir}" >&2
  exit 1
fi
if ! output_dir="$(realpath -e -- "${output_dir}")"; then
  printf '%s\n' '错误：无法解析输出目录。' >&2
  exit 1
fi
if ! build_dir="$(realpath -e -- "${build_dir}")"; then
  printf '%s\n' '错误：无法解析构建目录。' >&2
  exit 1
fi

output_python="${output_dir}/python"
output_library="${output_dir}/lib"
mkdir -p -- "${output_python}" "${output_library}"

temporary_root="$(mktemp -d "${TMPDIR:-/tmp}/tianji-xr-sdk.XXXXXX")"
cleanup() {
  rm -rf -- "${temporary_root}"
}
trap cleanup EXIT

staged_source="${temporary_root}/source"
staged_output="${temporary_root}/output/python"
mkdir -p -- "${staged_source}" "${staged_output}"
cp -a -- "${source_dir}/." "${staged_source}/"

case "$(uname -m)" in
  aarch64|arm64) staged_library_dir="${staged_source}/lib/aarch64" ;;
  *) staged_library_dir="${staged_source}/lib" ;;
esac
mkdir -p -- "${staged_library_dir}"
cp -L -- "${library_path}" "${staged_library_dir}/libPXREARobotSDK.so"

cmake_build_dir="$(mktemp -d "${build_dir}/run.XXXXXX")"
printf 'XR SDK source: %s\n' "${source_dir}"
printf 'XR SDK library: %s\n' "${library_path}"
printf 'XR SDK build: %s\n' "${cmake_build_dir}"
printf 'XR SDK output: %s\n' "${output_dir}"

cmake -S "${staged_source}" -B "${cmake_build_dir}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE="${python_bin_path}" \
  -Dpybind11_DIR="${pybind11_dir}" \
  -DCMAKE_LIBRARY_OUTPUT_DIRECTORY="${staged_output}" \
  -DCMAKE_RUNTIME_OUTPUT_DIRECTORY="${staged_output}" \
  -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY="${staged_output}"
cmake --build "${cmake_build_dir}" --parallel "${jobs}"

module_path="$(find "${staged_output}" -maxdepth 1 -type f -name 'xrobotoolkit_sdk*.so' -print -quit)"
[[ -n "${module_path}" ]] || {
  printf '错误：构建完成但未找到 xrobotoolkit_sdk*.so: %s\n' "${staged_output}" >&2
  exit 1
}

# The output directories are generated runtime assets.  Remove only stale
# modules with this package name so Python cannot select a previous ABI.
find "${output_python}" -maxdepth 1 -type f -name 'xrobotoolkit_sdk*.so' -delete
cp -L -- "${module_path}" "${output_python}/"
cp -L -- "${library_path}" "${output_library}/libPXREARobotSDK.so"

if ! PYTHONPATH="${output_python}${PYTHONPATH:+:${PYTHONPATH}}" \
     LD_LIBRARY_PATH="${output_library}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
     "${python_bin_path}" - <<'PY'
import importlib
import json
import sys

required = (
    "init",
    "get_headset_pose",
    "get_left_controller_pose",
    "get_right_controller_pose",
    "get_left_trigger",
    "get_right_trigger",
    "get_left_grip",
    "get_right_grip",
    "get_left_axis",
    "get_right_axis",
    "get_motion_timestamp_ns",
)
module = importlib.import_module("xrobotoolkit_sdk")
missing = [name for name in required if not callable(getattr(module, name, None))]
if missing:
    print("缺少 XR SDK callable: " + ", ".join(missing), file=sys.stderr)
    raise SystemExit(1)
print(json.dumps({
    "module": "xrobotoolkit_sdk",
    "module_path": getattr(module, "__file__", ""),
    "missing": [],
}, ensure_ascii=False, sort_keys=True))
PY
then
  printf '%s\n' '错误：xrobotoolkit_sdk 导入或 API 校验失败。' >&2
  exit 1
fi

printf '%s\n' 'XR SDK 构建完成。运行 vr_manus_xr_sim 时会自动发现:'
printf '  TIANJI_XR_SDK_PYTHONPATH=%s\n' "${output_python}"
printf '  TIANJI_XR_SDK_LIBRARY_DIR=%s\n' "${output_library}"
