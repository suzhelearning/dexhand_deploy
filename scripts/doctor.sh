#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

if [[ "$(uname -m)" != "x86_64" ]]; then
  printf '错误：厂商 SDK 仅打包了 x86_64 版本。\n' >&2
  exit 1
fi

activate_bundle_runtime

required_files=(
  "${BUNDLE_ROOT}/vendor/python/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so"
  "${BUNDLE_ROOT}/vendor/lib/libPXREARobotSDK.so"
  "${BUNDLE_ROOT}/vendor/python/marvin_sdk/libMarvinSDK.so"
  "${ZENOH_LIBRARY_ROOT}/libzenohc.so"
  "${ZENOH_C_INCLUDE_ROOT}/zenoh.h"
   "${ZENOH_CPP_INCLUDE_ROOT}/zenoh.hxx"
  "${BUNDLE_ROOT}/vendor/wuji-sdk/include/wuji_sdk.h"
  "${BUNDLE_ROOT}/vendor/wuji-sdk/lib/libwuji_sdk_c.so"
  "${BUNDLE_ROOT}/src/pico_body_tianji/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf"
  "${BUNDLE_ROOT}/src/pico_body_tianji/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4_mujoco.urdf"
  "${BUNDLE_ROOT}/src/pico_body_tianji/assets/marvin_m6_ccs/meshes/Link_Base.STL"
  "${BUNDLE_ROOT}/src/pico_body_tianji/assets/tianji_wuji2/tianji_wuji2.urdf"
  "${BUNDLE_ROOT}/src/pico_body_tianji/assets/tianji_wuji2/meshes/wuji2_r_wrist.STL"
  "${PROJECT_PREFIX}/lib/pico_body_tianji/tianji_kinematic_sim"
  "${PROJECT_PREFIX}/lib/pico_body_tianji/tianji_kinematic_sim.bin"
  "${PROJECT_PREFIX}/lib/pico_body_tianji/tianji_official_ik_probe"
  "${PROJECT_PREFIX}/lib/pico_body_tianji/tianji_official_ik_probe.bin"
  "${PROJECT_PREFIX}/lib/pico_body_tianji/tianji_official_ik_worker"
   "${PROJECT_PREFIX}/lib/pico_body_tianji/tianji_official_ik_worker.bin"
  "${PROJECT_PREFIX}/lib/pico_body_tianji/wuji_hand2_bridge"
  "${PROJECT_PREFIX}/lib/pico_body_tianji/wuji_hand2_bridge.bin"
  "${BUNDLE_ROOT}/runtime/tianji_official/kinematicsSDK/libKine.so"
  "${BUNDLE_ROOT}/runtime/tianji_official/CommonConfig/ccs_m6_40.MvKDCfg"
  "${ABI_LIBRARY_ROOT}/ld-linux-x86-64.so.2"
  "${PIN_LIBRARY_ROOT}/libpinocchio_default.so"
)
for file in "${required_files[@]}"; do
  if [[ ! -f "${file}" ]]; then
    printf '错误：缺少运行时文件：%s\n' "${file}" >&2
    exit 1
  fi
done

(
  cd "${BUNDLE_ROOT}"
  if ! sha256sum -c --quiet VENDOR_SHA256SUMS; then
    printf '错误：厂商运行时文件校验失败。\n' >&2
    exit 1
  fi
  expected_runtime_hash="$(awk 'NR == 1 {print $1}' RUNTIME_TREE_SHA256)"
  actual_runtime_hash="$(
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
  if [[
    -z "${expected_runtime_hash}" ||
    "${actual_runtime_hash}" != "${expected_runtime_hash}"
  ]]; then
    printf '错误：Zenoh/Pinocchio/ABI 运行时树校验失败。\n' >&2
    exit 1
  fi
)

for library in \
  "${BUNDLE_ROOT}/vendor/python/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so" \
  "${BUNDLE_ROOT}/vendor/lib/libPXREARobotSDK.so" \
  "${BUNDLE_ROOT}/vendor/python/marvin_sdk/libMarvinSDK.so" \
    "${ZENOH_LIBRARY_ROOT}/libzenohc.so" \
  "${BUNDLE_ROOT}/vendor/wuji-sdk/lib/libwuji_sdk_c.so"
do
  if ldd "${library}" | grep -q 'not found'; then
    printf '错误：动态库存在未满足依赖：%s\n' "${library}" >&2
    ldd "${library}" >&2
    exit 1
  fi
done

PICO_BODY_TIANJI_BUNDLE_ROOT="${BUNDLE_ROOT}" python - <<'PY'
import os
from pathlib import Path
import sys

import mujoco
import numpy
import scipy
import xrobotoolkit_sdk
import zenoh
from marvin_sdk.fx_robot import DCSS, Marvin_Robot
from pico_body_tianji.mujoco_urdf import portable_mujoco_urdf
from tianji_world_output.config_loader import TianjiConfig

# 显式传入随包配置路径，避开依赖 ament 索引的自动定位。
config = TianjiConfig.load(
    os.path.join(
        os.environ["PICO_BODY_TIANJI_BUNDLE_ROOT"],
        "vendor",
        "python",
        "tianji_world_output",
        "config",
        "tianji_robot.yaml",
    )
)
assert config.init_joints["left"].shape == (7,)
assert config.init_joints["right"].shape == (7,)
assert DCSS is not None
assert Marvin_Robot is not None
marvin = Marvin_Robot()
assert not marvin._connected
session = zenoh.open(zenoh.Config())
assert session is not None
try:
    session.close()
except zenoh.ZError as exc:
    # 已知环境问题：接口状态变化（如 enp129s0 down）时 close 超时；
    # open 成功已证明 zenoh 可用，close 超时降级为警告。
    print(f"zenoh close 警告（不影响可用性）: {exc}", file=sys.stderr)
assert Path(xrobotoolkit_sdk.__file__).is_file()
model_path = (
    Path(os.environ["PICO_BODY_TIANJI_BUNDLE_ROOT"])
    / "src"
    / "pico_body_tianji"
    / "assets"
    / "marvin_m6_ccs"
    / "urdf"
    / "marvin_m6_s_ccs_696_v4_mujoco.urdf"
)
xml, assets = portable_mujoco_urdf(model_path)
model = mujoco.MjModel.from_xml_string(xml, assets)
assert model.nq == 14
print("Python/厂商 SDK 导入检查通过")
print("zenoh 本地会话检查通过")
print("Marvin SDK", marvin.SDK_version(), "（仅加载，未连接）")
print(
    "numpy",
    numpy.__version__,
    "scipy",
    scipy.__version__,
    "mujoco",
    mujoco.__version__,
)
PY

printf '%s\n' \
  '环境和文件校验通过。Zenoh/Pinocchio/MuJoCo 已就绪；未连接设备。'
