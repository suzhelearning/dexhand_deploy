#!/usr/bin/env python3
"""把 wuji2 v6.12 右手以 r_mount_frame 为根挂到 tianji TCP_Link_R。

新手来源：
  /home/current/Downloads/wujihand2_v6.12_with_mount_marker/
  right_with_mount_marker.urdf

新手原树：
  r_base -> r_wrist
  r_base -> r_mount_frame（joint: r_base_to_mount）

``r_mount_frame`` 是法兰安装基准。脚本将该 fixed joint 反向，得到：
  r_mount_frame -> r_base -> r_wrist -> fingers
再用 JointWuji2_R 直接连接：
  TCP_Link_R -> r_mount_frame

两个安装原点严格重合（xyz=0）。方向关系由用户给定：
  TCP +x -> r_mount_frame +y
  TCP +y -> r_mount_frame +x
  TCP +z -> r_mount_frame -z
对应 R_tcp_rmount = [[0,1,0],[1,0,0],[0,0,-1]]，
URDF rpy(roll,pitch,yaw) = (pi, 0, pi/2)。
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

REPO_ROOT = Path(__file__).resolve().parent.parent
ASSET_ROOT = REPO_ROOT / "src/pico_body_tianji/assets"
TIANJI_URDF = (
    ASSET_ROOT
    / "marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4_mujoco.urdf"
)
WUJI2_URDF = ASSET_ROOT / "wuji2_right/right_with_mount_marker.urdf"
MESH_PREFIX = "package://pico_body_tianji/assets/marvin_m6_ccs/meshes/"
MOUNT_RPY = (math.pi, 0.0, math.pi / 2.0)
MOUNT_XYZ = (0.0, 0.0, 0.0)


def _remove_joint(root: ET.Element, name: str) -> ET.Element:
    for joint in root.findall("joint"):
        if joint.get("name") == name:
            root.remove(joint)
            return joint
    raise ValueError(f"缺少 joint: {name}")


def _reroot_at_mount_frame(wuji2: ET.Element) -> None:
    """反向 r_base_to_mount，使 r_mount_frame 成为新手 URDF 根。"""
    original = _remove_joint(wuji2, "r_base_to_mount")
    origin = original.find("origin")
    if origin is None:
        raise ValueError("r_base_to_mount 缺少 origin")
    xyz = [float(value) for value in origin.get("xyz", "0 0 0").split()]
    rpy = [float(value) for value in origin.get("rpy", "0 0 0").split()]
    expected_xyz = [-0.003, 0.00025, -0.0285]
    expected_rpy = [math.pi, 0.0, 0.0]
    if any(abs(actual - expected) > 1e-8 for actual, expected in zip(xyz, expected_xyz)):
        raise ValueError(f"r_base_to_mount xyz 已变化: {xyz}")
    if any(abs(actual - expected) > 1e-8 for actual, expected in zip(rpy, expected_rpy)):
        raise ValueError(f"r_base_to_mount rpy 已变化: {rpy}")

    # T_mount_base = inverse(T_base_mount)。Rx(pi) 自逆；
    # t_inv = -R^T t = [0.003, 0.00025, -0.0285]。
    inverse = ET.Element(
        "joint",
        {"name": "r_mount_frame_to_base", "type": "fixed"},
    )
    ET.SubElement(
        inverse,
        "origin",
        {"xyz": "0.003 0.00025 -0.0285", "rpy": f"{math.pi} 0 0"},
    )
    ET.SubElement(inverse, "parent", {"link": "r_mount_frame"})
    ET.SubElement(inverse, "child", {"link": "r_base"})
    wuji2.append(inverse)


def _prepare_visual_meshes(wuji2: ET.Element) -> None:
    """确认源码手部 mesh 已归档；源 URDF 已去除 collision。"""
    mesh_directory = ASSET_ROOT / "marvin_m6_ccs/meshes"
    for mesh in wuji2.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if not filename.startswith(MESH_PREFIX):
            raise ValueError(f"wuji2 mesh 不是包内 URI: {filename}")
        asset_name = filename.removeprefix(MESH_PREFIX)
        if Path(asset_name).name != asset_name:
            raise ValueError(f"wuji2 mesh 名无效: {filename}")
        if not (mesh_directory / asset_name).is_file():
            raise FileNotFoundError(mesh_directory / asset_name)


def _attach_mount_frame(tianji: ET.Element) -> None:
    mount = ET.SubElement(
        tianji,
        "joint",
        {"name": "JointWuji2_R", "type": "fixed"},
    )
    ET.SubElement(
        mount,
        "origin",
        {
            "xyz": " ".join(f"{value:.12g}" for value in MOUNT_XYZ),
            "rpy": " ".join(f"{value:.12g}" for value in MOUNT_RPY),
        },
    )
    ET.SubElement(mount, "parent", {"link": "TCP_Link_R"})
    ET.SubElement(mount, "child", {"link": "r_mount_frame"})



def _add_axis_visuals(root: ET.Element) -> None:
    """给 TCP 与安装基准添加可视化坐标轴（仅 visual，不影响动力学）。"""

    def add(link_name: str, length: float, radius: float, colors: tuple[str, ...]) -> None:
        link = next(
            (item for item in root.findall("link") if item.get("name") == link_name),
            None,
        )
        if link is None:
            raise ValueError(f"缺少坐标轴父 link: {link_name}")
        axes = (
            (f"{length / 2.0} 0 0", f"0 {math.pi / 2.0} 0", colors[0]),
            (f"0 {length / 2.0} 0", f"{math.pi / 2.0} 0 0", colors[1]),
            (f"0 0 {length / 2.0}", "0 0 0", colors[2]),
        )
        for index, (xyz, rpy, rgba) in enumerate(axes):
            visual = ET.SubElement(
                link,
                "visual",
                {"name": f"{link_name}_axis_{index}"},
            )
            ET.SubElement(visual, "origin", {"xyz": xyz, "rpy": rpy})
            geometry = ET.SubElement(visual, "geometry")
            ET.SubElement(
                geometry,
                "cylinder",
                {"length": str(length), "radius": str(radius)},
            )
            material = ET.SubElement(visual, "material")
            ET.SubElement(material, "color", {"rgba": rgba})

    # TCP：细短红(+x)/绿(+y)/蓝(+z)。
    add(
        "TCP_Link_R",
        0.05,
        0.003,
        ("1 0 0 1", "0 1 0 1", "0 0 1 1"),
    )
    # 新手 mount：粗长黄(+x)/品红(+y)/青(+z)，便于与 TCP 区分。
    add(
        "r_mount_frame",
        0.09,
        0.005,
        ("1 1 0 1", "1 0 1 1", "0 1 1 1"),
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    tianji = ET.parse(TIANJI_URDF).getroot()
    wuji2 = ET.parse(WUJI2_URDF).getroot()
    _reroot_at_mount_frame(wuji2)
    _prepare_visual_meshes(wuji2)
    _attach_mount_frame(tianji)

    for element in list(wuji2):
        tianji.append(element)
    _add_axis_visuals(tianji)

    output = args.out or (
        TIANJI_URDF.parent / "marvin_m6_s_ccs_696_v4_wuji2.urdf"
    )
    ET.indent(tianji)
    output.write_text(
        ET.tostring(tianji, encoding="unicode"),
        encoding="utf-8",
    )
    print(f"组合 URDF 已写出: {output}")
    print("安装: TCP_Link_R -> r_mount_frame, xyz=(0,0,0), "
          "rpy=(pi,0,pi/2)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
