#!/usr/bin/env python3
"""合并 tianji 双臂、right_arm marker 刚体与 wuji2 v6.12 右手。

marker 来源 ``assets/tianji_arm_marker/marker_frames.urdf``。SolidWorks
导出的三套 frame 属于同一块 8mm 刚体：``marker_tianji`` 在机器人侧
表面，``marker_mocap`` 在中心，``marker_wuji2`` 在手侧表面。脚本只
渲染中心 frame 的单份 mesh，并把 marker 树重根化到机器人侧。

新手以 ``r_mount_frame`` 为法兰安装基准。脚本反向原
``r_base_to_mount``，得到 ``r_mount_frame -> r_base -> wrist``。

最终链：
  TCP_Link_R -> marker_tianji -> marker_mocap -> marker_wuji2
             -> r_mount_frame -> r_base(wrist) -> fingers

方向由实测约定：
  marker +x->TCP +z，+y->TCP +x，+z->TCP +y；
  mount +x->TCP +y，+y->TCP +x，+z->TCP -z。
因此 TCP→marker rpy=(-pi/2,-pi/2,0)，marker→mount rpy=(0,-pi/2,0)。
各相邻安装面原点重合；marker 中心距两侧各 4mm，以 marker URDF
为几何真值。
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
MARKER_URDF = ASSET_ROOT / "tianji_arm_marker/marker_frames.urdf"
MESH_PREFIX = "package://pico_body_tianji/assets/marvin_m6_ccs/meshes/"


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
    expected_rpy = [0.0, 0.0, 0.0]
    if any(abs(actual - expected) > 1e-8 for actual, expected in zip(xyz, expected_xyz)):
        raise ValueError(f"r_base_to_mount xyz 已变化: {xyz}")
    if any(abs(actual - expected) > 1e-8 for actual, expected in zip(rpy, expected_rpy)):
        raise ValueError(f"r_base_to_mount rpy 已变化: {rpy}")

    # T_mount_base = inverse(T_base_mount)。新包 rpy=0(恒等)，
    # 反向平移 = -t = [0.003, -0.00025, 0.0285]。
    inverse = ET.Element(
        "joint",
        {"name": "r_mount_frame_to_base", "type": "fixed"},
    )
    ET.SubElement(
        inverse,
        "origin",
        {"xyz": "0.003 -0.00025 0.0285", "rpy": "0 0 0"},
    )
    ET.SubElement(inverse, "parent", {"link": "r_mount_frame"})
    ET.SubElement(inverse, "child", {"link": "r_base"})
    wuji2.append(inverse)


def _validate_visual_meshes(root: ET.Element, label: str) -> None:
    """确认源码 visual STL mesh 已归档到包内 assets（collision 会被删除）。"""
    mesh_directory = ASSET_ROOT / "marvin_m6_ccs/meshes"
    for mesh in root.findall(".//visual//mesh"):
        filename = mesh.get("filename", "")
        if not filename.endswith(".STL"):
            raise ValueError(f"{label} visual mesh 必须是 STL: {filename}")
        if not filename.startswith(MESH_PREFIX):
            raise ValueError(f"{label} mesh 不是包内 URI: {filename}")
        asset_name = filename.removeprefix(MESH_PREFIX)
        if Path(asset_name).name != asset_name:
            raise ValueError(f"{label} mesh 名无效: {filename}")
        if not (mesh_directory / asset_name).is_file():
            raise FileNotFoundError(mesh_directory / asset_name)


def _reroot_marker_at_tianji(marker: ET.Element) -> None:
    """把 marker_tianji（机器人侧表面）设为 marker 子树根。"""
    original = _remove_joint(marker, "marker_tianji")
    origin = original.find("origin")
    if origin is None:
        raise ValueError("marker_tianji joint 缺少 origin")
    xyz = [float(value) for value in origin.get("xyz", "0 0 0").split()]
    rpy = [float(value) for value in origin.get("rpy", "0 0 0").split()]
    if any(abs(actual - expected) > 1e-9 for actual, expected in zip(xyz, [-0.004, 0.0, 0.0])):
        raise ValueError(f"marker_tianji xyz 已变化: {xyz}")
    if any(abs(value) > 1e-9 for value in rpy):
        raise ValueError(f"marker_tianji rpy 已变化: {rpy}")
    inverse = ET.Element(
        "joint",
        {"name": "marker_tianji_to_mocap", "type": "fixed"},
    )
    ET.SubElement(inverse, "origin", {"xyz": "0.004 0 0", "rpy": "0 0 0"})
    ET.SubElement(inverse, "parent", {"link": "marker_tianji"})
    ET.SubElement(inverse, "child", {"link": "marker_mocap"})
    marker.append(inverse)


def _attach_marker_and_hand(tianji: ET.Element) -> None:
    """连接 TCP→marker 机器人侧面，以及 marker 手侧面→r_mount_frame。"""
    tcp_to_marker = ET.SubElement(
        tianji,
        "joint",
        {"name": "JointMarker_R", "type": "fixed"},
    )
    # marker +x→TCP +z, +y→TCP +x, +z→TCP +y。
    ET.SubElement(
        tcp_to_marker,
        "origin",
        {"xyz": "0 0 0", "rpy": f"{-math.pi / 2.0} {-math.pi / 2.0} 0"},
    )
    ET.SubElement(tcp_to_marker, "parent", {"link": "TCP_Link_R"})
    ET.SubElement(tcp_to_marker, "child", {"link": "marker_tianji"})

    marker_to_hand = ET.SubElement(
        tianji,
        "joint",
        {"name": "JointWuji2_R", "type": "fixed"},
    )
    # marker_wuji2→hand mount：Rx(180°) 后 Ry(-90°)，两侧表面原点重合。
    # 轴关系：marker +x→mount +z、+y→mount -y、+z→mount +x。
    ET.SubElement(
        marker_to_hand,
        "origin",
        {"xyz": "0 0 0", "rpy": f"{math.pi} {-math.pi / 2.0} 0"},
    )
    ET.SubElement(marker_to_hand, "parent", {"link": "marker_wuji2"})
    ET.SubElement(marker_to_hand, "child", {"link": "r_mount_frame"})



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
    # marker_mocap：中等长度 RGB，原点位于橙色刚体中心。
    add(
        "marker_mocap",
        0.07,
        0.004,
        ("1 0 0 1", "0 1 0 1", "0 0 1 1"),
    )
    # wuji2 r_wrist：粗长黄(+x)/品红(+y)/青(+z)，RL/H5 回放端点。
    add(
        "r_wrist",
        0.09,
        0.005,
        ("1 1 0 1", "1 0 1 1", "0 1 1 1"),
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    tianji = ET.parse(TIANJI_URDF).getroot()
    marker = ET.parse(MARKER_URDF).getroot()
    wuji2 = ET.parse(WUJI2_URDF).getroot()
    _reroot_marker_at_tianji(marker)
    _reroot_at_mount_frame(wuji2)
    # 新手 URDF 自带 <mujoco> 编译器段；组合模型只保留 tianji 的，
    # 否则 MuJoCo 报 repeated element 'mujoco'。
    for mujoco_node in wuji2.findall("mujoco"):
        wuji2.remove(mujoco_node)
    # 新手 collision 引用外部 .obj 凸包；组合模型只保留 visual。
    for link in wuji2.findall("link"):
        for collision in link.findall("collision"):
            link.remove(collision)
    _validate_visual_meshes(marker, "marker")
    _validate_visual_meshes(wuji2, "wuji2")
    _attach_marker_and_hand(tianji)

    for element in list(marker):
        tianji.append(element)
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
    print("安装链: TCP_Link_R -> marker_tianji -> marker_mocap "
          "-> marker_wuji2 -> r_mount_frame -> r_base")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
