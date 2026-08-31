#!/usr/bin/env python3
"""把 wuji hand2 beta1 左右手合并到 tianji 双臂 URDF，输出标准自包含资产。

用于给同事交付一份可直接加载的双臂+双手 URDF。输出目录包含：

  <out>/tianji_wuji2.urdf           完整模型（相对路径引用 meshes/）
  <out>/meshes/*.STL                全部引用的 STL mesh

右手沿用运行时链路（TCP_Link_R -> marker_* -> r_mount -> r_wrist），
左手为右手关于 XZ 平面的镜像：TCP_Link_L -> marker_*_L -> l_mount
-> l_wrist。

左右手 mesh 引用改写为相对路径 ``meshes/wuji2_<side>_*.STL``，
不依赖 ROS package，同事机器无 ROS 也可直接加载。

安装方向（右手实测，左手镜像）：
  right: TCP->marker rpy=(-pi/2,-pi/2,0), marker_wuji2->r_mount rpy=(0,-pi/2,0)
  left : TCP->marker rpy=( pi/2,-pi/2,0), marker_wuji2_L->l_mount rpy=(0,-pi/2,0)
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

REPO_ROOT = Path(__file__).resolve().parent.parent
ASSET_ROOT = REPO_ROOT / "src/tianji_teleop/assets"
TIANJI_URDF = (
    ASSET_ROOT
    / "marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf"
)
RIGHT_URDF = ASSET_ROOT / "wuji2_right/right_with_mount.urdf"
LEFT_URDF = ASSET_ROOT / "wuji2_left/left_with_mount.urdf"
MARKER_URDF = ASSET_ROOT / "tianji_arm_marker/marker_frames.urdf"
ASSET_MESH_DIR = ASSET_ROOT / "marvin_m6_ccs/meshes"

DEFAULT_OUT = Path.home() / "Documents/urdf/tianji_wuji2"


def _remove_joint(root: ET.Element, name: str) -> ET.Element:
    for joint in root.findall("joint"):
        if joint.get("name") == name:
            root.remove(joint)
            return joint
    raise ValueError(f"缺少 joint: {name}")


def _strip_collisions(root: ET.Element) -> None:
    """移除手部 collision（与 visual 重复的高面数 STL，交付只留 visual）。"""
    for link in root.findall("link"):
        for collision in link.findall("collision"):
            link.remove(collision)


def _validate_mount(root: ET.Element, side: str) -> None:
    """锁定 beta1 的 <side>_mount-><side>_wrist 固定安装变换。"""
    prefix = "l" if side == "left" else "r"
    mount = f"{prefix}_mount"
    wrist = f"{prefix}_wrist"
    joint_name = f"{prefix}_wrist_fixed"
    joint = next(
        (item for item in root.findall("joint")
         if item.get("name") == joint_name),
        None,
    )
    if joint is None:
        raise ValueError(f"缺少 joint: {joint_name}")
    parent = joint.find("parent")
    child = joint.find("child")
    origin = joint.find("origin")
    if parent is None or parent.get("link") != mount:
        raise ValueError(f"{joint_name} parent 必须为 {mount}")
    if child is None or child.get("link") != wrist:
        raise ValueError(f"{joint_name} child 必须为 {wrist}")
    if origin is None:
        raise ValueError(f"{joint_name} 缺少 origin")
    xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
    if side == "right":
        expected_xyz = [0.003, 0.00025016, -0.0285]
        expected_rpy = [0.0, 0.0, 1.6399e-05]
    else:
        expected_xyz = [0.00300004769907811, -0.000300109233680431, -0.0284998261308158]
        expected_rpy = [0.0, 0.0, 0.0]
    if any(abs(a - e) > 1e-6 for a, e in zip(xyz, expected_xyz)):
        raise ValueError(f"{joint_name} xyz 已变化: {xyz}")
    if any(abs(a - e) > 1e-6 for a, e in zip(rpy, expected_rpy)):
        raise ValueError(f"{joint_name} rpy 已变化: {rpy}")


def _reroot_marker_at_tianji(marker: ET.Element) -> None:
    """把 marker_tianji（机器人侧表面）设为 marker 子树根。"""
    original = _remove_joint(marker, "marker_tianji")
    origin = original.find("origin")
    if origin is None:
        raise ValueError("marker_tianji joint 缺少 origin")
    xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
    if any(abs(a - e) > 1e-9 for a, e in zip(xyz, [-0.004, 0.0, 0.0])):
        raise ValueError(f"marker_tianji xyz 已变化: {xyz}")
    if any(abs(v) > 1e-9 for v in rpy):
        raise ValueError(f"marker_tianji rpy 已变化: {rpy}")
    inverse = ET.Element(
        "joint", {"name": "marker_tianji_to_mocap", "type": "fixed"}
    )
    ET.SubElement(inverse, "origin", {"xyz": "0.004 0 0", "rpy": "0 0 0"})
    ET.SubElement(inverse, "parent", {"link": "marker_tianji"})
    ET.SubElement(inverse, "child", {"link": "marker_mocap"})
    marker.append(inverse)


def _clone_marker_tree(marker: ET.Element, side: str) -> ET.Element:
    """复制 marker 子树并加 <side> 后缀；返回右手树与左手树可共用的基元。

    由于 marker 树被 ``_reroot_marker_at_tianji`` 重根化到 marker_tianji，
    此处深拷贝整棵树，把 link/joint 名加上侧别后缀。
    """
    prefix = "l" if side == "left" else "r"
    root = ET.Element("robot", {"name": f"tianji_marker_{side}"})
    # link 名映射: marker_mocap -> marker_mocap_<p>, marker_wuji2 -> marker_wuji2_<p>,
    # marker_tianji -> marker_tianji_<p>
    name_map = {
        "marker_mocap": f"marker_mocap_{prefix}",
        "marker_wuji2": f"marker_wuji2_{prefix}",
        "marker_tianji": f"marker_tianji_{prefix}",
    }
    for link in marker.findall("link"):
        new_link = ET.SubElement(
            root, "link", {"name": name_map[link.get("name")]}
        )
        _copy_element(link, new_link)
    for joint in marker.findall("joint"):
        new_joint = ET.SubElement(
            root,
            "joint",
            {"name": _rename_joint(joint.get("name"), prefix),
             "type": joint.get("type", "fixed")},
        )
        # parent/child 映射
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is not None:
            ET.SubElement(
                new_joint, "parent",
                {"link": name_map.get(parent.get("link"), parent.get("link"))},
            )
        if child is not None:
            ET.SubElement(
                new_joint, "child",
                {"link": name_map.get(child.get("link"), child.get("link"))},
            )
        origin = joint.find("origin")
        if origin is not None:
            ET.SubElement(
                new_joint, "origin",
                {k: v for k, v in origin.attrib.items()},
            )
    return root


def _rename_joint(name: str, prefix: str) -> str:
    return f"{name}_{prefix}"


def _copy_element(src: ET.Element, dst: ET.Element) -> None:
    """深拷贝 src 的子节点到 dst（不复制属性）。"""
    for child in src:
        new = ET.SubElement(dst, child.tag, dict(child.attrib))
        _copy_element(child, new)


def _attach_side(tianji: ET.Element, marker_tree: ET.Element, hand: ET.Element,
                 side: str, mesh_map: dict[str, str]) -> None:
    """把单侧 marker 树 + 手部接到 tianji 的对应 TCP_Link_<P>。"""
    prefix = "l" if side == "left" else "r"
    tcp = f"TCP_Link_{prefix.upper()}"
    marker_tianji = f"marker_tianji_{prefix}"
    marker_mocap = f"marker_mocap_{prefix}"
    marker_wuji2 = f"marker_wuji2_{prefix}"
    mount = f"{prefix}_mount"
    wrist = f"{prefix}_wrist"

    if side == "right":
        tcp_to_marker_rpy = [-math.pi / 2.0, -math.pi / 2.0, 0.0]
    else:
        tcp_to_marker_rpy = [math.pi / 2.0, -math.pi / 2.0, 0.0]

    # TCP -> marker_tianji_<p>
    tcp_to_marker = ET.SubElement(
        tianji, "joint",
        {"name": f"JointMarker_{prefix.upper()}", "type": "fixed"},
    )
    ET.SubElement(
        tcp_to_marker, "origin",
        {"xyz": "0 0 0", "rpy": f"{tcp_to_marker_rpy[0]} {tcp_to_marker_rpy[1]} {tcp_to_marker_rpy[2]}"},
    )
    ET.SubElement(tcp_to_marker, "parent", {"link": tcp})
    ET.SubElement(tcp_to_marker, "child", {"link": marker_tianji})

    # marker_wuji2_<p> -> <p>_mount
    marker_to_hand = ET.SubElement(
        tianji, "joint",
        {"name": f"JointWuji2_{prefix.upper()}", "type": "fixed"},
    )
    ET.SubElement(
        marker_to_hand, "origin",
        {"xyz": "0 0 0", "rpy": f"0 {-math.pi / 2.0} 0"},
    )
    ET.SubElement(marker_to_hand, "parent", {"link": marker_wuji2})
    ET.SubElement(marker_to_hand, "child", {"link": mount})

    # 把手部 mesh（visual + collision）引用改写为输出目录相对路径并收集。
    # collision 与 visual 复用同一 STL；按 package:// 前缀提取文件名即可。
    for mesh in hand.findall(".//mesh"):
        filename = mesh.get("filename", "")
        prefix = "package://tianji_teleop/assets/marvin_m6_ccs/meshes/"
        if not filename.startswith(prefix):
            raise ValueError(f"手部 mesh 不是包内 URI: {filename}")
        asset_name = filename[len(prefix):]
        mesh.set("filename", f"meshes/{asset_name}")
        mesh_map[asset_name] = str(ASSET_MESH_DIR / asset_name)

    # 追加 marker 树与手部到 tianji
    for element in list(marker_tree):
        tianji.append(element)
    for element in list(hand):
        tianji.append(element)


def _strip_tcp_collision(root: ET.Element) -> None:
    """删除 TCP_Link_L/R 的 collision。

    tianji 原厂的 TCP_Link 是薄片状 STL（体积≈0），加载器做碰撞体积
    推导时会报 "mesh volume is too small"。TCP 是末端虚拟参考点，实际
    不需要碰撞体积；视觉 mesh 与坐标轴保留即可。
    """
    for link in root.findall("link"):
        if link.get("name") not in ("TCP_Link_L", "TCP_Link_R"):
            continue
        for collision in link.findall("collision"):
            link.remove(collision)


def _ensure_tcp_inertia(root: ET.Element) -> None:
    """给 TCP_Link_L/R 补一个非零集中惯性，避免加载器对零质量报错。

    tianji 原厂把 TCP 定为 mass=0 的虚拟坐标系。部分加载器（如 MuJoCo）
    对零质量 + 小体积 mesh 无法推导质量属性，会报 "mesh volume is too
    small"。这里给 TCP 一个 0.05kg 的集中质量（工具端参考质量），保持
    6 自由度惯性矩阵为零，不影响运动学，仅规避质量推导问题。
    """
    for link in root.findall("link"):
        if link.get("name") not in ("TCP_Link_L", "TCP_Link_R"):
            continue
        old_inertial = link.find("inertial")
        if old_inertial is not None:
            link.remove(old_inertial)
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(inertial, "mass", {"value": "0.05"})
        ET.SubElement(inertial, "inertia", {
            "ixx": "0", "ixy": "0", "ixz": "0",
            "iyy": "0", "iyz": "0", "izz": "0",
        })


def _add_axis_visuals(root: ET.Element) -> None:
    """给 TCP/wrist/mount 添加可视化坐标轴（仅 visual）。"""

    def add(link_name: str, length: float, radius: float, colors: tuple[str, ...]) -> None:
        link = next(
            (item for item in root.findall("link")
             if item.get("name") == link_name),
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
                link, "visual", {"name": f"{link_name}_axis_{index}"}
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

    # 双 TCP：细短 RGB。
    for tcp in ("TCP_Link_L", "TCP_Link_R"):
        add(tcp, 0.05, 0.003, ("1 0 0 1", "0 1 0 1", "0 0 1 1"))
    # 双 marker_mocap：中等 RGB。
    for mocap in ("marker_mocap_l", "marker_mocap_r"):
        add(mocap, 0.07, 0.004, ("1 0 0 1", "0 1 0 1", "0 0 1 1"))
    # 双 mount：cyan/洋红/黄短轴。
    for mount in ("l_mount", "r_mount"):
        add(mount, 0.06, 0.004, ("0 1 1 1", "1 0 1 1", "1 1 0 1"))
    # 双 wrist：粗长 RGB。
    for wrist in ("l_wrist", "r_wrist"):
        add(wrist, 0.09, 0.005, ("1 0 0 1", "0 1 0 1", "0 0 1 1"))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="把 wuji2 左右手合并到 tianji 双臂，输出自包含标准 URDF 资产"
    )
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT,
                        help=f"输出目录（默认 {DEFAULT_OUT}）")
    args = parser.parse_args()
    out: Path = args.out

    tianji = ET.parse(TIANJI_URDF).getroot()
    marker_src = ET.parse(MARKER_URDF).getroot()
    right_hand = ET.parse(RIGHT_URDF).getroot()
    left_hand = ET.parse(LEFT_URDF).getroot()

    _reroot_marker_at_tianji(marker_src)
    _validate_mount(right_hand, "right")
    _validate_mount(left_hand, "left")
    # 保留 wuji2 手部 collision 体：同事需要手指碰撞/抓取接触时可用。
    # 手部 collision 与 visual 复用同一 STL，mesh 文件不重复、体积不翻倍。

    marker_right = _clone_marker_tree(marker_src, "right")
    marker_left = _clone_marker_tree(marker_src, "left")

    mesh_map: dict[str, str] = {}
    _attach_side(tianji, marker_right, right_hand, "right", mesh_map)
    _attach_side(tianji, marker_left, left_hand, "left", mesh_map)
    _add_axis_visuals(tianji)
    _strip_tcp_collision(tianji)
    _ensure_tcp_inertia(tianji)

    # 改写 tianji 双臂自身的 package:// mesh 引用为相对路径并收集。
    # 此时 tianji 树仍含 <video>/<mesh> 引用，部分 url 为 package://tianji_teleop/...
    for mesh in tianji.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if not filename.startswith("package://tianji_teleop/assets/"
                                   "marvin_m6_ccs/meshes/"):
            continue
        asset_name = filename.removeprefix(
            "package://tianji_teleop/assets/marvin_m6_ccs/meshes/"
        )
        mesh.set("filename", f"meshes/{asset_name}")
        mesh_map[asset_name] = str(ASSET_MESH_DIR / asset_name)

    # 写出 URDF
    out.mkdir(parents=True, exist_ok=True)
    urdf_path = out / "tianji_wuji2.urdf"
    ET.indent(tianji)
    urdf_path.write_text(ET.tostring(tianji, encoding="unicode"), encoding="utf-8")

    # 复制 mesh
    meshes_dir = out / "meshes"
    meshes_dir.mkdir(parents=True, exist_ok=True)
    copied = 0
    missing = []
    for asset_name, src_path in sorted(mesh_map.items()):
        src = Path(src_path)
        if not src.is_file():
            missing.append(str(src))
            continue
        dst = meshes_dir / asset_name
        shutil.copy2(src, dst)
        copied += 1

    if missing:
        print("缺失源 mesh:", missing)
        return 1

    print(f"标准 URDF 已写出: {urdf_path}")
    print(f"复制 mesh: {copied} 个 -> {meshes_dir}")
    print("安装链:")
    print("  右: TCP_Link_R -> marker_tianji_r -> marker_mocap_r"
          " -> marker_wuji2_r -> r_mount -> r_wrist")
    print("  左: TCP_Link_L -> marker_tianji_l -> marker_mocap_l"
          " -> marker_wuji2_l -> l_mount -> l_wrist")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
