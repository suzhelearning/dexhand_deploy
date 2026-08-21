from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import h5py
import mujoco
import numpy as np

from .controller_only.mocap_h5 import reject_external_links


_HAND_ROOT_JOINT = "regrind_hand_root"
_OBJECT_ROOT_JOINT = "regrind_object_root"
_EXPECTED_FRAME = (
    "table centre origin, table top z=0, +x across width, "
    "+y along length, +z up"
)
_REQUIRED_DATASETS = (
    "regrind_retargeting_joints",
    "regrind_retargeting_root_pos",
    "regrind_retargeting_root_quat",
    "object_pos",
    "object_quat",
)


@dataclass(frozen=True)
class RegrindRecording:
    """Regrind 重定向后的 wuji2 手、自由根和物体逐帧轨迹。"""

    path: Path
    fps: float
    joint_names: tuple[str, ...]
    joints: np.ndarray
    root_position: np.ndarray
    root_quaternion_wxyz: np.ndarray
    object_position: np.ndarray
    object_quaternion_wxyz: np.ndarray
    dropped_leading_frames: int

    @property
    def frame_count(self) -> int:
        return int(self.joints.shape[0])

    @property
    def duration_s(self) -> float:
        return self.frame_count / self.fps

    def summary(self) -> dict[str, object]:
        return {
            "path": str(self.path),
            "frames": self.frame_count,
            "fps": self.fps,
            "duration_s": self.duration_s,
            "joint_count": len(self.joint_names),
            "root_link": "r_base",
            "quaternion_convention": "wxyz",
            "dataset": "regrind_retargeting_*",
            "dropped_leading_frames": self.dropped_leading_frames,
        }


@dataclass(frozen=True)
class RegrindQposLayout:
    hand_root_address: int
    object_root_address: int
    joint_addresses: tuple[int, ...]


def _text_attr(value: object) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return str(value)


def _finite_array(
    group: h5py.File,
    name: str,
    shape: tuple[int, ...],
) -> np.ndarray:
    values = np.asarray(group[name][:], dtype=np.float64)
    if values.shape != shape:
        raise ValueError(f"{name} 形状必须为 {shape}，实际 {values.shape}")
    if not np.isfinite(values).all():
        raise ValueError(f"{name} 包含 NaN/Inf")
    return values


def _validate_wxyz_quaternions(values: np.ndarray, name: str) -> None:
    norms = np.linalg.norm(values, axis=1)
    if not np.allclose(norms, 1.0, atol=1.0e-5, rtol=0.0):
        worst = float(np.max(np.abs(norms - 1.0)))
        raise ValueError(f"{name} 不是单位 WXYZ 四元数，最大模长误差={worst:.3g}")


def load_regrind_h5(path: str | Path) -> RegrindRecording:
    """严格加载 README 声明的 Regrind 50Hz 自由根手/物体轨迹。"""

    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(source)

    with h5py.File(source, "r") as h5:
        reject_external_links(h5)
        missing = [name for name in _REQUIRED_DATASETS if name not in h5]
        if missing:
            raise ValueError(
                "不是 Regrind wuji2 轨迹，缺少数据集：" + ", ".join(missing)
            )

        try:
            frame_count = int(h5.attrs["frames"])
            fps = float(h5.attrs["fps"])
            dropped = int(h5.attrs["dropped_leading_frames"])
        except KeyError as exc:
            raise ValueError(f"Regrind HDF5 缺少根属性：{exc.args[0]}") from exc
        if frame_count <= 0:
            raise ValueError("frames 必须为正整数")
        if not np.isfinite(fps) or fps <= 0.0:
            raise ValueError("fps 必须为正有限数值")
        if dropped < 1:
            raise ValueError(
                "dropped_leading_frames 必须 >= 1；禁止回放 Regrind 冷启动帧"
            )
        if _text_attr(h5.attrs.get("quat_convention", "")) != "wxyz":
            raise ValueError("Regrind 四元数约定必须为 WXYZ")
        if _text_attr(h5.attrs.get("root_link", "")) != "r_base":
            raise ValueError("Regrind root_link 必须为 r_base")
        if _text_attr(h5.attrs.get("frame", "")) != _EXPECTED_FRAME:
            raise ValueError("Regrind 世界系必须是 README 声明的桌面中心 z-up 坐标系")

        joint_names = tuple(
            item.strip()
            for item in _text_attr(h5.attrs.get("joint_order", "")).split(",")
            if item.strip()
        )
        if len(joint_names) != 20 or len(set(joint_names)) != 20:
            raise ValueError("joint_order 必须包含 20 个不重复的 wuji2 关节")

        joints = _finite_array(
            h5,
            "regrind_retargeting_joints",
            (frame_count, len(joint_names)),
        )
        root_position = _finite_array(
            h5, "regrind_retargeting_root_pos", (frame_count, 3)
        )
        root_quaternion = _finite_array(
            h5, "regrind_retargeting_root_quat", (frame_count, 4)
        )
        object_position = _finite_array(h5, "object_pos", (frame_count, 3))
        object_quaternion = _finite_array(h5, "object_quat", (frame_count, 4))
        _validate_wxyz_quaternions(
            root_quaternion, "regrind_retargeting_root_quat"
        )
        _validate_wxyz_quaternions(object_quaternion, "object_quat")

    return RegrindRecording(
        path=source,
        fps=fps,
        joint_names=joint_names,
        joints=joints,
        root_position=root_position,
        root_quaternion_wxyz=root_quaternion,
        object_position=object_position,
        object_quaternion_wxyz=object_quaternion,
        dropped_leading_frames=dropped,
    )


def _add_table_visual(world: ET.Element) -> None:
    visual = ET.SubElement(world, "visual", {"name": "table_top"})
    ET.SubElement(visual, "origin", {"xyz": "0 0 -0.01", "rpy": "0 0 0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box", {"size": "0.9 1.14 0.02"})
    material = ET.SubElement(visual, "material", {"name": "table"})
    ET.SubElement(material, "color", {"rgba": "0.28 0.31 0.34 1"})


def _rewrite_mesh_assets(
    root: ET.Element,
    base_directory: Path,
    prefix: str,
    assets: dict[str, bytes],
) -> None:
    base = base_directory.resolve()
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename", "")
        relative = Path(filename)
        if not filename or relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"非法 {prefix} mesh 路径：{filename!r}")
        source = (base / relative).resolve()
        if not source.is_relative_to(base) or not source.is_file():
            raise FileNotFoundError(source)
        asset_name = f"{prefix}_{'_'.join(relative.parts)}"
        existing = assets.get(asset_name)
        content = source.read_bytes()
        if existing is not None and existing != content:
            raise ValueError(f"mesh asset 名冲突：{asset_name}")
        assets[asset_name] = content
        mesh.set("filename", asset_name)


def _free_joint(name: str, parent: str, child: str) -> ET.Element:
    joint = ET.Element("joint", {"name": name, "type": "floating"})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    return joint


def _free_qpos_address(model: mujoco.MjModel, name: str) -> int:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise RuntimeError(f"MuJoCo 模型缺少自由关节：{name}")
    if int(model.jnt_type[joint_id]) != int(mujoco.mjtJoint.mjJNT_FREE):
        raise RuntimeError(f"MuJoCo 关节 {name} 不是 free joint")
    return int(model.jnt_qposadr[joint_id])


def build_regrind_mujoco_model(
    recording: RegrindRecording,
) -> tuple[mujoco.MjModel, RegrindQposLayout]:
    """从 H5 同目录 README 资产构建自由根 wuji2 + 锤子 + 桌面模型。"""

    package = recording.path.parent
    hand_path = package / "right.urdf"
    object_directory = package / "object"
    object_path = object_directory / "hammer.urdf"
    if not hand_path.is_file():
        raise FileNotFoundError(hand_path)
    if not object_path.is_file():
        raise FileNotFoundError(object_path)

    hand = ET.parse(hand_path).getroot()
    hand_joint_names = tuple(
        joint.get("name", "")
        for joint in hand.findall("joint")
        if joint.get("type") in ("revolute", "continuous")
    )
    if hand_joint_names != recording.joint_names:
        raise ValueError(
            "H5 joint_order 与同目录 right.urdf revolute joint 顺序不一致"
        )

    compiler = hand.find("./mujoco/compiler")
    if compiler is None:
        raise ValueError("right.urdf 缺少 mujoco/compiler")
    compiler.set("meshdir", "")
    compiler.set("strippath", "false")
    compiler.set("discardvisual", "false")

    assets: dict[str, bytes] = {}
    _rewrite_mesh_assets(hand, package, "hand", assets)

    hammer = ET.parse(object_path).getroot()
    for mujoco_node in hammer.findall("mujoco"):
        hammer.remove(mujoco_node)
    _rewrite_mesh_assets(hammer, object_directory, "object", assets)

    world = ET.Element("link", {"name": "world"})
    _add_table_visual(world)
    hand.insert(0, world)
    hand.append(_free_joint(_HAND_ROOT_JOINT, "world", "r_base"))
    for element in list(hammer):
        hand.append(element)
    hand.append(_free_joint(_OBJECT_ROOT_JOINT, "world", "bottom"))
    hand.set("name", "regrind_wuji2_hammer_replay")

    model = mujoco.MjModel.from_xml_string(
        ET.tostring(hand, encoding="unicode"), assets
    )
    hand_root_address = _free_qpos_address(model, _HAND_ROOT_JOINT)
    object_root_address = _free_qpos_address(model, _OBJECT_ROOT_JOINT)

    joint_addresses: list[int] = []
    for column, name in enumerate(recording.joint_names):
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, name
        )
        if joint_id < 0:
            raise RuntimeError(f"MuJoCo 模型缺少 H5 关节：{name}")
        if int(model.jnt_type[joint_id]) != int(mujoco.mjtJoint.mjJNT_HINGE):
            raise RuntimeError(f"H5 关节不是 hinge：{name}")
        if bool(model.jnt_limited[joint_id]):
            lower, upper = model.jnt_range[joint_id]
            minimum = float(np.min(recording.joints[:, column]))
            maximum = float(np.max(recording.joints[:, column]))
            if minimum < lower - 1.0e-8 or maximum > upper + 1.0e-8:
                raise ValueError(
                    f"{name} 超出 URDF 限位：轨迹 [{minimum}, {maximum}]，"
                    f"限位 [{lower}, {upper}]"
                )
        joint_addresses.append(int(model.jnt_qposadr[joint_id]))

    return model, RegrindQposLayout(
        hand_root_address=hand_root_address,
        object_root_address=object_root_address,
        joint_addresses=tuple(joint_addresses),
    )


def apply_regrind_frame(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    recording: RegrindRecording,
    layout: RegrindQposLayout,
    frame_index: int,
) -> None:
    """把一帧自由根、20 个关节和锤子位姿直接写入 MuJoCo qpos。"""

    index = int(frame_index)
    if not 0 <= index < recording.frame_count:
        raise IndexError(
            f"frame_index={index} 超出 [0, {recording.frame_count})"
        )

    hand = layout.hand_root_address
    data.qpos[hand : hand + 3] = recording.root_position[index]
    data.qpos[hand + 3 : hand + 7] = recording.root_quaternion_wxyz[index]
    obj = layout.object_root_address
    data.qpos[obj : obj + 3] = recording.object_position[index]
    data.qpos[obj + 3 : obj + 7] = recording.object_quaternion_wxyz[index]
    for column, address in enumerate(layout.joint_addresses):
        data.qpos[address] = recording.joints[index, column]
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
