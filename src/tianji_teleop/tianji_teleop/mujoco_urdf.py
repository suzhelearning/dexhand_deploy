from __future__ import annotations

from pathlib import Path
import struct
import xml.etree.ElementTree as ET


PACKAGE_MESH_PREFIX = (
    "package://tianji_teleop/assets/marvin_m6_ccs/meshes/"
)

# MuJoCo URDF 编译器对体积过小的 mesh 报 "mesh volume is too small"，
# 且无法通过 inertia 配置绕过。厂商标示类薄壳（如 TCP 轴标记 STL）
# 在旧合并资产中同样被替换为球；这里按 STL 体积探测后统一替换。
_THIN_MESH_VOLUME_LIMIT = 1.0e-8  # m³
_THIN_MESH_SPHERE_RADIUS = 0.025  # 与旧合并文件的 TCP sphere 视觉一致


def _stl_volume(data: bytes) -> float | None:
    """Binary STL 的三棱柱有符号体积；ASCII 或损坏文件返回 None。"""
    if data[:5] == b"solid" and b"facet" in data[:256]:
        return None
    if len(data) < 84:
        return None
    count = struct.unpack_from("<I", data, 80)[0]
    count = min(count, (len(data) - 84) // 50)
    if count <= 0:
        return None
    volume = 0.0
    for index in range(count):
        offset = 84 + index * 50
        values = struct.unpack_from("<12f", data, offset)
        x1, y1, z1 = values[0], values[1], values[2]
        x2, y2, z2 = values[3], values[4], values[5]
        x3, y3, z3 = values[6], values[7], values[8]
        volume += (
            x1 * (y2 * z3 - z2 * y3)
            - y1 * (x2 * z3 - z2 * x3)
            + z1 * (x2 * y3 - y2 * x3)
        ) / 6.0
    return abs(volume)


def _replace_thin_mesh_with_sphere(geometry: ET.Element) -> None:
    """把 geometry 内的 mesh 引用替换成小球（先删除旧子元素）。"""
    for child in list(geometry):
        geometry.remove(child)
    ET.SubElement(geometry, "sphere", {"radius": str(_THIN_MESH_SPHERE_RADIUS)})


def portable_mujoco_urdf(
    urdf_path: Path,
) -> tuple[str, dict[str, bytes]]:
    """Return relocatable URDF XML and in-memory MuJoCo mesh assets.

    支持两种 mesh 引用：
    - `package://tianji_teleop/assets/marvin_m6_ccs/meshes/<name>.STL`
      （旧组合资产，meshes 位于 urdf 上一级目录的兄弟目录）；
    - `meshes/<name>.STL`（与 urdf 同目录的 meshes 子目录，如
      assets/tianji_wuji2/tianji_wuji2.urdf 的原始引用）。

    纯 URDF（无 MuJoCo 扩展）也会自动注入最小 compiler 节点；
    visual 几何默认保留（discardvisual=false）；薄壳 STL（体积小于
    _THIN_MESH_VOLUME_LIMIT）替换为球体以避免 MuJoCo 编译失败。
    """
    source = Path(urdf_path).resolve()
    root = ET.parse(source).getroot()
    compiler = root.find("./mujoco/compiler")
    if compiler is None:
        # 纯 URDF（无 MuJoCo 扩展）也可加载：注入最小 compiler 节点。
        mujoco_node = root.find("./mujoco")
        if mujoco_node is None:
            mujoco_node = ET.Element("mujoco")
            root.insert(0, mujoco_node)
        compiler = ET.SubElement(mujoco_node, "compiler")
        compiler.set("balanceinertia", "true")
    compiler.set("meshdir", "")
    # MuJoCo 默认丢弃 URDF visual（仅留 collision）；本项目保持视觉。
    compiler.set("discardvisual", "false")

    package_mesh_directory = source.parent.parent / "meshes"
    sibling_mesh_directory = source.parent / "meshes"
    assets: dict[str, bytes] = {}
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith(PACKAGE_MESH_PREFIX):
            asset_name = filename.removeprefix(PACKAGE_MESH_PREFIX)
        elif filename.startswith("meshes/"):
            asset_name = filename[len("meshes/"):]
        else:
            continue
        if not asset_name or Path(asset_name).name != asset_name:
            raise ValueError(f"invalid mesh URI: {filename}")
        mesh_directory = (
            package_mesh_directory
            if filename.startswith(PACKAGE_MESH_PREFIX)
            else sibling_mesh_directory
        )
        asset_path = mesh_directory / asset_name
        data = asset_path.read_bytes()
        volume = _stl_volume(data)
        if volume is not None and volume < _THIN_MESH_VOLUME_LIMIT:
            # 薄壳 mesh：替换引用它的 geometry（visual/collision）为球，
            # 不加入 assets（MuJoCo 无法编译该 mesh）。
            for geometry in _referencing_geometries(root, mesh):
                _replace_thin_mesh_with_sphere(geometry)
            continue
        assets[asset_name] = data
        mesh.set("filename", asset_name)

    if not assets:
        raise ValueError("MuJoCo URDF contains no mesh assets")
    xml = ET.tostring(root, encoding="unicode")
    return xml, assets


def _referencing_geometries(root: ET.Element, mesh: ET.Element) -> list[ET.Element]:
    """返回引用了该 mesh 元素的 geometry（同一元素树内的父节点查找）。"""
    parent_of: dict[ET.Element, ET.Element] = {}
    for parent in root.iter():
        for child in parent:
            parent_of[child] = parent
    geometry = parent_of.get(mesh)
    if geometry is None:
        return []
    return [geometry]
