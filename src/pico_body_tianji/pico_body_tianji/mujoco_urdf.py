from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_MESH_PREFIX = (
    "package://pico_body_tianji/assets/marvin_m6_ccs/meshes/"
)


def portable_mujoco_urdf(
    urdf_path: Path,
) -> tuple[str, dict[str, bytes]]:
    """Return relocatable URDF XML and in-memory MuJoCo mesh assets."""
    source = Path(urdf_path).resolve()
    root = ET.parse(source).getroot()
    compiler = root.find("./mujoco/compiler")
    if compiler is None:
        raise ValueError("MuJoCo URDF is missing mujoco/compiler")
    compiler.set("meshdir", "")

    mesh_directory = source.parent.parent / "meshes"
    assets: dict[str, bytes] = {}
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if not filename.startswith(PACKAGE_MESH_PREFIX):
            continue
        asset_name = filename.removeprefix(PACKAGE_MESH_PREFIX)
        if not asset_name or Path(asset_name).name != asset_name:
            raise ValueError(f"invalid Marvin mesh URI: {filename}")
        asset_path = mesh_directory / asset_name
        assets[asset_name] = asset_path.read_bytes()
        mesh.set("filename", asset_name)

    if not assets:
        raise ValueError("MuJoCo URDF contains no Marvin mesh assets")
    xml = ET.tostring(root, encoding="unicode")
    return xml, assets
