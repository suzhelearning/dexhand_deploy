"""Read-only asset closure for the pinned flat, mesh-only SPARK MJCF.

This is not a general MuJoCo compiler. Unsupported path semantics fail rather
than producing a supposedly complete but partial recording manifest.
"""
from pathlib import Path
import xml.etree.ElementTree as ET


def flat_mujoco_asset_files(model, *, asset_root):
    root = Path(asset_root).resolve()

    def checked(path):
        path = Path(path).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            raise ValueError(f'model asset missing or outside asset root: {path}')
        return path

    model = checked(model)
    try:
        tree = ET.parse(model).getroot()
    except ET.ParseError as exc:
        raise ValueError(f'invalid model XML: {model}') from exc
    if tree.tag != 'mujoco' or tree.findall('.//include'):
        raise ValueError('asset closure requires flat MuJoCo XML without includes')
    compilers = tree.findall('compiler')
    if len(compilers) > 1:
        raise ValueError('asset closure requires a single compiler declaration')
    compiler = compilers[0].attrib if compilers else {}
    if 'assetdir' in compiler or compiler.get('strippath', 'false') != 'false':
        raise ValueError('unsupported model asset path transformation')
    meshdir = model.parent / compiler.get('meshdir', '')
    meshes = tree.findall('asset/mesh')
    paths = {model}
    for element in tree.iter():
        if 'file' not in element.attrib:
            continue
        if element not in meshes or not element.attrib['file'].strip():
            raise ValueError('asset closure supports explicit mesh files only')
        paths.add(checked(meshdir / element.attrib['file']))
    return sorted(paths)
