"""Resolve a simulation session's shared arm limits without editing robot assets."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from .coordination.arm_command_coordinator import ArmRobotConfig


def resolve_arm_config(arm_path: Path, urdf_path: Path, source: str) -> dict:
    value = yaml.safe_load(Path(arm_path).read_text(encoding='utf-8'))
    ArmRobotConfig.from_mapping(value)
    if source == 'yaml':
        return value
    if source != 'urdf':
        raise ValueError('joint-limit-source must be yaml or urdf')
    joints = {}
    for joint in ET.parse(urdf_path).getroot().findall('joint'):
        name = joint.get('name')
        if name in joints:
            raise ValueError(f'duplicate URDF joint: {name}')
        joints[name] = joint
    bounds = {}
    for side in ('left', 'right'):
        bounds[side] = []
        for name in value[f'{side}_joint_names']:
            joint = joints.get(name)
            if joint is None or joint.get('type') != 'revolute':
                raise ValueError(f'missing bounded revolute URDF joint: {name}')
            limit = joint.find('limit')
            if limit is None or 'lower' not in limit.attrib or 'upper' not in limit.attrib:
                raise ValueError(f'missing URDF limits: {name}')
            lo, hi = float(limit.get('lower')), float(limit.get('upper'))
            if not math.isfinite(lo) or not math.isfinite(hi) or lo >= hi:
                raise ValueError(f'invalid URDF limits: {name}')
            bounds[side].append((lo, hi))
    # The current arm schema shares seven bounds between the two sides.
    # Do not silently widen or intersect asymmetric URDFs.
    if bounds['left'] != bounds['right']:
        raise ValueError('left/right URDF limits differ; shared arm schema cannot represent them')
    value['lower_limits_rad'] = [lo for lo, _ in bounds['left']]
    value['upper_limits_rad'] = [hi for _, hi in bounds['left']]
    ArmRobotConfig.from_mapping(value)  # Includes Home-inside-limits validation.
    return value


def write_snapshot(value: dict, output: Path) -> None:
    # Native producer expects block joint names and inline numeric vectors.
    names = {key: value[key] for key in ('left_joint_names', 'right_joint_names')}
    text = yaml.safe_dump(names, sort_keys=False)
    for key in ('left_home_rad', 'right_home_rad', 'lower_limits_rad', 'upper_limits_rad'):
        text += f'{key}: {json.dumps(value[key], allow_nan=False)}\n'
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x', encoding='utf-8') as stream:
        stream.write(text)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm-config', required=True, type=Path)
    parser.add_argument('--urdf', required=True, type=Path)
    parser.add_argument('--source', required=True, choices=('yaml', 'urdf'))
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    write_snapshot(resolve_arm_config(args.arm_config, args.urdf, args.source), args.output)


if __name__ == '__main__':
    main()
