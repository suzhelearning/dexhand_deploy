"""Explicit side-specific bounds for the reference simulation route.

Not a replacement for the legacy shared-limit robot YAML. Deliberately exposes
only limits(side), so consumers cannot accidentally broadcast one arm's bounds.
"""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

from ..protocol.messages import ARM_JOINT_NAMES


def _vector(values):
    if (not isinstance(values, (list, tuple)) or len(values) != 7 or
            any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
        raise ValueError('robot vector must contain seven finite numbers')
    return tuple(float(v) for v in values)


@dataclass(frozen=True)
class BilateralArmRobotConfig:
    left_home_rad: tuple
    right_home_rad: tuple
    left_lower_limits_rad: tuple
    left_upper_limits_rad: tuple
    right_lower_limits_rad: tuple
    right_upper_limits_rad: tuple

    def __post_init__(self):
        for side in ('left', 'right'):
            for suffix in ('home_rad', 'lower_limits_rad', 'upper_limits_rad'):
                field = side + '_' + suffix
                object.__setattr__(self, field, _vector(getattr(self, field)))
            lo, hi = self.limits(side)
            if any(a >= b or not a <= q <= b for a, b, q in zip(lo, hi, getattr(self, side + '_home_rad'))):
                raise ValueError(f'{side} Home must lie inside ordered limits')

    @property
    def left_joint_names(self):
        return ARM_JOINT_NAMES['left']

    @property
    def right_joint_names(self):
        return ARM_JOINT_NAMES['right']

    @property
    def home_all(self):
        return self.left_home_rad + self.right_home_rad

    def limits(self, side):
        if side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        return getattr(self, side + '_lower_limits_rad'), getattr(self, side + '_upper_limits_rad')

    @classmethod
    def from_urdf(cls, path, left_home, right_home):
        joints = {}
        for joint in ET.parse(path).getroot().findall('joint'):
            name = joint.get('name')
            if name in joints:
                raise ValueError(f'duplicate URDF joint: {name}')
            joints[name] = joint
        bounds = []
        for side in ('left', 'right'):
            lower, upper = [], []
            for name in ARM_JOINT_NAMES[side]:
                joint = joints.get(name)
                limit = None if joint is None else joint.find('limit')
                if joint is None or joint.get('type') != 'revolute' or limit is None:
                    raise ValueError(f'missing bounded URDF joint: {name}')
                try:
                    lower.append(float(limit.attrib['lower']))
                    upper.append(float(limit.attrib['upper']))
                except (KeyError, ValueError) as exc:
                    raise ValueError(f'invalid URDF limits: {name}') from exc
            bounds.extend((lower, upper))
        return cls(left_home, right_home, *bounds)
