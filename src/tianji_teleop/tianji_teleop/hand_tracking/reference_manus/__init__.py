"""Pinned reference input boundary; deliberately NOT wrist-relative.

No ROS import occurs unless the original module main() is explicitly called.
The old manus.py converter remains unchanged for existing profiles.
"""
from .manus_hand_input import HandInputAssembler, HandInputFrame, RawvizHandInputProcessor
from .wuji2_hand_input import resolve_wuji2_keypoints, parse_rawviz_line
