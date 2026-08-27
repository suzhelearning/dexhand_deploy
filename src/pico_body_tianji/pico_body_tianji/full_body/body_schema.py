"""XRoboToolkit/PICO Body 的 24 点索引与 SMPL 兼容层级。"""

PICO_BODY_JOINT_NAMES = (
    "pelvis",
    "left_hip",
    "right_hip",
    "spine1",
    "left_knee",
    "right_knee",
    "spine2",
    "left_ankle",
    "right_ankle",
    "spine3",
    "left_foot",
    "right_foot",
    "neck",
    "left_collar",
    "right_collar",
    "head",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_hand",
    "right_hand",
)

PICO_BODY_JOINT_INDEX = {
    name: index for index, name in enumerate(PICO_BODY_JOINT_NAMES)
}

# PICO 的编号与 SMPL 24 点兼容。锁骨和颈部均从 Spine3 分叉；
# Collar 不是 Neck 的子节点。
PICO_BODY_PARENT_INDICES = (
    -1,
    0,
    0,
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    9,
    9,
    12,
    13,
    14,
    16,
    17,
    18,
    19,
    20,
    21,
)

PICO_BODY_BONES = tuple(
    (parent, child)
    for child, parent in enumerate(PICO_BODY_PARENT_INDICES)
    if parent >= 0
)
