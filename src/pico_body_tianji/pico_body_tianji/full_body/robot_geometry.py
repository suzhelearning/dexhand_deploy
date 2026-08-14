import numpy as np


# libKine 的胸廓/Base 坐标原点位于机械臂肩关节下方 174.5 mm。
# 所有送入 IK 和 RViz 的上肢关键点都使用这个同一原点。
MARVIN_SHOULDER_ORIGIN_M = np.array(
    [0.0, 0.0, 0.1745], dtype=np.float64
)

# Marvin URDF 中 Link_Stand→左右 Base 的固定 Z 偏移均为 140 mm。
# 完整 SMPL 骨架以两肩中心放在该高度，坐标使用 Link_Stand/机器人世界轴。
MARVIN_SHOULDER_CENTER_IN_STAND_M = np.array(
    [0.0, 0.0, 0.14], dtype=np.float64
)
