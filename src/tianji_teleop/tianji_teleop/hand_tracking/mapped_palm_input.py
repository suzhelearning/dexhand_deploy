"""Reference mapped-palm selection for receive-side jump gating; preserves raw bytes."""
from dataclasses import replace
import struct
import numpy as np
from scipy.spatial.transform import Rotation


def select_mapped_palm_frame(frame):
    if not frame.upper_limb_skeleton_valid or not frame.upper_limb_rotations_valid:
        raise ValueError('mapped palm requires valid corrected skeleton positions and rotations')
    # Reference selectMappedCorrectedPalm uses 1e-6 before normalization.
    for i in range(8):
        q = struct.unpack_from('<4d', frame.raw_packet, 396 + 32 * i)
        if abs(np.linalg.norm(q) - 1) > 1e-6:
            raise ValueError('mapped skeleton rotation is not unit length')
    bases = (np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]),
             np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]]))
    poses = []
    for i, basis in zip((3, 7), bases):
        rotation = Rotation.from_quat(frame.upper_limb_rotations_xyzw[i]).as_matrix() @ basis
        poses.append(np.r_[frame.upper_limb_points[i], Rotation.from_matrix(rotation).as_quat()])
    return replace(frame, left_pose=poses[0], right_pose=poses[1])
