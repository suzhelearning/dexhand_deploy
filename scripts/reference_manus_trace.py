"""Run the ORIGINAL hand bridge offline and emit a JSON reference trace.

Accepts only the NumPy pickle layout used by the supplied Manus recording.
Run in the pinned reference environment. No ROS node, socket or device starts.
Output goes to stdout so SSH can stream it without writing remote files.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import pickle
import sys
from pathlib import Path

import numpy as np


class RecordingUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if (module, name) == ("numpy", "dtype"):
            return np.dtype
        if (module, name) in {
            ("numpy._core.numeric", "_frombuffer"),
            ("numpy.core.numeric", "_frombuffer"),
        }:
            return np._core.numeric._frombuffer
        raise pickle.UnpicklingError(f"unsupported recording global: {module}.{name}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ("repository", "input", "left-config", "right-config"):
        parser.add_argument("--" + flag, type=Path, required=True)
    args = parser.parse_args()
    repository = args.repository.resolve()
    bridge_file = repository / "example/tj_wuji2_hand_bridge.py"
    spec = importlib.util.spec_from_file_location("original_hand_bridge", bridge_file)
    if spec is None or spec.loader is None:
        raise ValueError("cannot load original hand bridge")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    with args.input.open("rb") as stream:
        rows = RecordingUnpickler(stream).load()
    if not isinstance(rows, list) or not rows:
        raise ValueError("recording must contain a nonempty frame list")
    bridge = module.OfficialWujiHand2Bridge(
        repository,
        left_config=args.left_config.resolve(),
        right_config=args.right_config.resolve(),
    )
    times, output = [], []
    for index, row in enumerate(rows):
        t = float(row["t"])
        if not np.isfinite(t) or t < 0 or (times and t < times[-1]):
            raise ValueError(f"invalid time at frame {index}")
        hands = [np.asarray(row[key]) for key in ("right_fingers", "left_fingers")]
        if any(hand.shape != (21, 3) or not np.isfinite(hand).all() for hand in hands):
            raise ValueError(f"invalid hand at frame {index}")
        frame = bridge.retarget(np.concatenate([hand.reshape(-1) for hand in hands]),
                                index + 1, max(1, round(t * 1e9)))
        if not frame.left_valid or not frame.right_valid:
            raise ValueError(f"invalid retarget output at frame {index}")
        times.append(t)
        output.append(np.concatenate((frame.left, frame.right)))
    q = np.asarray(output, dtype="<f8")
    if not np.isfinite(q).all():
        raise ValueError("retarget produced non-finite output")
    digests = {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in {
        "input": args.input, "left_config": args.left_config,
        "right_config": args.right_config, "bridge": bridge_file,
    }.items()}
    json.dump({
        "schema_version": 1, "kind": "original_manus_hand_reference",
        "input_stage": "mediapipe21", "output_order": "left20_right20",
        "sequence_policy": "one_callback_per_recorded_row_starting_at_1",
        "frames": len(rows), "sha256": digests,
        "output_sha256": hashlib.sha256(q.tobytes()).hexdigest(),
        "joint_names": list(module.HAND2_JOINT_NAMES["left"]) + list(module.HAND2_JOINT_NAMES["right"]),
        "timestamps_s": times, "position_rad": q.tolist(),
    }, sys.stdout, allow_nan=False)
    print()


if __name__ == "__main__":
    main()
