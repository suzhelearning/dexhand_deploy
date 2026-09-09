from pathlib import Path
import tempfile
import unittest

import h5py

from tianji_teleop.protocol.messages import ArmTargetCommand, ProtocolEnvelope, ARM_FRAMES
from tianji_teleop.recording.session_h5 import (
    SessionH5Writer, SessionH5Reader, SessionH5Error, EXTENDED_SCHEMA_VERSION, SCHEMA_VERSION,
)


class ArmTargetRecordingValidityTest(unittest.TestCase):
    def write_targets(self, path, schema=EXTENDED_SCHEMA_VERSION, values=(True, False, True)):
        writer = SessionH5Writer(path, source_type=("hand_tracking_sim" if schema == EXTENDED_SCHEMA_VERSION else "mocap_live"),
                                 robot_model="marvin", router_zid="router", schema_version=schema)
        try:
            for sequence, valid in enumerate(values):
                writer.append_arm_target(ArmTargetCommand(
                    ProtocolEnvelope(1, "source", "router", sequence, sequence + 100),
                    None, "hand_tracking_target", "left", ARM_FRAMES["left"],
                    [0., 0., 0.], [0., 0., 0., 1.], [0., 0., 1.], tracking_valid=valid))
        finally:
            writer.close()

    def test_tracking_hold_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.h5"
            self.write_targets(path)
            with SessionH5Reader(path) as reader:
                self.assertEqual([row.get("tracking_valid") for row in reader.read_arm_target("left")],
                                 [True, False, True])

    def test_existing_files_default_to_valid(self):
        for schema in (SCHEMA_VERSION, EXTENDED_SCHEMA_VERSION):
            with self.subTest(schema=schema), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "session.h5"
                self.write_targets(path, schema, (True,))
                with h5py.File(path, "r+") as file:
                    for side in ("left", "right"):
                        group = file[f"target/arm/{side}"]
                        if "tracking_valid" in group:
                            del group["tracking_valid"]
                with SessionH5Reader(path) as reader:
                    self.assertIs(reader.read_arm_target("left")[0].get("tracking_valid"), True)

    def test_schema_10_rejects_hold_before_appending(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.h5"
            with self.assertRaisesRegex(SessionH5Error, "1.1"):
                self.write_targets(path, SCHEMA_VERSION, (False,))
            with SessionH5Reader(path) as reader:
                self.assertEqual(reader.read_arm_target("left"), [])

    def test_malformed_tracking_column_is_rejected(self):
        for corruption in ("length", "dtype"):
            with self.subTest(corruption=corruption), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "session.h5"
                self.write_targets(path)
                with h5py.File(path, "r+") as file:
                    group = file["target/arm/left"]
                    if corruption == "length":
                        group["tracking_valid"].resize((2,))
                    else:
                        del group["tracking_valid"]
                        group.create_dataset("tracking_valid", data=[1, 0, 1], maxshape=(None,), chunks=True)
                with self.assertRaises(SessionH5Error):
                    SessionH5Reader(path)
