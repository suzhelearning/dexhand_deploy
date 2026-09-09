"""Real MuJoCo/Python dry-retarget integration; no router or device required.

Arm assertions stop at the IK target boundary. This test does not substitute
another solver for the deployment's native IK producer.
"""
from pathlib import Path
import struct
import unittest

import mujoco
import numpy as np

from tianji_teleop.hand_tracking.pico import PICO_TO_MEDIAPIPE
from tianji_teleop.hand_tracking.runtime import ObservationRuntime
from tianji_teleop.hand_tracking.target_node import _load_config, create_bridge_from_config
from tianji_teleop.executors.mujoco.node import MujocoExecutor
from tianji_teleop.executors.wuji_hand2.node import WujiHandExecutor
from tianji_teleop.mujoco_urdf import portable_mujoco_urdf
from tianji_teleop.protocol.messages import HAND_JOINT_NAMES, HandTargetCommand, SessionState

ROOT = Path(__file__).resolve().parents[1]


def packet(displacement: float, bend: float) -> bytes:
    points = np.zeros((21, 3))
    for base in (1, 5, 9, 13, 17):
        for joint in range(4):
            points[base + joint] = (
                [0.02 + bend * joint, 0.003, 0.02 * (joint + 1)]
                if base == 1 else
                [0.002, 0.02 + bend * joint, 0.03 * (joint + 1)]
            )
    native = np.zeros((26, 3))
    native[PICO_TO_MEDIAPIPE] = points
    payload = bytearray(struct.pack('<BBBB7f', 1, 7, 26, 0, 0, 0, 1.6, 0, 0, 0, 1))
    for lateral in (0.2, -0.2):
        wrist = np.array([0.3 + displacement, lateral, 1.3])
        payload.extend(struct.pack('<BBBB7f', 1, 0, 0, 0, *wrist, 0, 0, 0, 1))
        for point in native:
            payload.extend(struct.pack('<BBBB7ff', 1, 0, 0, 0, *(point + wrist), 0, 0, 0, 1, 0.005))
    return struct.pack('<BBqI', 0xAB, 0x40, 1000, len(payload)) + payload


class PicoMujocoPipelineTest(unittest.TestCase):
    def test_pico_updates_both_simulated_hands_and_both_ik_targets(self):
        now = 1_000_000_000
        config = _load_config(ROOT / 'src/tianji_teleop/config/sources/hand_tracking_target.yaml')
        bridge = create_bridge_from_config(config, router_zid='test', observation_publisher_instance_id='obs')

        def publish(topic, value):
            if '/observation/hand/' in topic:
                bridge.ingest_hand_observation(value, now_ns=now)
            elif '/observation/arm_input/' in topic:
                bridge.ingest_arm_observation(value)

        runtime = ObservationRuntime(publish=publish, publisher_instance_id='obs', router_zid='test')
        xml, assets = portable_mujoco_urdf(ROOT / 'src/tianji_teleop/assets/tianji_wuji2/tianji_wuji2.urdf')
        model = mujoco.MjModel.from_xml_string(xml, assets)
        sim = MujocoExecutor(model=model, data=mujoco.MjData(model), publisher_instance_id='sim',
                             router_zid='test', coordinator_instance_id='coord',
                             hand_sides=('left', 'right'), hand_overlay=True, clock=lambda: now)
        hands = {side: WujiHandExecutor(mode='retarget', side=side, publisher_instance_id=f'hand-{side}',
                 router_zid='test', authorized_producer=f'retarget-{side}',
                 authorized_publisher_instance_id='target', coordinator_instance_id='coord',
                 dry_run=True, clock=lambda: now) for side in ('left', 'right')}
        self.addCleanup(sim.close)
        for hand in hands.values():
            self.addCleanup(hand.close)
        previous = {}
        for index, (displacement, bend) in enumerate(((0.0, 0.0), (0.02, 0.01))):
            now += 20_000_000
            runtime.ingest_pico_packet(packet(displacement, bend), receiver_instance_id='pico',
                                      connection_generation=1, receiver_frame_sequence=index,
                                      received_timestamp_ns=now)
            if index == 0:
                self.assertFalse(bridge.tick(now_ns=now).hand)
                bridge.start(now_ns=now)
            targets = bridge.tick(now_ns=now)
            self.assertEqual(len(targets.arm), 2)
            for target in targets.arm:
                home = config['arm_pose_mapper_config']['home_pose'][target.side]
                np.testing.assert_allclose(target.pose[:3], np.array(home[:3]) + [displacement, 0, 0], atol=1e-6)
            for target in targets.hand:
                hand = hands[target.side]
                hand.on_session_state(SessionState(1, index, now, 'teleop', 'test', 'target', 0, 'coord', 'test'))
                wire = HandTargetCommand(1, index, now, None, 'hand_tracking_target', target.side,
                                         'wrist_relative_mediapipe', target.keypoints_m.tolist(), 'target', 'test')
                self.assertTrue(hand.on_hand_target(wire))
                command = hand.tick()
                self.assertIsNotNone(command)
                self.assertEqual(tuple(command.names), HAND_JOINT_NAMES[target.side])
                self.assertTrue(sim.on_hand_command(command))
                sim.tick()
                values = sim.hand_state(target.side).position_rad
                np.testing.assert_allclose(values, command.position_rad)
                if index:
                    self.assertGreater(np.linalg.norm(np.array(values) - previous[target.side]), 0.01)
                previous[target.side] = values


if __name__ == '__main__':
    unittest.main()
