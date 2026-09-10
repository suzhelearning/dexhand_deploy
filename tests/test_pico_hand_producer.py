from dataclasses import replace
import os
from pathlib import Path
import unittest

from tests import test_official_pico_input as pico_fixture
from tests.test_pico_official_backend import SideClient
from tianji_teleop.producers import pico_official_hand
from tianji_teleop.protocol.messages import SessionState


class PicoHandProducerTest(unittest.TestCase):
    def make(self):
        clients = {side: SideClient(side) for side in ('left', 'right')}
        backend = pico_official_hand.PicoOfficialHandBackend(clients,
            receiver_instance_id='pico', connection_generation=3)
        self.assertTrue(hasattr(pico_official_hand, 'PicoOfficialHandProducer'))
        producer = pico_official_hand.PicoOfficialHandProducer(backend,
            publisher_instance_id='pico-hand', router_zid='router', coordinator_instance_id='coord',
            receiver_instance_id='pico', freshness_ns=100)
        return clients, producer

    def state(self, phase='teleop', seq=1):
        return SessionState(1, seq, 1000, phase, 'test', 'coordinator', None, 'coord', 'router')

    def frame(self):
        return pico_fixture.OfficialPicoInputTest().frame()

    def test_pico_uses_existing_authority_and_one_shot_command_gate(self):
        _, producer = self.make()
        self.assertTrue(producer.update_frame(self.frame(), now_ns=1000))
        self.assertEqual(producer.commands(1000), {})
        producer.update_session(self.state())
        commands = producer.commands(1000)
        self.assertEqual(set(commands), {'left', 'right'})
        self.assertEqual(commands['left'].publisher_instance_id, 'pico-hand')
        self.assertEqual(commands['left'].sequence, 8)
        self.assertEqual(commands['left'].timestamp_ns, 1000)
        self.assertEqual(producer.commands(1000), {})

    def test_stale_input_does_not_retarget_and_missing_side_does_not_command(self):
        clients, producer = self.make()
        producer.update_session(self.state())
        frame = self.frame()
        self.assertFalse(producer.update_frame(frame, now_ns=1101))
        self.assertFalse(clients['left'].calls)
        frame = replace(frame, hands=dict(frame.hands, left=replace(frame.hands['left'], valid=False)))
        self.assertTrue(producer.update_frame(frame, now_ns=1000))
        self.assertEqual(set(producer.commands(1000)), {'right'})
        self.assertEqual(producer.input_snapshot['valid_sides'], ['right'])

    def test_reconnection_requires_explicit_algorithm_state_recreation(self):
        _, producer = self.make()
        producer.update_session(self.state())
        self.assertTrue(producer.update_frame(self.frame(), now_ns=1000))
        frame = replace(self.frame(), connection_generation=4, receiver_frame_sequence=0)
        self.assertFalse(producer.update_frame(frame, now_ns=1000))
        self.assertFalse(producer.healthy)
        self.assertEqual(producer.commands(1000), {})
        producer.update_session(self.state(seq=2))
        self.assertFalse(producer.update_frame(replace(self.frame(), receiver_frame_sequence=9), now_ns=1000))

    @unittest.skipUnless(os.environ.get('WUJI_REFERENCE_TEST'), 'optional official Hand2 worker')
    def test_official_pico_commands_reach_authorized_mujoco_hands(self):
        from tests import test_authorized_hand_mujoco as sim_fixture
        from tianji_teleop.producers.hand_retarget import OfficialHandClient
        fixture = sim_fixture.AuthorizedHandMujocoTest()
        self.addCleanup(fixture.doCleanups)
        sim = fixture.make()
        root = Path(__file__).resolve().parents[1]
        clients = {}
        for side in ('left', 'right'):
            clients[side] = OfficialHandClient(
                python=root / 'tools/wuji_hand_native/.pixi/envs/default/bin/python',
                script=root / 'scripts/wuji_hand_worker.py', single_hand_side=side, startup_handshake=True)
            self.addCleanup(clients[side].close)
        backend = pico_official_hand.PicoOfficialHandBackend(clients,
            receiver_instance_id='pico', connection_generation=3)
        producer = pico_official_hand.PicoOfficialHandProducer(backend,
            publisher_instance_id='hand', router_zid='router', coordinator_instance_id='coord',
            receiver_instance_id='pico', freshness_ns=200_000_000)
        frame = replace(self.frame(), received_timestamp_ns=fixture.now)
        self.assertTrue(producer.update_frame(frame, now_ns=fixture.now))
        self.assertEqual(producer.commands(fixture.now), {})
        state = fixture.state('teleop', 1)
        sim.on_session_state(state)
        producer.update_session(state)
        commands = producer.commands(fixture.now)
        self.assertEqual(set(commands), {'left', 'right'})
        for command in commands.values():
            self.assertTrue(sim.on_hand_command(command), sim.last_hand_rejection)
        sim.tick()
        for side, command in commands.items():
            self.assertEqual(sim.hand_state(side).position_rad, command.position_rad)
        stopped = fixture.state('fault', 2)
        sim.on_session_state(stopped)
        producer.update_session(stopped)
        for command in commands.values():
            self.assertFalse(sim.on_hand_command(command))
        self.assertEqual(producer.commands(fixture.now), {})
