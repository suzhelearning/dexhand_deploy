import importlib.util
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
from dataclasses import replace
import time
import unittest

from tests import test_official_pico_input as pico_fixture
from tests.test_pico_official_backend import SideClient
from tianji_teleop.hand_tracking.pico import parse_pico_packet
from tianji_teleop.hand_tracking.runtime import ObservationRuntime
from tianji_teleop.producers.pico_official_hand import PicoOfficialHandBackend, PicoOfficialHandProducer
from tianji_teleop.protocol import topics
from tianji_teleop.protocol.messages import SessionState


class Session:
    def __init__(self):
        self.callbacks, self.messages = {}, []

    def declare_subscriber(self, topic, callback):
        self.callbacks[topic] = callback
        session = self
        class Handle:
            def undeclare(self):
                session.callbacks.pop(topic, None)
        return Handle()

    def put(self, topic, payload, **kwargs):
        self.messages.append((topic, json.loads(payload)))


def wire_frame():
    frame = pico_fixture.OfficialPicoInputTest().frame()
    payload = bytearray(struct.pack('<BBBB', 1, frame.flags, 26, 0))
    payload.extend(struct.pack('<7f', *frame.head_pose))
    for hand in frame.hands.values():
        payload.extend(struct.pack('<BBBB7f', int(hand.valid), 0, 0, 0, *hand.wrist_pose))
        for joint in hand.joints:
            payload.extend(struct.pack('<BBBB7ff', int(joint.valid), 0, 0, 0, *joint.pose, joint.radius_m))
    packet = struct.pack('<BBqI', 0xAB, 0x40, frame.source_timestamp_ms, len(payload)) + payload
    return parse_pico_packet(packet, receiver_instance_id='pico', connection_generation=3,
                             receiver_frame_sequence=7, received_timestamp_ns=1000)


class PicoHandServiceTest(unittest.TestCase):
    def make(self, capability='simulation', authority_guard=None, audit_processed_inputs=False):
        self.assertIsNotNone(importlib.util.find_spec('tianji_teleop.producers.pico_hand_service'))
        from tianji_teleop.producers.pico_hand_service import PicoHandService
        backend = PicoOfficialHandBackend({side: SideClient(side) for side in ('left', 'right')},
            receiver_instance_id='pico', connection_generation=3)
        producer = PicoOfficialHandProducer(backend, publisher_instance_id='hand', router_zid='router',
            coordinator_instance_id='coord', receiver_instance_id='pico', freshness_ns=100)
        session = Session()
        service = PicoHandService(session, producer, required_capability=capability,
                                  clock=lambda: 1000, authority_guard=authority_guard,
                                  audit_processed_inputs=audit_processed_inputs)
        self.addCleanup(service.close)
        return session, service

    def test_existing_observation_raw_topic_flows_through_authorized_shared_loop(self):
        session, service = self.make()
        self.assertEqual(set(session.callbacks), {topics.RAW_PICO_HAND_TRACKING, topics.SESSION_STATE})
        state = SessionState(1, 1, 1000, 'teleop', 'test', 'coordinator', None, 'coord', 'router')
        session.callbacks[topics.SESSION_STATE](state.to_dict())
        runtime = ObservationRuntime(publish=lambda topic, value: session.callbacks[topic](value)
            if topic in session.callbacks else None, publisher_instance_id='pico', router_zid='router')
        runtime.ingest_pico(wire_frame())
        deadline = time.monotonic() + 2
        while len(session.messages) < 2 and not service.failure and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertIsNone(service.failure)
        self.assertEqual({topic for topic, _ in session.messages}, {topics.hand_command('left'), topics.hand_command('right')})
        status = service.tick()
        self.assertTrue(status.ready)
        self.assertEqual(session.messages[-1][0], topics.PRODUCER_STATUS)
        service.close()
        self.assertEqual(session.callbacks, {})
        with self.assertRaises(RuntimeError):
            service.tick()

    def test_real_capability_is_not_enabled_by_new_hand_component(self):
        with self.assertRaisesRegex(ValueError, 'simulation'):
            self.make('real')

    def test_lost_authority_blocks_publication_and_reports_fault(self):
        from tianji_teleop.coordination.live_domain_guard import LiveDomainGuard
        token = 'tj/live/producer/hand/official_wuji_hand2/hand'
        guard = LiveDomainGuard({token})
        session, service = self.make(authority_guard=guard)
        guard.observe(token, present=False)
        state = SessionState(1, 1, 1000, 'teleop', 'test', 'coordinator', None, 'coord', 'router')
        session.callbacks[topics.SESSION_STATE](state.to_dict())
        session.callbacks[topics.RAW_PICO_HAND_TRACKING](dict(wire_frame().to_dict(), router_zid='router'))
        time.sleep(.03)
        self.assertFalse(session.messages)
        self.assertIn('authority lost', service.failure)
        self.assertFalse(service.tick().healthy)

    def test_bad_session_message_latches_fault_and_prevents_commands(self):
        session, service = self.make()
        session.callbacks[topics.SESSION_STATE]({'bad': 'state'})
        self.assertIsNotNone(service.failure)
        self.assertFalse(service.tick().healthy)
        session.callbacks[topics.RAW_PICO_HAND_TRACKING](dict(wire_frame().to_dict(), router_zid='router'))
        time.sleep(.02)
        self.assertFalse(any(topic in (topics.hand_command('left'), topics.hand_command('right'))
                             for topic, _ in session.messages))

    def test_tracking_loss_holds_after_authorization_but_cannot_start_invalid(self):
        session, service = self.make()
        frame = wire_frame()
        raw = bytearray(frame.raw_packet)
        raw[15] &= ~2  # left tracking lost, TCP and right hand remain valid
        missing = parse_pico_packet(bytes(raw), receiver_instance_id='pico', connection_generation=3,
                                    receiver_frame_sequence=8, received_timestamp_ns=1000)
        def feed(value, processed):
            session.callbacks[topics.RAW_PICO_HAND_TRACKING](dict(value.to_dict(), router_zid='router'))
            deadline = time.monotonic() + 2
            while service.tick().diagnostics['processed_callbacks'] < processed and time.monotonic() < deadline:
                time.sleep(.005)
        feed(frame, 1)
        state = SessionState(1, 1, 1000, 'teleop', 'test', 'coordinator', None, 'coord', 'router')
        session.callbacks[topics.SESSION_STATE](state.to_dict())
        feed(missing, 2)
        status = service.tick()
        self.assertTrue(status.ready)
        self.assertEqual(status.diagnostics['tracking_hold_sides'], ['left'])
        session.callbacks[topics.SESSION_STATE](replace(state, sequence=2, state='idle').to_dict())
        time.sleep(.02)
        self.assertFalse(service.tick().ready)

    @unittest.skipUnless(os.environ.get('WUJI_REFERENCE_TEST'), 'optional router and official Hand2 workers')
    def test_actual_router_official_workers_and_mujoco_hand_handoff(self):
        import mujoco
        import zenoh
        from tianji_teleop.producers.pico_hand_component import PicoHandComponent
        from tianji_teleop.producers.spark.factory import reference_robot_config
        from tianji_teleop.executors.mujoco.authorized_hand import AuthorizedHandMujoco
        from tianji_teleop.protocol.messages import HandJointCommand
        root = Path(__file__).resolve().parents[1]
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        endpoint = f'tcp/127.0.0.1:{port}'
        router = subprocess.Popen([str(root / 'vendor/zenoh-router/zenohd'), '-l', endpoint,
            '--no-multicast-scouting'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        def stop_router():
            router.terminate()
            try:
                router.wait(2)
            except subprocess.TimeoutExpired:
                router.kill()
                router.wait(2)
        self.addCleanup(stop_router)
        deadline = time.monotonic() + 3
        while True:
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=.1):
                    break
            except OSError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.02)
        def connect():
            session = zenoh.open(zenoh.Config.from_json5(json.dumps(dict(mode='client',
                connect=dict(endpoints=[endpoint]), scouting=dict(multicast=dict(enabled=False))))))
            self.addCleanup(session.close)
            return session
        owner, observer = connect(), connect()
        zid = str(next(iter(owner.info.routers_zid())))
        service = PicoHandComponent(owner, root=root, publisher_instance_id='hand', router_zid=zid,
            coordinator_instance_id='coord', receiver_instance_id='pico', connection_generation=3,
            expected_tokens={'tj/live/producer/hand/official_wuji_hand2/hand',
                             'tj/live/coordinator/arm/arm/coord'}, required_capability='simulation')
        self.addCleanup(service.close)
        received = {}
        def on_command(sample):
            command = HandJointCommand.from_dict(json.loads(bytes(sample.payload)))
            received[command.side] = command
        subscriptions = [observer.declare_subscriber(topics.hand_command(side), on_command)
                         for side in ('left', 'right')]
        for subscription in subscriptions:
            self.addCleanup(subscription.undeclare)
        def publish(topic, value):
            observer.put(topic, json.dumps(value).encode(), encoding='application/json')
        runtime = ObservationRuntime(publish=publish, publisher_instance_id='pico', router_zid=zid)
        # Complete model startup BEFORE producing live, expiring inputs.
        assets = root / 'src/tianji_teleop'
        model = mujoco.MjModel.from_xml_path(str(assets / 'assets/spark/marvin_m6_wuji2.xml'))
        robot = reference_robot_config(assets / 'config/producers/spark_reference.yaml',
            assets / 'assets/spark/marvin_m6_s_ccs_696_v4_local.urdf', assets / 'config/robot/arm.yaml')
        sim = AuthorizedHandMujoco(model=model, data=mujoco.MjData(model), robot_config=robot,
            publisher_instance_id='sim', router_zid=zid, coordinator_instance_id='coord',
            hand_producer_id='official_wuji_hand2', hand_producer_instance_id='hand')
        self.addCleanup(sim.close)
        sequence = 0
        deadline = time.monotonic() + 5
        while set(received) != {'left', 'right'} and time.monotonic() < deadline:
            sequence += 1
            now = time.monotonic_ns()
            state = SessionState(1, sequence, now, 'teleop', 'test', 'coordinator', None, 'coord', zid)
            publish(topics.SESSION_STATE, state.to_dict())
            runtime.ingest_pico(replace(wire_frame(), receiver_frame_sequence=sequence,
                                        received_timestamp_ns=now))
            time.sleep(.02)
        self.assertEqual(set(received), {'left', 'right'}, service.tick().to_dict())
        self.assertIsNone(service.failure)
        service.close()
        commands = dict(received)
        # Fresh authority at the handoff; do not rewrite command timestamps.
        sim.on_session_state(SessionState(1, sequence+1, time.monotonic_ns(), 'teleop', 'test',
                                         'coordinator', None, 'coord', zid))
        for command in commands.values():
            self.assertTrue(sim.on_hand_command(command), sim.last_hand_rejection)
        sim.tick()
        for side, command in commands.items():
            self.assertEqual(sim.hand_state(side).position_rad, command.position_rad)
