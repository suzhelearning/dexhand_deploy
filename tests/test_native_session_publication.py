"""Native publication parity, without opening Zenoh or commanding devices."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def manifest():
    return dict(run_id="test-run", algorithm="spark_upper_qpoases_headroom_feedforward_velocity_qp",
                model="/portable/model.xml", home=[[1.] * 7, [-1.] * 7],
                **{role + "_authority": dict(logical=logical, instance=role, router="router")
                   for role, logical in (("source", "tjvr"), ("producer", "ik_spark_headroom"),
                                         ("coordinator", "arm"), ("executor", "mujoco"))})


def cycle(state="idle", result=None):
    return dict(timestamp_ns=10000, ticks=17, state=state, state_epoch=3, reason="accepted",
                source=dict(sequence=13, revision=7, accepted=True, skeleton_valid=True,
                            rotations_valid=True), capture_failed=False,
                command=dict(left=[1.] * 7, right=[-1.] * 7),
                feedback=dict(left=[.9] * 7, right=[-.9] * 7), result=result, ik_adopted=bool(result))


def reference(row, config):
    from tianji_teleop.producers.spark.native_live_runner import _typed_cycle, _publish_native_snapshot
    class Sink:
        def __init__(self): self.rows = []
        def put_json(self, topic, value): self.rows.append([topic, value])
    sink = Sink()
    _publish_native_snapshot(sink, _typed_cycle(row, manifest=config, prefix="spark", sample=None))
    return sink.rows


@unittest.skipUnless(shutil.which("c++"), "C++ compiler required")
class NativePublicationTest(unittest.TestCase):
    def test_optional_hand_rows_share_arm_router_and_leave_arm_rows_unchanged(self):
        config=manifest()
        row=cycle('teleop')
        original=reference(row,config)
        config['hand_authorities']={
            'producer':dict(logical='official_wuji_hand2',instance='hand',router='router'),
            'left':dict(logical='wuji_left',instance='sim',router='router'),
            'right':dict(logical='wuji_right',instance='sim',router='router')}
        row.update(hand_feedback=[[0.]*20,[0.]*20],left_hand_command=[0.]*20)
        payload=dict(cycle=row,manifest=config)
        result=subprocess.run([str(self.binary)],input=json.dumps(payload)+'\n',text=True,capture_output=True,check=True)
        rows=json.loads(result.stdout)
        self.assertEqual(rows[:-3],original)
        self.assertEqual({r[0] for r in rows[-3:]},{'tianji/state/hand/left','tianji/state/hand/right','tianji/command/hand/left'})
        for authority in config['hand_authorities'].values():authority['router']='other'
        result=subprocess.run([str(self.binary)],input=json.dumps(payload)+'\n',text=True,capture_output=True)
        self.assertNotEqual(result.returncode,0)

    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix="native-publication-")
        cls.addClassCleanup(cls.directory.cleanup)
        cls.binary = Path(cls.directory.name) / "encode"
        subprocess.run(["c++", "-std=c++17", "-O2", "-pthread", "-Wall", "-Wextra", "-Werror",
                        "-I" + str(Path(sys.prefix) / "include"),
                        str(ROOT / "tests/cpp/session_publication_fixture.cpp"),
                        "-o", str(cls.binary)], check=True)

    def test_all_topics_match_python_for_each_phase_and_result_presence(self):
        rows = [cycle(state, result) for state in ("idle", "teleop", "returning", "fault")
                for result in (None, dict(tick_id=9, applied_sequence=6, input_live=True))]
        missing = cycle(); missing["source"] = {}; missing["reason"] = ""; rows.append(missing)
        failed = cycle(); failed["capture_failed"] = True; rows.append(failed)
        encoded = subprocess.check_output([str(self.binary)], text=True, timeout=10,
                    input="".join(json.dumps(dict(cycle=row, manifest=manifest())) + "\n" for row in rows))
        self.assertEqual([json.loads(line) for line in encoded.splitlines()],
                         [reference(row, manifest()) for row in rows])

    def test_invalid_authority_rejected_before_output(self):
        config = manifest(); config["coordinator_authority"]["router"] = "different-router"
        result = subprocess.run([str(self.binary)], text=True, capture_output=True, timeout=10,
                                input=json.dumps(dict(cycle=cycle(), manifest=config)))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, "")


class PublicationOwnershipTest(unittest.TestCase):
    def test_native_consumers_do_not_rebuild_python_cycle(self):
        from types import SimpleNamespace
        from unittest.mock import Mock, patch
        from tianji_teleop.producers.spark.native_live_runner import _NativeFrameConsumer
        config = manifest() | {'publication_backend': 'cpp', 'viewer_backend': 'cpp'}
        consumer = _NativeFrameConsumer(Mock(), manifest=config, prefix='spark', output=Mock(),
            capture=None, overlay=None, mapped_overlay=None, receiver_instance_id='source')
        with patch('tianji_teleop.producers.spark.native_live_runner._typed_cycle',
                   side_effect=AssertionError('unused Python cycle construction')):
            consumer._consume(SimpleNamespace(kind='cycle', payload=cycle()))
        self.assertIsNone(consumer.latest)
        self.assertEqual(consumer.control_status, ('idle', 3))
        self.assertEqual(consumer.counters['cycles'], 1)

    def test_viewer_capability_requires_handshake(self):
        from types import SimpleNamespace
        from tianji_teleop.producers.spark.native_gateway import NativeGatewayProcess
        with tempfile.NamedTemporaryFile() as manifest_file:
            gateway=NativeGatewayProcess(Path(sys.executable),Path(manifest_file.name),prefix='spark',viewer_backend='cpp')
            for ready,accepted in ((b'native_session_gateway_ready\n',False),
                                   (b'native_session_gateway_ready viewer=cpp\n',True)):
                with tempfile.TemporaryFile() as stream:
                    stream.write(ready);stream.seek(0)
                    gateway._process=SimpleNamespace(stdout=stream,poll=lambda:None)
                    if accepted: gateway._wait_ready()
                    else:
                        with self.assertRaisesRegex(RuntimeError,'handshake'): gateway._wait_ready()
                    gateway._process=None

    def test_viewer_reply_preserves_operator_action(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from tianji_teleop.producers.spark.native_live_runner import _NativeFrameConsumer
        consumer = _NativeFrameConsumer(Mock(), manifest=manifest() | {'viewer_backend':'cpp'},
            prefix='spark', output=Mock(), capture=None, overlay=None, mapped_overlay=None,
            receiver_instance_id='source')
        consumer._consume(SimpleNamespace(kind='reply', payload={
            'id': (1 << 63) | (1 << 3) | 3, 'accepted': True, 'reason': 'accepted'}))
        self.assertEqual(consumer.poll_reports()[0]['action'], 'shutdown')

    def test_recording_capability_requires_exact_handshake(self):
        import socket
        from types import SimpleNamespace
        from tianji_teleop.producers.spark.native_gateway import NativeGatewayProcess
        recording, peer = socket.socketpair()
        with tempfile.NamedTemporaryFile() as manifest_file, recording, peer:
            gateway = NativeGatewayProcess(Path(sys.executable), Path(manifest_file.name),
                prefix='spark', recording_fd=recording.fileno())
            for ready, accepted in ((b'native_session_gateway_ready\n', False),
                                    (b'native_session_gateway_ready recording=cpp\n', True)):
                with tempfile.TemporaryFile() as stream:
                    stream.write(ready); stream.seek(0)
                    gateway._process = SimpleNamespace(stdout=stream, poll=lambda: None)
                    if accepted: gateway._wait_ready()
                    else:
                        with self.assertRaisesRegex(RuntimeError, 'handshake'): gateway._wait_ready()
                    gateway._process = None

    def test_old_gateway_cannot_silently_disable_python_publication(self):
        from types import SimpleNamespace
        from tianji_teleop.producers.spark.native_gateway import NativeGatewayProcess
        with tempfile.NamedTemporaryFile() as manifest_file:
            for backend, ready, accepted in (
                    ('python', b'native_session_gateway_ready\n', True),
                    ('cpp', b'native_session_gateway_ready\n', False),
                    ('cpp', b'native_session_gateway_ready publication=cpp\n', True)):
                gateway = NativeGatewayProcess(Path(sys.executable), Path(manifest_file.name),
                                               prefix='spark', publication_backend=backend)
                with tempfile.TemporaryFile() as stream:
                    stream.write(ready); stream.seek(0)
                    gateway._process = SimpleNamespace(stdout=stream, poll=lambda: None)
                    if accepted:
                        gateway._wait_ready()
                    else:
                        with self.assertRaisesRegex(RuntimeError, 'handshake'):
                            gateway._wait_ready()
                    gateway._process = None

    def test_cpp_publication_disables_python_put_but_keeps_snapshot(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from tianji_teleop.producers.spark.native_live_runner import _NativeFrameConsumer
        for backend in ('python', 'cpp'):
            config = manifest() | {'publication_backend': backend}
            sink = Mock()
            consumer = _NativeFrameConsumer(Mock(), manifest=config, prefix='spark', output=sink,
                                            capture=None, overlay=None, mapped_overlay=None,
                                            receiver_instance_id='source')
            consumer._consume(SimpleNamespace(kind='cycle', payload=cycle()))
            self.assertIsNotNone(consumer.latest)
            self.assertEqual(sink.put_json.call_count, 11 if backend == 'python' else 0)
