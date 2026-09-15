from pathlib import Path
import subprocess
import tempfile
import unittest

from tests.test_native_raw_input import compile_driver
from tianji_teleop.producers.spark.native_gateway import decode_gateway_frame, FRAME_HEADER_STRUCT


class GatewaySummaryTest(unittest.TestCase):
    def test_reader_rejects_unnegotiated_transport(self):
        import socket
        import struct
        import sys
        from tianji_teleop.producers.spark.native_gateway import NativeGatewayProcess
        summary = struct.pack('<qQQQBBBBI',1,1,0,1,1,0,0,0,0)
        raw = struct.pack('<QqB3xI',1,1,1,1)+b'x'+bytes(655)
        with tempfile.NamedTemporaryFile() as manifest:
            for mode,kind,payload in (('full',7,summary),('summary',6,raw)):
                process=NativeGatewayProcess(Path(sys.executable),Path(manifest.name),
                    prefix='spark',diagnostic_transport=mode)
                parent,peer=socket.socketpair()
                with parent,peer:
                    process._socket=parent
                    peer.sendall(FRAME_HEADER_STRUCT.pack(b'TJSO',1,kind,0,len(payload),1,1)+payload)
                    peer.shutdown(socket.SHUT_WR)
                    process._read_frames()
                    self.assertIn('diagnostic transport mismatch',process.failure)
                    process._socket=None

    def test_summary_handshake_is_explicit(self):
        import sys
        from types import SimpleNamespace
        from tianji_teleop.producers.spark.native_gateway import NativeGatewayProcess
        with tempfile.NamedTemporaryFile() as manifest:
            process=NativeGatewayProcess(Path(sys.executable),Path(manifest.name),prefix='spark',
                diagnostic_transport='summary')
            for token in (b'',b' diagnostics=summary'):
                with tempfile.TemporaryFile() as stream:
                    stream.write(b'native_session_gateway_ready'+token+b'\n');stream.seek(0)
                    process._process=SimpleNamespace(stdout=stream,poll=lambda:None)
                    if token: process._wait_ready()
                    else:
                        with self.assertRaisesRegex(RuntimeError,'handshake'): process._wait_ready()
                    process._process=None

    def test_native_summary_throttles_without_losing_final_count(self):
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / 'summary'
            compile_driver('gateway_summary_fixture.cpp', binary)
            data = subprocess.check_output([str(binary)], timeout=5)
        frames = []
        while data:
            size = FRAME_HEADER_STRUCT.size + FRAME_HEADER_STRUCT.unpack_from(data)[4]
            frames.append(decode_gateway_frame(data[:size], prefix='spark'))
            data = data[size:]
        self.assertEqual([f.kind for f in frames], ['summary'] * 4 + ['complete'])
        self.assertEqual([f.payload['cycles'] for f in frames[:-1]], [1, 5, 25, 26])
        self.assertEqual(frames[-2].payload['state'], 'teleop')
        self.assertEqual(frames[-2].payload['ticks'], 26)
        self.assertEqual(frames[-2].payload['state_epoch'], 1)

    def test_malformed_summary_is_rejected(self):
        import struct
        payload = struct.pack('<qQQQBBBBI', 1, 26, 2, 26, 2, 0, 0, 0, 0)
        def wire(data):
            return FRAME_HEADER_STRUCT.pack(b'TJSO',1,7,0,len(data),26,1000)+data
        self.assertEqual(decode_gateway_frame(wire(payload),prefix='spark').payload['cycles'],26)
        for index, value in ((0,0),(24,0),(32,0),(33,2),(34,2),(35,1)):
            invalid=bytearray(payload);invalid[index]=value
            with self.assertRaises(ValueError): decode_gateway_frame(wire(invalid),prefix='spark')
        with self.assertRaises(ValueError): decode_gateway_frame(wire(payload+b'x'),prefix='spark')

    def test_summary_cannot_replace_python_consumers(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from tests.test_native_session_publication import manifest
        from tianji_teleop.producers.spark.native_live_runner import _NativeFrameConsumer
        for native in (False, True):
            config=manifest() | {'publication_backend':'cpp', 'viewer_backend':'cpp' if native else 'python'}
            consumer=_NativeFrameConsumer(Mock(),manifest=config,prefix='spark',output=Mock(),
                capture=None,overlay=None,mapped_overlay=None,receiver_instance_id='source')
            frame=SimpleNamespace(kind='summary',payload=dict(state='teleop',state_epoch=2,
                                 cycles=26,ticks=26,late_ticks=2))
            if native:
                consumer._consume(frame)
                self.assertEqual(consumer.control_status,('teleop',2))
                self.assertEqual(consumer.counters,dict(cycles=26,native_ticks=26,late_cycles=2))
                consumer._consume(SimpleNamespace(kind='complete',payload=dict(
                    complete=False,state='fault',epoch=2,reason='output failed')))
                self.assertEqual(consumer.control_status,('fault',2))
            else:
                with self.assertRaises(RuntimeError): consumer._consume(frame)
