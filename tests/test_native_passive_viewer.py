import os
import subprocess
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from threading import Event


class JoinedViewerTest(unittest.TestCase):
    def test_close_waits_for_renderer_destruction(self):
        from tianji_teleop.producers.spark.native_passive_viewer import joined_passive_viewer
        closing, destroyed = Event(), Event()
        handle = SimpleNamespace(close=closing.set)
        def launch(model, data, **kwargs):
            kwargs['handle_return'].put(handle)
            closing.wait(2)
            destroyed.set()
        with patch('mujoco.viewer._launch_internal', launch):
            with joined_passive_viewer(None, None) as viewer:
                self.assertIs(viewer, handle)
                self.assertFalse(destroyed.is_set())
            self.assertTrue(destroyed.is_set())

    def test_startup_failure_does_not_hang(self):
        from tianji_teleop.producers.spark.native_passive_viewer import joined_passive_viewer
        with patch('mujoco.viewer._launch_internal', side_effect=RuntimeError('GL init failed')):
            with self.assertRaisesRegex(RuntimeError, 'GL init failed'):
                with joined_passive_viewer(None, None):
                    self.fail('must not start')

    @unittest.skipUnless(os.environ.get('NATIVE_PASSIVE_VIEWER_TEST') == '1', 'requires desktop GL')
    def test_real_gl_close_and_immediate_process_exit(self):
        code = '''import mujoco
from tianji_teleop.producers.spark.native_passive_viewer import joined_passive_viewer
m=mujoco.MjModel.from_xml_string('<mujoco/>'); d=mujoco.MjData(m)
with joined_passive_viewer(m,d): pass
'''
        import sys
        for _ in range(5):
            p = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=20)
            self.assertEqual(p.returncode, 0, p.stderr)
