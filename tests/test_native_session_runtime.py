import json
from pathlib import Path
import random
import shlex
import shutil
import subprocess
import tempfile
from types import SimpleNamespace
import unittest

from tests.test_arm_coordinator import _status, _arm_state
from tianji_teleop.coordination.arm_command_coordinator import ArmCommandCoordinator

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which('c++'), 'C++ compiler required')
class NativeSessionRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        assert (ROOT / 'native/control/session_machine.hpp').is_file(), 'native state machine missing'
        cls.directory = tempfile.TemporaryDirectory(prefix='native-session-tests-')
        cls.addClassCleanup(cls.directory.cleanup)
        cls.driver = Path(cls.directory.name) / 'driver'
        subprocess.run(['c++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
            str(ROOT / 'tests/cpp/native_session_driver.cpp'), '-o', str(cls.driver)], check=True)

    def test_threaded_scheduler_and_safety_boundaries(self):
        executable = Path(self.directory.name) / 'runtime'
        subprocess.run(['c++', '-std=c++17', '-pthread', '-O2', '-Wall', '-Wextra', '-Werror',
            str(ROOT / 'tests/cpp/test_session_runtime.cpp'), '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True, timeout=10)

    def run_events(self, events, hands=False):
        text = ''.join(f'{op} {now} {flags} {rev} {json.dumps(arg, ensure_ascii=False)}\n'
                       for op, now, flags, rev, arg in events)
        result = subprocess.run([str(self.driver), '1000000000', '2000000000', str(int(hands))],
            input=text, capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        return [shlex.split(line) for line in result.stdout.splitlines()]

    def test_reference_state_transitions_and_reasons(self):
        now = [1_000_000_000]
        co = ArmCommandCoordinator(None, publisher_instance_id='co', router_zid='router-1',
            profile=dict(required_capability='simulation', active_sides=['left', 'right'],
                         bilateral_proposals=dict(run_id='run', execution_epoch=1)), clock=lambda: now[0])
        for role, name in (('source', 'src'), ('producer_arm', 'ik'), ('executor_arm', 'mujoco')):
            co.update_component(_status(role, name, now[0]))
        co.update_arm_state(_arm_state(now[0], co.robot.home_all))
        rng = random.Random(41)
        actions = ['tick', 'start', 'start', 'return', 'tick', 'start', 'shutdown', 'tick']
        actions += [rng.choice(('start', 'return', 'shutdown', 'tick', 'unsupported')) for _ in range(200)]
        events, expected = [], []
        for index, action in enumerate(actions):
            now[0] += 10_000_000
            flags = sum(int(value) << bit for bit, value in enumerate((
                co._domain_ready('source', now[0]), co._domain_ready('producer_arm', now[0]),
                co._domain_ready('executor_arm', now[0]), co._fresh(co._arm_state, now[0]),
                co._arm_at_home(now[0]), True, True, True, False, False, True)))
            if action == 'tick':
                co.tick(now_ns=now[0])
                accepted, result_reason = False, ''
            else:
                result = co.handle_intent(SimpleNamespace(action=action, sequence=index+1,
                    source='src', reason=action))
                accepted, result_reason = result.accepted, result.reason
            if co._commands_at_home():
                flags |= 1 << 8
            events.append((action, now[0], flags, index+1, action))
            expected.append((accepted, co.state.state, co.state.reason, result_reason))
        rows = self.run_events(events)
        self.assertEqual(len(rows), len(expected))
        for index, (row, exp) in enumerate(zip(rows, expected)):
            self.assertEqual((row[0]=='1', row[1], row[6], row[7]), exp, (index, events[index]))

    def test_fault_cannot_be_cleared_and_rearm_requires_ack_new_input(self):
        ready = 0b10111111111  # all start/Home gates, no stale proposal
        events = [('rearm', 100, ready, 1, '2'),
                  ('start', 101, ready, 1, ''),
                  ('start', 102, ready, 2, ''),
                  ('fault', 103, ready, 2, 'test fault'),
                  ('rearm', 104, ready, 3, '3'),
                  ('start', 105, ready, 3, '')]
        rows = self.run_events(events)
        self.assertEqual(rows[0][0:3], ['1', 'idle', '2'])
        self.assertEqual(rows[1][0], '0')
        self.assertEqual(rows[2][0:2], ['1', 'teleop'])
        self.assertEqual(rows[3][1], 'fault')
        self.assertEqual(rows[4][0], '0')
        self.assertEqual(rows[5][1], 'fault')
        self.assertEqual(rows[5][7], 'fault latched; restart required')

    def test_shutdown_waits_for_measured_and_commanded_home(self):
        ready = 0b10111111111
        rows = self.run_events([('shutdown', 100, ready, 1, ''),
            ('tick', 101, ready & ~(1<<4), 1, ''),
            ('tick', 102, ready & ~(1<<8), 1, ''),
            ('tick', 103, ready, 1, '')])
        self.assertEqual([row[1] for row in rows], ['returning']*3 + ['idle'])
        self.assertEqual([row[5] for row in rows], ['0', '0', '0', '1'])
