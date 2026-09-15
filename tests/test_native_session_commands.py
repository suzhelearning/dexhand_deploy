from pathlib import Path
from dataclasses import replace
import shlex
import shutil
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
import json

from tests.test_arm_coordinator import _status, _arm_state
from tests.test_bilateral_proposal import pair
from tianji_teleop.coordination.arm_command_coordinator import ArmCommandCoordinator

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which('c++'), 'C++ compiler required')
class NativeSessionCommandsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix='native-command-session-')
        cls.addClassCleanup(cls.directory.cleanup)
        cls.driver = Path(cls.directory.name) / 'driver'
        subprocess.run(['c++', '-std=c++17', '-O2', '-ffp-contract=off', '-Wall', '-Wextra', '-Werror',
            str(ROOT / 'tests/cpp/native_command_session_driver.cpp'), '-o', str(cls.driver)], check=True)

    def test_paired_gate_and_atomic_fault(self):
        executable = Path(self.directory.name) / 'gates'
        subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
            str(ROOT / 'tests/cpp/test_session_commands.cpp'), '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True, timeout=10)

    def test_reference_commands_home_and_fault(self):
        for clipping in (False, True):
            with self.subTest(clipping=clipping):
                self.compare(clipping)

    def compare(self, clipping):
        now = [1_000_000_000]
        settings = ArmCommandCoordinator._coordinator_config(None)
        settings.update(maximum_command_step_rad=.02, command_step_time_window_s=.05,
                        command_step_clipping_enabled=clipping)
        co = ArmCommandCoordinator(None, publisher_instance_id='co', router_zid='router-1',
            coordinator_config=settings, profile=dict(required_capability='simulation',
            active_sides=['left', 'right'], bilateral_proposals=dict(run_id='run-1', execution_epoch=1)),
            clock=lambda: now[0])
        self.addCleanup(co.close)
        home = co.robot.home_all
        lower = list(co.robot.limits('left')[0]) + list(co.robot.limits('right')[0])
        upper = list(co.robot.limits('left')[1]) + list(co.robot.limits('right')[1])
        lines = [' '.join(map(str, list(home) + lower + upper))]
        expected = []
        expected_receipts = []

        def step(op, tick=0, delta=0.):
            now[0] += 5_000_000
            for role, name in (('source','src'),('producer_arm','ik'),('executor_arm','mujoco')):
                co.update_component(replace(_status(role,name,now[0]),sequence=len(lines)))
            co.update_arm_state(replace(_arm_state(now[0],home),sequence=len(lines)))
            receipt = -1
            if op == 'proposal':
                value = pair(tick)
                for side in ('left','right'):
                    value[side]['timestamp_ns'] = now[0]
                    value[side]['position_rad'] = [v+delta for v in getattr(co.robot,side+'_home_rad')]
                accepted = co.update_bilateral_proposal(value)
                positions = value['left']['position_rad'] + value['right']['position_rad']
                lines.append(f'proposal {now[0]} {tick} {now[0]} ' + ' '.join(map(str,positions)))
            elif op == 'tick':
                had_pending = co._pending_bilateral is not None
                co.tick(now_ns=now[0])
                accepted = False
                if had_pending:
                    receipt = int(co.last_bilateral_receipt['accepted'])
                    expected_receipts.append(co.last_bilateral_receipt)
                lines.append(f'tick {now[0]}')
            else:
                result = co.handle_intent(SimpleNamespace(action=op,sequence=len(lines),source='src',reason=op))
                accepted = result.accepted
                lines.append(f'{op} {now[0]}')
            expected.append((accepted,co.state.state,co.state.reason,receipt,
                             co._safe_command['left'] + co._safe_command['right']))

        step('start')
        for tick in range(1,41):
            step('proposal',tick,.001*tick)
            step('tick')
        step('return')
        for _ in range(405):
            step('tick')
        step('start')
        step('proposal',41,.01)
        step('tick')
        step('proposal',42,99.)
        step('tick')
        step('start')
        result = subprocess.run([str(self.driver),'clip' if clipping else 'direct'],
            input='\n'.join(lines)+'\n',text=True,capture_output=True,timeout=10,check=True)
        rows = result.stdout.splitlines()
        self.assertEqual([json.loads(line) for line in result.stderr.splitlines()],expected_receipts)
        self.assertEqual(len(rows),len(expected))
        for index,(line,exp) in enumerate(zip(rows,expected)):
            row = shlex.split(line)
            self.assertEqual((row[0]=='1',row[1],row[2],int(row[3])), exp[:4], index)
            self.assertEqual(list(map(float,row[4:])),exp[4],index)
