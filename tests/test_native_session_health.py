from dataclasses import replace
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from tests.test_arm_coordinator import _status, _arm_state
from tianji_teleop.coordination.arm_command_coordinator import ArmCommandCoordinator

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which('c++'), 'C++ compiler required')
class NativeSessionHealthTest(unittest.TestCase):
    def compile(self, source, output):
        subprocess.run(['c++','-std=c++17','-pthread','-O2','-Wall','-Wextra','-Werror',
            str(ROOT/'tests/cpp'/source),'-o',str(output)],check=True)

    def test_cpp_gates_and_runtime(self):
        with tempfile.TemporaryDirectory(prefix='native-health-') as directory:
            for source in ('test_session_health.cpp','test_session_health_runtime.cpp'):
                executable=Path(directory)/source
                self.compile(source,executable)
                subprocess.run([str(executable)],check=True,timeout=10)

    def test_reference_freshness_home_and_readiness(self):
        now=[1_000_000_000]
        co=ArmCommandCoordinator(None,publisher_instance_id='co',router_zid='router-1',clock=lambda:now[0])
        self.addCleanup(co.close)
        lines=[' '.join(map(str,co.robot.home_all))]
        expected=[]
        roles=(('source','src'),('producer_arm','ik'),('executor_arm','mujoco'))
        for index in range(120):
            now[0]+=50_000_000
            if index%10<3:
                role=index%10
                ready=index%20<10
                co.update_component(replace(_status(*roles[role],now[0]),sequence=index+1,ready=ready))
                lines.append(f'status {now[0]} {index+1} {role} {int(ready)}')
            elif index%10==3:
                delta=(0.,.005,.1)[(index//10)%3]
                q=list(co.robot.home_all); q[0]+=delta
                co.update_arm_state(replace(_arm_state(now[0],q),sequence=index+1,publisher_instance_id='mujoco-instance'))
                lines.append(f'feedback {now[0]} {index+1} {delta}')
            else:
                # No health traffic: expire both status and feedback independently.
                if index%10==9: now[0]+=1_000_000_001
                lines.append(f'poll {now[0]} 0')
            fresh=co._fresh(co._arm_state,now[0])
            exact=fresh and co._arm_state.value.position_rad==list(co.robot.home_all)
            expected.append([int(co._domain_ready(role,now[0])) for role,_ in roles]+
                            [int(fresh),int(co._arm_at_home(now[0])),int(exact)])
        with tempfile.TemporaryDirectory(prefix='native-health-parity-') as directory:
            executable=Path(directory)/'driver'
            self.compile('native_health_driver.cpp',executable)
            result=subprocess.run([str(executable)],input='\n'.join(lines)+'\n',
                text=True,capture_output=True,check=True,timeout=10)
        self.assertEqual([list(map(int,row.split())) for row in result.stdout.splitlines()],expected)
