from pathlib import Path
import json
import subprocess
import tempfile
import unittest
from tianji_teleop.protocol.messages import HandJointCommand, HandJointState, HAND_JOINT_NAMES

ROOT=Path(__file__).resolve().parents[1]


class HandPublicationTest(unittest.TestCase):
    def test_existing_wire_contract_and_no_fabricated_command(self):
        with tempfile.TemporaryDirectory() as directory:
            binary=Path(directory)/'publication'
            subprocess.run(['c++','-std=c++17','-pthread','-O2','-Wall','-Wextra','-Werror',
                            str(ROOT/'tests/cpp/session_hand_publication.cpp'),'-o',str(binary)],check=True)
            authorities={'producer':{'logical':'official_wuji_hand2','instance':'hand','router':'router'},
                         'left':{'logical':'wuji_left','instance':'sim','router':'router'},
                         'right':{'logical':'wuji_right','instance':'sim','router':'router'}}
            for phase in ('idle','teleop','returning','fault'):
                enabled=phase in ('teleop','returning')
                p=subprocess.run([str(binary)],input=json.dumps(dict(phase=phase,command=enabled,authorities=authorities))+'\n',
                                 text=True,capture_output=True,check=True)
                rows=dict(json.loads(p.stdout))
                self.assertEqual(len(rows),3 if enabled else 2)
                for side in ('left','right'):
                    expected=HandJointState(1,7,1000,'wuji_'+side,side,list(HAND_JOINT_NAMES[side]),[0.]*20,None,'sim','router')
                    self.assertEqual(rows['tianji/state/hand/'+side],expected.to_dict())
                if enabled:
                    expected=HandJointCommand(1,7,1000,'official_wuji_hand2','left',list(HAND_JOINT_NAMES['left']),[0.]*20,'hand','router')
                    self.assertEqual(rows['tianji/command/hand/left'],expected.to_dict())
            authorities['left']['router']='wrong'
            p=subprocess.run([str(binary)],input=json.dumps(dict(phase='teleop',authorities=authorities))+'\n',text=True,capture_output=True)
            self.assertNotEqual(p.returncode,0)
