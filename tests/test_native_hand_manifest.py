from pathlib import Path
import unittest
from tianji_teleop.config_loader import load_yaml

ROOT=Path(__file__).resolve().parents[1]

class NativeHandManifestTest(unittest.TestCase):
    def test_hand_liveliness_uses_same_manifest_authorities(self):
        from contextlib import ExitStack
        from unittest.mock import Mock,patch
        from tianji_teleop.producers.spark.native_live_runner import _declare_native_authorities
        from tianji_teleop.producers.spark.native_hand_launch import hand_identities
        config={key:dict(logical=key,instance=key,router='router') for key in
            ('source_authority','producer_authority','coordinator_authority','executor_authority')}
        config.update(hand_identities('instance','router'))
        with patch('tianji_teleop.zenoh_util.declare_component_liveliness',side_effect=lambda *a,**k:Mock()) as declare:
            with ExitStack() as stack:
                tokens=_declare_native_authorities(Mock(),config,stack)
                self.assertEqual(len(tokens),8)
                self.assertIn('tj/live/source/manus/instance-manus',tokens)
                self.assertIn('tj/live/producer/hand/official_wuji_hand2/instance-hand',tokens)
                self.assertIn('tj/live/executor/hand/wuji_left/instance-sim',tokens)
                self.assertIn('tj/live/executor/hand/wuji_right/instance-sim',tokens)
                self.assertEqual(declare.call_count,8)

    def test_arm_manifest_requires_explicit_native_hand_opt_in(self):
        import importlib.util
        from tianji_teleop.producers.spark.native_gateway import build_native_gateway_manifest
        spec=importlib.util.spec_from_file_location('hand_manifest_cli',ROOT/'scripts/vr_manus_live.py')
        cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
        _,resolved=cli.resolve(['--disable-hands','--scheduler-backend','cpp'])
        resolved['config']['hands_enabled']=True
        kwargs=dict(run_id='run',instance_id='instance',router_zid='router',tjvr_bind='127.0.0.1',tjvr_port=15000)
        with self.assertRaises(ValueError):build_native_gateway_manifest(ROOT,resolved,**kwargs)
        row=build_native_gateway_manifest(ROOT,resolved,native_hands=True,**kwargs)
        self.assertEqual(row['run_id'],'run')
    def test_uses_existing_limits_and_native_worker(self):
        from tianji_teleop.producers.spark.native_hand_launch import build_hand_manifest
        result=build_hand_manifest(ROOT,run_id='run',instance_id='instance',router='router',stdout_fd=10,
            right_glove='right-id',left_glove='left-id')
        config=load_yaml(ROOT/'src/tianji_teleop/config/robot/wuji_hand2.yaml')
        hand=result['hand_runtime']
        self.assertEqual(hand['stdout_fd'],10)
        self.assertEqual(hand['lower'],[config['lower_limits_rad']]*2)
        self.assertEqual(hand['zero'],[config['zero_position_rad']]*2)
        self.assertEqual(hand['right_glove'],'right-id')
        self.assertIn('--startup-handshake',hand['worker_command'])
        self.assertIn('--native-scheduler',hand['worker_command'])
        self.assertEqual(result['manus_source_authority']['instance'],'instance-manus')
        self.assertEqual(result['hand_authorities']['left']['instance'],'instance-sim')
        self.assertEqual(result['hand_authorities']['right']['instance'],'instance-sim')
        for kwargs in (dict(stdout_fd=True),dict(stdout_fd=2),dict(right_glove='same',left_glove='same'),
                       dict(instance_id=''),dict(router='x\0y')):
            with self.assertRaises(ValueError):
                build_hand_manifest(ROOT,**(dict(run_id='run',instance_id='instance',router='router',stdout_fd=10)|kwargs))
