from pathlib import Path
import subprocess
import tempfile
import unittest
import sys
import json

ROOT=Path(__file__).resolve().parents[1]

class NativeXZCalibrationTest(unittest.TestCase):
    def test_shell_rejects_non_cpp_before_device_start(self):
        command=['bash',str(ROOT/'scripts/run_session.sh'),'--profile','pico_vr_manus_sim',
                 '--mapped-palm-xz-calibration','--ik-backend','pico_ee_mapped_corrected_palm_velocity_qp']
        result=subprocess.run(command,text=True,capture_output=True)
        self.assertEqual(result.returncode,2,result.stderr)
        self.assertIn('X/Z',result.stderr)
        self.assertIn('C++',result.stderr)

    def test_cli_isolated_full_cpp_route(self):
        with tempfile.TemporaryDirectory() as folder:
            command=[sys.executable,str(ROOT/'scripts/vr_manus_live.py'),'--check',
                '--disable-hands','--mapped-palm-xz-calibration',
                '--ik-backend','pico_ee_mapped_corrected_palm_velocity_qp',
                '--scheduler-backend','cpp','--publication-backend','cpp',
                '--viewer','--viewer-backend','cpp','--recording-adapter','cpp',
                '--record',str(Path(folder)/'test.h5')]
            result=subprocess.run(command,text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertTrue(json.loads(result.stdout)['mapped_palm_xz_calibration'])
            common=subprocess.run(command+['--mapped-palm-common-x-reference'],text=True,capture_output=True)
            self.assertEqual(common.returncode,0,common.stderr)
            self.assertTrue(json.loads(common.stdout)['mapped_palm_common_x_reference'])
            rejected=subprocess.run([v for v in command if v!='--mapped-palm-xz-calibration']+
                                    ['--mapped-palm-common-x-reference'],text=True,capture_output=True)
            self.assertNotEqual(rejected.returncode,0)
            self.assertIn('requires --mapped-palm-xz-calibration',rejected.stderr)
            for extra in (['--mapped-palm-height-calibration'],['--scheduler-backend','python'],
                          ['--ik-backend','spark_upper_qpoases_headroom_feedforward_velocity_qp']):
                result=subprocess.run(command+extra,text=True,capture_output=True)
                self.assertNotEqual(result.returncode,0,result.stdout)

    def test_stable_xz_and_failed_retry(self):
        with tempfile.TemporaryDirectory() as folder:
            exe=Path(folder)/'test'
            subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror',
                str(ROOT/'tests/cpp/native_xz_calibration.cpp'),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
