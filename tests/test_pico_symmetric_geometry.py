import importlib.util
import sys
import tempfile
import unittest
from unittest.mock import patch
from dataclasses import dataclass
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
SCRIPTS=ROOT/'vendor/pico_tracker/src/pico_bridge/scripts'

class SymmetricGeometryTest(unittest.TestCase):
    def test_profile_integrity_and_default_isolation(self):
        sys.path.insert(0,str(SCRIPTS))
        self.addCleanup(lambda:sys.path.remove(str(SCRIPTS)))
        import pico_symmetric_geometry as module
        @dataclass(frozen=True)
        class Geometry:
            upper_arm_length_m: float
            forearm_length_m: float
        geometries={'left':Geometry(.2705,.1762),'right':Geometry(.2763,.2475)}
        self.assertIs(module.apply_profile(geometries,{'left':'','right':''})[0],geometries)
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);source=root/'original';source.mkdir();output=root/'symmetric'
            for name in module.FILES:
                side='left' if '_left_' in name else 'right'
                g=geometries[side]
                (source/name).write_text(json.dumps(vars(g)))
            original={name:(source/name).read_bytes() for name in module.FILES}
            # Unit-test policy plumbing independently of the existing artifact validator.
            with patch.object(module,'validate_artifact') as validate:
                module.create_profile(source,output)
                self.assertEqual(validate.call_count,2)
                with self.assertRaises(FileExistsError):module.create_profile(source,output)
            paths={s:str(output/f'pico_{s}_arm_geometry.yaml') for s in geometries}
            effective,policy=module.apply_profile(geometries,paths)
            for side in geometries:
                self.assertEqual(effective[side],Geometry(.2763,.2475))
            self.assertEqual(geometries['left'].forearm_length_m,.1762)
            for name,data in original.items():
                self.assertEqual((source/name).read_bytes(),data)
                self.assertEqual((output/name).read_bytes(),data)
            policy_path=output/module.POLICY_FILE
            saved=policy_path.read_text()
            for field,value in [('effective_lengths_m',{'upper_arm':.4,'forearm':.3}),
                                ('original_lengths_m',{}),('policy','unknown')]:
                invalid=json.loads(saved);invalid[field]=value
                policy_path.write_text(json.dumps(invalid))
                with self.subTest(field=field),self.assertRaises(ValueError):
                    module.apply_profile(geometries,paths)
            policy_path.write_text(saved)
            with self.assertRaises(ValueError):
                module.apply_profile({'left':None,'right':geometries['right']},paths)
            with self.assertRaises(ValueError):
                module.apply_profile(geometries,{**paths,'right':str(source/'pico_right_arm_geometry.yaml')})
            policy_path.write_text('{"complete": false}')
            with self.assertRaisesRegex(ValueError,'incomplete'):module.apply_profile(geometries,paths)
            policy_path.write_text(saved)
            (output/module.FILES[0]).write_text('tampered')
            with self.assertRaisesRegex(ValueError,'fingerprint'):module.apply_profile(geometries,paths)
            with patch.object(module,'validate_artifact',side_effect=ValueError('invalid calibration')):
                with self.assertRaisesRegex(ValueError,'invalid calibration'):
                    module.create_profile(source,root/'invalid')
                self.assertFalse((root/'invalid').exists())

    def test_profile_and_actual_reconstruction(self):
        sys.path.insert(0,str(SCRIPTS))
        self.addCleanup(lambda:sys.path.remove(str(SCRIPTS)))
        import pico_symmetric_geometry as module
        from pico_palm_skeleton_filter_core import SideCalibration,correct_side
        lengths=module.symmetric_lengths({'left':(.2705,.1762),'right':(.2763,.2475)})
        self.assertEqual(lengths,(.2763,.2475))
        with self.assertRaises(ValueError):module.symmetric_lengths({'left':(.1,.2),'right':(.3,.2)})
        positions=np.zeros((24,3));rotations=np.tile([0.,0.,0.,1.],(24,1))
        for side,indices,y in [('left',(16,18,20,22),.2),('right',(17,19,21,23),-.2)]:
            shoulder,elbow,wrist,hand=indices
            positions[shoulder]=[0,y,1];positions[elbow]=[.2,y,.8]
            positions[wrist]=[.4,y,1];positions[hand]=[.43,y,1]
            calibration=SideCalibration(lengths[0],lengths[1],np.zeros(3),np.array([0.,0.,0.,1.]))
            result=correct_side(positions,rotations,positions[hand],rotations[hand],calibration,side,
                                wrist_to_palm_distance_m=.03)
            output=result.positions
            self.assertAlmostEqual(np.linalg.norm(output[elbow]-output[shoulder]),lengths[0],places=9)
            self.assertAlmostEqual(np.linalg.norm(output[wrist]-output[elbow]),lengths[1],places=9)
