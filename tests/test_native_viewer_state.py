import json
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest

import numpy as np

from tests.test_native_raw_input import compile_driver
from tests.test_reference_tjvr_receiver import packet
from tianji_teleop.hand_tracking.native_binary_results import _SPECS, _STRUCTS, decode_result
from tianji_teleop.hand_tracking.reference_tjvr import parse_reference_tjvr_packet
from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReceivedTjvrFrame
from tianji_teleop.executors.mujoco.spark_overlay import SparkOverlay
from tianji_teleop.executors.mujoco.mapped_palm_overlay import MappedPalmOverlay

ROOT=Path(__file__).resolve().parents[1]

def result_wire(prefix,tick,seq,epoch,invalid=None):
    values=[]
    for key,fmt in _SPECS[prefix]:
        if fmt.endswith('d'):
            count=int(fmt[:-1] or '1')
            value=[0.]*count
            if key.endswith('target_quaternion_xyzw'): value=[0.,0.,0.,1.]
            if key=='target_height_offsets_m': value=[-.2,-.3]
            if invalid=='quaternion' and key.endswith('target_quaternion_xyzw'): value=[0.]*4
            if invalid=='height' and key=='target_height_offsets_m': value=[2.,3.]
            if invalid and key.endswith('target_position'): value=[9.,9.,9.]
            values.extend(value)
        else:
            values.append(dict(tick_id=tick,timestamp_ns=1000,applied_sequence=seq,applied_epoch=epoch,
                               input_live=1,_height_present=1).get(key,0))
    return struct.pack('<4sBBH',b'TJBR',1,1 if prefix=='spark' else 2,_STRUCTS[prefix].size)+_STRUCTS[prefix].pack(*values)

class ViewerStateTest(unittest.TestCase):
    def test_overlay_geometry_matches_existing_python(self):
        self.assertTrue((ROOT/'native/control/viewer_state.hpp').is_file())
        with tempfile.TemporaryDirectory() as directory:
            binary=Path(directory)/'state'
            compile_driver('viewer_state_fixture.cpp',binary)
            for prefix in ('spark','mapped_palm'):
                overlay=SparkOverlay('source') if prefix=='spark' else MappedPalmOverlay('source')
                raw=packet(1)
                observation=parse_reference_tjvr_packet(raw,receiver_instance_id='source',receiver_frame_sequence=1,received_timestamp_ns=1000)
                frame=observation.frame
                rows=[dict(raw=list(raw),sequence=1,received_ns=1000,now_ns=1001)]
                overlay.ingest_raw(observation)
                expected=[overlay.geometry(1001)]
                for tick,active,now,packet_present,epoch,invalid in ((1,True,1001,True,1,None),
                      (2,True,1001,False,1,'quaternion'),
                      (2,True,1001,False,1,'height' if prefix=='mapped_palm' else 'quaternion'),
                      (1,False,1001,False,1,None),(2,True,50001001,False,1,None),
                      (3,True,200001001,False,1,None),(4,False,1001,False,1,None),(1,True,1001,False,2,None)):
                    wire=result_wire(prefix,tick,frame.sequence,frame.tracking_epoch,invalid)
                    native=decode_result(wire,prefix)
                    row=dict(wire=list(wire),epoch=epoch,active=active,now_ns=now)
                    attempt=None
                    if packet_present:
                        row.update(packet=list(raw),received_ns=1000)
                        attempt=dict(tick_id=tick,timestamp_ns=1000,sample=ReceivedTjvrFrame(observation).to_dict())
                    if prefix=='spark': overlay.ingest_native(native,execution_epoch=epoch)
                    else: overlay.ingest_cycle(native,attempt,execution_epoch=epoch,active=active)
                    rows.append(row);expected.append(overlay.geometry(now))
                result=subprocess.run([str(binary),prefix],input='\n'.join(map(json.dumps,rows)),text=True,capture_output=True,timeout=10)
                self.assertEqual(result.returncode,0,result.stderr)
                actual=list(map(json.loads,result.stdout.splitlines()))
                self.assertEqual(len(actual),len(expected))
                for got,(markers,bones) in zip(actual,expected):
                    np.testing.assert_allclose(got['bones'],np.asarray(bones).tolist(),atol=1e-12)
                    self.assertEqual(len(got['markers']),len(markers))
                    for a,b in zip(got['markers'],markers):
                        self.assertEqual(a['label'],b['label'])
                        for field in ('position','color'): np.testing.assert_allclose(a[field],b[field],atol=1e-12)
                        if b['rotation'] is None: self.assertIsNone(a['rotation'])
                        else: np.testing.assert_allclose(np.asarray(a['rotation']).reshape(3,3),b['rotation'],atol=1e-12)
