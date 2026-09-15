"""Opt-in full native joint chain, synthetic input only; briefly opens C++ Viewer."""
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from collections import deque
import h5py
import zenoh

ROOT=Path(__file__).resolve().parents[1]

@unittest.skipUnless(os.environ.get('NATIVE_JOINT_GATEWAY_TEST')=='1','opt-in graphical offline joint integration')
class NativeJointGatewayTest(unittest.TestCase):
    def test_real_workers_recording_and_rearm(self):
        from tianji_teleop.producers.spark.native_gateway import NativeGatewayProcess,build_native_gateway_manifest,write_native_gateway_manifest
        from tianji_teleop.producers.spark.native_hand_launch import build_hand_manifest
        from tianji_teleop.hand_tracking.native_manus_process import NativeManusProcess
        from tianji_teleop.recording.native_owner import NativeRecordingOwner
        def free_port(kind):
            with socket.socket(socket.AF_INET,kind) as s:s.bind(('127.0.0.1',0));return s.getsockname()[1]
        port=free_port(socket.SOCK_STREAM);endpoint=f'tcp/127.0.0.1:{port}'
        router=subprocess.Popen([str(ROOT/'vendor/zenoh-router/zenohd'),'-l',endpoint,'--no-multicast-scouting'],
            stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        session=None
        try:
            deadline=time.monotonic()+5
            while True:
                try:
                    with socket.create_connection(('127.0.0.1',port),timeout=.1):break
                except OSError:
                    if time.monotonic()>deadline:raise
                    time.sleep(.02)
            cfg=zenoh.Config();cfg.insert_json5('mode','"client"');cfg.insert_json5('connect/endpoints',json.dumps([endpoint]))
            cfg.insert_json5('scouting/multicast/enabled','false');session=zenoh.open(cfg)
            zid=str(session.info.routers_zid()[0])
            spec=importlib.util.spec_from_file_location('joint_cli',ROOT/'scripts/vr_manus_live.py')
            cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
            for prefix,backend,xz in (('spark','spark_upper_qpoases_headroom_feedforward_velocity_qp',False),
                ('mapped_palm','pico_ee_mapped_corrected_palm_velocity_qp',False),
                ('mapped_palm','pico_ee_mapped_corrected_palm_velocity_qp',True)):
                with self.subTest(prefix=prefix,xz=xz),tempfile.TemporaryDirectory() as folder:
                    _,resolved=cli.resolve(['--disable-hands','--scheduler-backend','cpp','--ik-backend',backend])
                    resolved['config']['hands_enabled']=True
                    if xz: resolved.update(mapped_palm_height_calibration=True,mapped_palm_xz_calibration=True)
                    udp=free_port(socket.SOCK_DGRAM);path=Path(folder)/'joint.h5'
                    owner=NativeRecordingOwner(path,root=ROOT,router_zid=zid,robot_model=prefix,metadata={})
                    driver=NativeManusProcess(command=[sys.executable,str(ROOT/'tests/native_joint_input_fixture.py'),str(udp),'150'],cwd=ROOT)
                    gateway=None
                    history=deque(maxlen=12)
                    try:
                        manifest=build_native_gateway_manifest(ROOT,resolved,run_id='joint-offline',instance_id='fixture',
                            router_zid=zid,tjvr_bind='127.0.0.1',tjvr_port=udp,native_hands=True)
                        manifest.update(build_hand_manifest(ROOT,run_id='joint-offline',instance_id='fixture',router=zid,stdout_fd=driver.fileno()))
                        manifest.update(publication_backend='cpp',viewer_backend='cpp',diagnostic_transport='summary',
                            publication_endpoint=endpoint,recording_fd=owner.fileno(),recording_origin_ns=owner.origin_ns)
                        mp=write_native_gateway_manifest(Path(folder),manifest)
                        gateway=NativeGatewayProcess(ROOT/'build/control-native/tianji_native_session_gateway',mp,prefix=prefix,
                            publication_backend='cpp',viewer_backend='cpp',diagnostic_transport='summary',
                            recording_fd=owner.fileno(),manus_fd=driver.fileno(),frame_capacity=4096)
                        gateway.start();driver.mark_transferred();owner.release_descriptor()
                        epoch=1
                        def action(name):
                            nonlocal epoch
                            try:ident=gateway.send_action(name,next_epoch=epoch+1 if name=='rearm' else 0)
                            except (OSError,RuntimeError) as exc:
                                frames=[]
                                try:
                                    while (frame:=gateway.get_frame(.01)) is not None:frames.append((frame.kind,frame.payload))
                                except RuntimeError as failure:frames.append(('failure',str(failure)))
                                raise RuntimeError(str(exc)+' '+gateway.stderr+' '+repr(frames)) from exc
                            end=time.monotonic()+10
                            while time.monotonic()<end:
                                try:frame=gateway.get_frame(.1)
                                except RuntimeError as exc:
                                    raise RuntimeError(f'{exc}; exit={gateway.process.poll()}; stderr={gateway.stderr}') from exc
                                if frame:history.append((frame.kind,frame.payload))
                                if frame and frame.kind=='summary':epoch=frame.payload['state_epoch']
                                if frame and frame.kind=='complete':raise RuntimeError('unexpected completion '+repr(frame.payload)+' '+gateway.stderr)
                                if frame and frame.kind=='reply' and frame.payload['id']==ident:return frame.payload['accepted']
                                if gateway.failure:raise RuntimeError(gateway.failure+' '+gateway.stderr)
                            raise RuntimeError('no action reply: '+name+' '+gateway.stderr)
                        if xz:
                            end=time.monotonic()+10
                            while not action('calibrate'):
                                if time.monotonic()>end:raise RuntimeError('calibration readiness timed out '+gateway.stderr)
                                time.sleep(.1)
                            time.sleep(3)
                        end=time.monotonic()+10
                        while not action('start'):
                            if time.monotonic()>end:raise RuntimeError('start readiness timed out '+gateway.stderr)
                            time.sleep(.1)
                        time.sleep(2);self.assertTrue(action('return'))
                        end=time.monotonic()+10
                        while not action('start'):
                            if time.monotonic()>end:raise RuntimeError('automatic rearm timed out '+gateway.stderr)
                            time.sleep(.1)
                        time.sleep(2);self.assertTrue(action('return'))
                        end=time.monotonic()+10
                        while not action('rearm'):
                            if time.monotonic()>end:raise RuntimeError('manual rearm timed out '+gateway.stderr)
                            time.sleep(.1)
                        end=time.monotonic()+10
                        while not action('start'):
                            if time.monotonic()>end:raise RuntimeError('post-manual-rearm start timed out '+gateway.stderr)
                            time.sleep(.1)
                        time.sleep(2);self.assertTrue(action('shutdown'))
                        end=time.monotonic()+30
                        completed=None
                        while time.monotonic()<end:
                            frame=gateway.get_frame(.1)
                            if frame:history.append((frame.kind,frame.payload))
                            if frame and frame.kind=='complete':
                                completed=frame.payload;break
                        self.assertIsNotNone(completed,repr(list(history)))
                        self.assertTrue(completed['complete'],repr(completed))
                        self.assertEqual(gateway.process.wait(timeout=5),0,gateway.stderr)
                        owner.close(complete=True)
                    finally:
                        if sys.exc_info()[0]:
                            print('gateway diagnostics',list(history),gateway.stderr if gateway else '',file=sys.stderr)
                        if gateway:gateway.close(graceful=False)
                        driver.close();owner.close(complete=False)
                        if sys.exc_info()[0] and path.exists():
                            try:
                                with h5py.File(path) as failed:
                                    audits=[json.loads(p) for k,p in zip(failed['meta/dual_audit/kind'].asstr()[:],
                                        failed['meta/dual_audit/payload_json'].asstr()[:]) if k=='native_cycle']
                                    results=[r for c in audits for r in c.get('native_hand_results',[])]
                                    print('hand tail',[(r['reason'],r['result']['phase'],
                                        (r['observed_timestamp_ns']-r['result']['input_timestamp_ns'])/1e6)
                                        for r in results[-5:]],file=sys.stderr)
                            except (OSError,KeyError):pass
                    with h5py.File(path) as file:
                        self.assertTrue(file.attrs['complete'])
                        self.assertGreater(len(file['raw/manus_callbacks/time_ns']),0)
                        for side in ('left','right'):
                            self.assertGreater(len(file[f'joint/command/hand/{side}/time_ns']),0)
                        cycles=[json.loads(p) for k,p in zip(file['meta/dual_audit/kind'].asstr()[:],
                            file['meta/dual_audit/payload_json'].asstr()[:]) if k=='native_cycle']
                        self.assertTrue(any(c.get('native_hand_results') for c in cycles))
                        if xz:
                            attempts=[c['native'] for c in cycles if c.get('native')]
                            self.assertTrue(attempts, 'missing recorded native attempts')
                            self.assertTrue(any('target_x_offsets_m' in a and 'target_height_offsets_m' in a for a in attempts),
                                            'X/Z offsets missing from native HDF5 audit')
                        results=[r for c in cycles for r in c.get('native_hand_results',[])]
                        self.assertGreater(len(file['raw/manus_callbacks/time_ns']),len(results))
                        self.assertTrue(any(b['result']['input_sequence']>a['result']['input_sequence']+1
                            for a,b in zip(results,results[1:])), 'overload must coalesce compute, not raw recording')
        finally:
            if session:session.close()
            router.terminate()
            try:router.wait(timeout=5)
            except subprocess.TimeoutExpired:router.kill();router.wait(timeout=5)
