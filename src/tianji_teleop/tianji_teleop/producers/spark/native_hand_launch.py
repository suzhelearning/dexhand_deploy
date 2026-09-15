"""Cold-path configuration for the bilateral native Hand2 worker, not a driver."""
import os
from pathlib import Path
from ...config_loader import load_yaml

def hand_identities(instance_id, router):
    for value in (instance_id,router):
        if not isinstance(value,str) or not value.strip() or len(value)>240 or '\0' in value:
            raise ValueError('explicit native hand identities required')
    def identity(logical,suffix):return dict(logical=logical,instance=instance_id+suffix,router=router)
    return dict(manus_source_authority=identity('manus','-manus'),
        hand_authorities=dict(producer=identity('official_wuji_hand2','-hand'),
            left=identity('wuji_left','-sim'),right=identity('wuji_right','-sim')))

def build_hand_manifest(root, *, run_id, instance_id, router, stdout_fd,
                        right_glove=None, left_glove=None):
    for value in (run_id, instance_id, router):
        if not isinstance(value, str) or not value.strip() or len(value)>240 or '\0' in value:
            raise ValueError('explicit bounded native hand identities required')
    if type(stdout_fd) is not int or not 3<=stdout_fd<2**31:
        raise ValueError('native Manus descriptor must be >= 3')
    right_glove='' if right_glove is None else right_glove
    left_glove='' if left_glove is None else left_glove
    for glove in (right_glove,left_glove):
        if not isinstance(glove,str) or len(glove)>256 or '\0' in glove:
            raise ValueError('invalid explicit glove binding')
    if right_glove and right_glove==left_glove:
        raise ValueError('left and right glove bindings must differ')
    root=Path(root).resolve()
    python=root/'tools/wuji_hand_native/.pixi/envs/default/bin/python'
    launcher=root/'scripts/wuji_hand_native_scheduler_launcher.py'
    worker=root/'build/hand-native/tianji_hand_native_scheduler'
    for path in (python,launcher,worker):
        if not path.is_file():raise RuntimeError(f'missing native hand launch asset: {path}')
    for path in (python,worker):
        if not os.access(path,os.X_OK):raise RuntimeError(f'native hand executable required: {path}')
    config=load_yaml(root/'src/tianji_teleop/config/robot/wuji_hand2.yaml')
    from ...executors.wuji_hand2.config import WujiHandConfig
    WujiHandConfig.from_mapping(config)  # Same robot contract as the Python route.
    hand=dict(stdout_fd=stdout_fd,generation=1,timeout_ms=2000,capacity=1024,age_ns=200_000_000,
        right_glove=right_glove,left_glove=left_glove,
        worker_command=[str(python),str(launcher),'--native-scheduler',str(worker),
                        '--period-ns','5000000','--freshness-ns','200000000',
                        '--filter-continuity-ns','200000000',
                        '--output-capacity','1024','--startup-handshake'])
    for field,key in (('lower','lower_limits_rad'),('upper','upper_limits_rad'),
                      ('zero','zero_position_rad'),('zero_tolerance','zero_tolerance_rad')):
        hand[field]=[list(config[key]),list(config[key])]
    return dict(hand_runtime=hand,**hand_identities(instance_id,router))
