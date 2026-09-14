"""One explicit selection of isolated native worker, model, configuration and identity."""
from ...hand_tracking.input_modes import SPARK_BACKEND, MAPPED_PALM_BACKEND
from ...hand_tracking.spark_worker_client import SparkWorkerClient
from ...hand_tracking.mapped_palm_worker_client import MappedPalmWorkerClient


def bilateral_assets(root, backend=SPARK_BACKEND):
    assets = root / 'src/tianji_teleop'
    if backend == SPARK_BACKEND:
        return dict(worker=root / 'build/spark-native/spark_native_worker',
            lock=root / 'tools/spark_native/pixi.lock',
            config=assets / 'config/producers/spark_reference.yaml',
            model=assets / 'assets/spark/marvin_m6_wuji2.xml',
            urdf=assets / 'assets/spark/marvin_m6_s_ccs_696_v4_local.urdf',
            client=SparkWorkerClient, producer_id='ik_spark_headroom')
    if backend == MAPPED_PALM_BACKEND:
        return dict(worker=root / 'build/mapped-palm-native/mapped_palm_native_worker',
            lock=root / 'tools/mapped_palm_native/pixi.lock',
            config=assets / 'src/ik/mapped_palm/config/bandwidth.yaml',
            model=assets / 'assets/mapped_palm/marvin_m6_wuji2.xml',
            urdf=assets / 'assets/mapped_palm/marvin_m6_s_ccs_696_v4_local.urdf',
            manifest=assets / 'src/ik/mapped_palm/source_manifest.json',
            client=MappedPalmWorkerClient, producer_id='ik_mapped_palm')
    raise ValueError('unsupported bilateral reference backend: ' + str(backend))
