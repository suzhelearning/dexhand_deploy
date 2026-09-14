"""Bilateral backend factory: native dependencies remain in their own process."""
from pathlib import Path
import yaml

from ...coordination.arm_command_coordinator import ArmRobotConfig
from ...hand_tracking.input_modes import InputMode, SPARK_BACKEND, MAPPED_PALM_BACKEND, validate_ik_input
from ...hand_tracking.mapped_palm_worker_client import MappedPalmWorkerClient
from ...hand_tracking.spark_worker_client import SparkWorkerClient
from ...coordination.bilateral_robot import BilateralArmRobotConfig


def create_bilateral_backend(name, input_mode, *, required_capability, **worker_options):
    if name not in (SPARK_BACKEND, MAPPED_PALM_BACKEND):
        raise ValueError(f'unknown bilateral backend: {name}; single-arm backends use their existing factory')
    if not isinstance(input_mode, InputMode):
        raise ValueError('explicit resolved input mode required')
    validate_ik_input(input_mode, name)
    if required_capability != 'simulation':
        raise ValueError('SPARK bilateral backend is simulation-only until separate real validation')
    client = SparkWorkerClient if name == SPARK_BACKEND else MappedPalmWorkerClient
    return client(required_capability=required_capability, **worker_options)


def reference_robot_config(config_path, urdf_path, base_arm_config):
    """Explicit reference initial state and URDF bounds; no environment writes."""
    config = yaml.safe_load(Path(config_path).read_text())
    controller = config['controller']
    if controller.get('initial_posture_enabled') is not True:
        raise ValueError('SPARK session requires an explicit reference initial posture')
    # Check the canonical joint naming, but never reuse shared bounds/Home.
    ArmRobotConfig.load(base_arm_config)
    return BilateralArmRobotConfig.from_urdf(urdf_path, controller['initial_left_q_rad'],
                                            controller['initial_right_q_rad'])
