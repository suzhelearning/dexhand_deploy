"""Legacy Tianji robot configuration loader used by mocap/replay paths."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import yaml


@dataclass
class TianjiConfig:
    """Read-only subset of the historical Tianji output configuration."""

    raw: Dict[str, Any]
    robot_ip: str
    init_joints: Dict[str, np.ndarray]
    init_pos: Dict[str, np.ndarray]
    init_rot: Dict[str, np.ndarray]
    init_quat: Dict[str, np.ndarray]
    world_to_chest_quat: Dict[str, np.ndarray]
    world_to_chest_trans: Dict[str, np.ndarray]
    arm_init_pos: Dict[str, np.ndarray]
    arm_init_quat: Dict[str, np.ndarray]
    pico_to_robot: np.ndarray
    mocap_to_robot: np.ndarray
    zsp_type: int
    default_zsp_para: Dict[str, list]
    zsp_angle: float
    dgr: list
    tracker_serial_map: Dict[str, str]
    config_path: Path

    @classmethod
    def load(
        cls,
        config_path: Optional[str | os.PathLike[str]] = None,
        *,
        use_ros: bool = True,
    ) -> "TianjiConfig":
        path = Path(config_path) if config_path is not None else Path(cls._find_config_file(use_ros))
        if not path.is_file():
            raise FileNotFoundError(f"Config file not found: {path}")
        with path.open("r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
        if not isinstance(raw, dict):
            raise ValueError(f"Tianji config must be a mapping: {path}")
        return cls._parse(raw, path)

    @classmethod
    def _find_config_file(cls, use_ros: bool = True) -> str:
        configured = os.environ.get("TIANJI_WORLD_OUTPUT_CONFIG")
        if configured:
            return configured
        if use_ros:
            try:
                from ament_index_python.packages import get_package_share_directory

                candidate = Path(get_package_share_directory("tianji_world_output")) / "config" / "tianji_robot.yaml"
                if candidate.is_file():
                    return str(candidate)
            except Exception:
                pass
        candidate = Path(__file__).resolve().parent / "config" / "tianji_robot.yaml"
        if candidate.is_file():
            return str(candidate)
        raise FileNotFoundError(
            "Cannot find tianji_robot.yaml in the Tianji world-output compatibility package"
        )

    @classmethod
    def _parse(cls, raw: Dict[str, Any], config_path: Path) -> "TianjiConfig":
        def arrays(value: Any) -> Dict[str, np.ndarray]:
            if not isinstance(value, dict):
                return {}
            return {str(key): np.asarray(item, dtype=np.float64) for key, item in value.items()}

        identity = np.eye(3, dtype=np.float64)
        mocap = np.asarray(raw.get("mocap_to_robot", identity), dtype=np.float64)
        if mocap.shape != (3, 3) or not np.isfinite(mocap).all():
            raise ValueError("mocap_to_robot must be a finite 3x3 matrix")
        if not np.allclose(mocap @ mocap.T, identity, atol=1.0e-6) or not np.isclose(np.linalg.det(mocap), 1.0):
            raise ValueError("mocap_to_robot must be a proper rotation matrix")

        return cls(
            raw=raw,
            robot_ip=str(raw.get("robot_ip", "192.168.1.190")),
            init_joints=arrays(raw.get("init_joints", {})),
            init_pos=arrays(raw.get("init_pos", {})),
            init_rot=arrays(raw.get("init_rot", {})),
            init_quat=arrays(raw.get("init_quat", {})),
            world_to_chest_quat=arrays(raw.get("world_to_chest_quat", {})),
            world_to_chest_trans=arrays(raw.get("world_to_chest_trans", {})),
            arm_init_pos=arrays(raw.get("arm_init_pos", {})),
            arm_init_quat=arrays(raw.get("arm_init_quat", {})),
            pico_to_robot=np.asarray(raw.get("pico_to_robot", identity), dtype=np.float64),
            mocap_to_robot=mocap,
            zsp_type=int(raw.get("zsp_type", 1)),
            default_zsp_para=dict(raw.get("default_zsp_para", {})),
            zsp_angle=float(raw.get("zsp_angle", 0.0)),
            dgr=list(raw.get("dgr", [5.0, 5.0, 5.0])),
            tracker_serial_map=dict(raw.get("tracker_serial_map", {})),
            config_path=config_path,
        )

    def get_world_to_chest_rotation(self, side: str) -> np.ndarray:
        from scipy.spatial.transform import Rotation

        return Rotation.from_quat(self.world_to_chest_quat[side]).as_matrix()

    def get_chest_to_world_rotation(self, side: str) -> np.ndarray:
        return self.get_world_to_chest_rotation(side).T

    def get_default_zsp_direction(self, side: str) -> np.ndarray:
        fallback = [0.0, -1.0, -0.5, 0.0, 0.0, 0.0] if side == "left" else [0.0, 1.0, -0.5, 0.0, 0.0, 0.0]
        values = self.default_zsp_para.get(side, fallback)
        direction = np.asarray(values[:3], dtype=np.float64)
        norm = float(np.linalg.norm(direction))
        if direction.shape != (3,) or not np.isfinite(direction).all() or norm <= 1.0e-9:
            raise ValueError(f"default_zsp_para.{side} must contain a finite non-zero direction")
        return direction / norm

    def get_kine_config_path(self) -> str:
        filename = str(self.raw.get("kine_config_file", "ccs_m6.MvKDCfg"))
        return str(Path(__file__).resolve().parent / "config" / filename)


_config_instance: TianjiConfig | None = None


def get_config(use_ros: bool = True) -> TianjiConfig:
    global _config_instance
    if _config_instance is None:
        _config_instance = TianjiConfig.load(use_ros=use_ros)
    return _config_instance


def reload_config(config_path: Optional[str | os.PathLike[str]] = None, use_ros: bool = True) -> TianjiConfig:
    global _config_instance
    _config_instance = TianjiConfig.load(config_path=config_path, use_ros=use_ros)
    return _config_instance
