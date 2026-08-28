"""Task 8 的配置树、router endpoint 和 component config 入口。

所有产品入口都从这个模块解析配置根和 endpoint，避免源码、staging、runtime
各自推导出不同路径。配置文件使用扁平 YAML；未知字段一律拒绝。
"""
from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any, Mapping

import yaml

DEFAULT_ROUTER_ENDPOINT = "tcp/127.0.0.1:7447"
ROUTER_ENDPOINT_ENV = "TIANJI_ROUTER_ENDPOINT"


def canonical_config_root() -> Path:
    """返回本源码/安装包对应的唯一 ``config`` 根目录。

    ``TIANJI_CONFIG_ROOT`` 供受管 runtime 明确指定安装后的 config；源码运行时
    以包所在的 ``src/pico_body_tianji/config`` 为默认值。不会回退到旧
    ``config/mode`` 树。
    """
    configured = os.environ.get("TIANJI_CONFIG_ROOT")
    if configured:
        root = Path(configured).expanduser().resolve()
        if not root.is_dir():
            raise ValueError(f"TIANJI_CONFIG_ROOT does not exist: {root}")
        return root

    bundle = os.environ.get("PICO_BODY_TIANJI_BUNDLE_ROOT")
    if bundle:
        candidates = (
            Path(bundle) / "runtime" / "pico_body_tianji" / "share" / "pico_body_tianji" / "config",
            Path(bundle) / "src" / "pico_body_tianji" / "config",
            Path(bundle) / "config",
        )
        for candidate in candidates:
            if candidate.is_dir():
                return candidate.resolve()
        raise ValueError(f"unable to locate config under bundle root {bundle}")

    source_root = Path(__file__).resolve().parents[1] / "config"
    if source_root.is_dir():
        return source_root
    raise ValueError(f"unable to locate canonical config root: {source_root}")


def router_endpoint(environment: Mapping[str, Any] | None = None) -> str:
    """读取唯一 router endpoint；未设置时显式使用默认地址。"""
    values = os.environ if environment is None else environment
    raw = values.get(ROUTER_ENDPOINT_ENV, DEFAULT_ROUTER_ENDPOINT)
    endpoint = str(raw).strip()
    if not endpoint:
        raise ValueError(f"{ROUTER_ENDPOINT_ENV} must not be empty")
    if "/" not in endpoint or endpoint.startswith("/"):
        raise ValueError(f"invalid Zenoh router endpoint: {endpoint!r}")
    return endpoint


def load_yaml(path: str | os.PathLike[str]) -> dict[str, Any]:
    """加载一个 object YAML，并拒绝空文档/非 mapping。"""
    location = Path(path).expanduser()
    try:
        value = yaml.safe_load(location.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ValueError(f"unable to read config {location}: {exc}") from exc
    if not isinstance(value, Mapping):
        raise ValueError(f"config {location} must contain a mapping")
    return dict(value)


def load_component_config(
    path: str | os.PathLike[str],
    *,
    allowed_keys: set[str] | frozenset[str] | None = None,
    required_keys: set[str] | frozenset[str] | None = None,
) -> dict[str, Any]:
    """加载组件配置并执行 canonical field 集合校验。"""
    value = load_yaml(path)
    if allowed_keys is not None:
        extra = set(value) - set(allowed_keys)
        if extra:
            raise ValueError(f"unknown config fields in {path}: {sorted(extra)}")
    if required_keys is not None:
        missing = set(required_keys) - set(value)
        if missing:
            raise ValueError(f"missing config fields in {path}: {sorted(missing)}")
    return value


def component_path(component: str) -> Path:
    """解析 canonical 相对路径，禁止 ``..`` 和旧 mode 树。"""
    relative = Path(component)
    if relative.is_absolute() or ".." in relative.parts or relative.parts[0:1] == ("mode",):
        raise ValueError(f"non-canonical component config path: {component!r}")
    return canonical_config_root() / relative


def require_finite_positive(value: Any, field: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise ValueError(f"{field} must be finite and positive")
    return result


__all__ = [
    "DEFAULT_ROUTER_ENDPOINT",
    "ROUTER_ENDPOINT_ENV",
    "canonical_config_root",
    "component_path",
    "load_component_config",
    "load_yaml",
    "require_finite_positive",
    "router_endpoint",
]
