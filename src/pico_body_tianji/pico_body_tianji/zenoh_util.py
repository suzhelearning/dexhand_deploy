"""Zenoh 通讯公共层（替代 ROS 2 rclpy 链路）。

契约（zenoh-migration-contract）：
- key 表达式沿用原 ROS 话题路径，如 /pico_body/left_arm_target_pose
- Pose/Vector/JointState 消息用 JSON（UTF-8）；String/Bool 用裸文本
- stamp 为 {"sec": int, "nanosec": int}，缺失或 0 表示未打时间戳
- 存活注册：tj/live/<node-name>
- 事件 + 初始值（原 transient_local latched）用 LatchedKey：PUT 更新存储，
  GET 返回最近值；订阅者启动时可主动 get 一次
"""

from __future__ import annotations

import argparse
import json
import os
import threading
import time
from typing import Any, Callable, Dict, Optional

from .protocol.messages import strict_loads
import zenoh


from .config_loader import router_endpoint

_LIVELINESS_ROLES = {"source", "producer/arm", "producer/hand", "coordinator/arm", "executor/arm", "executor/hand", "recorder"}


def declare_component_liveliness(
    session: object,
    *,
    role: str,
    logical_id: str,
    instance_id: str,
) -> object | None:
    """Declare the canonical token ``tj/live/<role>/<logical>/<instance>``."""
    if role not in _LIVELINESS_ROLES:
        raise ValueError(f"unsupported liveliness role: {role}")
    if not logical_id or not instance_id or "/" in logical_id or "/" in instance_id:
        raise ValueError("logical_id and instance_id must be non-empty path components")
    live = getattr(session, "liveliness", None)
    if not callable(live):
        return None
    return live().declare_token(f"tj/live/{role}/{logical_id}/{instance_id}")


# ---------------------------------------------------------------- 时间戳

def stamp_now() -> Dict[str, int]:
    """返回当前秒 + 纳秒结构（对齐 ROS Time 语义）。"""
    t = time.time_ns()
    return {"sec": t // 1_000_000_000, "nanosec": t % 1_000_000_000}


def stamp_ns(stamp: Optional[Dict[str, int]]) -> int:
    """stamp 转纳秒；缺失/非法返回 0。"""
    if not stamp:
        return 0
    try:
        return int(stamp["sec"]) * 1_000_000_000 + int(stamp["nanosec"])
    except (KeyError, TypeError, ValueError):
        return 0


# ---------------------------------------------------------------- 会话

def key(name: str) -> str:
    """ROS 风格话题名 → Zenoh key 表达式（zenoh 禁止前导斜杠）。"""
    return name.lstrip("/")


def open_session(endpoint: str | None = None) -> zenoh.Session:
    """Open a client session using the single configured router endpoint."""
    endpoint = router_endpoint() if endpoint is None else endpoint.strip()
    if not endpoint:
        raise ValueError("router endpoint must not be empty")
    config = zenoh.Config.from_json5(
        json.dumps({"mode": "client", "connect": {"endpoints": [endpoint]}})
    )
    return zenoh.open(config)


class LiveToken:
    """进程存活标记：tj/live/<node-name>，退出时自动注销。"""

    def __init__(self, session: zenoh.Session, node_name: str):
        self._token = session.liveliness().declare_token(f"tj/live/{node_name}")

    def close(self) -> None:
        try:
            self._token.undeclare()
        except Exception:
            pass

    def __enter__(self) -> "LiveToken":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


# ---------------------------------------------------------------- 发布

class ZenohPub:
    """可靠发布器：JSON 或裸文本。"""

    def __init__(self, session: zenoh.Session, key: str):
        self._pub = session.declare_publisher(
            key, reliability=zenoh.Reliability.RELIABLE
        )

    def put_json(self, obj: Any) -> None:
        payload = json.dumps(obj, separators=(",", ":")).encode("utf-8")
        self._pub.put(payload, encoding="application/json")

    def put_text(self, text: Any) -> None:
        self._pub.put(str(text).encode("utf-8"))

    def put_bytes(self, payload: bytes) -> None:
        """发布裸二进制负载（如 float32 数组）。"""
        self._pub.put(payload)

    def close(self) -> None:
        try:
            self._pub.undeclare()
        except Exception:
            pass

    def __enter__(self) -> "ZenohPub":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


# ---------------------------------------------------------------- 订阅

class _SafeSub:
    """包装 subscriber：解码 + 异常隔离。

    注意：zenoh-python 1.10 的 declare_subscriber 不接受 reliability
    参数（仅 declare_publisher 有），可靠语义为 zenoh 默认。
    """

    def __init__(
        self,
        session: zenoh.Session,
        key: str,
        handler: Callable[[zenoh.Sample], None],
    ):
        def safe_handler(sample: zenoh.Sample) -> None:
            try:
                handler(sample)
            except Exception:
                import traceback
                traceback.print_exc()

        self._sub = session.declare_subscriber(key, safe_handler)

    def close(self) -> None:
        try:
            self._sub.undeclare()
        except Exception:
            pass


class ZenohJsonSub(_SafeSub):
    """JSON 消息订阅；handler 收到解码后的对象。"""

    def __init__(self, session, key, handler):
        def decoded(sample):
            payload = bytes(sample.payload)
            if not payload:
                return
            handler(strict_loads(payload))

        super().__init__(session, key, decoded)


class ZenohTextSub(_SafeSub):
    """裸文本订阅；handler 收到 str。"""

    def __init__(self, session, key, handler):
        def decoded(sample):
            payload = bytes(sample.payload)
            if not payload:
                return
            handler(payload.decode("utf-8"))

        super().__init__(session, key, decoded)


# ---------------------------------------------------------------- 事件 + 初始值

class LatchedKey:
    """替代 ROS transient_local latched 语义。

    PUT 存储最近值并应答 GET；订阅方需要初始值时先 get() 一次。
    """

    def __init__(
        self,
        session: zenoh.Session,
        key: str,
        initial: Optional[bytes] = None,
    ):
        self._session = session
        self._key = key
        self._lock = threading.Lock()
        self._value = initial
        self._queryable = session.declare_queryable(key, self._on_query)
        self._sub = _SafeSub(session, key, self._on_put)

    def _on_put(self, sample: zenoh.Sample) -> None:
        with self._lock:
            self._value = bytes(sample.payload)

    def _on_query(self, query: zenoh.Query) -> None:
        with self._lock:
            value = self._value
        query.reply(self._key, value if value is not None else b"")

    def put(self, payload: bytes) -> None:
        with self._lock:
            self._value = payload
        self._session.put(self._key, payload)

    def put_text(self, text: Any) -> None:
        self.put(str(text).encode("utf-8"))

    def get_text(self) -> Optional[str]:
        with self._lock:
            value = self._value
        if value is None:
            return None
        return value.decode("utf-8")

    def close(self) -> None:
        try:
            self._queryable.undeclare()
        except Exception:
            pass
        try:
            self._sub.close()
        except Exception:
            pass

    def __enter__(self) -> "LatchedKey":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


# ---------------------------------------------------------------- 参数

def load_tianji_config():
    """显式加载随包 tianji_robot.yaml（避开 ament 索引依赖）。

    优先使用环境变量 PICO_BODY_TIANJI_BUNDLE_ROOT（activate_bundle_runtime
    导出）；缺失时回退到源码树相对定位。
    """
    from tianji_world_output.config_loader import TianjiConfig

    bundle_root = os.environ.get("PICO_BODY_TIANJI_BUNDLE_ROOT", "")
    if bundle_root:
        config_path = os.path.join(
            bundle_root,
            "vendor",
            "python",
            "tianji_world_output",
            "config",
            "tianji_robot.yaml",
        )
        return TianjiConfig.load(config_path)
    # 回退：本文件位于 <root>/src/pico_body_tianji/pico_body_tianji/
    here = os.path.dirname(os.path.abspath(__file__))
    fallback = os.path.join(
        here,
        "..",
        "..",
        "..",
        "vendor",
        "python",
        "tianji_world_output",
        "config",
        "tianji_robot.yaml",
    )
    return TianjiConfig.load(fallback)


def parse_cli_args(
    argv: Optional[list] = None,
    extra: Optional[Dict[str, Any]] = None,
) -> argparse.Namespace:
    """统一 CLI：--config <yaml> 与 --param key:=value（可多次）。"""
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--config", default="", help="节点参数 YAML 文件")
    parser.add_argument("--param", action="append", default=[], metavar="key:=value")
    if extra:
        for key, kwargs in extra.items():
            parser.add_argument(key, **kwargs)
    return parser.parse_args(argv)


def _coerce_like(default, value):
    """把字符串覆盖值转换为默认值同类型（bool/int/float/list）。"""
    if isinstance(default, bool):
        return str(value).strip().lower() in ("1", "true", "yes")
    if isinstance(default, int):
        return int(float(value))
    if isinstance(default, float):
        return float(value)
    if isinstance(default, (list, tuple)):
        return json.loads(value)
    return value


def load_node_config(
    yaml_path: str,
    node_name: str,
    defaults: Dict[str, Any],
    overrides: Optional[Dict[str, str]] = None,
) -> Dict[str, Any]:
    """从 YAML 读取节点参数并套用默认值。

    兼容两种结构：
    - ROS 旧格式：{node_name: {ros__parameters: {key: value}}}
    - 扁平格式：{key: value}
    最后应用 --param key:=value 覆盖（按默认值类型转换）。
    """
    import yaml

    params = dict(defaults)
    if yaml_path:
        with open(yaml_path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        if isinstance(data, dict):
            section = data.get(node_name, data)
            if isinstance(section, dict) and "ros__parameters" in section:
                section = section["ros__parameters"]
            if isinstance(section, dict):
                params.update(section)
    if overrides:
        params.update(
            {
                k: _coerce_like(defaults.get(k), v)
                for k, v in overrides.items()
            }
        )
    return params


def parse_param_override(spec: str) -> tuple:
    """'key:=value' → (key, value)；无 ':= ' 时报错。"""
    if ":=" not in spec:
        raise ValueError(f"非法 --param 格式：{spec}（需要 key:=value）")
    key, _, value = spec.partition(":=")
    return key.strip(), value.strip()

def require_single_router(session: object, expected_zid: str | None = None) -> str:
    """Return the one connected router ZID, failing closed on mismatch."""
    info = getattr(session, "info", None)
    routers_zid = getattr(info, "routers_zid", None)
    if not callable(routers_zid):
        raise RuntimeError("session.info.routers_zid() is required")
    routers = [str(value) for value in routers_zid()]
    if len(routers) != 1 or not routers[0]:
        raise RuntimeError(f"expected exactly one router ZID, got {len(routers)}")
    if expected_zid is not None and routers[0] != expected_zid:
        raise RuntimeError(
            f"router ZID mismatch: expected {expected_zid!r}, got {routers[0]!r}"
        )
    return routers[0]
