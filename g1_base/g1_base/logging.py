"""
g1_base 日志模块
================
为 g1_base 提供统一的 Python logging 配置，将日志输出到：
  bot_mind/logs/g1_base.log（独立文件，带自动轮转）

与 navigation_manager.log 分离：
  - g1_base.log   → SDK 调用、control_server 请求/响应、耗时分析
  - navigation_manager.log → ROS2 节点日志、导航状态、定位信息（通过 get_logger()）

用法:
    from g1_base.logging import get_logger
    logger = get_logger("my_module")
    logger.info("message")

    # 带耗时记录
    from g1_base.logging import log_elapsed
    with log_elapsed(logger, "动作执行"):
        do_something()  # 会自动打印 "[动作执行] 耗时 1.234s"
"""

import logging
import os
import time
from contextlib import contextmanager
from logging.handlers import RotatingFileHandler
from pathlib import Path

# ── 配置 ──
# 日志目录：优先从环境变量读取，否则回退到 ~/bot_mind/logs/
_BOT_MIND_HOME = os.environ.get(
    "BOT_MIND_HOME",
    os.path.expanduser("~/bot_mind"),
)
_log_dir_env = os.environ.get("G1_BASE_LOG_DIR", "").strip()
LOG_DIR = Path(_log_dir_env) if _log_dir_env else Path(_BOT_MIND_HOME) / "logs"
LOG_FILE = LOG_DIR / "g1_base.log"
LOG_LEVEL = os.environ.get("G1_BASE_LOG_LEVEL", "DEBUG").upper()
LOG_MAX_BYTES = int(os.environ.get("G1_BASE_LOG_MAX_BYTES", str(10 * 1024 * 1024)))  # 10 MB
LOG_BACKUP_COUNT = int(os.environ.get("G1_BASE_LOG_BACKUP_COUNT", "5"))

# ── 格式 ──
_LOG_FORMAT = (
    "%(asctime)s.%(msecs)03d | %(levelname)-8s | "
    "%(name)s:%(funcName)s:%(lineno)d - %(message)s"
)
_LOG_DATEFMT = "%Y-%m-%d %H:%M:%S"

# ── 全局初始化（仅一次）──
_initialized = False
_root_logger_name = "g1_base"


def _ensure_initialized():
    """确保 g1_base 根 logger 已配置（幂等）。"""
    global _initialized
    if _initialized:
        return

    root = logging.getLogger(_root_logger_name)
    root.setLevel(getattr(logging, LOG_LEVEL, logging.DEBUG))
    root.propagate = False  # 不冒泡到 root logger

    formatter = logging.Formatter(_LOG_FORMAT, datefmt=_LOG_DATEFMT)

    # 仅使用文件 handler，不输出到 stdout。
    # 这样 SDK/control 日志只写入 g1_base.log，
    # 不会通过 subprocess stdout 混入 navigation_manager.log。
    # 导航相关日志由 ROS2 的 get_logger() 输出到 stdout。
    try:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        file_handler = RotatingFileHandler(
            str(LOG_FILE),
            maxBytes=LOG_MAX_BYTES,
            backupCount=LOG_BACKUP_COUNT,
            encoding="utf-8",
        )
        file_handler.setLevel(getattr(logging, LOG_LEVEL, logging.DEBUG))
        file_handler.setFormatter(formatter)
        root.addHandler(file_handler)
    except OSError as exc:
        # 文件日志创建失败时回退到 stderr，不影响运行
        import sys
        fallback = logging.StreamHandler(sys.stderr)
        fallback.setLevel(logging.WARNING)
        fallback.setFormatter(formatter)
        root.addHandler(fallback)
        root.warning("无法创建日志文件 %s: %s，回退到 stderr", LOG_FILE, exc)

    _initialized = True


def get_logger(name: str) -> logging.Logger:
    """获取 g1_base 子 logger。

    Args:
        name: 模块名，最终 logger 名称为 ``g1_base.<name>``。

    Returns:
        配置好的 Logger 实例。
    """
    _ensure_initialized()
    return logging.getLogger(f"{_root_logger_name}.{name}")


@contextmanager
def log_elapsed(logger: logging.Logger, label: str, level: int = logging.INFO):
    """上下文管理器，自动记录代码块耗时。

    用法::

        with log_elapsed(logger, "SDK 初始化"):
            sdk.init()
        # -> [SDK 初始化] 耗时 1.234s

    Args:
        logger: 目标 logger。
        label: 操作标签。
        level: 日志级别，默认 INFO。
    """
    t0 = time.monotonic()
    logger.log(level, "[%s] 开始", label)
    try:
        yield
    except Exception:
        elapsed = time.monotonic() - t0
        logger.log(level, "[%s] 失败，耗时 %.3fs", label, elapsed)
        raise
    else:
        elapsed = time.monotonic() - t0
        logger.log(level, "[%s] 完成，耗时 %.3fs", label, elapsed)
