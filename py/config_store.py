"""配置持久化：config.json（调试参数） / layout.json（窗口布局）。

文件放在 hmi.py 同目录，与参考工具一致。写入走「临时文件 + os.replace」，
中途崩掉也不会留下半截 JSON。

**永远不保存 enabled / sending**：重启后必须是失能、不发送。
"""
from __future__ import annotations

import json
import os
import tempfile

APP_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_NAME = "config.json"
LAYOUT_NAME = "layout.json"


def config_path() -> str:
    return os.path.join(APP_DIR, CONFIG_NAME)


def layout_path() -> str:
    return os.path.join(APP_DIR, LAYOUT_NAME)


def _atomic_write(path: str, data) -> None:
    d = os.path.dirname(path) or "."
    fd, tmp = tempfile.mkstemp(dir=d, prefix=".tmp_", suffix=".json")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            json.dump(data, fh, ensure_ascii=False, indent=2)
        os.replace(tmp, path)
    except Exception:
        try:
            os.unlink(tmp)
        except Exception:
            pass
        raise


def load_json(path: str):
    """读 JSON；文件不存在或格式不对都返回 None。"""
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
        return data if isinstance(data, dict) else None
    except Exception:
        return None


def save_config(data: dict) -> str:
    path = config_path()
    _atomic_write(path, data)
    return path


def save_layout(data: dict) -> str:
    path = layout_path()
    _atomic_write(path, data)
    return path


def offscreen() -> bool:
    return os.environ.get("QT_QPA_PLATFORM", "").lower() in ("offscreen", "minimal")
