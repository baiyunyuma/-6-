"""Small atomic JSON store for the ROS 2 HMI (registry + layout).

Never persist enabled/sending flags: a restart must come up disarmed.
"""
from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path

BASE_DIR = Path.home() / ".config" / "zfy_arm"
#: 拖动录制的轨迹库（每条轨迹一个 JSON）
TRAJ_DIR = BASE_DIR / "trajectories"
REGISTRY_NAME = "hmi_registry.json"
LAYOUT_NAME = "hmi_layout.json"
LIMITS_NAME = "hmi_limits.json"
#: 增益 / 前馈 / 夹爪参数（「保存参数」按钮写入，启动时自动载入）
SETTINGS_NAME = "hmi_settings.json"


def offscreen() -> bool:
    return os.environ.get("QT_QPA_PLATFORM", "").lower() in ("offscreen", "minimal")


def load(name: str):
    path = BASE_DIR / name
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
        return data if isinstance(data, dict) else None
    except Exception:
        return None


def save(name: str, data: dict) -> None:
    if offscreen():
        return
    save_path(BASE_DIR / name, data)


def save_path(path, data) -> None:
    """Atomic JSON write to an arbitrary path (trajectory library etc.)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(
        dir=str(path.parent), prefix=".tmp_", suffix=".json"
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            json.dump(data, fh, ensure_ascii=False, separators=(",", ":"))
        os.replace(tmp, str(path))
    except Exception:
        try:
            os.unlink(tmp)
        except Exception:
            pass
        raise


def load_path(path):
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
        return data if isinstance(data, dict) else None
    except Exception:
        return None
