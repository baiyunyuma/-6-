"""
关节电机配置（注册表）。

MIT 模式下 q / dq / tau 的**编码量程由型号决定**，填错会导致
「下发的角度 ≠ 实际到达的角度」，而且不会报任何错。型号必须与实物一致。

量程取自官方 SDK 的 limit_param 表：
    DM4310 : q ±12.5 rad, dq ±30 rad/s, tau ±10 N·m
    DM4340 : q ±12.5 rad, dq ±10 rad/s, tau ±28 N·m

**channel 很重要**：实测这台模块的电机接在 **CAN 通道 1**，而所有官方例程
默认用通道 0 —— 在通道 0 上发得出去，电机根本收不到。
"""

from dataclasses import dataclass, asdict
from typing import List

#: 默认 CAN 通道。
#:
#: 当前这台 USB2CANFD 是**单通道**模块：通道 0 能正常配置波特率并通信，
#: 通道 1 的 set_channel_baudrate 失败、回读全 0（实测）。所以默认用 0。
#: 如果换成双通道模块、电机接在第二路，把注册表里的「通道」改成 1 即可。
DEFAULT_CHANNEL = 0

#: 型号名 -> (SDK 枚举名, q_max, dq_max, tau_max)
MODELS = {
    "4310":     ("DM4310",     12.5, 30.0, 10.0),
    "4310_48V": ("DM4310_48V", 12.5, 50.0, 10.0),
    "4340":     ("DM4340",     12.5, 10.0, 28.0),
    "4340_48V": ("DM4340_48V", 12.5, 20.0, 28.0),
    "6006":     ("DM6006",     12.5, 45.0, 12.0),
    "8006":     ("DM8006",     12.5, 45.0, 20.0),
    "8009":     ("DM8009",     12.5, 45.0, 54.0),
    "10010":    ("DM10010",    12.5, 20.0, 200.0),
    "10010L":   ("DM10010L",   12.5, 25.0, 200.0),
}

#: 控制模式：寄存器 10 的取值
MODE_CODES = {"MIT": 1, "POS_VEL": 2, "VEL": 3, "POS_FORCE": 4}

#: 模式名 -> SDK 的 Control_Mode 枚举名（是「命令 CAN ID 偏移」，不是寄存器值）
MODE_ENUM_NAMES = {
    "MIT": "MIT_MODE",
    "POS_VEL": "POS_VEL_MODE",
    "VEL": "VEL_MODE",
    "POS_FORCE": "POS_FORCE_MODE",
}

#: SDK 的 Control_Mode 数值 -> 模式名（读驱动器实际模式用）
MODE_BY_VALUE = {0x000: "MIT", 0x100: "POS_VEL", 0x200: "VEL", 0x300: "POS_FORCE"}


@dataclass
class MotorSpec:
    name: str
    can_id: int
    mst_id: int
    model: str = "4310"
    channel: int = DEFAULT_CHANNEL
    mode: str = "MIT"

    @property
    def limits(self):
        return MODELS.get(self.model, MODELS["4310"])[1:]

    def to_dict(self):
        return asdict(self)


def default_specs() -> List[MotorSpec]:
    """
    默认注册表。型号是**按常见布局猜的**（基座大、腕部小），
    上电后请用 scan 或在界面里核对。
    """
    return [
        MotorSpec("joint1", 0x01, 0x01, "4340", DEFAULT_CHANNEL),
        MotorSpec("joint2", 0x02, 0x02, "4340", DEFAULT_CHANNEL),
        MotorSpec("joint3", 0x03, 0x03, "4340", DEFAULT_CHANNEL),
        MotorSpec("joint4", 0x04, 0x04, "4310", DEFAULT_CHANNEL),
        MotorSpec("joint5", 0x05, 0x05, "4310", DEFAULT_CHANNEL),
        MotorSpec("joint6", 0x06, 0x06, "4310", DEFAULT_CHANNEL),
    ]
