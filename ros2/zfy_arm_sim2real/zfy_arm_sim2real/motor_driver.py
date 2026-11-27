"""ROS 2 driver for the real ZFY arm (Damiao motors, MIT mode).

The CAN/USB layer is the HMI's ``py/core.MotorBus`` -- one owner of the
USB2CANFD adapter, a callback per feedback frame and the same enable sequence
as the Qt HMI.  This node only turns that into ROS 2 interfaces.

Control flow mirrors the Qt HMI's two switches:

1. ``~/enable`` (true) sends only the 0xFC enable command -- no control frame,
   so the motor is enabled but outputs no torque.
2. ``~/sending`` (true) starts the 250 Hz MIT stream (hold gains at the
   current pose until ``/mit_command`` arrives); false pauses the stream and
   keeps the last pose with the hold gains.  This is the HMI's "发送数据".
3. ``~/release`` or ``~/enable`` (false) stops the stream, sends one zero
   torque frame and disables with 0xFD.

Topics
------
``/joint_states``
    ``sensor_msgs/JointState`` at ``state_rate``.  Positions are mapped from
    motor angles to URDF coordinates as ``sign * (q - offset)``; fill
    ``joint_offsets`` from the raw angles at the URDF "fold" pose and set
    ``joint_signs`` to +1/-1 per joint.  While not sending, the driver polls a
    ``0xCC`` status refresh every ``state_refresh_period`` so the topic stays
    alive without commanding motion.
``/mit_command``
    ``std_msgs/Float64MultiArray`` with ``5*N`` values:
    ``[kp(N), kd(N), q(N), dq(N), tau(N)]``.  Only accepted while sending.
    MIT uses all fields; POS_VEL uses q/dq, VEL uses dq, POS_FORCE uses q/tau.

Services
--------
``~/enable``   (``std_srvs/SetBool``)  true = 0xFC only; false = zero torque +
                                      disable.
``~/sending``  (``std_srvs/SetBool``)  true = start MIT stream; false = stop it
                                      (stays enabled).
``~/release``  (``std_srvs/Trigger``)  stop stream, zero torque, disable.
``~/set_mode``  (``zfy_arm_interfaces/SetMotorMode``)  switch one joint's
    CTRL_MODE; only while disabled and not sending.
``~/read_param`` / ``~/write_param`` / ``~/save_param``
    (``zfy_arm_interfaces``)  RID access on one motor; writes are refused while
    the MIT stream is running.
``~/status``  (``zfy_arm_interfaces/GetMotorStatus``)  modes, mapped angles and
    ERR for the HMI.

Safety: the node never enables the motors by itself, requires a responding
motor and a fresh pose before enabling, enables in MIT mode, clamps commands
to the motor ratings and falls back to a hold pose when ``/mit_command`` stops
arriving.
"""

from __future__ import annotations

import csv
import math
import os
import queue
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, Int32
from std_srvs.srv import SetBool, Trigger
from zfy_arm_interfaces.msg import JointGoal, MotorMitCommand, MotorRegistryEntry
from zfy_arm_interfaces.srv import (
    ConfigureMotors,
    GetMotorStatus,
    ReadMotorParam,
    SaveMotorParam,
    SetMotorEnable,
    SetMotorMode,
    SetMotorSending,
    WriteMotorParam,
)

#: 达妙 CTRL_MODE 寄存器里 MIT 的编码
_MIT_MODE_CODE = 1

#: 机械/模型关节限位（URDF 弧度，J1..J6），与 URDF 和 MuJoCo 模型一致。
#: 可用 ROS 参数 joint_lower / joint_upper 收窄；驱动在所有指令路径裁剪。
DEFAULT_JOINT_LOWER = [
    -2.6179939, 0.0, -3.1415927,
    -1.5707963, -1.5707963, -6.2831853,
]
DEFAULT_JOINT_UPPER = [
    2.6179939, 3.0543262, 0.0,
    1.5707963, 1.5707963, 6.2831853,
]

#: MIT 反馈帧首字节高半字节 ERR 的含义（官方 FAQ；1 是"使能正常"，不是故障）
_ERR_TEXT = {
    0: "失能/无故障",
    1: "使能/正常",
    2: "输出编码器未校准",
    3: "输出编码器校准量过大",
    4: "输出编码器校准偏差过大",
    5: "传感器读取错误",
    7: "电机参数读取错误",
    8: "过压",
    9: "欠压",
    10: "过流",
    11: "MOS 过温",
    12: "线圈过温",
    13: "通信丢失",
    14: "过载",
}


def _err_text(err: int) -> str:
    return "%d(%s)" % (err, _ERR_TEXT.get(err, "未知"))


def _find_hmi_py(explicit: str) -> Path:
    """Locate the HMI ``py`` directory that holds core.py / motor_config.py."""
    candidates: List[Path] = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    env = os.environ.get("ZFY_HMI_PY")
    if env:
        candidates.append(Path(env).expanduser())

    here = Path(__file__).resolve()
    for base in (here.parent, *here.parents):
        candidates.append(base / "py")
        candidates.append(base)

    seen = set()
    for cand in candidates:
        try:
            cand = cand.resolve()
        except OSError:
            continue
        if cand in seen:
            continue
        seen.add(cand)
        if (cand / "core.py").is_file() and (cand / "motor_config.py").is_file():
            return cand

    raise RuntimeError(
        "找不到上位机的 py 目录（core.py / motor_config.py）；"
        "请设置参数 hmi_py_path 或环境变量 ZFY_HMI_PY"
    )


def _load_bus_types(explicit_path: str, backend: str = "vendor"):
    py_dir = _find_hmi_py(explicit_path)
    if str(py_dir) not in sys.path:
        sys.path.insert(0, str(py_dir))
    from motor_config import MODE_CODES, MODELS, MotorSpec

    backend = str(backend or "vendor").strip().lower()
    if backend == "socketcan":
        from socketcan_bus import SocketCanBus

        return SocketCanBus, MotorSpec, MODELS, MODE_CODES
    if backend != "vendor":
        raise RuntimeError(
            f"未知电机后端 {backend!r}；可选 vendor / socketcan"
        )
    from core import MotorBus

    return MotorBus, MotorSpec, MODELS, MODE_CODES


class _Gravity:
    def __init__(self, model, qidx, didx):
        import mujoco
        import numpy as np
        self._mj = mujoco
        self.model = model
        self.data = mujoco.MjData(model)
        self.qidx = np.asarray(qidx, dtype=int)
        self.didx = np.asarray(didx, dtype=int)
        self._tau = np.zeros(int(model.nv))

    def bias(self, q):
        import numpy as np
        self.data.qpos[self.qidx] = np.asarray(q, dtype=float)
        self.data.qvel[:] = 0.0
        self._mj.mj_kinematics(self.model, self.data)
        self._mj.mj_comPos(self.model, self.data)
        self._tau[:] = 0.0
        self._mj.mj_rne(self.model, self.data, 0, self._tau)
        return self._tau[self.didx].copy()


class _JointServo:
    """Joint interpolation: legacy cosine, or quintic C2 with velocity FF.

    ``cosine`` is the original profile and stays the default so existing
    behaviour does not change.  ``quintic`` is a minimum-jerk segment with
    zero velocity/acceleration at both ends; it also yields analytic dq/ddq
    for the MIT velocity reference and future inertial feed-forward.
    """

    def __init__(self, n: int):
        self.n = n
        self._lock = threading.Lock()
        self.kp = [5.0] * n
        self.kd = [0.8] * n
        self.q_cmd = [0.0] * n
        self.q_start = [0.0] * n
        self.q_target = [0.0] * n
        self.dq_cmd = [0.0] * n
        self.ddq_cmd = [0.0] * n
        self.t0 = 0.0
        self.dur = 0.0
        self.dwell = 0.0
        self.dwell_t0 = 0.0
        self.phase = "idle"
        self.direct_t = 0.0
        #: DIRECT 指令带了速度（POS_VEL 回放）；MIT 默认不看它
        self.direct_dq = False
        self.active = False
        self.pending = deque()
        self.profile = "cosine"
        self.velocity_feedforward = False
        self.max_velocity = 0.0
        self.max_acceleration = 0.0

    def configure(self, profile: str, velocity_feedforward: bool,
                  max_velocity: float, max_acceleration: float) -> None:
        with self._lock:
            name = str(profile).strip().lower()
            if name not in ("cosine", "quintic"):
                raise ValueError("未知轨迹 profile: %r" % profile)
            self.profile = name
            self.velocity_feedforward = bool(velocity_feedforward)
            self.max_velocity = max(0.0, float(max_velocity))
            self.max_acceleration = max(0.0, float(max_acceleration))

    def set_gains(self, kp: float, kd: float) -> None:
        with self._lock:
            self.kp = [float(kp)] * self.n
            self.kd = [float(kd)] * self.n

    def gains(self):
        with self._lock:
            return list(self.kp), list(self.kd)

    def snapshot(self):
        with self._lock:
            return list(self.q_cmd), self.active, len(self.pending)

    def reset(self, q) -> None:
        with self._lock:
            self.q_cmd = [float(v) for v in q]
            self.q_start = list(self.q_cmd)
            self.q_target = list(self.q_cmd)
            self.dq_cmd = [0.0] * self.n
            self.ddq_cmd = [0.0] * self.n
            self.direct_dq = False
            self.active = False
            self.phase = "idle"
            self.dwell = 0.0
            self.pending.clear()

    def _scaled_duration_locked(self, q_from, q_to, dur: float) -> float:
        """Time-scale a quintic segment to respect velocity/acceleration."""
        dur = max(0.05, float(dur))
        if self.profile != "quintic":
            return dur
        scale = 1.0
        for a, b in zip(q_from, q_to):
            h = abs(float(b) - float(a))
            if h < 1e-9:
                continue
            if self.max_velocity > 0.0:
                scale = max(scale, 1.875 * h / (self.max_velocity * dur))
            if self.max_acceleration > 0.0:
                scale = max(
                    scale,
                    math.sqrt(
                        5.773502691896258 * h
                        / (self.max_acceleration * dur * dur)
                    ),
                )
        return dur * scale

    def _start_next_locked(self, now: float) -> None:
        if not self.pending:
            self.active = False
            self.phase = "idle"
            self.dwell = 0.0
            self.dq_cmd = [0.0] * self.n
            self.ddq_cmd = [0.0] * self.n
            return
        self.q_start = list(self.q_cmd)
        target, requested_dur, dwell = self.pending.popleft()
        self.q_target = list(target)
        self.dwell = float(dwell)
        self.dur = self._scaled_duration_locked(
            self.q_start, self.q_target, requested_dur)
        self.t0 = now
        self.phase = "move"
        self.active = True

    def _sample_full_locked(self, now: float):
        """Return (q, dq, ddq) for the active profile at time ``now``."""
        if not self.active:
            # DIRECT 指令可能自带速度（POS_VEL 回放）；其余路径 dq_cmd
            # 都已被清零，所以这里直接返回它是安全的。
            return (
                list(self.q_cmd), list(self.dq_cmd), list(self.ddq_cmd)
            )
        if self.phase == "dwell":
            if now - self.dwell_t0 < self.dwell:
                return list(self.q_cmd), [0.0] * self.n, [0.0] * self.n
            self._start_next_locked(now)
            if not self.active or self.phase != "move":
                return list(self.q_cmd), [0.0] * self.n, [0.0] * self.n
        s = 0.0 if self.dur <= 0.0 else min(1.0, (now - self.t0) / self.dur)
        if self.profile == "quintic":
            u = 10.0 * s ** 3 - 15.0 * s ** 4 + 6.0 * s ** 5
            du = (30.0 * s ** 2 - 60.0 * s ** 3 + 30.0 * s ** 4) \
                / max(self.dur, 1e-9)
            ddu = (60.0 * s - 180.0 * s ** 2 + 120.0 * s ** 3) \
                / max(self.dur * self.dur, 1e-9)
        else:
            # 原来的余弦插值，保留用于 A/B 对比
            u = 0.5 - 0.5 * math.cos(math.pi * s)
            du = (0.5 * math.pi * math.sin(math.pi * s)) \
                / max(self.dur, 1e-9)
            ddu = (0.5 * math.pi * math.pi * math.cos(math.pi * s)) \
                / max(self.dur * self.dur, 1e-9)
        self.q_cmd = [
            a + (b - a) * u for a, b in zip(self.q_start, self.q_target)
        ]
        self.dq_cmd = [
            (b - a) * du for a, b in zip(self.q_start, self.q_target)
        ]
        self.ddq_cmd = [
            (b - a) * ddu for a, b in zip(self.q_start, self.q_target)
        ]
        if s >= 1.0 - 1e-9:
            self.q_cmd = list(self.q_target)
            self.dq_cmd = [0.0] * self.n
            self.ddq_cmd = [0.0] * self.n
            if self.dwell > 1e-3:
                self.phase = "dwell"
                self.dwell_t0 = now
                return (
                    list(self.q_cmd),
                    list(self.dq_cmd),
                    list(self.ddq_cmd),
                )
            self._start_next_locked(now)
        return list(self.q_cmd), list(self.dq_cmd), list(self.ddq_cmd)

    def _sample_locked(self, now: float):
        q, _dq, _ddq = self._sample_full_locked(now)
        return q

    def sample(self, now: float):
        with self._lock:
            return self._sample_locked(now)

    def sample_full(self, now: float):
        with self._lock:
            return self._sample_full_locked(now)

    def move_to(self, q, dur: float, dwell: float = 0.0) -> None:
        with self._lock:
            now = time.monotonic()
            self.q_start = self._sample_locked(now)
            self.q_target = [float(v) for v in q]
            self.t0 = now
            self.dur = self._scaled_duration_locked(
                self.q_start, self.q_target, dur)
            self.dwell = max(0.0, float(dwell))
            self.direct_dq = False
            self.phase = "move"
            self.active = True
            self.pending.clear()

    def append(self, q, dur: float, dwell: float = 0.0) -> None:
        with self._lock:
            now = time.monotonic()
            target = [float(v) for v in q]
            requested = max(0.05, float(dur))
            dwell = max(0.0, float(dwell))
            item = (target, requested, dwell)
            if not self.active and not self.pending:
                self.q_start = list(self.q_cmd)
                self.q_target = list(target)
                self.dwell = dwell
                self.dur = self._scaled_duration_locked(
                    self.q_start, self.q_target, requested)
                self.t0 = now
                self.direct_dq = False
                self.phase = "move"
                self.active = True
            else:
                self.pending.append(item)

    def stop_here(self) -> None:
        with self._lock:
            now = time.monotonic()
            cur = self._sample_locked(now)
            self.pending.clear()
            self.q_start = list(cur)
            self.q_target = list(cur)
            self.t0 = now
            self.dur = 0.25
            self.dwell = 0.0
            self.dq_cmd = [0.0] * self.n
            self.ddq_cmd = [0.0] * self.n
            self.direct_dq = False
            self.phase = "move"
            self.active = True

    def idle_for_float(self, now: float) -> bool:
        """没有轨迹、也没有刚收到的点动指令时，才允许期望角跟着实测走。"""
        with self._lock:
            if self.active:
                return False
            return (now - self.direct_t) > 0.2

    def follow_measured(self, q) -> None:
        with self._lock:
            if self.active:
                return
            self.q_cmd = [float(v) for v in q]
            self.q_start = list(self.q_cmd)
            self.q_target = list(self.q_cmd)
            self.dq_cmd = [0.0] * self.n
            self.ddq_cmd = [0.0] * self.n
            self.direct_dq = False

    def set_direct(self, q, dq=None) -> None:
        """Direct setpoint.  ``dq`` (optional) feeds POS_VEL/VEL modes."""
        with self._lock:
            self.q_cmd = [float(v) for v in q]
            self.q_start = list(self.q_cmd)
            self.q_target = list(self.q_cmd)
            if dq is None:
                self.dq_cmd = [0.0] * self.n
                self.direct_dq = False
            else:
                self.dq_cmd = [float(v) for v in dq]
                self.direct_dq = True
            self.ddq_cmd = [0.0] * self.n
            self.active = False
            self.phase = "idle"
            self.dwell = 0.0
            self.direct_t = time.monotonic()
            self.pending.clear()

    def direct_velocity_active(self) -> bool:
        with self._lock:
            return bool(self.direct_dq)


def _joint_index(name: str) -> int:
    """Map joint1..joint6 to 0..5; unknown names fall back to 0."""
    text = str(name).strip().lower()
    digits = "".join(ch for ch in text if ch.isdigit())
    try:
        n = int(digits) if digits else 1
    except ValueError:
        n = 1
    return max(0, min(5, n - 1))


class _CsvLogger:
    """Non-blocking CSV writer for dynamic-identification data.

    The control loop only enqueues one row per tick; a daemon thread does the
    file I/O, so a slow disk never stretches the 250/1000 Hz control period.
    """

    def __init__(self, path: str, header: List[str], flush_every: int = 50):
        self.path = str(path)
        self.header = list(header)
        self.flush_every = max(1, int(flush_every))
        self.dropped = 0
        self._q: "queue.Queue" = queue.Queue(maxsize=50000)
        self._thread: Optional[threading.Thread] = None
        self._fh = None
        self._writer = None

    def start(self) -> None:
        self._fh = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._fh)
        self._writer.writerow(self.header)
        self._thread = threading.Thread(
            target=self._run, name="ident-csv", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        n = 0
        try:
            while True:
                row = self._q.get()
                if row is None:
                    break
                try:
                    self._writer.writerow(row)
                except Exception:
                    self.dropped += 1
                    continue
                n += 1
                if n % self.flush_every == 0:
                    try:
                        self._fh.flush()
                    except Exception:
                        pass
        finally:
            try:
                self._fh.flush()
                self._fh.close()
            except Exception:
                pass

    def log(self, row) -> None:
        try:
            self._q.put_nowait(row)
        except queue.Full:
            self.dropped += 1

    def stop(self) -> None:
        try:
            self._q.put_nowait(None)
        except queue.Full:
            pass
        th = self._thread
        if th is not None and th.is_alive():
            th.join(timeout=2.0)
        self._thread = None


@dataclass
class _MotorRec:
    """Per-motor runtime state. Kept in registry order (J1..J6)."""

    spec: object                     # MotorSpec
    joint_index: int
    sign: float
    offset: float
    q_max: float
    dq_max: float
    tau_max: float
    hold_kp: float
    hold_kd: float
    #: motor radians per URDF joint radian (external transmission ratio)
    scale: float = 1.0
    enabled: bool = False
    sending: bool = False
    pose_known: bool = False
    zero_torque: bool = False
    per_cmd: Optional[Tuple[float, float, float, float, float]] = None
    per_cmd_t: float = 0.0
    per_cmd_timeout: float = 0.0
    stale_since: Optional[float] = None
    hold_q: float = 0.0              # motor frame
    last_target_q: float = 0.0       # motor frame
    last_tau: float = 0.0            # motor frame
    #: 软件限位换算到电机帧（已按 sign 交换上下界）
    q_lo_motor: float = -1.0e9
    q_hi_motor: float = 1.0e9


class MotorDriver(Node):
    def __init__(self):
        super().__init__("motor_driver")

        self.declare_parameter("sim", True)
        self.declare_parameter("canfd", False)
        self.declare_parameter("nom_baud", 1_000_000)
        self.declare_parameter("dat_baud", 1_000_000)
        self.declare_parameter("set_baudrate", True)
        self.declare_parameter("device_index", 0)
        self.declare_parameter("device_type", "single")
        #: 电机后端：vendor = 厂商 USB SDK；socketcan = gs_usb 固件 + can0
        self.declare_parameter("backend", "vendor")
        self.declare_parameter("can_channel", "can0")
        #: channel index -> interface name.  For the dual USB2CANFD set e.g.
        #: ["can1", "can2"] so J1..J3 (channel 0) and J4..J6 (channel 1)
        #: use different buses.  Empty means [can_channel].
        self.declare_parameter("can_channels", [""])
        self.declare_parameter("can_bitrate", 1_000_000)
        self.declare_parameter("hmi_py_path", "")
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter("can_ids", [1, 2, 3, 4, 5, 6])
        self.declare_parameter("mst_ids", [1, 2, 3, 4, 5, 6])
        self.declare_parameter(
            "models", ["4340", "4340", "4340", "4310", "4310", "4310"]
        )
        self.declare_parameter("channels", [0, 0, 0, 0, 0, 0])
        #: +1/-1 per joint: motor positive direction in URDF coordinates
        self.declare_parameter("joint_signs", [1.0] * 6)
        #: motor-frame q at the URDF zero pose (rad), subtracted before publishing
        self.declare_parameter("joint_offsets", [0.0] * 6)
        #: motor radians per URDF joint radian (external transmission ratio);
        #: q_motor = sign * scale * q_urdf + offset
        self.declare_parameter("joint_scales", [1.0] * 6)
        #: 软件关节限位（URDF 弧度）。默认取模型限位的前 n 个。
        self.declare_parameter("joint_lower", list(DEFAULT_JOINT_LOWER))
        self.declare_parameter("joint_upper", list(DEFAULT_JOINT_UPPER))
        #: per-joint gravity feedforward trim (multiplies the MuJoCo torque)
        self.declare_parameter("gravity_gains", [1.0] * 6)
        #: per-joint constant torque trim in the URDF frame (N*m)
        self.declare_parameter("gravity_biases", [0.0] * 6)
        self.declare_parameter("hold_kp", [5.0] * 6)
        self.declare_parameter("hold_kd", [0.3] * 6)
        self.declare_parameter("control_rate", 250.0)
        self.declare_parameter("state_rate", 100.0)
        self.declare_parameter("command_timeout", 0.3)
        self.declare_parameter("feedback_timeout", 0.5)
        #: 反馈持续超时后零力矩并失能，避免盲发保持力矩
        self.declare_parameter("feedback_release_timeout", 1.0)
        #: 指令位置相对上一帧的最大速度；拦住单帧大跳变。0 = 不限制
        self.declare_parameter("max_joint_velocity", 3.5)
        #: 轨迹插值：cosine = 原来的余弦（默认，行为不变）；
        #: quintic = 五次多项式，C2 连续，可用于速度前馈
        self.declare_parameter("trajectory_profile", "cosine")
        #: 把轨迹的解析速度送进 MIT 的 dq_d；False 保持原来的 dq_d=0
        self.declare_parameter("velocity_feedforward", False)
        #: quintic 轨迹的时间缩放上限（rad/s^2）；0 = 不限制
        self.declare_parameter("max_joint_acceleration", 10.0)
        self.declare_parameter("state_refresh_period", 0.5)
        self.declare_parameter(
            "gravity_scene",
            os.path.expanduser(
                "~/zfy_arm/zfy_arm_control/model/scene_robot_arm_torque.xml"
            ),
        )
        self.declare_parameter("clear_error_before_enable", False)
        self.declare_parameter("force_mode_write", False)
        #: identification CSV logging; can also be toggled at runtime with
        #: the ~/set_logging service (std_srvs/SetBool)
        self.declare_parameter("log_csv", False)
        self.declare_parameter("log_path", "/tmp/zfy_ident.csv")
        self.declare_parameter("log_every", 1)

        self.joint_names = [str(v) for v in self._get("joint_names")]
        n = len(self.joint_names)
        if n == 0:
            raise RuntimeError("joint_names 不能为空")
        if len(set(self.joint_names)) != n:
            raise RuntimeError("joint_names 有重复")

        self.can_ids = self._int_list("can_ids", n)
        self.mst_ids = self._int_list("mst_ids", n)
        self.channels = self._int_list("channels", n)
        self.models = self._string_list("models", n)
        self.joint_signs = self._float_list("joint_signs", n)
        if any(v not in (-1.0, 1.0) for v in self.joint_signs):
            raise RuntimeError("joint_signs 只能是 +1 或 -1")
        self.joint_offsets = self._float_list("joint_offsets", n)
        raw_scales = list(self._get("joint_scales"))
        if len(raw_scales) != n:
            if len(raw_scales) == 6 and all(float(v) == 1.0 for v in raw_scales):
                raw_scales = [1.0] * n
            else:
                raise RuntimeError(
                    f"参数 joint_scales 长度 {len(raw_scales)} != 关节数 {n}")
        self.joint_scales = [float(v) for v in raw_scales]
        if any(v <= 0.0 or not math.isfinite(v) for v in self.joint_scales):
            raise RuntimeError("joint_scales 必须为正的有限数")
        raw_lower = [float(v) for v in self._get("joint_lower")]
        raw_upper = [float(v) for v in self._get("joint_upper")]
        if len(raw_lower) != n or len(raw_upper) != n:
            # 默认值是 6 轴表；单轴/两轴台架配置可只给前 n 个。
            if len(raw_lower) == 6 and len(raw_upper) == 6 and n < 6:
                raw_lower = raw_lower[:n]
                raw_upper = raw_upper[:n]
            else:
                raise RuntimeError(
                    "joint_lower / joint_upper 长度必须等于关节数 %d" % n
                )
        if (any(not math.isfinite(v) for v in raw_lower)
                or any(not math.isfinite(v) for v in raw_upper)):
            raise RuntimeError("joint_lower / joint_upper 必须是有限数")
        if any(lo >= hi for lo, hi in zip(raw_lower, raw_upper)):
            raise RuntimeError("joint_lower 必须小于 joint_upper")
        self.joint_lower = raw_lower
        self.joint_upper = raw_upper
        raw_gains = list(self._get("gravity_gains"))
        if len(raw_gains) != n:
            if len(raw_gains) == 6 and all(float(v) == 1.0 for v in raw_gains):
                raw_gains = [1.0] * n
            else:
                raise RuntimeError(
                    f"参数 gravity_gains 长度 {len(raw_gains)} != 关节数 {n}")
        self.gravity_gains = [float(v) for v in raw_gains]
        if any(v <= 0.0 or not math.isfinite(v) for v in self.gravity_gains):
            raise RuntimeError("gravity_gains 必须为正的有限数")
        raw_biases = list(self._get("gravity_biases"))
        if len(raw_biases) != n:
            if len(raw_biases) == 6 and all(float(v) == 0.0 for v in raw_biases):
                raw_biases = [0.0] * n
            else:
                raise RuntimeError(
                    f"参数 gravity_biases 长度 {len(raw_biases)} != 关节数 {n}")
        self.gravity_biases = [float(v) for v in raw_biases]
        if any(not math.isfinite(v) for v in self.gravity_biases):
            raise RuntimeError("gravity_biases 必须是有限数")
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.hold_kp = self._float_list("hold_kp", n)
        self.hold_kd = self._float_list("hold_kd", n)
        if len(set(self.can_ids)) != n:
            raise RuntimeError("can_ids 有重复")

        self.sim = bool(self._get("sim"))
        self.backend = str(self._get("backend")).strip().lower()
        self.can_channel = str(self._get("can_channel"))
        self.can_channels = [
            str(v).strip() for v in self._get("can_channels") if str(v).strip()
        ]
        self.can_bitrate = int(self._get("can_bitrate"))
        self.control_rate = float(self._get("control_rate"))
        self.state_rate = float(self._get("state_rate"))
        self.command_timeout = float(self._get("command_timeout"))
        self.feedback_timeout = float(self._get("feedback_timeout"))
        self.feedback_release_timeout = float(self._get("feedback_release_timeout"))
        self.max_joint_velocity = float(self._get("max_joint_velocity"))
        self.trajectory_profile = str(
            self._get("trajectory_profile")).strip().lower()
        if self.trajectory_profile not in ("cosine", "quintic"):
            raise RuntimeError(
                "trajectory_profile 必须是 cosine 或 quintic")
        self.velocity_feedforward = bool(self._get("velocity_feedforward"))
        self.max_joint_acceleration = float(
            self._get("max_joint_acceleration"))
        if (not math.isfinite(self.max_joint_acceleration)
                or self.max_joint_acceleration < 0.0):
            raise RuntimeError(
                "max_joint_acceleration 必须是非负有限数；0 表示不限制")
        self.state_refresh_period = float(self._get("state_refresh_period"))
        self.clear_error_before_enable = bool(self._get("clear_error_before_enable"))
        self.force_mode_write = bool(self._get("force_mode_write"))
        self.log_csv = bool(self._get("log_csv"))
        self.log_path = str(self._get("log_path"))
        self.log_every = max(1, int(self._get("log_every")))
        self._csv: Optional[_CsvLogger] = None
        self._log_counter = 0
        if not 1.0 <= self.control_rate <= 1000.0:
            raise RuntimeError("control_rate 必须在 1..1000 Hz")
        if not 1.0 <= self.state_rate <= 1000.0:
            raise RuntimeError("state_rate 必须在 1..1000 Hz")
        if self.command_timeout <= 0.0 or self.feedback_timeout <= 0.0:
            raise RuntimeError("command_timeout / feedback_timeout 必须为正")
        if self.feedback_release_timeout <= 0.0:
            raise RuntimeError("feedback_release_timeout 必须为正")
        if self.max_joint_velocity < 0.0:
            raise RuntimeError("max_joint_velocity 不能为负；0 表示不限速")
        if self.state_refresh_period <= 0.0:
            raise RuntimeError("state_refresh_period 必须为正")
        if any(x < 0.0 for x in self.hold_kp) or any(x < 0.0 for x in self.hold_kd):
            raise RuntimeError("hold_kp / hold_kd 必须非负")

        MotorBus, MotorSpec, models, mode_codes = _load_bus_types(
            str(self._get("hmi_py_path")), self.backend
        )
        self.mode_codes = dict(mode_codes)
        self._valid_models = set(models)
        self._motor_spec_cls = MotorSpec
        for model in self.models:
            if model not in models:
                raise RuntimeError(
                    f"未知型号 {model!r}，可选: {', '.join(sorted(models))}"
                )
        specs = [
            MotorSpec(name, cid, mid, model, ch, "MIT")
            for name, cid, mid, model, ch in zip(
                self.joint_names, self.can_ids, self.mst_ids,
                self.models, self.channels,
            )
        ]
        # ---- 运行时注册表：records 是唯一状态源，下面的数组是派生缓存 ----
        self._enabled_cids: set = set()
        self._sending_cids: set = set()
        self._sign_by_name = dict(zip(self.joint_names, self.joint_signs))
        self._offset_by_name = dict(zip(self.joint_names, self.joint_offsets))
        self._kp_by_name = dict(zip(self.joint_names, self.hold_kp))
        self._kd_by_name = dict(zip(self.joint_names, self.hold_kd))
        self._servo = _JointServo(len(specs))
        self._recs: List[_MotorRec] = self._make_records(specs)
        self._apply_joint_limits()
        self._rebuild_cache()

        # Filled by the SDK receive thread as soon as the bus is open.
        self._lock = threading.Lock()
        self._latest: Dict[int, Tuple[float, float, float, int]] = {}
        self._latest_t: Dict[int, float] = {}
        self._missing_warned = False
        self._online: Dict[int, bool] = {cid: True for cid in self.can_ids}
        self._probed_mode: Dict[int, Optional[int]] = {
            cid: None for cid in self.can_ids
        }

        self._last_refresh = 0.0
        self._tx_dropped_seen = 0
        self._last_cmd: Optional[
            Tuple[List[float], List[float], List[float], List[float], List[float]]
        ] = None
        self._last_cmd_time = 0.0
        self._watchdog_warned = False
        self._warn_times: Dict[str, float] = {}
        self._use_servo = False
        self._ff_gravity = False
        self._ff_scale = 1.0
        self._ff_friction = False
        self._ff_coulomb = 0.10
        self._ff_viscous = 0.05
        self._ff_slope = 100.0
        self._drag_teach = False
        self._grav = None
        self._grav_failed = False
        self._float_logged = False

        self._state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._cmd_pub = self.create_publisher(JointState, "/joint_commands", 10)
        self._motion_pub = self.create_publisher(Bool, "/motor_driver/motion_active", 10)
        self._pending_pub = self.create_publisher(Int32, "/motor_driver/motion_pending", 10)
        self.create_subscription(
            Float64MultiArray, "/mit_command", self._on_mit_command, 10
        )
        self.create_subscription(JointGoal, "/arm_joint_goal", self._on_joint_goal, 10)
        self.create_service(SetBool, "~/enable", self._srv_enable)
        self.create_service(SetBool, "~/sending", self._srv_sending)
        self.create_service(SetBool, "~/set_logging", self._srv_set_logging)
        self.create_service(Trigger, "~/release", self._srv_release)
        self.create_service(SetMotorMode, "~/set_mode", self._srv_set_mode)
        self.create_service(ReadMotorParam, "~/read_param", self._srv_read_param)
        self.create_service(WriteMotorParam, "~/write_param", self._srv_write_param)
        self.create_service(SaveMotorParam, "~/save_param", self._srv_save_param)
        self.create_service(GetMotorStatus, "~/status", self._srv_status)
        self.create_service(
            ConfigureMotors, "~/configure_motors", self._srv_configure_motors
        )
        self.create_service(
            SetMotorEnable, "~/set_motor_enable", self._srv_set_motor_enable
        )
        self.create_service(
            SetMotorSending, "~/set_motor_sending", self._srv_set_motor_sending
        )
        self.create_subscription(
            MotorMitCommand, "/motor_driver/motor_mit", self._on_motor_mit, 20
        )
        self.create_timer(1.0 / self.state_rate, self._publish_state)
        self.create_timer(1.0 / self.control_rate, self._control_tick)

        self._bus = None
        try:
            bus_kwargs = dict(
                nom_baud=int(self._get("nom_baud")),
                dat_baud=int(self._get("dat_baud")),
                sim=self.sim,
                canfd=bool(self._get("canfd")),
                set_baudrate=bool(self._get("set_baudrate")),
                device_index=int(self._get("device_index")),
                device_type=str(self._get("device_type")),
                on_sample=self._on_sample,
            )
            if self.backend == "socketcan":
                bus_kwargs["channel"] = self.can_channel
                bus_kwargs["bitrate"] = self.can_bitrate
                bus_kwargs["channels"] = (
                    self.can_channels or [self.can_channel]
                )
            self._bus = MotorBus(specs, **bus_kwargs)
        except Exception as exc:
            raise RuntimeError(f"打开电机总线失败: {exc}") from exc

        self.get_logger().info(
            "电机驱动已启动：%s，后端 %s，%s，控制 %.0f Hz，状态 %.0f Hz"
            % (
                "仿真" if self.sim else "实机",
                self.backend,
                "CAN FD" if bool(self._get("canfd")) else "经典 CAN",
                self.control_rate,
                self.state_rate,
            )
        )
        self.get_logger().info(
            "关节 (控制ID/反馈ID): "
            + ", ".join(
                f"{s.name}=0x{s.can_id:02X}/0x{s.mst_id:02X}"
                f"({s.model},ch{s.channel})" for s in specs
            )
        )
        self.get_logger().info(
            "轨迹: profile=%s, velocity_feedforward=%s, "
            "max_joint_acceleration=%.2f rad/s^2"
            % (
                self.trajectory_profile,
                self.velocity_feedforward,
                self.max_joint_acceleration,
            )
        )
        self.get_logger().info(
            "软件限位: "
            + ", ".join(
                f"J{i + 1}[{lo:+.3f},{hi:+.3f}]"
                for i, (lo, hi) in enumerate(
                    zip(self.joint_lower, self.joint_upper)
                )
            )
            + " rad"
        )
        if not self.sim:
            self._log_channel_status()
            self._probe_motors()
            self._log_status("使能前状态")
        self.get_logger().warn(
            "电机未使能。确认机械臂周围安全后依次调用: "
            "1) ros2 service call /motor_driver/enable std_srvs/srv/SetBool \"{data: true}\" "
            "2) ros2 service call /motor_driver/sending std_srvs/srv/SetBool \"{data: true}\""
        )
        if self.log_csv:
            try:
                self._start_logging()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error("启动 CSV 记录失败: %s" % exc)

    def destroy_node(self):
        self._stop_logging()
        super().destroy_node()

    def _on_set_parameters(self, params):
        """Allow live trimming of per-joint gravity gain/bias and limits."""
        for p in params:
            if p.name in ("joint_lower", "joint_upper"):
                vals = [float(v) for v in p.value]
                n = len(self.joint_names)
                if len(vals) != n or any(
                    not math.isfinite(v) for v in vals
                ):
                    return SetParametersResult(
                        successful=False,
                        reason="%s 必须是每个关节一个有限数" % p.name)
                if p.name == "joint_lower":
                    lo, hi = vals, list(self.joint_upper)
                else:
                    lo, hi = list(self.joint_lower), vals
                if any(a >= b for a, b in zip(lo, hi)):
                    return SetParametersResult(
                        successful=False,
                        reason="joint_lower 必须小于 joint_upper")
                self.joint_lower = lo
                self.joint_upper = hi
                if hasattr(self, "_recs"):
                    self._apply_joint_limits()
                self.get_logger().info(
                    "joint limits = [%s] .. [%s]" % (
                        ", ".join("%.4f" % v for v in lo),
                        ", ".join("%.4f" % v for v in hi),
                    )
                )
            elif p.name == "gravity_gains":
                vals = list(p.value)
                if (len(vals) != len(self.joint_names)
                        or any(float(v) <= 0.0 or not math.isfinite(float(v))
                               for v in vals)):
                    return SetParametersResult(
                        successful=False,
                        reason="gravity_gains 必须是每个关节一个正数")
                self.gravity_gains = [float(v) for v in vals]
                self.get_logger().info(
                    "gravity_gains = " + str(self.gravity_gains))
            elif p.name == "gravity_biases":
                vals = list(p.value)
                if (len(vals) != len(self.joint_names)
                        or any(not math.isfinite(float(v)) for v in vals)):
                    return SetParametersResult(
                        successful=False,
                        reason="gravity_biases 必须是每个关节一个有限数")
                self.gravity_biases = [float(v) for v in vals]
                self.get_logger().info(
                    "gravity_biases = " + str(self.gravity_biases))
            elif p.name == "trajectory_profile":
                value = str(p.value).strip().lower()
                if value not in ("cosine", "quintic"):
                    return SetParametersResult(
                        successful=False,
                        reason="trajectory_profile 必须是 cosine 或 quintic")
                if self._servo.snapshot()[1]:
                    return SetParametersResult(
                        successful=False,
                        reason="轨迹执行中不能切换 trajectory_profile")
                self.trajectory_profile = value
                self._apply_servo_profile()
                self.get_logger().info(
                    "trajectory_profile = " + value)
            elif p.name == "velocity_feedforward":
                if self._servo.snapshot()[1]:
                    return SetParametersResult(
                        successful=False,
                        reason="轨迹执行中不能切换 velocity_feedforward")
                self.velocity_feedforward = bool(p.value)
                self._apply_servo_profile()
                self.get_logger().info(
                    "velocity_feedforward = %s" % self.velocity_feedforward)
            elif p.name == "max_joint_acceleration":
                value = float(p.value)
                if not math.isfinite(value) or value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="max_joint_acceleration 必须是非负有限数")
                self.max_joint_acceleration = value
                self._apply_servo_profile()
                self.get_logger().info(
                    "max_joint_acceleration = %.2f" % value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------- registry helpers
    @property
    def _active(self) -> bool:
        return bool(self._enabled_cids)

    @property
    def _sending(self) -> bool:
        return bool(self._sending_cids)

    def _apply_joint_limits(self) -> None:
        """Convert URDF software limits into each motor's frame."""
        for rec in self._recs:
            try:
                i = self.joint_names.index(rec.spec.name)
            except ValueError:
                continue
            if not (0 <= i < len(self.joint_lower)):
                continue
            a = float(rec.sign) * float(rec.scale)
            lo = a * self.joint_lower[i] + float(rec.offset)
            hi = a * self.joint_upper[i] + float(rec.offset)
            if lo > hi:
                lo, hi = hi, lo
            rec.q_lo_motor = lo
            rec.q_hi_motor = hi

    def _make_records(self, specs) -> List[_MotorRec]:
        ordered = sorted(
            enumerate(specs),
            key=lambda item: (_joint_index(item[1].name), item[0]),
        )
        recs: List[_MotorRec] = []
        for _idx, spec in ordered:
            q_max, dq_max, tau_max = spec.limits
            name = spec.name
            recs.append(
                _MotorRec(
                    spec=spec,
                    joint_index=_joint_index(name),
                    sign=float(self._sign_by_name.get(name, 1.0)),
                    offset=float(self._offset_by_name.get(name, 0.0)),
                    q_max=float(q_max),
                    dq_max=float(dq_max),
                    tau_max=float(tau_max),
                    hold_kp=float(self._kp_by_name.get(name, 5.0)),
                    hold_kd=float(self._kd_by_name.get(name, 0.3)),
                    scale=float(self.joint_scales[_joint_index(name)]),
                )
            )
        return recs

    def _rebuild_cache(self) -> None:
        """Refresh the legacy arrays from self._recs (source of truth)."""
        recs = self._recs
        self.joint_names = [r.spec.name for r in recs]
        self.can_ids = [int(r.spec.can_id) for r in recs]
        self.mst_ids = [int(r.spec.mst_id) for r in recs]
        self.channels = [int(r.spec.channel) for r in recs]
        self.models = [str(r.spec.model) for r in recs]
        self.joint_signs = [float(r.sign) for r in recs]
        self.joint_offsets = [float(r.offset) for r in recs]
        self.joint_scales = [float(r.scale) for r in recs]
        self.hold_kp = [float(r.hold_kp) for r in recs]
        self.hold_kd = [float(r.hold_kd) for r in recs]
        self._q_max = [float(r.q_max) for r in recs]
        self._dq_max = [float(r.dq_max) for r in recs]
        self._tau_max = [float(r.tau_max) for r in recs]
        self.modes = {int(r.spec.can_id): str(r.spec.mode) for r in recs}
        self._hold_q = [float(r.hold_q) for r in recs]
        self._last_target_q = [float(r.last_target_q) for r in recs]
        self._last_tau = [float(r.last_tau) for r in recs]
        self._last_tau_urdf = [0.0] * len(recs)
        try:
            kp, kd = self._servo.gains()
            kp0 = float(kp[0]) if kp else 5.0
            kd0 = float(kd[0]) if kd else 0.8
        except Exception:
            kp0, kd0 = 5.0, 0.8
        self._servo = _JointServo(len(recs))
        self._servo.set_gains(kp0, kd0)
        self._apply_servo_profile()
        self._rec_by_can = {int(r.spec.can_id): r for r in recs}
        self._enabled_cids &= set(self._rec_by_can)
        self._sending_cids &= set(self._rec_by_can)

    def _apply_servo_profile(self) -> None:
        if not hasattr(self, "_servo"):
            return
        self._servo.configure(
            profile=self.trajectory_profile,
            velocity_feedforward=self.velocity_feedforward,
            max_velocity=self.max_joint_velocity,
            max_acceleration=self.max_joint_acceleration,
        )

    # ------------------------------------------------------------- parameters
    def _get(self, name: str):
        return self.get_parameter(name).value

    def _int_list(self, name: str, n: int) -> List[int]:
        values = list(self._get(name))
        if len(values) != n:
            raise RuntimeError(f"参数 {name} 长度 {len(values)} != 关节数 {n}")
        return [int(v) for v in values]

    def _float_list(self, name: str, n: int) -> List[float]:
        values = list(self._get(name))
        if len(values) != n:
            raise RuntimeError(f"参数 {name} 长度 {len(values)} != 关节数 {n}")
        return [float(v) for v in values]

    def _string_list(self, name: str, n: int) -> List[str]:
        values = list(self._get(name))
        if len(values) != n:
            raise RuntimeError(f"参数 {name} 长度 {len(values)} != 关节数 {n}")
        return [str(v) for v in values]

    # ------------------------------------------------------------- bus wiring
    def _on_sample(self, can_id, t, q, dq, tau, err):
        # Runs in the SDK receive thread: keep it short, never raise.
        cid = int(can_id)
        with self._lock:
            self._latest[cid] = (float(q), float(dq), float(tau), int(err))
            self._latest_t[cid] = time.monotonic()

    def _log_channel_status(self):
        status = getattr(self._bus, "channel_status", {}) or {}
        if self.backend == "socketcan":
            for _ch, st in sorted(status.items()):
                self.get_logger().info(
                    "CAN 通道 %d (%s): operstate=%s driver=%s bitrate=%s"
                    % (_ch, st.get("iface") or self.can_channel,
                       st.get("operstate", "unknown"),
                       st.get("driver") or "未知", st.get("nom"))
                )
            return
        for ch, st in sorted(status.items()):
            if st.get("ok") is False:
                self.get_logger().warn(
                    "通道 %d 波特率回读不符（canfd=%s nom=%s data=%s），"
                    "请核对设备类型与通道号"
                    % (ch, st.get("canfd"), st.get("nom"), st.get("dat"))
                )

    def _probe_motors(self):
        answers = []
        for spec in self._bus.specs:
            value = None
            for _ in range(2):
                try:
                    value = self._bus.read_param(spec.can_id, 10, wait=0.2)
                except Exception:
                    value = None
                if value is not None:
                    break
                time.sleep(0.05)
            answers.append((spec, value))
            self._online[spec.can_id] = value is not None
            self._probed_mode[spec.can_id] = (
                None if value is None else int(value)
            )
        text = ", ".join(
            f"{spec.name}(0x{spec.can_id:02X})="
            + (f"应答(mode={value})" if value is not None else "无应答")
            for spec, value in answers
        )
        if all(value is not None for _, value in answers):
            self.get_logger().info("连接探测: " + text)
        else:
            self.get_logger().warn("连接探测: " + text + "；检查供电/CAN 通道/CAN ID")

    def _send_refresh(self):
        bus = self._bus
        mc = getattr(bus, "mc", None) if bus is not None else None
        if self.sim or mc is None:
            return
        for cid, ch in zip(self.can_ids, self.channels):
            try:
                motor = mc.getMotor(ch, cid)
                if motor is not None:
                    mc.refresh_motor_status(motor)
            except Exception as exc:
                self._warn_key(
                    f"refresh-{cid}",
                    f"刷新状态失败 (CAN 0x{cid:02X}): {exc}",
                    2.0,
                )

    def _log_status(self, tag: str):
        bus = self._bus
        mc = getattr(bus, "mc", None) if bus is not None else None
        if self.sim or mc is None:
            return
        self._send_refresh()
        time.sleep(0.1)
        now = time.monotonic()
        with self._lock:
            snapshot = {cid: self._latest.get(cid) for cid in self.can_ids}
            stamps = {cid: self._latest_t.get(cid) for cid in self.can_ids}
        parts = []
        for name, cid in zip(self.joint_names, self.can_ids):
            state = snapshot[cid]
            if state is None:
                parts.append(f"{name}=无反馈")
            else:
                age = now - stamps[cid] if stamps[cid] is not None else float("inf")
                parts.append(
                    f"{name}: q={state[0]:.3f} rad, err={_err_text(state[3])}, "
                    "反馈年龄 %.2fs" % age
                )
        self.get_logger().info(
            "%s: %s（接收 %.0f Hz）"
            % (tag, ", ".join(parts), float(mc.rx_hz))
        )

    # ------------------------------------------------------------------ state
    def _publish_state(self):
        with self._lock:
            snapshot = {cid: self._latest.get(cid) for cid in self.can_ids}
        missing = [
            name for name, cid in zip(self.joint_names, self.can_ids)
            if snapshot[cid] is None
        ]
        if missing and not self._missing_warned:
            self.get_logger().warn(
                "尚未收到反馈: " + ", ".join(missing) + "；/joint_states 暂按 0 发布"
            )
            self._missing_warned = True
        elif not missing and self._missing_warned:
            self.get_logger().info("全部关节反馈正常")
            self._missing_warned = False

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        for i, cid in enumerate(self.can_ids):
            state = snapshot[cid]
            if state is None:
                msg.position.append(0.0)
                msg.velocity.append(0.0)
                msg.effort.append(0.0)
            else:
                q, dq, tau, _ = state
                sign = self.joint_signs[i]
                scale = self.joint_scales[i]
                msg.position.append(sign * (q - self.joint_offsets[i]) / scale)
                msg.velocity.append(sign * dq / scale)
                msg.effort.append(sign * tau * scale)
        self._state_pub.publish(msg)
        _q_cmd, active, pending = self._servo.snapshot()
        cmd = JointState()
        cmd.header.stamp = msg.header.stamp
        cmd.name = list(self.joint_names)
        cmd.position = [
            r.sign * (r.last_target_q - r.offset) / r.scale
            for r in self._recs
        ]
        cmd.effort = [r.sign * r.last_tau * r.scale for r in self._recs]
        self._cmd_pub.publish(cmd)
        self._motion_pub.publish(Bool(data=bool(active)))
        self._pending_pub.publish(Int32(data=int(pending)))

    # --------------------------------------------------------------- commands
    def _on_mit_command(self, msg: Float64MultiArray):
        if not self._active:
            self._warn_key(
                "inactive", "收到 MIT 指令但电机未使能；先调用 /motor_driver/enable", 5.0
            )
            return
        if not self._sending:
            self._warn_key(
                "not-sending",
                "电机已使能但未开始发送 MIT；先调用 "
                "/motor_driver/sending (data: true)，忽略 /mit_command",
                5.0,
            )
            return

        n = len(self.joint_names)
        data = list(msg.data)
        if len(data) != 5 * n:
            self._warn_key(
                "length",
                f"MIT 指令长度 {len(data)}，应为 5*{n}={5 * n}",
                5.0,
            )
            return
        if not all(math.isfinite(float(v)) for v in data):
            self._warn_key("nonfinite", "MIT 指令含 NaN/Inf，已丢弃", 2.0)
            return

        kp = [float(v) for v in data[0:n]]
        kd = [float(v) for v in data[n:2 * n]]
        q = [float(v) for v in data[2 * n:3 * n]]
        dq = [float(v) for v in data[3 * n:4 * n]]
        tau = [float(v) for v in data[4 * n:5 * n]]
        if any(v < 0.0 for v in kp) or any(v < 0.0 for v in kd):
            self._warn_key("gains", "kp/kd 不能为负，已丢弃本帧", 2.0)
            return

        clipped = False
        for i in range(n):
            sign = self.joint_signs[i]
            scale = self.joint_scales[i]
            q[i] = sign * scale * q[i] + self.joint_offsets[i]
            dq[i] = sign * scale * dq[i]
            tau[i] = sign * tau[i] / scale
            rec_i = self._recs[i] if i < len(self._recs) else None
            if rec_i is not None:
                q_c = self._clamp_rec_q(rec_i, q[i])
            else:
                q_c = min(max(q[i], -self._q_max[i]), self._q_max[i])
            dq_c = min(max(dq[i], -self._dq_max[i]), self._dq_max[i])
            tau_c = min(max(tau[i], -self._tau_max[i]), self._tau_max[i])
            if (q_c, dq_c, tau_c) != (q[i], dq[i], tau[i]):
                clipped = True
            q[i], dq[i], tau[i] = q_c, dq_c, tau_c
        if clipped:
            self._warn_key("clamp", "指令超出软件限位/电机量程，已裁剪", 2.0)

        now = time.monotonic()
        q = self._slew_q(q, now)
        self._use_servo = False
        self._last_cmd = (kp, kd, q, dq, tau)
        self._last_cmd_time = now
        self._last_target_q = list(q)
        self._last_tau = list(tau)
        for i, rec in enumerate(self._recs):
            rec.last_target_q = float(q[i])
            rec.last_tau = float(tau[i])

    def _slew_q(self, q: List[float], now: float) -> List[float]:
        """Limit how fast the commanded motor angle can move.

        The step is measured from the previous command, not from the measured
        angle, so a normal tracking error can still produce holding torque.
        """
        if self.max_joint_velocity <= 0.0:
            return q
        if self._last_cmd_time <= 0.0:
            dt = 1.0 / self.control_rate
        else:
            dt = min(0.05, max(1.0 / self.control_rate, now - self._last_cmd_time))
        step = self.max_joint_velocity * dt
        measured = self._latest_pose(max_age=self.feedback_timeout)
        limited = False
        out = []
        for i, target in enumerate(q):
            # 手拖后上位机从实测角重新起步时，这一帧就在手臂当前位置，不要再被旧指令钉住
            if measured is not None and abs(target - measured[i]) <= 0.1:
                out.append(target)
                continue
            prev = self._last_target_q[i]
            nxt = min(max(target, prev - step), prev + step)
            if nxt != target:
                limited = True
            out.append(nxt)
        if limited:
            self._warn_key(
                "slew",
                "指令跳变超过 %.2f rad/s，已限速" % self.max_joint_velocity,
                2.0,
            )
        return out

    def _clamp_urdf(self, q, where: str):
        """Clip a URDF-frame joint vector to the configured software limits."""
        out = []
        clipped = []
        for i, value in enumerate(q):
            v = float(value)
            lo = float(self.joint_lower[i])
            hi = float(self.joint_upper[i])
            c = min(max(v, lo), hi)
            if c != v:
                clipped.append(i)
            out.append(c)
        if clipped:
            self._warn_key(
                "joint-limit-" + where,
                "%s: %s 超出软件限位，已裁剪" % (
                    where,
                    ", ".join("J%d" % (i + 1) for i in clipped),
                ),
                2.0,
            )
        return out

    def _clamp_rec_q(self, rec: _MotorRec, q: float) -> float:
        """Final per-motor clamp in the motor frame (software + motor range)."""
        lo = max(float(rec.q_lo_motor), -float(rec.q_max))
        hi = min(float(rec.q_hi_motor), float(rec.q_max))
        value = float(q)
        c = min(max(value, lo), hi)
        if c != value:
            self._warn_key(
                "jlim-%d" % int(rec.spec.can_id),
                "%s: 目标超出软件限位，已裁剪" % rec.spec.name,
                5.0,
            )
        return c

    def _on_joint_goal(self, msg: JointGoal) -> None:
        cmd = int(msg.command)
        if cmd in (
            int(JointGoal.MOVE),
            int(JointGoal.APPEND),
            int(JointGoal.DIRECT),
            int(JointGoal.STOP),
            int(JointGoal.DRAG_ON),
        ) and len(self._recs) != 6:
            self._warn_key(
                "registry-partial",
                "关节/轨迹/拖动示教需要注册全部 6 轴；当前注册: "
                + ", ".join(r.spec.name for r in self._recs),
                2.0,
            )
            return
        if cmd == int(JointGoal.GAINS):
            self._servo.set_gains(float(msg.kp), float(msg.kd))
            self._ff_gravity = bool(msg.gravity)
            self._ff_scale = float(msg.gravity_scale)
            self._ff_friction = bool(msg.friction)
            self._ff_coulomb = float(msg.coulomb)
            self._ff_viscous = float(msg.viscous)
            self._ff_slope = float(msg.friction_slope)
            self._use_servo = True
            return
        if cmd == int(JointGoal.DRAG_ON):
            if not self._sending:
                self._warn_key("drag", "未开始发送，不能进入拖动示教", 2.0)
                return
            measured = self._measured_urdf(0.5)
            if measured is None:
                self._warn_key("drag", "没有实测角度，不能进入拖动示教", 2.0)
                return
            self._servo.reset(measured)
            self._drag_teach = True
            self._use_servo = True
            self.get_logger().info("拖动示教：期望角度、速度和重力前馈锁定到当前关节")
            return
        if cmd == int(JointGoal.DRAG_OFF):
            self._drag_teach = False
            measured = self._measured_urdf(1.0)
            if measured is not None:
                self._servo.reset(measured)
            self._use_servo = True
            self.get_logger().info("退出拖动示教，位置锁在当前角度")
            return
        if self._drag_teach:
            self._warn_key("drag-busy", "拖动示教中，先退出再发送运动目标", 2.0)
            return
        if not self._sending:
            self._warn_key(
                "goal-idle", "收到关节目标但未开始发送；先调用 /motor_driver/sending", 2.0
            )
            return
        self._use_servo = True
        if cmd == int(JointGoal.STOP):
            self._servo.stop_here()
            return
        q = [float(v) for v in msg.position]
        if len(q) != len(self.joint_names) or not all(math.isfinite(v) for v in q):
            self._warn_key("goal-q", "关节目标长度或数值无效，已丢弃", 2.0)
            return
        q = self._clamp_urdf(q, "关节目标")
        dwell = max(0.0, float(getattr(msg, "dwell", 0.0)))
        if cmd == int(JointGoal.DIRECT):
            dq = [float(v) for v in getattr(msg, "velocity", [])]
            if (len(dq) != len(q)
                    or not all(math.isfinite(v) for v in dq)):
                dq = None
            self._servo.set_direct(q, dq)
            return
        if cmd == int(JointGoal.APPEND):
            self._servo.append(q, float(msg.duration), dwell)
            return
        measured = self._measured_urdf(0.5)
        if measured is None:
            measured = self._servo.snapshot()[0]
        self._servo.reset(measured)
        self._servo.move_to(q, float(msg.duration), dwell)

    def _measured_urdf(self, max_age: float) -> Optional[List[float]]:
        raw = self._latest_pose(max_age)
        if raw is None:
            return None
        return [
            self.joint_signs[i]
            * (raw[i] - self.joint_offsets[i])
            / self.joint_scales[i]
            for i in range(len(self.joint_names))
        ]

    def _measured_dq_urdf(self) -> List[float]:
        now = time.monotonic()
        with self._lock:
            snapshot = {cid: self._latest.get(cid) for cid in self.can_ids}
            stamps = {cid: self._latest_t.get(cid) for cid in self.can_ids}
        out = []
        for i, cid in enumerate(self.can_ids):
            state = snapshot[cid]
            stamp = stamps[cid]
            if state is None or stamp is None or now - stamp > 0.5:
                out.append(0.0)
            else:
                out.append(
                    self.joint_signs[i] * float(state[1])
                    / self.joint_scales[i])
        return out

    def _measured_tau_urdf(self) -> List[float]:
        now = time.monotonic()
        with self._lock:
            snapshot = {cid: self._latest.get(cid) for cid in self.can_ids}
            stamps = {cid: self._latest_t.get(cid) for cid in self.can_ids}
        out = []
        for i, cid in enumerate(self.can_ids):
            state = snapshot[cid]
            stamp = stamps[cid]
            if state is None or stamp is None or now - stamp > 0.5:
                out.append(float("nan"))
            else:
                out.append(
                    self.joint_signs[i] * float(state[2])
                    * self.joint_scales[i])
        return out

    def _ensure_gravity(self) -> bool:
        if self._grav is not None:
            return True
        if self._grav_failed:
            return False
        try:
            site = os.environ.get(
                "MUJOCO_SITE_PACKAGES",
                "/home/sky/venvs/mujoco/lib/python3.10/site-packages",
            )
            if site and site not in sys.path:
                sys.path.append(site)
            import mujoco
            scene = str(self._get("gravity_scene"))
            model = mujoco.MjModel.from_xml_path(scene)
            qidx = []
            didx = []
            for name in self.joint_names:
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if jid < 0:
                    raise RuntimeError(f"重力模型没有关节 {name}")
                qidx.append(int(model.jnt_qposadr[jid]))
                didx.append(int(model.jnt_dofadr[jid]))
            self._grav = _Gravity(model, qidx, didx)
            self.get_logger().info("重力模型已加载: " + scene)
            return True
        except Exception as exc:
            self._grav_failed = True
            self.get_logger().error("重力模型加载失败，前馈关闭: %s" % exc)
            return False

    def _tau_ff(self, q_urdf: List[float], force_gravity: bool = False) -> List[float]:
        n = len(self.joint_names)
        tau = [0.0] * n
        if (self._ff_gravity or force_gravity) and self._ensure_gravity():
            bias = self._grav.bias(q_urdf)
            tau = [
                self._ff_scale * self.gravity_gains[i] * float(bias[i])
                + self.gravity_biases[i]
                for i in range(n)
            ]
        if self._ff_friction:
            dq = self._measured_dq_urdf()
            for i in range(n):
                tau[i] += (
                    self._ff_coulomb * math.tanh(self._ff_slope * dq[i])
                    + self._ff_viscous * dq[i]
                )
        for i, cid in enumerate(self.can_ids):
            if self.modes.get(cid, "MIT") not in ("MIT", "?"):
                tau[i] = 0.0
        self._last_tau_urdf = list(tau)
        return tau

    def _has_velocity_mode(self) -> bool:
        """True when any registered motor is in a mode that consumes dq."""
        return any(
            str(rec.spec.mode) in ("POS_VEL", "VEL")
            for rec in self._recs
        )

    def _servo_step(self, now: float) -> None:
        q_urdf, dq_urdf, _ddq_urdf = self._servo.sample_full(now)
        kp, kd = self._servo.gains()
        measured = self._measured_urdf(0.5)
        if self._drag_teach and measured is not None:
            self._servo.follow_measured(measured)
            q_urdf = measured
            dq_urdf = self._measured_dq_urdf()
            tau = self._tau_ff(measured, force_gravity=True)
        else:
            # POS_VEL / VEL 指令需要 dq；MIT 只有开启 velocity_feedforward
            # 才用。没有速度需求时保持旧行为（dq=0），不变。
            if not (self.velocity_feedforward
                    or self._servo.direct_velocity_active()
                    or self._has_velocity_mode()):
                dq_urdf = None
            tau = self._tau_ff(measured if measured is not None else q_urdf)
        self._commit_urdf(kp, kd, q_urdf, tau, now, dq_urdf)

    def _commit_urdf(self, kp, kd, q_urdf, tau_urdf, now: float,
                     dq_urdf=None) -> None:
        n = len(self.joint_names)
        q_urdf = self._clamp_urdf(list(q_urdf), "伺服")
        q = []
        dq = []
        tau = []
        for i in range(n):
            sign = self.joint_signs[i]
            scale = self.joint_scales[i]
            q.append(sign * scale * float(q_urdf[i]) + self.joint_offsets[i])
            if dq_urdf is None:
                dq.append(0.0)
            else:
                dq.append(sign * scale * float(dq_urdf[i]))
            tau.append(sign * float(tau_urdf[i]) / scale)
        q = self._slew_q(q, now)
        kp_l = [float(v) for v in kp]
        kd_l = [float(v) for v in kd]
        self._last_cmd = (kp_l, kd_l, q, dq, tau)
        self._last_cmd_time = now
        self._last_target_q = list(q)
        self._last_tau = list(tau)
        for i, rec in enumerate(self._recs):
            rec.last_target_q = float(q[i])
            rec.last_tau = float(tau[i])

    def _hold_command(self):
        return (
            list(self.hold_kp),
            list(self.hold_kd),
            list(self._last_target_q),
            [0.0] * len(self.joint_names),
            list(self._last_tau),
        )

    def _selected_command(self, now: float):
        timed_out = (
            self._last_cmd is not None
            and now - self._last_cmd_time > self.command_timeout
        )
        if timed_out:
            if not self._watchdog_warned:
                self.get_logger().warn(
                    "超过 %.2f s 没有新的 /mit_command，保持最后一帧目标位姿"
                    % self.command_timeout
                )
                self._watchdog_warned = True
            return self._hold_command()
        if self._watchdog_warned:
            self.get_logger().info("收到新的 MIT 指令，退出保持")
            self._watchdog_warned = False
        if self._last_cmd is None:
            return self._hold_command()
        return self._last_cmd

    # -------------------------------------------------------------- control loop
    def _command_for(self, rec: _MotorRec, idx: int, base, now: float):
        """Pick this motor's command: per-motor override > zero > hold > global."""
        if rec.per_cmd is not None:
            timeout = (
                rec.per_cmd_timeout
                if rec.per_cmd_timeout > 0.0
                else self.command_timeout
            )
            if now - rec.per_cmd_t <= timeout:
                result = rec.per_cmd
            else:
                result = (
                    rec.hold_kp, rec.hold_kd, rec.last_target_q, 0.0, 0.0
                )
        elif rec.zero_torque:
            result = (0.0, 0.0, rec.hold_q, 0.0, 0.0)
        elif base is not None and len(base[0]) == len(self._recs):
            result = (
                float(base[0][idx]), float(base[1][idx]),
                float(base[2][idx]), float(base[3][idx]), float(base[4][idx]),
            )
        elif not rec.pose_known:
            result = (0.0, 0.0, rec.hold_q, 0.0, 0.0)
        else:
            result = (rec.hold_kp, rec.hold_kd, rec.hold_q, 0.0, 0.0)
        # 最后一道闸：无论目标来自关节/轨迹/单轴/外部 MIT，都在这里裁剪。
        return (
            float(result[0]), float(result[1]),
            self._clamp_rec_q(rec, result[2]),
            float(result[3]), float(result[4]),
        )

    def _control_tick(self):
        bus = self._bus
        if bus is None:
            return
        now = time.monotonic()
        if self.sim:
            try:
                bus.sample_sim(now)
            except Exception as exc:
                self._warn_key("sim", f"仿真采样失败: {exc}", 2.0)
        self._check_tx_drops(bus)
        if not self._sending_cids:
            # 不发控制帧时达妙 MIT 不主动回帧，用 0xCC 查询让状态仍然可见。
            if now - self._last_refresh >= self.state_refresh_period:
                self._last_refresh = now
                self._send_refresh()
            self._check_tx_drops(bus)
            return

        base = None
        if self._use_servo and len(self._recs) == 6:
            self._servo_step(now)
            base = self._last_cmd
        elif self._last_cmd is not None:
            base = self._selected_command(now)

        # 只有 CSV 记录打开时才构建行；否则每 tick 白建 6 个 tuple。
        rows = [] if self._csv is not None else None
        for idx, rec in enumerate(self._recs):
            cid = int(rec.spec.can_id)
            mode = str(rec.spec.mode)
            kp, kd, q, dq, tau = self._command_for(rec, idx, base, now)
            if rows is not None:
                rows.append((kp, kd, q, dq, tau, mode))
            if cid not in self._sending_cids:
                continue
            try:
                if mode == "POS_VEL":
                    bus.pos_vel(cid, q, dq)
                elif mode == "VEL":
                    bus.vel(cid, dq)
                elif mode == "POS_FORCE":
                    bus.pos_force(cid, q, tau)
                else:
                    bus.mit(cid, kp, kd, q, dq, tau)
            except Exception as exc:
                self._warn_key(
                    f"mit-{cid}", f"发送控制帧失败 (CAN 0x{cid:02X}): {exc}", 2.0
                )
        if rows is not None:
            self._log_sample(now, rows)
        self._check_tx_drops(bus)
        self._check_feedback(now)

    def _check_tx_drops(self, bus) -> None:
        """非阻塞发送丢帧时，按 2 s 限频告警一次（不刷屏）。"""
        dropped = int(getattr(bus, "tx_dropped", 0))
        if dropped <= self._tx_dropped_seen:
            return
        delta = dropped - self._tx_dropped_seen
        self._tx_dropped_seen = dropped
        err = getattr(bus, "last_tx_error", None)
        self._warn_key(
            "tx-drop",
            f"CAN 发送丢帧 {delta} 次（累计 {dropped}）"
            + (f": {err}" if err else "")
            + "；检查 USB/CAN 负载或总线状态",
            2.0,
        )

    def _check_feedback(self, now: float):
        if not self._sending_cids:
            return
        stale = []
        for cid in list(self._sending_cids):
            rec = self._rec_by_can.get(int(cid))
            if rec is None:
                continue
            stamp = self._latest_t.get(int(cid))
            if stamp is None or now - stamp > self.feedback_timeout:
                stale.append(rec.spec.name)
                if rec.stale_since is None:
                    rec.stale_since = now
                if now - rec.stale_since >= self.feedback_release_timeout:
                    self.get_logger().error(
                        "%s 反馈持续超过 %.1f s，单轴零力矩并失能"
                        % (rec.spec.name, self.feedback_release_timeout)
                    )
                    self._disable_cids([int(cid)], reason="feedback timeout")
            else:
                rec.stale_since = None
        if stale:
            self._warn_key(
                "feedback",
                "反馈超时: " + ", ".join(stale) + "；仍保持目标位姿，请检查 CAN",
                2.0,
            )

    # ----------------------------------------------------------------- services
    def _srv_enable(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            ok, message = self._enable_cids(list(self._rec_by_can))
        else:
            ok, message = self._disable_cids(list(self._enabled_cids))
        if ok:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)
        response.success = ok
        response.message = message
        return response

    def _srv_sending(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            ok, message = self._set_sending_cids(
                list(self._enabled_cids), True
            )
        else:
            ok, message = self._set_sending_cids(
                list(self._sending_cids), False
            )
        if ok:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)
        response.success = ok
        response.message = message
        return response

    # -------------------------------------------------- identification logging
    def _csv_header(self) -> List[str]:
        n = len(self.joint_names)
        names = ["j%d" % (i + 1) for i in range(n)]
        cols = ["t_mono", "t_ros"]
        for p in ("q", "dq", "tau", "q_des", "dq_des",
                  "tau_cmd", "kp", "kd", "mode"):
            cols += ["%s_%s" % (p, name) for name in names]
        cols += ["enabled", "sending"]
        return cols

    def _start_logging(self) -> str:
        if self._csv is not None:
            return self._csv.path
        path = Path(self.log_path).expanduser()
        stamp = time.strftime("%Y%m%d-%H%M%S")
        if "{stamp}" in str(path):
            path = Path(str(path).replace("{stamp}", stamp))
        else:
            path = path.with_name(path.stem + "_" + stamp + path.suffix)
        path.parent.mkdir(parents=True, exist_ok=True)
        logger = _CsvLogger(str(path), self._csv_header())
        logger.start()
        self._csv = logger
        self._log_counter = 0
        self.get_logger().info("CSV 记录开始: " + str(path))
        return str(path)

    def _stop_logging(self) -> str:
        logger = self._csv
        self._csv = None
        if logger is None:
            return ""
        logger.stop()
        self.get_logger().info(
            "CSV 记录结束: %s（丢弃 %d 行）" % (logger.path, logger.dropped))
        return logger.path

    def _srv_set_logging(self, request: SetBool.Request,
                         response: SetBool.Response):
        try:
            if request.data:
                path = self._start_logging()
                response.success = True
                response.message = "logging: " + path
            else:
                path = self._stop_logging()
                response.success = True
                response.message = "stopped: " + (path or "not logging")
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
        return response

    def _log_sample(self, now: float, rows) -> None:
        logger = self._csv
        if logger is None:
            return
        self._log_counter += 1
        if self._log_counter % self.log_every:
            return
        n = len(self.joint_names)
        q = self._measured_urdf(0.5)
        if q is None:
            q = [float("nan")] * n
        dq = self._measured_dq_urdf()
        tau = self._measured_tau_urdf()
        q_des, dq_des, tau_cmd = [], [], []
        kp, kd, modes = [], [], []
        for i, (r_kp, r_kd, r_q, r_dq, r_tau, mode) in enumerate(rows):
            sign = float(self.joint_signs[i])
            off = float(self.joint_offsets[i])
            scale = float(self.joint_scales[i])
            if abs(sign) < 1e-9:
                sign = 1.0
            if scale <= 0.0:
                scale = 1.0
            q_des.append((float(r_q) - off) / (sign * scale))
            dq_des.append(float(r_dq) / (sign * scale))
            tau_cmd.append(float(r_tau) * scale / sign)
            kp.append(float(r_kp))
            kd.append(float(r_kd))
            modes.append(str(mode))
        row = [now, self.get_clock().now().nanoseconds * 1e-9]
        row += list(q) + list(dq) + list(tau)
        row += q_des + dq_des + tau_cmd + kp + kd + modes
        row += [int(bool(self._enabled_cids)), int(bool(self._sending_cids))]
        logger.log(row)

    def _srv_release(self, _request: Trigger.Request, response: Trigger.Response):
        ok, message = self._disable_cids(list(self._enabled_cids))
        if ok:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)
        response.success = ok
        response.message = message
        return response

    # ---------------------------------------------------- per-motor services
    def _srv_configure_motors(self, request, response):
        try:
            entries = [
                {
                    "joint_name": str(e.joint_name),
                    "can_id": int(e.can_id),
                    "mst_id": int(e.mst_id),
                    "model": str(e.model),
                    "mode": str(e.mode),
                    "channel": int(e.channel),
                    "sign": float(e.sign),
                    "offset": float(e.offset),
                }
                for e in request.motors
            ]
            ok, message, joint_names, can_ids = self._apply_registry(
                entries, bool(request.apply)
            )
        except Exception as exc:
            ok, message, joint_names, can_ids = False, str(exc), [], []
        response.success = ok
        response.message = message
        response.joint_names = joint_names
        response.can_ids = can_ids
        if ok:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)
        return response

    def _srv_set_motor_enable(self, request, response):
        cids = [int(c) for c in request.can_ids]
        if request.all or not cids:
            cids = list(self._rec_by_can)
        if not cids:
            response.success = False
            response.message = "没有注册电机"
            response.enabled_can_ids = sorted(self._enabled_cids)
            response.failed_can_ids = []
            return response
        if request.enable:
            ok, message = self._enable_cids(cids)
            failed = [c for c in cids if c not in self._enabled_cids]
        else:
            ok, message = self._disable_cids(cids)
            failed = [c for c in cids if c in self._enabled_cids]
        response.success = ok
        response.message = message
        response.enabled_can_ids = sorted(self._enabled_cids)
        response.failed_can_ids = failed
        if ok:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)
        return response

    def _srv_set_motor_sending(self, request, response):
        cids = [int(c) for c in request.can_ids]
        if request.all or not cids:
            cids = list(self._enabled_cids if request.sending else self._sending_cids)
        if not cids:
            response.success = False
            response.message = (
                "没有已使能电机" if request.sending else "当前没有在发送的电机"
            )
            response.sending_can_ids = sorted(self._sending_cids)
            response.failed_can_ids = []
            return response
        ok, message = self._set_sending_cids(cids, bool(request.sending))
        if request.sending:
            failed = [c for c in cids if c not in self._sending_cids]
        else:
            failed = [c for c in cids if c in self._sending_cids]
        response.success = ok
        response.message = message
        response.sending_can_ids = sorted(self._sending_cids)
        response.failed_can_ids = failed
        if ok:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)
        return response

    @staticmethod
    def _param_is_int(rid: int) -> bool:
        # 与 SDK 的 Motor_Control.is_in_ranges 一致：这些 RID 是整数寄存器
        return (7 <= rid <= 10) or (13 <= rid <= 16) or (35 <= rid <= 36)

    def _srv_set_mode(self, request, response):
        cid = int(request.can_id)
        mode = str(request.mode).strip().upper()
        response.actual_mode = self.modes.get(cid, "")
        if cid not in self._rec_by_can:
            response.success = False
            response.message = f"CAN 0x{cid:02X} 不在注册表里"
            return response
        if mode not in self.mode_codes:
            response.success = False
            response.message = f"未知模式 {mode!r}；可选 {', '.join(self.mode_codes)}"
            return response
        if cid in self._sending_cids:
            response.success = False
            response.message = "先停止该轴发送控制帧，再切换模式"
            return response
        if cid in self._enabled_cids:
            response.success = False
            response.message = "先失能（0xFD）再切换模式"
            return response
        ok = bool(self._bus.set_mode(cid, mode)) if self._bus is not None else False
        if ok:
            self.modes[cid] = mode
            rec = self._rec_by_can.get(cid)
            if rec is not None:
                rec.spec.mode = mode
            self._probed_mode[cid] = self.mode_codes[mode]
            response.message = f"CAN 0x{cid:02X} 已切到 {mode}"
            self.get_logger().info(response.message)
        else:
            response.message = f"CAN 0x{cid:02X} 切换 {mode} 失败（驱动器未确认）"
            self.get_logger().warn(response.message)
        response.success = ok
        if self._bus is not None:
            response.actual_mode = self._bus.mode_of(cid)
        return response

    def _srv_read_param(self, request, response):
        cid = int(request.can_id)
        rid = int(request.rid)
        if cid not in self._rec_by_can:
            response.success = False
            response.message = f"CAN 0x{cid:02X} 不在注册表里"
            return response
        value = self._bus.read_param(cid, rid, wait=0.25) if self._bus else None
        if value is None:
            response.success = False
            response.message = "读取超时/无回复"
            return response
        response.success = True
        response.value = float(value)
        response.is_integer = self._param_is_int(rid)
        response.message = f"CAN 0x{cid:02X} RID {rid} = {value}"
        return response

    def _srv_write_param(self, request, response):
        cid = int(request.can_id)
        rid = int(request.rid)
        if cid not in self._rec_by_can:
            response.success = False
            response.message = f"CAN 0x{cid:02X} 不在注册表里"
            return response
        if rid == 10:
            response.success = False
            response.message = "CTRL_MODE 请用 ~/set_mode 切换，避免状态不一致"
            return response
        if cid in self._sending_cids:
            response.success = False
            response.message = "先停止该轴发送控制帧，再写参数"
            return response
        value = int(round(float(request.value))) if request.is_integer else float(request.value)
        try:
            acked = self._bus.write_param(cid, rid, value)
        except Exception as exc:
            response.success = False
            response.message = f"写入失败: {exc}"
            return response
        check = self._bus.read_param(cid, rid, wait=0.2) if self._bus else None
        if check is None:
            response.success = False
            response.message = (
                f"写入 RID {rid} ← {value}：未收到回读"
                + ("，写应答也缺失" if acked is False else "")
            )
            self.get_logger().warn(response.message)
            return response
        if request.is_integer:
            match = int(round(float(check))) == int(value)
        else:
            match = abs(float(check) - value) <= max(1e-6, 1e-4 * abs(value))
        response.success = bool(match)
        if match:
            response.message = f"已写入 RID {rid} ← {value}，回读 {check}"
            self.get_logger().info(response.message)
        else:
            response.message = (
                f"写入 RID {rid} ← {value}，但回读 {check} 不一致"
                "（写入可能未生效）"
            )
            self.get_logger().warn(response.message)
        return response

    def _srv_save_param(self, request, response):
        cid = int(request.can_id)
        if cid not in self._rec_by_can:
            response.success = False
            response.message = f"CAN 0x{cid:02X} 不在注册表里"
            return response
        if cid in self._enabled_cids or cid in self._sending_cids:
            response.success = False
            response.message = (
                "先失能该轴再保存 Flash。保存流程会发 0xFD，"
                "使能状态下保存会让这一轴偷偷失能"
            )
            return response
        try:
            acked = self._bus.save(cid)
        except Exception as exc:
            response.success = False
            response.message = f"保存失败: {exc}"
            return response
        if acked is False:
            response.success = False
            response.message = (
                f"CAN 0x{cid:02X} 保存未收到 0xAA 应答；"
                "参数可能未写入 Flash，请重试"
            )
            self.get_logger().warn(response.message)
            return response
        response.success = True
        response.message = f"CAN 0x{cid:02X} 参数已保存到 Flash"
        return response

    def _srv_status(self, _request, response):
        now = time.monotonic()
        with self._lock:
            snapshot = {cid: self._latest.get(cid) for cid in self.can_ids}
            stamps = {cid: self._latest_t.get(cid) for cid in self.can_ids}
        response.success = True
        response.message = "ok"
        response.enabled = bool(self._active)
        response.sending = bool(self._sending)
        response.can_ids = [int(cid) for cid in self.can_ids]
        response.modes = [str(self.modes.get(cid, "MIT")) for cid in self.can_ids]
        response.joint_names = [r.spec.name for r in self._recs]
        response.joint_indices = [int(r.joint_index) for r in self._recs]
        response.mst_ids = [int(r.spec.mst_id) for r in self._recs]
        response.models = [str(r.spec.model) for r in self._recs]
        response.channels = [int(r.spec.channel) for r in self._recs]
        response.signs = [float(r.sign) for r in self._recs]
        response.offsets = [float(r.offset) for r in self._recs]
        response.enabled_each = [
            bool(cid in self._enabled_cids) for cid in self.can_ids
        ]
        response.sending_each = [
            bool(cid in self._sending_cids) for cid in self.can_ids
        ]
        response.zero_torque_each = [bool(r.zero_torque) for r in self._recs]
        response.hold_q = [
            r.sign * (r.hold_q - r.offset) / r.scale for r in self._recs
        ]
        for i, cid in enumerate(self.can_ids):
            state = snapshot[cid]
            stamp = stamps[cid]
            response.age.append(
                999.0 if stamp is None else max(0.0, now - float(stamp))
            )
            if state is None:
                response.q.append(0.0)
                response.dq.append(0.0)
                response.tau.append(0.0)
                response.err.append(-1)
                continue
            q, dq, tau, err = state
            sign = self.joint_signs[i]
            scale = self.joint_scales[i]
            response.q.append(sign * (q - self.joint_offsets[i]) / scale)
            response.dq.append(sign * dq / scale)
            response.tau.append(sign * tau * scale)
            response.err.append(int(err))
        return response

    # ---------------------------------------------------- per-motor helpers
    def _fresh_pose_for(self, cids, max_age: float = 1.0):
        """Refresh only the requested motors; return {can_id: motor_q} or None."""
        bus = self._bus
        if bus is None:
            return None
        cids = [int(c) for c in cids]
        if self.sim:
            bus.sample_sim(time.monotonic())
            states = bus.states()
            out = {}
            for cid in cids:
                st = states.get(cid)
                if st is None:
                    return None
                out[cid] = float(st.get("q", 0.0))
            return out
        mc = getattr(bus, "mc", None)
        if mc is not None:
            for cid in cids:
                motor = bus._motor(cid)
                if motor is not None:
                    try:
                        mc.refresh_motor_status(motor)
                    except Exception:
                        pass
        time.sleep(0.1)
        now = time.monotonic()
        out = {}
        with self._lock:
            for cid in cids:
                state = self._latest.get(cid)
                stamp = self._latest_t.get(cid)
                if state is None or stamp is None or now - stamp > max_age:
                    return None
                out[cid] = float(state[0])
        return out

    def _wait_pose(self, cid: int, timeout: float):
        """Wait for a feedback frame after a zero-torque probe."""
        start = time.monotonic()
        if self.sim:
            self._bus.sample_sim(time.monotonic())
            st = self._bus.states().get(cid)
            return None if st is None else float(st.get("q", 0.0))
        while time.monotonic() - start < timeout:
            with self._lock:
                state = self._latest.get(cid)
                stamp = self._latest_t.get(cid)
            if state is not None and stamp is not None and stamp >= start:
                return float(state[0])
            time.sleep(0.005)
        with self._lock:
            state = self._latest.get(cid)
            stamp = self._latest_t.get(cid)
        if (
            state is not None
            and stamp is not None
            and time.monotonic() - stamp <= timeout
        ):
            return float(state[0])
        return None

    def _slew_rec(self, rec: _MotorRec, q: float, now: float) -> float:
        if self.max_joint_velocity <= 0.0:
            return q
        cid = int(rec.spec.can_id)
        with self._lock:
            state = self._latest.get(cid)
            stamp = self._latest_t.get(cid)
        if (
            state is not None
            and stamp is not None
            and now - stamp <= self.feedback_timeout
            and abs(q - float(state[0])) <= 0.1
        ):
            return q
        prev_t = (
            rec.per_cmd_t
            if rec.per_cmd_t > 0.0
            else now - 1.0 / self.control_rate
        )
        dt = min(0.05, max(1.0 / self.control_rate, now - prev_t))
        step = self.max_joint_velocity * dt
        return min(max(q, rec.last_target_q - step), rec.last_target_q + step)

    def _on_motor_mit(self, msg: MotorMitCommand):
        cid = int(msg.can_id)
        rec = self._rec_by_can.get(cid)
        if rec is None:
            self._warn_key(
                "mit-unknown", f"单轴指令: CAN 0x{cid:02X} 未注册", 2.0
            )
            return
        if not rec.enabled:
            self._warn_key(
                "mit-disabled",
                f"单轴指令: {rec.spec.name} 未使能，忽略",
                2.0,
            )
            return
        if cid not in self._sending_cids:
            self._warn_key(
                "mit-nosend",
                f"单轴指令: {rec.spec.name} 未开始发送，先点『发送数据』",
                2.0,
            )
            return
        now = time.monotonic()
        cmd = int(msg.command)
        mode = str(rec.spec.mode)
        if cmd == int(MotorMitCommand.ZERO_TORQUE):
            if mode != "MIT":
                self._warn_key(
                    "mit-zero-mode",
                    f"{rec.spec.name}: 零力矩只在 MIT 模式下支持",
                    2.0,
                )
                return
            for _ in range(5):
                try:
                    self._bus.mit(cid, 0.0, 0.0, rec.hold_q, 0.0, 0.0)
                except Exception:
                    pass
                time.sleep(0.002)
            rec.zero_torque = True
            rec.per_cmd = None
            rec.last_target_q = rec.hold_q
            rec.last_tau = 0.0
            return
        if cmd == int(MotorMitCommand.HOLD):
            q = rec.hold_q if rec.pose_known else rec.last_target_q
            rec.per_cmd = (rec.hold_kp, rec.hold_kd, q, 0.0, 0.0)
            rec.per_cmd_t = now
            rec.per_cmd_timeout = 0.0
            rec.zero_torque = False
            return
        # MIT setpoint (URDF frame -> motor frame)
        kp = max(0.0, float(msg.kp))
        kd = max(0.0, float(msg.kd))
        q = rec.sign * rec.scale * float(msg.q) + rec.offset
        dq = rec.sign * rec.scale * float(msg.dq)
        tau = rec.sign * float(msg.tau) / rec.scale
        q = self._clamp_rec_q(rec, q)
        dq = min(max(dq, -rec.dq_max), rec.dq_max)
        tau = min(max(tau, -rec.tau_max), rec.tau_max)
        q = self._slew_rec(rec, q, now)
        rec.per_cmd = (kp, kd, q, dq, tau)
        rec.per_cmd_t = now
        rec.per_cmd_timeout = float(msg.timeout)
        rec.last_target_q = q
        rec.last_tau = tau
        rec.zero_torque = False

    def _enable_cids(self, cids) -> Tuple[bool, str]:
        bus = self._bus
        if bus is None:
            return False, "总线未打开"
        cids = [int(c) for c in cids]
        failed = []
        for cid in cids:
            rec = self._rec_by_can.get(cid)
            if rec is None:
                failed.append(cid)
                continue
            if rec.enabled:
                continue
            mode = str(rec.spec.mode)
            try:
                if not self.sim and self.clear_error_before_enable:
                    for _ in range(3):
                        bus.mc.control_cmd(cid, 0xFB, rec.spec.channel)
                        time.sleep(0.003)
                if mode != "MIT":
                    poses = self._fresh_pose_for([cid], max_age=1.0)
                    if poses is None:
                        raise RuntimeError(
                            "非 MIT 模式使能前读不到当前位置（0xCC 无应答）"
                        )
                    rec.hold_q = poses[cid]
                    rec.last_target_q = poses[cid]
                    rec.pose_known = True
                switch = self.force_mode_write or (
                    self._probed_mode.get(cid)
                    != self.mode_codes.get(mode, _MIT_MODE_CODE)
                )
                bus.enable(cid, True, switch_mode=switch)
                if switch:
                    self._clear_param_gate(bus, cids=[cid])
                if mode == "MIT":
                    probe_q = rec.hold_q
                    if self.sim:
                        st = bus.states().get(cid)
                        if st is not None:
                            probe_q = float(st.get("q", 0.0))
                    bus.mit(cid, 0.0, 0.0, probe_q, 0.0, 0.0)
                    pose = self._wait_pose(cid, timeout=self.feedback_timeout)
                    if pose is None:
                        raise RuntimeError("使能后未收到反馈")
                    rec.hold_q = pose
                    rec.last_target_q = pose
                rec.last_tau = 0.0
                rec.pose_known = True
                rec.zero_torque = False
                rec.per_cmd = None
                rec.per_cmd_t = 0.0
                rec.enabled = True
                rec.sending = False
                self._enabled_cids.add(cid)
                self._online[cid] = True
                self._probed_mode[cid] = self.mode_codes.get(mode)
            except Exception as exc:
                failed.append(cid)
                try:
                    bus.release(cid)
                except Exception:
                    pass
                try:
                    bus.enable(cid, False)
                except Exception:
                    pass
                rec.enabled = False
                rec.sending = False
                rec.pose_known = False
                rec.per_cmd = None
                self._enabled_cids.discard(cid)
                self._sending_cids.discard(cid)
                self.get_logger().warn(
                    f"使能 CAN 0x{cid:02X} 失败: {exc}"
                )
        if self._enabled_cids:
            self._last_refresh = time.monotonic()
        names = [
            self._rec_by_can[c].spec.name
            for c in sorted(self._enabled_cids)
            if c in self._rec_by_can
        ]
        if failed:
            return False, (
                "部分使能失败: "
                + ", ".join(f"0x{c:02X}" for c in failed)
                + "；已使能: "
                + (", ".join(names) or "无")
            )
        if not self._enabled_cids:
            return False, "没有电机被使能"
        return True, (
            "已使能: " + ", ".join(names)
            + "（仅 0xFC/零力矩探测，尚未开始发送）"
        )

    def _disable_cids(self, cids, reason: str = "") -> Tuple[bool, str]:
        bus = self._bus
        if bus is None:
            return False, "总线未打开"
        errors = []
        cids = [int(c) for c in cids]
        for cid in cids:
            rec = self._rec_by_can.get(cid)
            if rec is None:
                continue
            self._sending_cids.discard(cid)
            try:
                bus.release(cid)
            except Exception as exc:
                errors.append(f"release 0x{cid:02X}: {exc}")
            try:
                bus.enable(cid, False)
            except Exception as exc:
                errors.append(f"disable 0x{cid:02X}: {exc}")
            rec.enabled = False
            rec.sending = False
            rec.pose_known = False
            rec.zero_torque = False
            rec.per_cmd = None
            rec.stale_since = None
            rec.last_tau = 0.0
            self._enabled_cids.discard(cid)
        self._use_servo = False
        self._last_cmd = None
        self._last_cmd_time = 0.0
        if not self._enabled_cids:
            self._drag_teach = False
        msg = (
            "已失能: "
            + (", ".join(f"0x{c:02X}" for c in cids) or "无")
            + (f"（{reason}）" if reason else "")
        )
        if errors:
            return False, msg + "；" + "；".join(errors)
        return True, msg

    def _set_sending_cids(self, cids, on: bool) -> Tuple[bool, str]:
        bus = self._bus
        if bus is None:
            return False, "总线未打开"
        cids = [int(c) for c in cids]
        failed = []
        if on:
            if not cids:
                return False, "没有已使能电机"
            for cid in cids:
                rec = self._rec_by_can.get(cid)
                if rec is None or not rec.enabled:
                    failed.append(cid)
                    continue
                self._sending_cids.add(cid)
                rec.sending = True
                rec.zero_torque = False
                rec.stale_since = None
            if failed:
                return False, (
                    "未使能/未注册，拒绝发送: "
                    + ", ".join(f"0x{c:02X}" for c in failed)
                )
            self._last_refresh = 0.0
            return True, (
                "开始发送: "
                + ", ".join(f"0x{c:02X}" for c in sorted(self._sending_cids))
            )
        for cid in cids:
            self._sending_cids.discard(cid)
            rec = self._rec_by_can.get(cid)
            if rec is not None:
                rec.sending = False
                rec.zero_torque = False
                try:
                    if rec.pose_known:
                        bus.mit(
                            cid, rec.hold_kp, rec.hold_kd,
                            rec.last_target_q, 0.0, 0.0,
                        )
                    else:
                        bus.mit(cid, 0.0, 0.0, rec.last_target_q, 0.0, 0.0)
                except Exception:
                    pass
        if not self._sending_cids:
            self._use_servo = False
            self._last_cmd = None
            self._last_cmd_time = 0.0
        return True, (
            "停止发送: "
            + (", ".join(f"0x{c:02X}" for c in cids) or "无")
        )

    def _apply_registry(self, entries, apply: bool):
        bus = self._bus
        if bus is None:
            raise RuntimeError("总线未打开")
        if not entries:
            raise ValueError("注册表为空")
        if len(entries) > 6:
            raise ValueError("最多注册 6 台电机")
        valid_joints = {f"joint{i}" for i in range(1, 7)}
        seen_joint = set()
        seen_id = set()
        parsed = []
        for e in entries:
            jn = str(e["joint_name"]).strip().lower()
            if jn not in valid_joints:
                raise ValueError(f"关节名必须是 joint1..joint6: {jn!r}")
            if jn in seen_joint:
                raise ValueError(f"关节重复: {jn}")
            seen_joint.add(jn)
            cid = int(e["can_id"])
            mid = int(e["mst_id"])
            ch = int(e["channel"])
            model = str(e["model"])
            mode = str(e["mode"]).strip().upper()
            sign = float(e["sign"])
            offset = float(e["offset"])
            if not (0 < cid <= 0x7FF) or not (0 < mid <= 0x7FF):
                raise ValueError(
                    f"CAN/MST ID 必须在 1..0x7FF: 0x{cid:02X}/0x{mid:02X}"
                )
            if model not in self._valid_models:
                raise ValueError(f"未知型号 {model!r}")
            if mode not in self.mode_codes:
                raise ValueError(f"未知模式 {mode!r}")
            if ch not in bus._open_channels:
                raise ValueError(
                    f"通道 {ch} 未在驱动启动时打开；换通道需要失能后重启驱动"
                )
            if sign not in (-1.0, 1.0):
                raise ValueError(f"{jn}: 方向只能是 +1 或 -1")
            if not math.isfinite(offset):
                raise ValueError(f"{jn}: 零位必须是有限数")
            # CAN 和 MST 可以是同一个值（常规配置），只和别的电机比较
            for ident in {cid, mid}:
                key = (ch, ident)
                if key in seen_id:
                    raise ValueError(
                        f"通道 {ch} 的 CAN/MST ID 0x{ident:02X} 冲突"
                    )
            seen_id.update({(ch, cid), (ch, mid)})
            parsed.append(
                {
                    "joint_name": jn,
                    "can_id": cid,
                    "mst_id": mid,
                    "model": model,
                    "mode": mode,
                    "channel": ch,
                    "sign": sign,
                    "offset": offset,
                }
            )
        parsed.sort(key=lambda p: _joint_index(p["joint_name"]))
        joint_names = [p["joint_name"] for p in parsed]
        can_ids = [p["can_id"] for p in parsed]
        if not apply:
            return True, "注册表校验通过（未应用）", joint_names, can_ids

        old_by_can = dict(self._rec_by_can)
        new_by_can = {p["can_id"]: p for p in parsed}
        remove_cids = [cid for cid in old_by_can if cid not in new_by_can]
        changed_cids = []
        for cid, p in new_by_can.items():
            old = old_by_can.get(cid)
            if old is None:
                changed_cids.append(cid)
                continue
            if (
                old.spec.name != p["joint_name"]
                or int(old.spec.mst_id) != p["mst_id"]
                or str(old.spec.model) != p["model"]
                or int(old.spec.channel) != p["channel"]
                or str(old.spec.mode) != p["mode"]
                or float(old.sign) != p["sign"]
                or float(old.offset) != p["offset"]
            ):
                changed_cids.append(cid)
        new_cids = [cid for cid in new_by_can if cid not in old_by_can]

        if self._sending_cids:
            self._set_sending_cids(list(self._sending_cids), False)

        affected = set(remove_cids) | set(changed_cids)
        for cid in sorted(affected):
            if cid in self._enabled_cids:
                self._disable_cids([cid], reason="registry change")

        for cid in sorted(set(remove_cids) | set(changed_cids)):
            try:
                bus.unregister(cid)
            except Exception as exc:
                self.get_logger().warn(f"注销 CAN 0x{cid:02X} 失败: {exc}")

        for cid in sorted(set(new_cids) | set(changed_cids)):
            p = new_by_can[cid]
            spec = self._motor_spec_cls(
                p["joint_name"], p["can_id"], p["mst_id"],
                p["model"], p["channel"], p["mode"],
            )
            try:
                bus.register(spec)
            except Exception as exc:
                raise RuntimeError(
                    f"注册 CAN 0x{cid:02X} 失败: {exc}"
                ) from exc
            self._online[cid] = True
            self._probed_mode[cid] = None
            self._latest.pop(cid, None)
            self._latest_t.pop(cid, None)

        new_recs = []
        for p in parsed:
            cid = p["can_id"]
            old = old_by_can.get(cid)
            preserve = old is not None and cid not in changed_cids and cid not in remove_cids
            spec = old.spec if preserve else self._motor_spec_cls(
                p["joint_name"], cid, p["mst_id"], p["model"],
                p["channel"], p["mode"],
            )
            q_max, dq_max, tau_max = spec.limits
            rec = _MotorRec(
                spec=spec,
                joint_index=_joint_index(p["joint_name"]),
                sign=p["sign"],
                offset=p["offset"],
                q_max=float(q_max),
                dq_max=float(dq_max),
                tau_max=float(tau_max),
                hold_kp=float(old.hold_kp) if old is not None else 5.0,
                hold_kd=float(old.hold_kd) if old is not None else 0.3,
                scale=float(self.joint_scales[_joint_index(p["joint_name"])]),
            )
            if preserve and old.enabled:
                rec.enabled = True
                rec.pose_known = old.pose_known
                rec.hold_q = old.hold_q
                rec.last_target_q = old.last_target_q
                rec.last_tau = old.last_tau
            new_recs.append(rec)
        new_recs.sort(key=lambda r: (r.joint_index, r.spec.name))
        self._recs = new_recs
        self._apply_joint_limits()
        self._enabled_cids = {r.spec.can_id for r in new_recs if r.enabled}
        self._sending_cids = set()
        self._rebuild_cache()
        self._last_cmd = None
        self._last_cmd_time = 0.0
        self._use_servo = False
        self._missing_warned = False
        for cid in list(self._online):
            if cid not in self._rec_by_can:
                self._online.pop(cid, None)
                self._probed_mode.pop(cid, None)
                self._latest.pop(cid, None)
                self._latest_t.pop(cid, None)
        text = ", ".join(
            f"{n}=0x{c:02X}" for n, c in zip(joint_names, can_ids)
        )
        return True, "注册表已应用: " + text, joint_names, can_ids

    def _enable(self) -> Tuple[bool, str]:
        bus = self._bus
        if bus is None:
            return False, "总线未打开"
        if self._active:
            return True, "电机已处于使能状态"

        offline = [
            name for name, cid in zip(self.joint_names, self.can_ids)
            if not self._online.get(cid, False)
        ]
        if offline:
            return False, (
                "连接探测无应答，拒绝使能: " + ", ".join(offline)
                + "；检查供电、CAN 通道与 CAN ID"
            )

        if not self.sim and self.clear_error_before_enable:
            mc = bus.mc
            for cid, ch in zip(self.can_ids, self.channels):
                try:
                    for _ in range(3):
                        mc.control_cmd(cid, 0xFB, ch)
                        time.sleep(0.003)
                except Exception as exc:
                    self._warn_key(
                        f"clear-{cid}",
                        f"清除故障失败 (CAN 0x{cid:02X}): {exc}",
                        2.0,
                    )
            time.sleep(0.1)
            self._log_status("清除故障后状态")

        if self.sim:
            states = bus.states()
            pose = [float(states.get(cid, {}).get("q", 0.0)) for cid in self.can_ids]
        else:
            pose = self._fresh_pose(max_age=1.0)
            if pose is None:
                return False, (
                    "使能前读不到当前位置（0xCC 刷新无有效应答），拒绝使能，"
                    "以免保持目标跳到 0 位；先确认适配器能收到反馈"
                )

        # 发送流启动前的基准位姿；此时不发控制帧，电机输出为零力矩
        self._hold_q = list(pose)
        self._last_target_q = list(pose)

        enabled = []
        skipped_mode_write = []
        try:
            for cid in self.can_ids:
                want_mode = self.modes.get(cid, "MIT")
                switch = self.force_mode_write or (
                    self._probed_mode.get(cid) != self.mode_codes[want_mode]
                )
                if not switch:
                    skipped_mode_write.append(cid)
                bus.enable(cid, True, switch_mode=switch)
                enabled.append(cid)
        except Exception as exc:
            for cid in enabled:
                try:
                    bus.enable(cid, False)
                except Exception:
                    pass
            return False, f"使能失败: {exc}"

        if skipped_mode_write:
            names = [
                name for name, cid in zip(self.joint_names, self.can_ids)
                if cid in skipped_mode_write
            ]
            self.get_logger().info(
                "已在 MIT 模式，跳过 CTRL_MODE 写入: " + ", ".join(names)
            )

        self._last_cmd = None
        self._last_cmd_time = 0.0
        self._last_tau = [0.0] * len(self.joint_names)
        self._active = True
        self._sending = False
        self._last_refresh = 0.0
        self._watchdog_warned = False

        modes = ", ".join(
            f"{name}={bus.mode_of(cid)}"
            for name, cid in zip(self.joint_names, self.can_ids)
        )
        pose_text = ", ".join(
            f"{name}={q:.4f} rad"
            for name, q in zip(self.joint_names, self._hold_q)
        )

        self._log_status("使能后状态")
        self._last_refresh = time.monotonic()
        self.get_logger().warn(
            "已使能 %d 台电机（模式: %s），只发了 0xFC，未发任何控制帧；当前 %s。"
            "开始运动: /motor_driver/sending (data: true)"
            % (len(self.can_ids), modes, pose_text)
        )
        return True, "已使能（仅 0xFC，未发送控制帧）；用 ~/sending 开始发送"

    def _start_sending(self) -> Tuple[bool, str]:
        bus = self._bus
        if bus is None:
            return False, "总线未打开"
        if not self._active:
            return False, "电机未使能；先调用 /motor_driver/enable (data: true)"
        if self._sending:
            return True, "已经在发送 MIT 指令"

        if self.sim:
            pose = self._latest_pose(max_age=max(self.feedback_timeout, 0.2))
        else:
            # 发送前再查一次 0xCC，按真正的当前位姿起保持帧
            pose = self._fresh_pose(max_age=1.0)
        if pose is None:
            pose = list(self._hold_q)
            self.get_logger().warn(
                "开始发送前读不到当前位置，改用使能时记录的位姿作为保持目标: %s"
                % ", ".join(
                    f"{name}={q:.4f}" for name, q in zip(self.joint_names, pose)
                )
            )
        self._hold_q = list(pose)
        self._last_target_q = list(pose)
        self._last_cmd = None
        self._last_cmd_time = 0.0
        self._last_tau = [0.0] * len(self.joint_names)
        self._watchdog_warned = False
        urdf = [
            self.joint_signs[i]
            * (pose[i] - self.joint_offsets[i])
            / self.joint_scales[i]
            for i in range(len(self.joint_names))
        ]
        self._servo.reset(urdf)
        self._use_servo = True

        if not self.sim:
            # 使能时若写过 CTRL_MODE，先确认 SDK 参数写闸门已放开，
            # 否则第一批 MIT 反馈会被当成参数应答丢掉。
            self._clear_param_gate(bus)

        self._sending = True
        self._send_hold_once()
        pose_text = ", ".join(
            f"{name}={q:.4f} rad"
            for name, q in zip(self.joint_names, self._hold_q)
        )
        self.get_logger().warn(
            "开始发送 MIT（%.0f Hz），按当前位置保持: %s（hold_kp=%s, hold_kd=%s）"
            % (
                self.control_rate,
                pose_text,
                [round(v, 3) for v in self.hold_kp],
                [round(v, 3) for v in self.hold_kd],
            )
        )
        return True, "已开始发送 MIT 指令（按当前位置保持）"

    def _stop_sending(self) -> Tuple[bool, str]:
        bus = self._bus
        if bus is None:
            return False, "总线未打开"
        if not self._sending:
            return True, "当前未在发送 MIT 指令"

        pose = self._latest_pose(max_age=max(self.feedback_timeout, 0.2))
        if pose is None:
            pose = list(self._last_target_q)
            self.get_logger().warn(
                "停止发送前反馈不新鲜，改为保持最后一帧目标位姿"
            )
        self._hold_q = list(pose)
        self._last_target_q = list(pose)

        # 补几帧“当前位置保持”，避免流停止后电机继续执行最后一条运动指令
        kp, kd, q, dq, tau = self._hold_command()
        for _ in range(5):
            for i, cid in enumerate(self.can_ids):
                try:
                    bus.mit(cid, kp[i], kd[i], q[i], dq[i], tau[i])
                except Exception as exc:
                    self._warn_key(
                        f"stop-{cid}",
                        f"停止前保持帧发送失败 (CAN 0x{cid:02X}): {exc}",
                        2.0,
                    )
            time.sleep(0.005)

        self._sending = False
        self._drag_teach = False
        self._last_cmd = None
        self._last_cmd_time = 0.0
        self._last_refresh = 0.0
        self._watchdog_warned = False
        pose_text = ", ".join(
            f"{name}={q:.4f} rad"
            for name, q in zip(self.joint_names, self._hold_q)
        )
        self.get_logger().warn(
            "已停止发送 MIT（电机仍使能，按 hold 增益保持 %s）；"
            "恢复: /motor_driver/sending (data: true)，失能: /motor_driver/release"
            % pose_text
        )
        return True, "已停止发送 MIT；电机仍使能并按保持增益锁住当前位置"

    def _release_and_disable(self) -> Tuple[bool, str]:
        bus = self._bus
        if bus is None:
            return False, "总线未打开"
        self._active = False
        self._sending = False
        self._drag_teach = False
        self._last_cmd = None
        self._last_cmd_time = 0.0
        self._stale_since = None
        self._last_tau = [0.0] * len(self.joint_names)
        self._last_refresh = 0.0
        self._watchdog_warned = False
        errors = []
        for cid in self.can_ids:
            try:
                bus.release(cid)
            except Exception as exc:
                errors.append(f"release 0x{cid:02X}: {exc}")
        for cid in self.can_ids:
            try:
                bus.enable(cid, False)
            except Exception as exc:
                errors.append(f"disable 0x{cid:02X}: {exc}")
        if errors:
            return False, "；".join(errors)
        return True, "已下发零力矩并失能全部电机"

    def _send_hold_once(self):
        bus = self._bus
        if bus is None:
            return
        kp, kd, q, dq, tau = self._hold_command()
        for i, cid in enumerate(self.can_ids):
            try:
                bus.mit(cid, kp[i], kd[i], q[i], dq[i], tau[i])
            except Exception as exc:
                self._warn_key(
                    f"hold-{cid}", f"发送保持指令失败 (CAN 0x{cid:02X}): {exc}", 2.0
                )

    def _fresh_pose(self, max_age: float = 1.0) -> Optional[List[float]]:
        """用 0xCC 刷新读一次当前位置；任一关节数据过期就返回 None。"""
        self._send_refresh()
        time.sleep(0.1)
        return self._latest_pose(max_age)

    def _latest_pose(self, max_age: float) -> Optional[List[float]]:
        now = time.monotonic()
        with self._lock:
            snapshot = {cid: self._latest.get(cid) for cid in self.can_ids}
            stamps = {cid: self._latest_t.get(cid) for cid in self.can_ids}
        stale = [
            name for name, cid in zip(self.joint_names, self.can_ids)
            if snapshot[cid] is None
            or stamps[cid] is None
            or now - stamps[cid] > max_age
        ]
        if stale:
            # 这个函数每 tick 都会被 _slew_q / _tau_ff 调用；反馈掉线时
            # 不能 250 Hz 刷日志，否则日志本身就会拖垮控制环。
            self._warn_key(
                "pose-stale",
                "位置不可用（无反馈或已过期）: " + ", ".join(stale),
                1.0,
            )
            return None
        return [float(snapshot[cid][0]) for cid in self.can_ids]

    # ------------------------------------------------------------------ helpers
    def _clear_param_gate(self, bus, cids=None, timeout: float = 0.1):
        # switchControlMode 写 RID 10 后，如果电机没有回写应答，SDK 的
        # read_write_save 会一直置位，后续 MIT 反馈会被当成参数帧丢弃。
        gate = getattr(getattr(bus, "mc", None), "read_write_save", None)
        if gate is None:
            return
        if cids is None:
            cids = list(self._sending_cids) or list(self._enabled_cids)
        cids = [int(c) for c in cids]
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and gate.is_set():
            for cid in cids:
                rec = self._rec_by_can.get(cid)
                if rec is None:
                    continue
                try:
                    bus.mit(
                        cid, rec.hold_kp, rec.hold_kd,
                        rec.last_target_q, 0.0, 0.0,
                    )
                except Exception:
                    pass
            time.sleep(0.005)
        if gate.is_set():
            gate.clear()
            self.get_logger().warn(
                "SDK 参数写闸门未自动清除，已手动清除；"
                "否则后续 MIT 反馈会被静默丢弃"
            )

    def _warn_key(self, key: str, message: str, period: float):
        now = time.monotonic()
        if now - self._warn_times.get(key, -1e9) >= period:
            self._warn_times[key] = now
            self.get_logger().warn(message)

    def shutdown(self):
        bus = self._bus
        if bus is None:
            return
        try:
            self._disable_cids(list(self._enabled_cids))
        finally:
            self._bus = None
            try:
                bus.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = MotorDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[motor_driver] 启动失败: {exc}", file=sys.stderr)
        code = 1
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return code


if __name__ == "__main__":
    sys.exit(main())
