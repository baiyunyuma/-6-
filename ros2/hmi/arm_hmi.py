#!/usr/bin/env python3
"""ZFY 实机机械臂上位机（ROS 2 + PyQt5）。

    source /opt/ros/humble/setup.bash
    source ~/zfy_arm/Zfy_Arm_real_control/install/setup.bash
    python3 ~/zfy_arm/ros2_ws/tools/arm_hmi.py

本程序不直接打开 CAN，只通过 ROS 2 操作 /motor_driver 和 /gripper_driver：
  /joint_states                       实时关节角
  /mit_command                        MIT / POS_VEL / VEL / POS_FORCE 目标
  ~/enable, ~/sending, ~/release      使能、开始/停止发送、失能
  ~/set_mode, ~/read_param, ...      模式和 RID 参数
  ~/status                            模式、ERR、真实反馈年龄
  /gripper_driver/...                 独立夹爪 MIT 节点（使能/自适应抓取/保存高刚度）

六个页面：
  基础    使能/失能、单轴运动、模式切换、参数读写、q/dq/tau 曲线
  关节    六轴关节目标、TCP 位姿与软件限位（可收窄并同步到驱动）
  轨迹    记录多个关节位姿并顺序执行
  逆解    TCP 的 x/y/z -> 阻尼最小二乘逆解（位置，不含姿态）
  辨识    激励轨迹 -> 驱动 CSV -> 重力/摩擦参数拟合，可回填前馈
  夹爪    独立第 7 电机：使能/MIT、自适应抓取、高刚度保持与保存

安全：不会自动使能；运动必须同时满足“已使能 + 已开始控制 + 反馈新鲜”；
重力补偿默认关闭（关节零位未标定前不要打开）；关闭窗口会请求失能；
轨迹和逆解只做关节空间插值，不做碰撞检测。
"""
from __future__ import annotations

import argparse
import fcntl
import glob
import ipaddress
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64MultiArray, Int32
from std_srvs.srv import SetBool, Trigger
from zfy_arm_interfaces.msg import (
    JointGoal,
    MotorMitCommand,
    MotorRegistryEntry,
)
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
import hmi_store

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

MUJOCO_SITE = os.environ.get(
    "MUJOCO_SITE_PACKAGES",
    "/home/sky/venvs/mujoco/lib/python3.10/site-packages",
)
if MUJOCO_SITE and os.path.isdir(MUJOCO_SITE) and MUJOCO_SITE not in sys.path:
    sys.path.append(MUJOCO_SITE)

import mujoco  # noqa: E402
import pyqtgraph as pg  # noqa: E402

pg.setConfigOptions(antialias=False)
from arm_hmi_3d import Preview3D, _rot_axis  # noqa: E402
from PyQt5.QtCore import QEvent, Qt, QTimer, pyqtSignal  # noqa: E402
from PyQt5.QtGui import QFont, QFontDatabase  # noqa: E402
from PyQt5.QtWidgets import (  # noqa: E402
    QAbstractItemView,
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QInputDialog,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
MODES = ["MIT", "POS_VEL", "VEL", "POS_FORCE"]
#: 拖动录制的轨迹库（每条轨迹一个 JSON，文件名带时间戳，永不覆盖）
TRAJ_DIR = os.path.expanduser("~/.config/zfy_arm/trajectories")
GRIP_STATE_NAMES = [
    "IDLE", "ENABLED", "OPENING", "ADAPTIVE",
    "CONTACT", "HOLD_HIGH", "HOLDING", "FAULT",
]
#: Must match py/motor_config.py MODELS
MODEL_NAMES = [
    "4310", "4310_48V", "4340", "4340_48V",
    "6006", "8006", "8009", "10010", "10010L",
]
DEFAULT_SCENE = os.path.expanduser(
    "~/zfy_arm/zfy_arm_control/model/scene_robot_arm_torque.xml"
)
#: 离线重力/摩擦拟合脚本与激励节点（见 Zfy_Arm_real_control/docs/动力学参数辨识.md）
IDENT_SCRIPT = os.path.expanduser(
    "~/zfy_arm/Zfy_Arm_real_control/tools/identify_dynamics.py"
)
IDENT_PKG = "zfy_arm_sim2real"
IDENT_EXCITE = "ident_excite"

#: 常见寄存器；第三项 True = 整数寄存器
PARAM_REGS = [
    ("CAN ID (ESC_ID)", 8, True),
    ("回传 ID (MST_ID)", 7, True),
    ("控制模式 (CTRL_MODE)", 10, True),
    ("加速度 (ACC)", 4, False),
    ("减速度 (DEC)", 5, False),
    ("最大速度 (MAX_SPD)", 6, False),
    ("速度环 KP", 25, False),
    ("速度环 KI", 26, False),
    ("位置环 KP", 27, False),
    ("位置环 KI", 28, False),
    ("超时 (TIMEOUT)", 9, True),
    ("CAN 波特率", 35, True),
]

MOTOR_COLORS = (
    (229, 57, 53),
    (67, 160, 71),
    (30, 136, 229),
    (255, 152, 0),
    (156, 39, 176),
    (0, 188, 212),
    (236, 64, 122),   # J7 夹爪
)

STYLE = """
QWidget { background: #1e2228; color: #d8dee9; font-size: 12px; }
QMainWindow { background: #1e2228; }
QGroupBox { border: 1px solid #3b4252; border-radius: 4px; margin-top: 9px; padding-top: 6px; font-weight: bold; }
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; color: #88c0d0; }
QPushButton { background: #3b4252; border: 1px solid #4c566a; border-radius: 3px; padding: 5px 10px; }
QPushButton:hover { background: #434c5e; }
QPushButton:disabled { color: #6c7380; background: #2b3038; }
QPushButton#good { background: #2e7d32; }
QPushButton:checked { background: #1565c0; }
QPushButton#danger { background: #b71c1c; }
QTabWidget::pane { border: 1px solid #3b4252; }
QTabBar::tab { background: #2b3038; padding: 6px 14px; }
QTabBar::tab:selected { background: #3b4252; color: #88c0d0; }
QTableWidget { background: #23272e; gridline-color: #3b4252; alternate-background-color: #262b33; }
QHeaderView::section { background: #2b3038; color: #aab4c0; border: 1px solid #3b4252; padding: 3px; }
QLineEdit, QComboBox, QDoubleSpinBox, QSpinBox { background: #2b3038; border: 1px solid #4c566a; border-radius: 3px; padding: 3px; }
QPlainTextEdit { background: #16191e; color: #a3be8c; font-family: monospace; }
QLabel#hint { color: #8f98a5; }
QLabel#warn { color: #ebcb8b; }
"""


CJK_FONT_CANDIDATES = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    # WSL 一般没有中文字体，直接借 Windows 的，免安装
    "/mnt/c/Windows/Fonts/msyh.ttc",
    "/mnt/c/Windows/Fonts/msyhbd.ttc",
    "/mnt/c/Windows/Fonts/simhei.ttf",
    "/mnt/c/Windows/Fonts/simsun.ttc",
]


def load_cjk_font(app):
    families = []
    for path in CJK_FONT_CANDIDATES:
        if not os.path.exists(path):
            continue
        fid = QFontDatabase.addApplicationFont(path)
        if fid >= 0:
            fams = QFontDatabase.applicationFontFamilies(fid)
            if fams:
                families.append(fams[0])
    font = QFont()
    font.setFamilies(families + ["DejaVu Sans"] if families else ["DejaVu Sans"])
    font.setPointSize(9)
    app.setFont(font)
    return families


def _quat_from_R(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).reshape(9))
    return q


def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def _quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


class GravityComp:
    """六轴全链重力力矩（q=0 对应 URDF fold）。

    用独立的 MjData 做运动学 + RNE（flg_acc=0），等价于 qfrc_bias
    在 qvel=0 时的值，但不做碰撞检测，适合在 100 Hz 控制循环里跑。
    """

    def __init__(self, model, qpos_idx, dof_idx):
        self.model = model
        self.data = mujoco.MjData(model)
        self.qpos_idx = np.asarray(qpos_idx, dtype=int)
        self.dof_idx = np.asarray(dof_idx, dtype=int)
        self._tau = np.zeros(int(model.nv))

    def bias(self, q):
        q = np.asarray(q, dtype=float)
        self.data.qpos[self.qpos_idx] = q
        self.data.qvel[:] = 0.0
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)
        self._tau[:] = 0.0
        mujoco.mj_rne(self.model, self.data, 0, self._tau)
        return self._tau[self.dof_idx].copy()


# --------------------------------------------------------------------------- #
# MuJoCo 运动学：TCP 正解 + 位置逆解（阻尼最小二乘）
# --------------------------------------------------------------------------- #
class ArmKinematics:
    def __init__(self, scene: str):
        self.scene = scene
        self.model = mujoco.MjModel.from_xml_path(scene)
        self.data = mujoco.MjData(self.model)
        self.joint_ids = []
        for name in JOINTS:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise RuntimeError(f"MuJoCo 模型里没有 {name}")
            self.joint_ids.append(jid)
        self.qpos_idx = np.array(
            [int(self.model.jnt_qposadr[j]) for j in self.joint_ids], dtype=int
        )
        self.dof_idx = np.array(
            [int(self.model.jnt_dofadr[j]) for j in self.joint_ids], dtype=int
        )
        self.limits = np.array(
            [self.model.jnt_range[j] for j in self.joint_ids], dtype=float
        )
        self.tcp_site = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "tcp"
        )
        if self.tcp_site < 0:
            raise RuntimeError("MuJoCo 模型里没有 tcp site")
        self._jacp = np.zeros((3, self.model.nv))
        self._jacr = np.zeros((3, self.model.nv))

    def _kinematics(self, q: np.ndarray) -> None:
        self.data.qpos[self.qpos_idx] = q
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)

    def fk(self, q) -> np.ndarray:
        q = np.clip(np.asarray(q, dtype=float), self.limits[:, 0], self.limits[:, 1])
        self._kinematics(q)
        return self.data.site_xpos[self.tcp_site].copy()

    def ik(self, target, seed, max_iter: int = 400, tol: float = 5e-4):
        target = np.asarray(target, dtype=float)
        q = np.clip(np.asarray(seed, dtype=float), self.limits[:, 0], self.limits[:, 1])
        best_err = float("inf")
        best_q = q.copy()
        for it in range(1, max_iter + 1):
            self._kinematics(q)
            err = target - self.data.site_xpos[self.tcp_site]
            e = float(np.linalg.norm(err))
            if e < best_err:
                best_err, best_q = e, q.copy()
            if e < tol:
                return best_q, best_err, it
            mujoco.mj_jacSite(
                self.model, self.data, self._jacp, self._jacr, self.tcp_site
            )
            jac = self._jacp[:, self.dof_idx]
            lam = 0.08 if e > 0.01 else 0.02
            dq = jac.T @ np.linalg.solve(
                jac @ jac.T + (lam * lam) * np.eye(3), err
            )
            step = 0.5 if e > 0.01 else 1.0
            dq *= step
            n = float(np.linalg.norm(dq))
            if n > 0.25:
                dq *= 0.25 / n
            q = np.clip(q + dq, self.limits[:, 0], self.limits[:, 1])
            if n < 1e-9:
                break
        self._kinematics(best_q)
        return best_q, best_err, max_iter

    def tcp_pose(self, q):
        q = np.clip(np.asarray(q, dtype=float), self.limits[:, 0], self.limits[:, 1])
        self._kinematics(q)
        return (
            self.data.site_xpos[self.tcp_site].copy(),
            self.data.site_xmat[self.tcp_site].reshape(3, 3).copy(),
        )

    def ik_pose(self, target_pos, target_R, seed, max_iter: int = 400,
                tol_pos: float = 5e-4, tol_rot: float = 5e-3):
        """6-DOF pose IK: position (m) + orientation (rotation matrix)."""
        target_pos = np.asarray(target_pos, dtype=float)
        target_R = np.asarray(target_R, dtype=float).reshape(3, 3)
        q = np.clip(np.asarray(seed, dtype=float), self.limits[:, 0], self.limits[:, 1])
        best_q = q.copy()
        best_cost = float("inf")
        best_pos = float("inf")
        best_rot = float("inf")
        for it in range(1, max_iter + 1):
            self._kinematics(q)
            p = self.data.site_xpos[self.tcp_site]
            R = self.data.site_xmat[self.tcp_site].reshape(3, 3)
            e_pos = target_pos - p
            e_pos_n = float(np.linalg.norm(e_pos))
            qe = _quat_mul(_quat_from_R(target_R), _quat_conj(_quat_from_R(R)))
            if qe[0] < 0.0:
                qe = -qe
            v = qe[1:]
            nv = float(np.linalg.norm(v))
            angle = 2.0 * math.atan2(nv, qe[0]) if nv > 1e-12 else 0.0
            e_rot = (v / nv * angle) if nv > 1e-12 else np.zeros(3)
            cost = e_pos_n + 0.05 * angle
            if cost < best_cost:
                best_cost = cost
                best_q = q.copy()
                best_pos = e_pos_n
                best_rot = angle
            if e_pos_n < tol_pos and angle < tol_rot:
                return q.copy(), e_pos_n, angle, it
            mujoco.mj_jacSite(
                self.model, self.data, self._jacp, self._jacr, self.tcp_site
            )
            jac = np.vstack([
                self._jacp[:, self.dof_idx],
                self._jacr[:, self.dof_idx],
            ])
            err = np.concatenate([e_pos, e_rot])
            far = e_pos_n > 0.01 or angle > 0.1
            lam = 0.06 if far else 0.01
            dq = jac.T @ np.linalg.solve(
                jac @ jac.T + (lam * lam) * np.eye(6), err
            )
            if far:
                dq *= 0.5
            n = float(np.linalg.norm(dq))
            if n > 0.2:
                dq *= 0.2 / n
            q = np.clip(q + dq, self.limits[:, 0], self.limits[:, 1])
            if n < 1e-9:
                break
        self._kinematics(best_q)
        return best_q, best_pos, best_rot, max_iter


# --------------------------------------------------------------------------- #
# 关节空间运动：余弦插值 + 轨迹队列（全部在 GUI 侧算，100 Hz 下发）
# --------------------------------------------------------------------------- #
class JointMotion:
    def __init__(self, n: int):
        self.n = n
        self._lock = threading.RLock()
        self.kp = np.full(n, 5.0)
        self.kd = np.full(n, 0.8)
        self.q_cmd = np.zeros(n)
        self.q_start = np.zeros(n)
        self.q_target = np.zeros(n)
        self.t0 = 0.0
        self.dur = 0.0
        self.active = False
        self.pending = deque()

    def set_gains(self, kp: float, kd: float) -> None:
        with self._lock:
            self.kp[:] = float(kp)
            self.kd[:] = float(kd)

    def gains(self):
        with self._lock:
            return self.kp.copy(), self.kd.copy()

    def reset(self, q) -> None:
        with self._lock:
            self.q_cmd = np.asarray(q, dtype=float).copy()
            self.q_start = self.q_cmd.copy()
            self.q_target = self.q_cmd.copy()
            self.active = False
            self.pending.clear()

    def _sample_locked(self, now: float) -> np.ndarray:
        if not self.active:
            return self.q_cmd.copy()
        s = 0.0 if self.dur <= 0.0 else min(1.0, (now - self.t0) / self.dur)
        u = 0.5 - 0.5 * math.cos(math.pi * s)
        self.q_cmd = self.q_start + (self.q_target - self.q_start) * u
        if s >= 1.0 - 1e-9:
            self.q_cmd = self.q_target.copy()
            if self.pending:
                self.q_start = self.q_cmd.copy()
                self.q_target, self.dur = self.pending.popleft()
                self.t0 = now
            else:
                self.active = False
        return self.q_cmd.copy()

    def sample(self, now: float) -> np.ndarray:
        with self._lock:
            return self._sample_locked(now)

    def move_to(self, q, dur: float) -> None:
        with self._lock:
            now = time.monotonic()
            self.q_start = self._sample_locked(now).copy()
            self.q_target = np.asarray(q, dtype=float).copy()
            self.t0 = now
            self.dur = max(0.05, float(dur))
            self.active = True
            self.pending.clear()

    def append(self, q, dur: float) -> None:
        with self._lock:
            now = time.monotonic()
            target = np.asarray(q, dtype=float).copy()
            if not self.active and not self.pending:
                self.q_start = self.q_cmd.copy()
                self.q_target = target
                self.t0 = now
                self.dur = max(0.05, float(dur))
                self.active = True
            else:
                self.pending.append((target, max(0.05, float(dur))))

    def stop_here(self) -> None:
        with self._lock:
            now = time.monotonic()
            cur = self._sample_locked(now).copy()
            self.pending.clear()
            self.q_start = cur
            self.q_target = cur.copy()
            self.t0 = now
            self.dur = 0.25
            self.active = True

    def set_direct(self, q) -> None:
        """Direct target for teleop; cancels any queued trajectory."""
        with self._lock:
            self.q_cmd = np.asarray(q, dtype=float).copy()
            self.q_start = self.q_cmd.copy()
            self.q_target = self.q_cmd.copy()
            self.active = False
            self.pending.clear()

    def state(self):
        with self._lock:
            return self.active, len(self.pending)

    def commanded(self):
        """Current interpolated command. Does not advance the trajectory."""
        with self._lock:
            return self.q_cmd.copy(), self.active


# --------------------------------------------------------------------------- #
# ROS 桥
# --------------------------------------------------------------------------- #
PAD_TOKEN = "ZFY1"


class GamepadInput:
    """UDP receiver for xbox_udp.ps1: ZFY1,lx,ly,rx,ry,lt,rt,buttons."""

    def __init__(self, port: int = 47800):
        self.port = port
        self.sock = None
        self.error = None
        self.axes = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.buttons = 0
        self.stamp = 0.0
        self.source = ""
        self.allowed_sources = self._allowed_sources()
        self.allowed_networks = self._allowed_networks()
        self.rejected_sources = set()
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", int(port)))
            sock.setblocking(False)
            self.sock = sock
        except OSError as exc:
            self.error = str(exc)

    @staticmethod
    def _allowed_sources():
        sources = {"127.0.0.1", "::1"}
        try:
            out = subprocess.check_output(
                ["ip", "route", "show", "default"], text=True, timeout=1.0
            )
            tokens = out.split()
            if "via" in tokens:
                idx = tokens.index("via")
                if idx + 1 < len(tokens):
                    sources.add(tokens[idx + 1])
        except Exception:
            pass
        return sources

    @staticmethod
    def _allowed_networks():
        """Local IPv4 subnets (the pad script runs on another LAN host)."""
        nets = []
        try:
            out = subprocess.check_output(
                ["ip", "-4", "-o", "addr", "show"], text=True, timeout=1.0
            )
            for line in out.splitlines():
                parts = line.split()
                if "inet" not in parts:
                    continue
                token = parts[parts.index("inet") + 1]
                try:
                    net = ipaddress.ip_network(token, strict=False)
                except ValueError:
                    continue
                if not net.is_loopback:
                    nets.append(net)
        except Exception:
            pass
        return nets

    def poll(self) -> bool:
        if self.sock is None:
            return False
        got = False
        while True:
            try:
                data, addr = self.sock.recvfrom(256)
            except (BlockingIOError, OSError):
                break
            src = addr[0] if addr else ""
            allowed = src in self.allowed_sources
            if not allowed and src:
                try:
                    ip = ipaddress.ip_address(src)
                    allowed = any(ip in net for net in self.allowed_networks)
                except ValueError:
                    allowed = False
            if not allowed:
                if src not in self.rejected_sources:
                    self.rejected_sources.add(src)
                continue
            try:
                parts = data.decode("ascii", "ignore").strip().split(",")
                if len(parts) < 8 or parts[0] != PAD_TOKEN:
                    continue
                axes = tuple(float(v) for v in parts[1:7])
                buttons = int(float(parts[7]))
                if not all(math.isfinite(v) for v in axes):
                    continue
                axes = tuple(max(-1.0, min(1.0, v)) for v in axes)
            except (ValueError, UnicodeError, OverflowError):
                continue
            self.axes = axes
            self.buttons = buttons
            self.source = src
            self.stamp = time.monotonic()
            got = True
        return got

    def age(self) -> float:
        return time.monotonic() - self.stamp if self.stamp else 999.0

    def connected(self) -> bool:
        return self.sock is not None and self.age() < 0.4

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None


class CmdMirror:
    """Driver-side trajectory, mirrored here for the UI and 3D preview."""

    def __init__(self, n: int):
        self._lock = threading.Lock()
        self.q = np.zeros(n)
        self.active = False
        self.pending = 0
        self.kp = 5.0
        self.kd = 0.8

    def reset(self, q=None) -> None:
        return

    def set_gains(self, kp: float, kd: float) -> None:
        self.kp = float(kp)
        self.kd = float(kd)

    def commanded(self):
        with self._lock:
            return self.q.copy(), self.active

    def state(self):
        with self._lock:
            return self.active, self.pending

    def update(self, q, tau) -> None:
        with self._lock:
            self.q[:] = q
            self._tau = np.asarray(tau, dtype=float).copy()

    def set_flags(self, active: bool, pending: int) -> None:
        with self._lock:
            self.active = bool(active)
            self.pending = int(pending)


class RosBridge(Node):
    def __init__(self, log_cb, n: int = 6, rate: float = 100.0, kin=None):
        super().__init__("zfy_arm_hmi")
        self.n = n
        self.log_cb = log_cb
        self.rate = float(rate)
        self._lock = threading.RLock()
        # 软件限位（URDF 弧度）。所有 JointGoal / 单轴 MIT 目标都先过这里。
        if kin is not None:
            lim = np.asarray(kin.limits, dtype=float)
            self.joint_lo = lim[:, 0].copy()
            self.joint_hi = lim[:, 1].copy()
        else:
            self.joint_lo = np.full(n, -1.0e9)
            self.joint_hi = np.full(n, 1.0e9)
        self._limit_warn_t = 0.0
        self.q = np.zeros(n)
        self.dq = np.zeros(n)
        self.tau = np.zeros(n)
        self.q_stamp = 0.0
        self.q_seen = False
        # 与 /joint_states 对齐：100 Hz × 10 s。画线时再抽点，不在这里降频。
        self._hist_hz = 100.0
        self._hist_cap = 1000
        self._hist_t = np.full(self._hist_cap, np.nan)
        self._hist_q = np.zeros((self._hist_cap, n))
        self._hist_dq = np.zeros((self._hist_cap, n))
        self._hist_tau = np.zeros((self._hist_cap, n))
        self._hist_qd = np.zeros((self._hist_cap, n))
        #: 夹爪（J7）历史，与 6 轴共用同一时间轴；无反馈时写 NaN 断开曲线。
        self._hist_gq = np.full(self._hist_cap, np.nan)
        self._hist_gdq = np.full(self._hist_cap, np.nan)
        self._hist_gtau = np.full(self._hist_cap, np.nan)
        self._hist_i = 0
        self._hist_count = 0
        self._hist_next = 0.0
        self.modes = ["?"] * n
        self.errs = [-1] * n
        self.ages = [999.0] * n
        self.can_ids = list(range(1, n + 1))
        self.active_mask = np.ones(n, dtype=bool)
        self.enabled_each = [False] * n
        self.sending_each = [False] * n
        self._joint_stamp = [0.0] * n
        self.registry_names = []
        self.registry_can_ids = []
        self.registry_mst_ids = []
        self.registry_models = []
        self.registry_channels = []
        self.registry_signs = []
        self.registry_offsets = []
        self.enabled = False
        self.sending = False
        self.driver_enabled = False
        self.driver_sending = False
        self.motion = CmdMirror(n)
        self._kp = 5.0
        self._kd = 0.8
        self.gravity_enabled = False
        self.gravity_scale = 1.0
        self.friction_enabled = False
        self.coulomb = 0.10
        self.viscous = 0.05
        self.friction_slope = 100.0
        self.last_tau_ff = np.zeros(n)
        self._last_q_des = np.zeros(n)
        self._stale_warn_t = 0.0
        self._status_busy = False
        self._status_t0 = 0.0
        self._status_seq = 0
        self._state_rev = 0
        self.stopping = False
        # 夹爪（独立 gripper_driver）
        self.grip_q = 0.0
        self.grip_dq = 0.0
        self.grip_tau = 0.0
        self.grip_stamp = 0.0
        self.grip_state = -1
        self.grip_contact = False

        self.create_subscription(JointState, "/joint_states", self._on_state, 20)
        self.create_subscription(
            JointState, "/gripper_driver/state", self._on_grip_state, 10
        )
        self.create_subscription(
            Int32, "/gripper_driver/status", self._on_grip_status, 10
        )
        self.create_subscription(
            Bool, "/gripper_driver/contact", self._on_grip_contact, 10
        )
        self.create_subscription(JointState, "/joint_commands", self._on_command, 10)
        self.create_subscription(Bool, "/motor_driver/motion_active", self._on_motion_active, 10)
        self.create_subscription(Int32, "/motor_driver/motion_pending", self._on_pending, 10)
        self.goal_pub = self.create_publisher(JointGoal, "/arm_joint_goal", 10)
        self.heartbeat_pub = self.create_publisher(
            Empty, "/arm_hmi/heartbeat", 10
        )
        self.cli_enable = self.create_client(SetBool, "/motor_driver/enable")
        self.cli_sending = self.create_client(SetBool, "/motor_driver/sending")
        self.cli_release = self.create_client(Trigger, "/motor_driver/release")
        self.cli_mode = self.create_client(SetMotorMode, "/motor_driver/set_mode")
        self.cli_read = self.create_client(ReadMotorParam, "/motor_driver/read_param")
        self.cli_write = self.create_client(
            WriteMotorParam, "/motor_driver/write_param"
        )
        self.cli_save = self.create_client(
            SaveMotorParam, "/motor_driver/save_param"
        )
        self.cli_status = self.create_client(
            GetMotorStatus, "/motor_driver/status"
        )
        self.cli_configure = self.create_client(
            ConfigureMotors, "/motor_driver/configure_motors"
        )
        self.cli_motor_enable = self.create_client(
            SetMotorEnable, "/motor_driver/set_motor_enable"
        )
        self.cli_motor_sending = self.create_client(
            SetMotorSending, "/motor_driver/set_motor_sending"
        )
        self.cli_logging = self.create_client(
            SetBool, "/motor_driver/set_logging"
        )
        self.cli_grip_enable = self.create_client(
            SetBool, "/gripper_driver/enable"
        )
        self.cli_grip_sending = self.create_client(
            SetBool, "/gripper_driver/sending"
        )
        self.cli_grip_open = self.create_client(
            Trigger, "/gripper_driver/open"
        )
        self.cli_grip_grasp = self.create_client(
            Trigger, "/gripper_driver/adaptive_grasp"
        )
        self.cli_grip_hold = self.create_client(
            Trigger, "/gripper_driver/hold_high"
        )
        self.cli_grip_save = self.create_client(
            Trigger, "/gripper_driver/save_grasp"
        )
        self.cli_grip_stop = self.create_client(
            Trigger, "/gripper_driver/stop"
        )
        self.cli_grip_get_params = self.create_client(
            GetParameters, "/gripper_driver/get_parameters"
        )
        self.cli_grip_set_params = self.create_client(
            SetParameters, "/gripper_driver/set_parameters"
        )
        self.mit_pub = self.create_publisher(
            MotorMitCommand, "/motor_driver/motor_mit", 20
        )
        self.create_timer(0.1, self._publish_heartbeat)
        self.create_timer(1.0, self._poll_status)

    # ------------------------------------------------------------- 基础状态
    def log(self, text: str) -> None:
        if self.log_cb is not None:
            self.log_cb(time.strftime("%H:%M:%S ") + str(text))

    def _publish_heartbeat(self) -> None:
        if not self.stopping:
            self.heartbeat_pub.publish(Empty())

    def _on_state(self, msg: JointState) -> None:
        index = {name: i for i, name in enumerate(msg.name)}
        now = time.monotonic()
        with self._lock:
            for i, name in enumerate(JOINTS):
                j = index.get(name)
                if j is None:
                    continue
                if j < len(msg.position):
                    self.q[i] = float(msg.position[j])
                    self._joint_stamp[i] = now
                if j < len(msg.velocity):
                    self.dq[i] = float(msg.velocity[j])
                if j < len(msg.effort):
                    self.tau[i] = float(msg.effort[j])
            self.q_stamp = now
            self.q_seen = True
            if self._hist_next <= 0.0 or now >= self._hist_next:
                i = self._hist_i
                self._hist_t[i] = now
                self._hist_q[i] = self.q
                self._hist_dq[i] = self.dq
                self._hist_tau[i] = self.tau
                self._hist_qd[i] = self._last_q_des
                fresh = (
                    self.grip_stamp > 0.0
                    and now - self.grip_stamp <= 0.5
                )
                self._hist_gq[i] = self.grip_q if fresh else np.nan
                self._hist_gdq[i] = self.grip_dq if fresh else np.nan
                self._hist_gtau[i] = self.grip_tau if fresh else np.nan
                self._hist_i = (i + 1) % self._hist_cap
                self._hist_count = min(self._hist_cap, self._hist_count + 1)
                self._hist_next = now + 1.0 / self._hist_hz

    def current_q(self) -> np.ndarray:
        with self._lock:
            return self.q.copy()

    def feedback_age_for(self, joint_index: int) -> float:
        with self._lock:
            stamp = (
                self._joint_stamp[joint_index]
                if 0 <= joint_index < len(self._joint_stamp)
                else 0.0
            )
            return time.monotonic() - stamp if stamp > 0.0 else 999.0

    def feedback_age(self) -> float:
        with self._lock:
            now = time.monotonic()
            ages = [
                now - self._joint_stamp[i]
                for i in range(self.n)
                if self.active_mask[i] and self._joint_stamp[i] > 0.0
            ]
        return max(ages) if ages else 999.0

    def motor_age_for(self, joint_index: int) -> float:
        with self._lock:
            if 0 <= joint_index < len(self.ages):
                return float(self.ages[joint_index])
            return 999.0

    def motor_age(self) -> float:
        with self._lock:
            ages = [
                float(self.ages[i])
                for i in range(min(self.n, len(self.ages)))
                if self.active_mask[i]
            ]
        return max(ages) if ages else 999.0

    def snapshot(self):
        with self._lock:
            count = self._hist_count
            if count < 2:
                return None
            if count < self._hist_cap:
                idx = np.arange(count)
            else:
                idx = (np.arange(self._hist_cap) + self._hist_i) % self._hist_cap
            return (
                self._hist_t[idx].copy(),
                self._hist_q[idx].copy(),
                self._hist_dq[idx].copy(),
                self._hist_tau[idx].copy(),
                self._hist_qd[idx].copy(),
                self._hist_gq[idx].copy(),
                self._hist_gdq[idx].copy(),
                self._hist_gtau[idx].copy(),
            )

    def _on_command(self, msg: JointState) -> None:
        n = self.n
        index = {name: i for i, name in enumerate(msg.name)}
        q = np.zeros(n)
        tau = np.zeros(n)
        found = False
        if index:
            for i, name in enumerate(JOINTS):
                j = index.get(name)
                if j is None:
                    continue
                if j < len(msg.position):
                    q[i] = float(msg.position[j])
                    found = True
                if j < len(msg.effort):
                    tau[i] = float(msg.effort[j])
        elif len(msg.position) >= n:  # legacy publisher without names
            q = np.array([float(v) for v in msg.position[:n]], dtype=float)
            if len(msg.effort) >= n:
                tau = np.array([float(v) for v in msg.effort[:n]], dtype=float)
            found = True
        if not found:
            return
        with self._lock:
            self._last_q_des = q.copy()
            self.last_tau_ff = tau.copy()
        self.motion.update(q, tau)

    def _on_motion_active(self, msg: Bool) -> None:
        self.motion.set_flags(bool(msg.data), self.motion.pending)

    def _on_pending(self, msg: Int32) -> None:
        _q, active = self.motion.commanded()
        self.motion.set_flags(active, int(msg.data))

    # ------------------------------------------------------------- 夹爪
    def _on_grip_state(self, msg: JointState) -> None:
        if not msg.position:
            return
        with self._lock:
            self.grip_q = float(msg.position[0])
            self.grip_dq = (
                float(msg.velocity[0]) if msg.velocity else 0.0
            )
            self.grip_tau = (
                float(msg.effort[0]) if msg.effort else 0.0
            )
            self.grip_stamp = time.monotonic()

    def _on_grip_status(self, msg: Int32) -> None:
        with self._lock:
            self.grip_state = int(msg.data)

    def _on_grip_contact(self, msg: Bool) -> None:
        with self._lock:
            self.grip_contact = bool(msg.data)

    def gripper_enable(self, on: bool) -> None:
        req = SetBool.Request()
        req.data = bool(on)
        self._call(
            self.cli_grip_enable, req,
            "夹爪使能" if on else "夹爪失能",
        )

    def gripper_sending(self, on: bool) -> None:
        req = SetBool.Request()
        req.data = bool(on)
        self._call(
            self.cli_grip_sending, req,
            "夹爪开始发送" if on else "夹爪停止发送",
        )

    def gripper_open(self) -> None:
        self._call(self.cli_grip_open, Trigger.Request(), "夹爪张开")

    def gripper_grasp(self) -> None:
        self._call(
            self.cli_grip_grasp, Trigger.Request(), "夹爪自适应抓取"
        )

    def gripper_hold_high(self) -> None:
        self._call(
            self.cli_grip_hold, Trigger.Request(), "夹爪高刚度保持"
        )

    def gripper_save(self) -> None:
        self._call(
            self.cli_grip_save, Trigger.Request(), "夹爪保存抓取"
        )

    def gripper_stop(self) -> None:
        self._call(self.cli_grip_stop, Trigger.Request(), "夹爪零力矩停止")

    def gripper_get_parameters(self, names, cb=None) -> bool:
        if not self.cli_grip_get_params.service_is_ready():
            self.log("夹爪读取参数: 服务未就绪")
            return False
        req = GetParameters.Request()
        req.names = [str(n) for n in names]
        future = self.cli_grip_get_params.call_async(req)

        def done(fut):
            try:
                res = fut.result()
            except Exception as exc:
                self.log("夹爪读取参数失败: %s" % exc)
                return
            if res is None:
                return
            if cb is not None:
                cb(list(res.values))

        future.add_done_callback(done)
        return True

    def gripper_set_parameters(self, values, cb=None) -> bool:
        if not self.cli_grip_set_params.service_is_ready():
            self.log("夹爪写参数: 服务未就绪")
            return False
        req = SetParameters.Request()
        for name, value in values.items():
            p = Parameter()
            p.name = str(name)
            pv = ParameterValue()
            if isinstance(value, bool):
                pv.type = ParameterType.PARAMETER_BOOL
                pv.bool_value = bool(value)
            elif isinstance(value, int):
                pv.type = ParameterType.PARAMETER_INTEGER
                pv.integer_value = int(value)
            else:
                pv.type = ParameterType.PARAMETER_DOUBLE
                pv.double_value = float(value)
            p.value = pv
            req.parameters.append(p)
        future = self.cli_grip_set_params.call_async(req)

        def done(fut):
            try:
                res = fut.result()
            except Exception as exc:
                self.log("夹爪写参数失败: %s" % exc)
                return
            if res is None:
                return
            if cb is not None:
                cb(res)

        future.add_done_callback(done)
        return True

    def set_joint_limits(self, lower, upper) -> None:
        lo = np.asarray(lower, dtype=float).reshape(-1)
        hi = np.asarray(upper, dtype=float).reshape(-1)
        if lo.size != self.n or hi.size != self.n:
            raise ValueError("joint limits length must equal joint count")
        if np.any(lo >= hi):
            raise ValueError("joint lower must be < upper")
        with self._lock:
            self.joint_lo = lo.copy()
            self.joint_hi = hi.copy()

    def _clamp_goal_position(self, position):
        raw = np.asarray(position, dtype=float)
        if raw.size != self.n:
            return position
        q = np.clip(raw, self.joint_lo, self.joint_hi)
        if np.any(np.abs(q - raw) > 1e-9):
            now = time.monotonic()
            if now - self._limit_warn_t > 2.0:
                self._limit_warn_t = now
                bad = [
                    f"J{i + 1}" for i in range(self.n)
                    if abs(q[i] - raw[i]) > 1e-9
                ]
                self.log("关节目标超出软件限位，已裁剪: " + ", ".join(bad))
        return q

    def _publish_goal(self, command: int, position=None, duration: float = 0.0,
                      dwell: float = 0.0, velocity=None) -> None:
        msg = JointGoal()
        msg.command = int(command)
        msg.duration = float(duration)
        msg.dwell = float(dwell)
        if position is not None:
            position = self._clamp_goal_position(position)
        msg.position = [] if position is None else [float(v) for v in position]
        if velocity is not None and len(velocity) == self.n:
            msg.velocity = [float(v) for v in velocity]
        else:
            msg.velocity = []
        msg.kp = float(self._kp)
        msg.kd = float(self._kd)
        msg.gravity = bool(self.gravity_enabled)
        msg.gravity_scale = float(self.gravity_scale)
        msg.friction = bool(self.friction_enabled)
        msg.coulomb = float(self.coulomb)
        msg.viscous = float(self.viscous)
        msg.friction_slope = float(self.friction_slope)
        self.goal_pub.publish(msg)

    def publish_setup(self) -> None:
        self._publish_goal(JointGoal.GAINS)

    def drag_teach(self, on: bool) -> None:
        self._publish_goal(JointGoal.DRAG_ON if on else JointGoal.DRAG_OFF)
        self.log("进入拖动示教" if on else "退出拖动示教，停在当前位置")

    # --------------------------------------------------------------- 服务
    def _call(self, client, request, label: str, cb=None) -> bool:
        if not client.service_is_ready():
            self.log(f"{label}: 服务未就绪（驱动启动了吗？）")
            return False
        future = client.call_async(request)

        def done(fut):
            try:
                res = fut.result()
            except Exception as exc:
                self.log(f"{label}: 调用异常 {exc}")
                return
            if res is None:
                self.log(f"{label}: 无应答")
                return
            self.log(f"{label}: {'成功' if res.success else '失败'} {res.message}")
            if cb is not None:
                cb(res)

        future.add_done_callback(done)
        return True

    def set_enabled(self, on: bool, cb=None) -> None:
        req = SetBool.Request()
        req.data = bool(on)

        def handle(res):
            if res.success:
                self.enabled = bool(on)
                self.driver_enabled = bool(on)
                if not on:
                    self.sending = False
                    self.driver_sending = False
                    self.last_tau_ff = np.zeros(self.n)
                    self.motion.reset(self.current_q())
            self._state_rev += 1
            if cb is not None:
                cb(res)

        self._call(self.cli_enable, req, "使能" if on else "失能", handle)

    def set_sending(self, on: bool, cb=None) -> None:
        req = SetBool.Request()
        req.data = bool(on)

        def handle(res):
            if res.success:
                self.sending = bool(on)
                self.driver_sending = bool(on)
                if on:
                    self.motion.reset(self.current_q())
                else:
                    self.last_tau_ff = np.zeros(self.n)
            self._state_rev += 1
            if cb is not None:
                cb(res)

        self._call(self.cli_sending, req, "开始控制" if on else "停止发送", handle)

    def set_logging(self, on: bool, cb=None) -> None:
        """Toggle the driver's identification CSV logger (~/set_logging)."""
        req = SetBool.Request()
        req.data = bool(on)
        self._call(
            self.cli_logging, req,
            "开始记录 CSV" if on else "停止记录 CSV", cb,
        )

    def release(self, cb=None) -> None:
        def handle(res):
            if res.success:
                self.enabled = False
                self.sending = False
                self.driver_enabled = False
                self.driver_sending = False
                self.last_tau_ff = np.zeros(self.n)
                self.motion.reset(self.current_q())
            self._state_rev += 1
            if cb is not None:
                cb(res)

        self._call(self.cli_release, Trigger.Request(), "急停/失能", handle)

    def set_mode(self, can_id: int, mode: str, cb=None) -> None:
        req = SetMotorMode.Request()
        req.can_id = int(can_id)
        req.mode = str(mode)
        self._call(self.cli_mode, req, f"切模式 0x{can_id:02X}->{mode}", cb)

    def read_param(self, can_id: int, rid: int, cb=None) -> None:
        req = ReadMotorParam.Request()
        req.can_id = int(can_id)
        req.rid = int(rid)
        self._call(self.cli_read, req, f"读参数 0x{can_id:02X} RID {rid}", cb)

    def write_param(self, can_id: int, rid: int, value: float,
                    is_integer: bool, cb=None) -> None:
        req = WriteMotorParam.Request()
        req.can_id = int(can_id)
        req.rid = int(rid)
        req.value = float(value)
        req.is_integer = bool(is_integer)
        self._call(self.cli_write, req, f"写参数 0x{can_id:02X} RID {rid}", cb)

    def save_param(self, can_id: int, cb=None) -> None:
        req = SaveMotorParam.Request()
        req.can_id = int(can_id)
        self._call(self.cli_save, req, f"保存 Flash 0x{can_id:02X}", cb)

    # ------------------------------------------------------ registry / per-motor
    def configure_motors(self, entries, cb=None) -> None:
        req = ConfigureMotors.Request()
        for e in entries:
            m = MotorRegistryEntry()
            m.joint_name = str(e["joint_name"])
            m.can_id = int(e["can_id"])
            m.mst_id = int(e["mst_id"])
            m.model = str(e["model"])
            m.mode = str(e["mode"])
            m.channel = int(e["channel"])
            m.sign = float(e["sign"])
            m.offset = float(e["offset"])
            req.motors.append(m)
        req.apply = True
        self._call(self.cli_configure, req, "应用电机注册表", cb)

    def set_motor_enabled(self, can_ids, on: bool, all_motors: bool = False,
                          cb=None) -> None:
        req = SetMotorEnable.Request()
        req.can_ids = [int(c) for c in can_ids]
        req.all = bool(all_motors)
        req.enable = bool(on)
        label = "全部使能" if all_motors else (
            ("使能 " if on else "失能 ")
            + ", ".join(f"0x{c:02X}" for c in req.can_ids)
        )
        self._call(self.cli_motor_enable, req, label, cb)

    def set_motor_sending(self, can_ids, on: bool, all_motors: bool = False,
                          cb=None) -> None:
        req = SetMotorSending.Request()
        req.can_ids = [int(c) for c in can_ids]
        req.all = bool(all_motors)
        req.sending = bool(on)
        label = "全部开始发送" if all_motors else (
            ("开始发送 " if on else "停止发送 ")
            + ", ".join(f"0x{c:02X}" for c in req.can_ids)
        )
        self._call(self.cli_motor_sending, req, label, cb)

    def publish_motor_mit(self, can_id: int, kp: float, kd: float,
                          q: float, dq: float, tau: float,
                          timeout: float = 1.0) -> None:
        slot = self._slot_for_can(can_id)
        if slot is not None and 0 <= slot < self.n:
            q_raw = float(q)
            q = float(
                min(max(q_raw, self.joint_lo[slot]), self.joint_hi[slot])
            )
            if q != q_raw:
                now = time.monotonic()
                if now - self._limit_warn_t > 2.0:
                    self._limit_warn_t = now
                    self.log(
                        f"J{slot + 1} 单轴 MIT 目标超出软件限位，已裁剪"
                    )
        msg = MotorMitCommand()
        msg.command = MotorMitCommand.MIT
        msg.can_id = int(can_id)
        msg.kp = float(kp)
        msg.kd = float(kd)
        msg.q = float(q)
        msg.dq = float(dq)
        msg.tau = float(tau)
        msg.timeout = float(timeout)
        self.mit_pub.publish(msg)

    def hold_motor(self, can_id: int) -> None:
        msg = MotorMitCommand()
        msg.command = MotorMitCommand.HOLD
        msg.can_id = int(can_id)
        self.mit_pub.publish(msg)

    def zero_motor(self, can_id: int) -> None:
        msg = MotorMitCommand()
        msg.command = MotorMitCommand.ZERO_TORQUE
        msg.can_id = int(can_id)
        self.mit_pub.publish(msg)

    def _slot_for_can(self, can_id: int):
        with self._lock:
            try:
                return self.can_ids.index(int(can_id))
            except ValueError:
                return None

    def motor_enabled(self, can_id: int) -> bool:
        i = self._slot_for_can(can_id)
        with self._lock:
            return bool(i is not None and 0 <= i < len(self.enabled_each)
                        and self.enabled_each[i])

    def motor_sending(self, can_id: int) -> bool:
        i = self._slot_for_can(can_id)
        with self._lock:
            return bool(i is not None and 0 <= i < len(self.sending_each)
                        and self.sending_each[i])

    def registered_can_ids(self):
        with self._lock:
            return [int(c) for c in self.can_ids if c]

    def _poll_status(self) -> None:
        if self.stopping:
            return
        now = time.monotonic()
        if self._status_busy:
            if now - self._status_t0 < 2.0:
                return
            self._status_seq += 1
            self._status_busy = False
        if not self.cli_status.service_is_ready():
            return
        self._status_busy = True
        self._status_t0 = now
        self._status_seq += 1
        seq = self._status_seq
        rev = self._state_rev
        future = self.cli_status.call_async(GetMotorStatus.Request())

        def done(fut):
            if seq != self._status_seq:
                return
            self._status_busy = False
            if self.stopping:
                return
            try:
                res = fut.result()
            except Exception:
                return
            if res is None or not res.success:
                return
            if rev != self._state_rev:
                return
            drv_enabled = bool(getattr(res, "enabled", True))
            drv_sending = bool(getattr(res, "sending", True))
            names = list(getattr(res, "joint_names", []))
            can_ids = list(res.can_ids)
            if names and len(names) == len(can_ids):
                idx = {nm: k for k, nm in enumerate(names)}
                modes = ["未注册"] * self.n
                errs = [-1] * self.n
                ages = [999.0] * self.n
                enabled_each = [False] * self.n
                sending_each = [False] * self.n
                can_slot = [0] * self.n
                mst_slot = [0] * self.n
                model_slot = [""] * self.n
                channel_slot = [0] * self.n
                sign_slot = [1.0] * self.n
                offset_slot = [0.0] * self.n
                for i, jn in enumerate(JOINTS):
                    k = idx.get(jn)
                    if k is None:
                        continue
                    can_slot[i] = int(can_ids[k])
                    if k < len(res.modes):
                        modes[i] = str(res.modes[k])
                    if k < len(res.err):
                        errs[i] = int(res.err[k])
                    if k < len(res.age):
                        ages[i] = float(res.age[k])
                    if k < len(res.enabled_each):
                        enabled_each[i] = bool(res.enabled_each[k])
                    if k < len(res.sending_each):
                        sending_each[i] = bool(res.sending_each[k])
                    mst_slot[i] = (
                        int(res.mst_ids[k])
                        if k < len(res.mst_ids) else can_slot[i]
                    )
                    if k < len(res.models):
                        model_slot[i] = str(res.models[k])
                    if k < len(res.channels):
                        channel_slot[i] = int(res.channels[k])
                    if k < len(res.signs):
                        sign_slot[i] = float(res.signs[k])
                    if k < len(res.offsets):
                        offset_slot[i] = float(res.offsets[k])
                active = np.array([c != 0 for c in can_slot], dtype=bool)
                drv_enabled = any(enabled_each)
                drv_sending = any(sending_each)
                with self._lock:
                    self.modes = modes
                    self.errs = errs
                    self.ages = ages
                    self.enabled_each = enabled_each
                    self.sending_each = sending_each
                    self.can_ids = can_slot
                    self.active_mask = active
                    self.registry_names = names
                    self.registry_can_ids = list(res.can_ids)
                    self.registry_mst_ids = list(getattr(res, "mst_ids", []))
                    self.registry_models = list(getattr(res, "models", []))
                    self.registry_channels = list(getattr(res, "channels", []))
                    self.registry_signs = list(getattr(res, "signs", []))
                    self.registry_offsets = list(getattr(res, "offsets", []))
                    self.driver_enabled = drv_enabled
                    self.driver_sending = drv_sending
            else:
                with self._lock:
                    self.modes = list(res.modes)
                    self.errs = list(res.err)
                    self.ages = list(res.age)
                    self.can_ids = list(res.can_ids)
                    self.driver_enabled = drv_enabled
                    self.driver_sending = drv_sending
            was_enabled, was_sending = self.enabled, self.sending
            self.enabled = bool(drv_enabled)
            self.sending = bool(drv_sending)
            if not drv_enabled and (was_enabled or was_sending):
                self.last_tau_ff = np.zeros(self.n)
                self.log("驱动实际已失能，HMI 已同步停止发送")
            elif not drv_sending and was_sending:
                self.log("驱动实际已停止发送，HMI 已同步停止")

        future.add_done_callback(done)

    # ------------------------------------------------------------- 运动入口
    def ready_for_motion(self) -> bool:
        return bool(self.enabled and self.sending)

    def move_to(self, q, duration: float, dwell: float = 0.0) -> bool:
        if not self.ready_for_motion():
            self.log("未使能或未开始控制，忽略运动指令")
            return False
        self._publish_goal(JointGoal.MOVE, q, duration, dwell)
        return True

    def append_waypoint(self, q, duration: float, dwell: float = 0.0) -> bool:
        if not self.ready_for_motion():
            self.log("未使能或未开始控制，忽略轨迹点")
            return False
        self._publish_goal(JointGoal.APPEND, q, duration, dwell)
        return True

    def set_direct(self, q, dq=None) -> None:
        """Direct setpoint; ``dq`` is used by POS_VEL replay."""
        self._publish_goal(JointGoal.DIRECT, q, 0.0, velocity=dq)

    def stop_motion(self) -> None:
        self._publish_goal(JointGoal.STOP)
        self.log("已请求平滑停止在当前指令位姿")


# --------------------------------------------------------------------------- #
# 曲线
# --------------------------------------------------------------------------- #
class PlotPanel(QWidget):
    def __init__(self, n: int, parent=None):
        super().__init__(parent)
        self.n = n
        self._last_sig = None
        self._build()

    def set_active_mask(self, mask):
        if not hasattr(self, "curves"):
            return
        for j in range(self.n):
            visible = bool(mask[j]) if j < len(mask) else True
            for kind in ("pos", "vel", "tau", "cmd"):
                curve = self.curves.get((j, kind))
                if curve is not None:
                    curve.setVisible(visible)

    def _build(self):
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        bar = QHBoxLayout()
        self.chk_pause = QCheckBox("暂停滚动")
        self.chk_cmd = QCheckBox("显示目标")
        self.chk_cmd.setChecked(False)
        self.chk_grip = QCheckBox("夹爪 J7")
        self.chk_grip.setChecked(True)
        bar.addWidget(self.chk_pause)
        bar.addWidget(self.chk_cmd)
        bar.addWidget(self.chk_grip)
        bar.addStretch(1)
        lay.addLayout(bar)

        self.gw = pg.GraphicsLayoutWidget()
        self.plots = [
            self.gw.addPlot(row=0, col=0, title="位置 (rad)"),
            self.gw.addPlot(row=1, col=0, title="速度 (rad/s)"),
            self.gw.addPlot(row=2, col=0, title="力矩 (N·m)"),
        ]
        for p in self.plots:
            p.showGrid(x=False, y=False)
            p.setDownsampling(auto=False)
            p.enableAutoRange(axis="y", enable=False)
            p.getAxis("left").enableAutoSIPrefix(False)
            p.getAxis("left").setWidth(52)
        self.plots[1].setXLink(self.plots[0])
        self.plots[2].setXLink(self.plots[0])
        self.plots[0].setYRange(-3.2, 3.2, padding=0)
        self.plots[1].setYRange(-2.0, 2.0, padding=0)
        self.plots[2].setYRange(-5.0, 5.0, padding=0)
        self.plots[0].addLegend(offset=(8, 8))
        lay.addWidget(self.gw, 1)

        self.curves = {}
        self._last_sig = None
        self._cmd_shown = False
        self._x_range = None
        self._y_stamp = 0.0
        self._data_kw = {"skipFiniteCheck": True}
        for j in range(self.n):
            color = MOTOR_COLORS[j % len(MOTOR_COLORS)]
            pen = pg.mkPen(color=color, width=1)
            dash = pg.mkPen(color=color, width=1, style=Qt.DashLine)
            label = f"J{j + 1}"
            self.curves[(j, "pos")] = self.plots[0].plot(pen=pen, name=label)
            self.curves[(j, "vel")] = self.plots[1].plot(pen=pen)
            self.curves[(j, "tau")] = self.plots[2].plot(pen=pen)
            self.curves[(j, "cmd")] = self.plots[0].plot(pen=dash)

        # 夹爪（J7，独立 gripper_driver）单独一组曲线，画粗一点便于区分
        g = self.n
        gpen = pg.mkPen(
            color=MOTOR_COLORS[g % len(MOTOR_COLORS)], width=2
        )
        self.curves[(g, "pos")] = self.plots[0].plot(
            pen=gpen, name="J7 夹爪"
        )
        self.curves[(g, "vel")] = self.plots[1].plot(pen=gpen)
        self.curves[(g, "tau")] = self.plots[2].plot(pen=gpen)
        self.chk_grip.toggled.connect(self._apply_grip_visible)

    def _apply_grip_visible(self, on: bool) -> None:
        if not hasattr(self, "curves"):
            return
        for kind in ("pos", "vel", "tau"):
            curve = self.curves.get((self.n, kind))
            if curve is not None:
                curve.setVisible(bool(on))

    def _set_curve(self, curve, x, y) -> None:
        try:
            curve.setData(x, y, **self._data_kw)
        except TypeError:
            self._data_kw = {}
            curve.setData(x, y)

    def _set_grip_curve(self, curve, x, y) -> None:
        # 夹爪断线时历史里是 NaN，connect="finite" 让曲线在那里断开。
        curve.setData(x, y, connect="finite")

    def update_data(self, arrays, show_cmd: bool):
        if not arrays:
            return
        ts, q, dq, tau, qd, gq, gdq, gtau = arrays
        if len(ts) < 2:
            return
        sig = (
            q[-1].tobytes(),
            dq[-1].tobytes(),
            tau[-1].tobytes(),
            qd[-1].tobytes() if show_cmd else b"",
            gq[-1].tobytes(),
            gdq[-1].tobytes(),
            gtau[-1].tobytes(),
        )
        if sig == self._last_sig:
            return
        self._last_sig = sig
        # 历史仍按 100 Hz 采样；GUI 只按 20 Hz 重画，折线最多约 250 点
        npts = len(ts)
        stride = max(1, npts // 250)
        if stride > 1:
            idx = np.arange(0, npts, stride)
            if int(idx[-1]) != npts - 1:
                idx = np.append(idx, npts - 1)
            ts = ts[idx]
            q = q[idx]
            dq = dq[idx]
            tau = tau[idx]
            qd = qd[idx]
            gq = gq[idx]
            gdq = gdq[idx]
            gtau = gtau[idx]
        x = ts - ts[-1]
        for j in range(self.n):
            self._set_curve(self.curves[(j, "pos")], x, q[:, j])
            self._set_curve(self.curves[(j, "vel")], x, dq[:, j])
            self._set_curve(self.curves[(j, "tau")], x, tau[:, j])
            if show_cmd:
                self._set_curve(self.curves[(j, "cmd")], x, qd[:, j])
            elif self._cmd_shown:
                self.curves[(j, "cmd")].setData([], [])
        self._cmd_shown = show_cmd

        g = self.n
        self._set_grip_curve(self.curves[(g, "pos")], x, gq)
        self._set_grip_curve(self.curves[(g, "vel")], x, gdq)
        self._set_grip_curve(self.curves[(g, "tau")], x, gtau)

        lo = max(-10.0, float(x[0]))
        rng = (round(lo, 2), 0.0)
        if rng != self._x_range:
            self._x_range = rng
            self.plots[0].setXRange(lo, 0.0, padding=0)
        now = time.monotonic()
        if now - self._y_stamp < 1.0:
            return
        self._y_stamp = now
        pos_all = np.concatenate([q, qd]) if show_cmd else q
        if self.chk_grip.isChecked():
            pos_all = np.concatenate([pos_all.ravel(), gq])
            vel_all = np.concatenate([dq.ravel(), gdq])
            tau_all = np.concatenate([tau.ravel(), gtau])
        else:
            vel_all, tau_all = dq, tau
        for plot, arr, pad in (
            (self.plots[0], pos_all, 1.0),
            (self.plots[1], vel_all, 1.0),
            (self.plots[2], tau_all, 2.0),
        ):
            span = self._finite_span(arr)
            if span is None or span[1] - span[0] < 1e-6:
                plot.setYRange(-pad, pad, padding=0)
            else:
                plot.setYRange(span[0], span[1], padding=0.15)

    @staticmethod
    def _finite_span(arr):
        vals = arr[np.isfinite(arr)]
        if vals.size == 0:
            return None
        return float(vals.min()), float(vals.max())


# --------------------------------------------------------------------------- #
# 主窗口
# --------------------------------------------------------------------------- #
class MainWindow(QMainWindow):
    sig_log = pyqtSignal(str)
    sig_param = pyqtSignal(str)
    sig_registry = pyqtSignal(object)
    sig_grip_params = pyqtSignal(object)
    sig_grip_set = pyqtSignal(object)

    def __init__(self, kin: ArmKinematics, rate: float = 100.0):
        super().__init__()
        self.kin = kin
        self.n = len(JOINTS)
        # 物理/模型限位作为不可突破的上界；用户可在「关节」页收窄并持久化。
        self._model_limits = np.array(kin.limits, dtype=float)
        self.joint_limits = self._load_joint_limits()
        self.kin.limits = self.joint_limits.copy()
        self._closing = False
        self.waypoints: list[np.ndarray] = []
        self.last_ik = None
        self._preview_initialized = False
        self._pose_spin_guard = False
        self._last_follow = 0.0
        self.registry: list[dict] = []
        self._reg_sync = False
        self._driver_registry_synced = False
        self._driver_proc = None
        self._driver_log = None
        self._driver_connect_deadline = 0.0
        self._driver_sim = False
        self._driver_ready_logged = False
        self._driver_sync_pending = False
        # 夹爪独立进程
        self._gripper_proc = None
        self._gripper_log = None
        self._gripper_log_path = ""
        self._grip_params_loaded = False
        self._grip_saved_mtime = 0.0
        #: 「保存参数」写入的夹爪参数；驱动一连上就自动写回去
        self._grip_restore: dict = {}
        #: 驱动从「未就绪」变「就绪」时补发一次 GAINS
        self._driver_ready_prev = False
        saved = hmi_store.load(hmi_store.REGISTRY_NAME)
        if saved and isinstance(saved.get("motors"), list):
            for ent in saved["motors"]:
                if isinstance(ent, dict):
                    self.registry.append(self._normalize_registry_entry(ent))
        self.gamepad = GamepadInput()
        self._gamepad_active = False
        self._pad_stop_latch = False
        self._jog_pos = np.zeros(3)
        self._jog_R = np.eye(3)
        self._jog_q = np.zeros(self.n)
        self._jog_t = 0.0
        self._pad_warn_t = 0.0
        self._pad_rejected_logged = set()
        self._pad_source_logged = ""
        self._pad_buttons_prev = 0
        self._seen_motion = False
        self._keys = set()
        self._keyboard_active = False
        self._key_suspend = False

        # 动力学参数辨识（激励节点 + 驱动 CSV + 离线拟合）
        self._ident_phase = "idle"
        self._ident_proc = None
        self._ident_analysis_proc = None
        self._ident_log = None
        self._ident_log_path = ""
        self._ident_log_offset = 0
        self._ident_analysis_log = ""
        self._ident_analysis_offset = 0
        self._ident_csv = ""
        self._ident_json = ""
        self._ident_result = None
        self._ident_log_state = "idle"
        self._ident_msg = ""
        self._ident_t0 = 0.0
        self._ident_wall_t0 = 0.0
        self._ident_deadline = 0.0
        self._ident_stop_requested = False
        self._ident_ui_saved = {}
        self._ident_tab_saved = []

        # 拖动录制 / 轨迹复现（HMI 侧位置回放，V1）
        self._rec_active = False
        self._rec_q = []
        self._rec_t = []
        self._rec_vel = []          # 与 _rec_q 对齐的 dq（POS_VEL 回放用）
        self._rec_name = ""         # 当前轨迹名（录制或从文件载入）
        self._rec_path = ""         # 当前轨迹对应的磁盘文件（未保存为空）
        self._rec_t0 = 0.0
        self._rec_last_warn = 0.0
        self._rec_stale = 0
        self._rec_bad = []
        self._replay_active = False
        self._replay_phase = "idle"
        self._replay_q = []
        self._replay_t = []
        self._replay_vel = []
        self._replay_mode = "MIT"
        self._replay_t0 = 0.0
        self._replay_i = 0
        self._replay_seen_motion = False
        self._replay_approach_t0 = 0.0
        self._replay_approach_dur = 0.0
        self._replay_target0 = np.zeros(self.n)

        self.setWindowTitle("ZFY Arm HMI")
        self.resize(1360, 860)
        self.move(40, 30)

        self.bridge = RosBridge(
            log_cb=self.sig_log.emit, n=self.n, rate=rate, kin=self.kin
        )
        self.bridge.set_joint_limits(
            self.joint_limits[:, 0], self.joint_limits[:, 1]
        )
        self.sig_log.connect(self._append_log)
        self.sig_param.connect(self._set_param_text)
        self.sig_registry.connect(self._on_registry_applied)
        self.sig_grip_params.connect(self._on_grip_params)
        self.sig_grip_set.connect(self._on_grip_set_result)
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.bridge)
        self._spin_thread = threading.Thread(
            target=self.executor.spin, daemon=True
        )
        self._spin_thread.start()

        self._build_ui()
        self._restore_settings()   # 内部会同步 kp/kd 与前馈到 bridge
        self._update_limits_to_spins()
        self.cb_driver_config.setCurrentIndex(1)
        QTimer.singleShot(0, self._restore_layout)
        self.bridge.log(
            "重力补偿默认关闭。零位和关节方向未写入驱动前，"
            "不要打开重力，也不要回 fold、逆解或手柄。"
        )
        QApplication.instance().installEventFilter(self)
        if self.gamepad.error:
            self.sig_log.emit(
                f"手柄 UDP 端口 {self.gamepad.port} 绑定失败: {self.gamepad.error}"
            )
        else:
            self.sig_log.emit(
                f"手柄 UDP 监听 0.0.0.0:{self.gamepad.port}；"
                "Windows 侧运行 xbox_udp.ps1"
            )

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._tick_ui)
        self.ui_timer.start(100)
        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self._tick_plots)
        self.plot_timer.start(50)
        self.pad_timer = QTimer(self)
        self.pad_timer.timeout.connect(self._tick_gamepad)
        self.pad_timer.start(10)
        self.mit_timer = QTimer(self)
        self.mit_timer.timeout.connect(self._tick_mit_live)
        self.mit_timer.start(50)
        self.ident_timer = QTimer(self)
        self.ident_timer.timeout.connect(self._tick_ident)
        self.ident_timer.start(200)
        self.grip_timer = QTimer(self)
        self.grip_timer.timeout.connect(self._tick_gripper)
        self.grip_timer.start(100)
        self.rec_timer = QTimer(self)
        self.rec_timer.setInterval(7)
        self.rec_timer.timeout.connect(self._rec_tick)
        self.replay_timer = QTimer(self)
        self.replay_timer.setInterval(7)
        self.replay_timer.timeout.connect(self._replay_tick)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        root.addWidget(self._build_top_bar())

        self.vsplit = QSplitter(Qt.Vertical)
        self.outer = QSplitter(Qt.Horizontal)
        self.outer.addWidget(self._build_registry_panel())

        self.inner = QSplitter(Qt.Horizontal)
        self.inner.addWidget(self._build_tabs())
        self.plot_panel = PlotPanel(self.n)
        self.inner.addWidget(self.plot_panel)
        self.inner.setStretchFactor(0, 1)
        self.inner.setStretchFactor(1, 1)
        self.inner.setSizes([560, 800])
        self.splitter = self.inner  # legacy name used by _on_tab_changed
        self.tabs.currentChanged.connect(self._on_tab_changed)

        self.outer.addWidget(self.inner)
        self.outer.setStretchFactor(0, 0)
        self.outer.setStretchFactor(1, 1)
        self.outer.setSizes([360, 1000])

        self.vsplit.addWidget(self.outer)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(600)
        self.vsplit.addWidget(self.log_view)
        self.vsplit.setSizes([700, 130])
        root.addWidget(self.vsplit, 1)
        self._update_joint_availability()

    def _build_registry_panel(self):
        box = QGroupBox("电机注册表")
        box.setMinimumWidth(330)
        lay = QVBoxLayout(box)

        conn = QHBoxLayout()
        self.chk_sim = QCheckBox("仿真")
        self.chk_sim.setChecked(False)
        self.cb_driver_config = QComboBox()
        for label, name in (
            ("实机 SocketCAN 双路 (motors_socketcan_dual.yaml)",
             "motors_socketcan_dual.yaml"),
            ("实机 SocketCAN 单路 can0（自动连接模块）",
             "motors_socketcan_wsl.yaml"),
            ("两电机台架 (motors_two.yaml)", "motors_two.yaml"),
            ("实机 6 轴 vendor (motors_real.yaml)", "motors_real.yaml"),
            ("仿真 6 轴 (motors.yaml)", "motors.yaml"),
        ):
            self.cb_driver_config.addItem(label, name)
        self.cb_driver_config.setCurrentIndex(1)
        self.btn_connect = QPushButton("连接驱动")
        self.btn_connect.clicked.connect(self.on_connect_module)
        self.lbl_driver = QLabel("驱动: 未连接")
        self.lbl_driver.setObjectName("hint")
        conn.addWidget(self.chk_sim)
        conn.addWidget(self.cb_driver_config, 1)
        conn.addWidget(self.btn_connect)
        lay.addLayout(conn)
        lay.addWidget(self.lbl_driver)

        self.tbl_reg = QTableWidget(0, 11)
        self.tbl_reg.setHorizontalHeaderLabels(
            ["使能", "发送", "关节", "CAN ID", "MST ID", "型号",
             "模式", "通道", "符号", "零位", "状态"]
        )
        self.tbl_reg.verticalHeader().setVisible(False)
        hdr = self.tbl_reg.horizontalHeader()
        hdr.setSectionResizeMode(QHeaderView.Interactive)
        hdr.setStretchLastSection(True)
        for i, w in enumerate((40, 40, 74, 66, 66, 76, 76, 46, 48, 70)):
            self.tbl_reg.setColumnWidth(i, w)
        lay.addWidget(self.tbl_reg, 1)

        row1 = QHBoxLayout()
        for text, slot in (
            ("添加", self.on_registry_add),
            ("删除选中", self.on_registry_delete),
            ("应用注册表", self.on_registry_apply),
            ("读取驱动注册表", self.on_registry_read_driver),
        ):
            b = QPushButton(text)
            b.clicked.connect(slot)
            row1.addWidget(b)
        lay.addLayout(row1)

        row2 = QHBoxLayout()
        for text, slot in (
            ("全部使能", self.on_enable_all),
            ("全部失能", self.on_disable_all),
            ("全部开始发送", self.on_send_all),
            ("全部停止发送", self.on_stop_all),
        ):
            b = QPushButton(text)
            b.clicked.connect(slot)
            row2.addWidget(b)
        lay.addLayout(row2)

        hint = QLabel(
            "先注册并应用，再使能；使能后点『发送数据』才会收发。"
            "同一关节/CAN/MST 不能重复。"
        )
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        lay.addWidget(hint)
        self._rebuild_registry_table()
        return box

    # ---------------------------------------------------------- driver connect
    def _bridge_ready(self) -> bool:
        try:
            return bool(self.bridge.cli_status.service_is_ready())
        except Exception:
            return False

    def on_connect_module(self):
        if self._rec_active or self._replay_active:
            self.bridge.log("录制/回放中，不能连接/断开驱动")
            return
        if self._driver_proc is not None:
            self._stop_driver_process()
            self._set_driver_state(self._bridge_ready())
            return
        if self._bridge_ready():
            self.bridge.log("驱动已在运行（外部启动）")
            self._set_driver_state(True)
            return
        self._start_driver_process()

    def _ensure_usb_attached(self):
        """Make sure the USB-CANFD adapter is visible in WSL.

        Supports both the gs_usb firmware (1d50:606f) and the vendor
        firmware (34b7:6877).  If the module is bound but not attached,
        run usbipd attach automatically.  Returns (ok, message).
        """
        vids = ("1d50:606f", "34b7:6877")

        def present() -> bool:
            try:
                out = subprocess.check_output(
                    ["lsusb"], text=True, timeout=3
                )
                low = out.lower()
                return any(vid in low for vid in vids)
            except Exception:
                return False

        if present():
            return True, ""
        try:
            out = subprocess.check_output(
                ["usbipd.exe", "list"], text=True, timeout=5,
                stderr=subprocess.STDOUT,
            )
        except Exception as exc:
            return False, f"usbipd 不可用: {exc}"
        busid = None
        line_text = ""
        for line in out.splitlines():
            low = line.lower()
            if any(vid in low for vid in vids):
                parts = line.split()
                if parts:
                    busid = parts[0]
                line_text = low
                break
        if busid is None:
            return False, (
                "Windows 侧没有看到 USB2CANFD 模块。请把模块插好，再点连接。"
            )
        if "attached" in line_text:
            time.sleep(0.5)
            return (
                (True, "") if present() else
                (False, "usbipd 显示已 attach，但 WSL 仍看不到设备；试拔插模块")
            )
        if "not shared" in line_text:
            return False, (
                "USB2CANFD 未共享。请在 Windows 管理员 PowerShell 执行：\n"
                f"usbipd bind --busid {busid}\n然后重试。"
            )
        try:
            res = subprocess.run(
                ["usbipd.exe", "attach", "--wsl", "--busid", busid],
                text=True, timeout=8, capture_output=True,
            )
        except Exception as exc:
            return False, f"usbipd attach 失败: {exc}"
        for _ in range(10):
            time.sleep(0.5)
            if present():
                return True, ""
        detail = (res.stderr or res.stdout or "").strip()
        return False, (
            "usbipd attach 后 WSL 仍看不到设备"
            + (f"（{detail}）" if detail else "；试拔插模块/换 USB 口")
        )

    def _driver_config_path(self, name: str) -> str:
        local = os.path.join(PROJECT_ROOT, "config", name)
        if os.path.isfile(local):
            return local
        return os.path.expanduser(
            "~/zfy_arm/Zfy_Arm_real_control/zfy_arm_sim2real/config/"
            + name
        )

    @staticmethod
    def _can_ifaces(cfg_path: str):
        try:
            import yaml
            with open(cfg_path, "r", encoding="utf-8") as fh:
                data = yaml.safe_load(fh) or {}
            node = (data.get("motor_driver") or {}).get(
                "ros__parameters") or {}
            ifaces = node.get("can_channels") or [
                node.get("can_channel") or "can0"]
            ifaces = [str(i) for i in ifaces if i]
            bitrate = int(node.get("can_bitrate") or 1000000)
            return ifaces or ["can0"], bitrate
        except Exception:
            return ["can0"], 1000000

    @staticmethod
    def _run_root(cmd: str):
        """Run a root-only shell command; sudo -n first, then WSL 的 root."""
        attempts = [["sudo", "-n", "bash", "-lc", cmd]]
        distro = os.environ.get("WSL_DISTRO_NAME")
        wsl = ["wsl.exe"]
        if distro:
            wsl += ["-d", distro]
        attempts.append(wsl + ["-u", "root", "-e", "bash", "-lc", cmd])
        last = ""
        for argv in attempts:
            try:
                res = subprocess.run(
                    argv, text=True, timeout=20, capture_output=True)
            except Exception as exc:
                last = str(exc)
                continue
            if res.returncode == 0:
                return True, ""
            last = ((res.stderr or res.stdout or "").strip()
                    or f"rc={res.returncode}")
        return False, last

    @staticmethod
    def _can_state_ok(iface: str, bitrate: int) -> bool:
        try:
            with open(f"/sys/class/net/{iface}/operstate") as fh:
                if fh.read().strip() != "up":
                    return False
        except Exception:
            return False
        try:
            out = subprocess.check_output(
                ["ip", "-details", "link", "show", iface],
                text=True, timeout=3)
        except Exception:
            return False
        return f"bitrate {int(bitrate)}" in out

    def _ensure_can_ready(self, cfg_path: str):
        """Bring every can_channels entry up at the configured bitrate."""
        ifaces, bitrate = self._can_ifaces(cfg_path)
        for iface in ifaces:
            for _ in range(10):
                if os.path.exists(f"/sys/class/net/{iface}"):
                    break
                time.sleep(0.5)
            if not os.path.exists(f"/sys/class/net/{iface}"):
                ok, msg = self._run_root("modprobe gs_usb")
                if ok:
                    for _ in range(10):
                        time.sleep(0.5)
                        if os.path.exists(f"/sys/class/net/{iface}"):
                            break
                if not os.path.exists(f"/sys/class/net/{iface}"):
                    return False, (
                        f"CAN 接口 {iface} 不存在"
                        + (f"（modprobe gs_usb 失败: {msg}）" if not ok else "")
                        + "；确认模块已 attach、驱动已加载"
                    )
            if self._can_state_ok(iface, bitrate):
                continue
            ok, msg = self._run_root(
                f"ip link set {iface} type can bitrate {int(bitrate)} && "
                f"ip link set {iface} up"
            )
            if not ok:
                return False, f"配置 {iface} 失败: {msg}"
            for _ in range(6):
                time.sleep(0.3)
                if self._can_state_ok(iface, bitrate):
                    break
            if not self._can_state_ok(iface, bitrate):
                return False, f"{iface} 已配置但未进入 up/{bitrate}"
            self.bridge.log(f"CAN {iface} 已配置: {bitrate} bps, up")
        self.bridge.log("CAN 已就绪: " + ", ".join(ifaces))
        return True, ""

    def _start_driver_process(self):
        cfg_name = self.cb_driver_config.currentData() or (
            "motors_socketcan_wsl.yaml")
        cfg = self._driver_config_path(str(cfg_name))
        self._driver_sim = bool(self.chk_sim.isChecked())
        use_socketcan = "socketcan" in str(cfg_name)
        if not self._driver_sim:
            ok, msg = self._ensure_usb_attached()
            if not ok:
                self.bridge.log("USB: " + msg)
                QMessageBox.warning(self, "连接驱动", msg)
                return
            if use_socketcan:
                ok, msg = self._ensure_can_ready(cfg)
                if not ok:
                    self.bridge.log("CAN: " + msg)
                    QMessageBox.warning(self, "连接驱动", msg)
                    return
        sim = "true" if self._driver_sim else "false"
        install = os.path.join(PROJECT_ROOT, "ros2", "install", "setup.bash")
        if os.path.isfile(install):
            install_cmd = "source " + install
        else:
            install_cmd = (
                "source $HOME/zfy_arm/Zfy_Arm_real_control/install/setup.bash"
            )
        inner = (
            "source /opt/ros/humble/setup.bash && "
            + install_cmd + " && "
            "exec ros2 launch zfy_arm_sim2real motor_driver.launch.py "
            f"sim:={sim} config:={cfg}"
        )
        self.bridge.log("启动驱动: " + inner)
        try:
            self._driver_log = open(
                "/tmp/zfy_hmi_driver.log", "wb", buffering=0
            )
            self._driver_proc = subprocess.Popen(
                ["bash", "-lc", inner],
                stdout=self._driver_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception as exc:
            self._close_driver_log()
            self._driver_proc = None
            QMessageBox.critical(self, "连接模块", f"启动驱动失败: {exc}")
            return
        self._driver_connect_deadline = time.monotonic() + 12.0
        self._driver_ready_logged = False
        self._driver_sync_pending = True
        self.btn_connect.setEnabled(False)
        self._set_text(self.lbl_driver, "驱动: 正在启动…")
        self.lbl_driver.setStyleSheet("color: #ebcb8b;")

    def _close_driver_log(self):
        log = self._driver_log
        self._driver_log = None
        if log is not None:
            try:
                log.close()
            except Exception:
                pass

    def _stop_driver_process(self):
        proc = self._driver_proc
        self._driver_proc = None
        if proc is not None:
            try:
                pgid = os.getpgid(proc.pid)
            except Exception:
                pgid = None
            if pgid is not None:
                try:
                    os.killpg(pgid, signal.SIGINT)
                except Exception:
                    pass
            try:
                proc.wait(timeout=3.0)
            except Exception:
                if pgid is not None:
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                    except Exception:
                        pass
                else:
                    try:
                        proc.kill()
                    except Exception:
                        pass
        self._close_driver_log()

    def _log_driver_tail(self, n: int = 10):
        try:
            with open(
                "/tmp/zfy_hmi_driver.log", "r", encoding="utf-8",
                errors="replace",
            ) as fh:
                lines = fh.readlines()[-n:]
        except Exception:
            return
        for line in lines:
            self.bridge.log("驱动日志: " + line.rstrip())

    def _set_driver_state(self, connected: bool):
        if connected:
            mode = "仿真" if self._driver_sim else (
                    "SocketCAN" if "socketcan" in str(
                        self.cb_driver_config.currentData()) else "USB-CANFD")
            if self._driver_proc is not None:
                self._set_text(self.btn_connect, "断开驱动")
                self.btn_connect.setEnabled(True)
                self._set_text(self.lbl_driver, f"驱动: 已连接（{mode}）")
            else:
                self._set_text(self.btn_connect, "已连接（外部）")
                self.btn_connect.setEnabled(False)
                self._set_text(self.lbl_driver, "驱动: 已连接（外部）")
            self.lbl_driver.setStyleSheet("color: #a3be8c;")
        else:
            self._set_text(self.btn_connect, "连接驱动")
            self.btn_connect.setEnabled(True)
            self._set_text(self.lbl_driver, "驱动: 未连接")
            self.lbl_driver.setStyleSheet("color: #8f98a5;")

    def _tick_driver_state(self):
        ready = self._bridge_ready()
        if ready and not self._driver_ready_prev:
            # 驱动晚于 HMI 启动时，启动时发的 GAINS 会丢；连上后补一次。
            self.bridge.publish_setup()
            self.bridge.log("驱动已就绪：已下发当前 kp/kd 与前馈设置")
        self._driver_ready_prev = ready
        if self._driver_proc is not None:
            if ready:
                if not self._driver_ready_logged:
                    self._driver_ready_logged = True
                    mode = "仿真" if self._driver_sim else (
                    "SocketCAN" if "socketcan" in str(
                        self.cb_driver_config.currentData()) else "USB-CANFD")
                    self.bridge.log(f"驱动已连接（{mode}）")
                self._set_driver_state(True)
            elif self._driver_proc.poll() is not None:
                self.bridge.log(
                    "驱动进程已退出；错误见下方驱动日志"
                )
                self._log_driver_tail()
                self._close_driver_log()
                self._driver_proc = None
                self._set_driver_state(False)
            elif time.monotonic() > self._driver_connect_deadline:
                self.bridge.log(
                    "驱动 12 s 内未就绪；检查 USB 是否已 usbipd attach"
                )
                self._log_driver_tail()
                self._stop_driver_process()
                self._set_driver_state(False)
            else:
                self.btn_connect.setEnabled(False)
                self._set_text(self.lbl_driver, "驱动: 正在启动…")
                self.lbl_driver.setStyleSheet("color: #ebcb8b;")
        else:
            self._set_driver_state(ready)

    # ------------------------------------------------------------ registry UI
    @staticmethod
    def _normalize_registry_entry(ent: dict) -> dict:
        can = int(ent.get("can_id", 1))
        return {
            "joint_name": str(ent.get("joint_name", "joint1")),
            "can_id": can,
            "mst_id": int(ent.get("mst_id", can)),
            "model": str(ent.get("model", "4340")),
            "mode": str(ent.get("mode", "MIT")).upper(),
            "channel": int(ent.get("channel", 0)),
            "sign": float(ent.get("sign", 1.0)),
            "offset": float(ent.get("offset", 0.0)),
        }

    def _rebuild_registry_table(self):
        if not hasattr(self, "tbl_reg"):
            return
        self._reg_sync = True
        try:
            self.tbl_reg.setRowCount(0)
            for ent in self.registry:
                self._add_registry_row(ent)
        finally:
            self._reg_sync = False
        self._refresh_mit_motor_combo()
        self._update_joint_availability()
        try:
            self.tbl_reg.scrollToTop()
        except Exception:
            pass

    def _add_registry_row(self, ent: dict):
        r = self.tbl_reg.rowCount()
        self.tbl_reg.insertRow(r)

        chk_en = QCheckBox()
        chk_send = QCheckBox()
        self.tbl_reg.setCellWidget(r, 0, chk_en)
        self.tbl_reg.setCellWidget(r, 1, chk_send)
        chk_en.toggled.connect(
            lambda checked, e=ent: self._on_row_enable(e, checked)
        )
        chk_send.toggled.connect(
            lambda checked, e=ent: self._on_row_send(e, checked)
        )

        cb_joint = QComboBox()
        for i, name in enumerate(JOINTS):
            cb_joint.addItem(f"J{i + 1} {name}", name)
        idx = cb_joint.findData(ent["joint_name"])
        cb_joint.setCurrentIndex(max(0, idx))
        cb_joint.currentIndexChanged.connect(
            lambda _i, cb=cb_joint, e=ent: self._registry_update(
                e, "joint_name", cb.currentData()
            )
        )
        self.tbl_reg.setCellWidget(r, 2, cb_joint)

        ed_can = QLineEdit(f"0x{ent['can_id']:02X}")
        ed_mst = QLineEdit(f"0x{ent['mst_id']:02X}")
        ed_can.editingFinished.connect(
            lambda ed=ed_can, e=ent: self._registry_update_hex(e, "can_id", ed)
        )
        ed_mst.editingFinished.connect(
            lambda ed=ed_mst, e=ent: self._registry_update_hex(e, "mst_id", ed)
        )
        self.tbl_reg.setCellWidget(r, 3, ed_can)
        self.tbl_reg.setCellWidget(r, 4, ed_mst)

        cb_model = QComboBox()
        cb_model.addItems(MODEL_NAMES)
        cb_model.setCurrentText(ent["model"])
        cb_model.currentTextChanged.connect(
            lambda t, e=ent: self._registry_update(e, "model", t)
        )
        self.tbl_reg.setCellWidget(r, 5, cb_model)

        cb_mode = QComboBox()
        cb_mode.addItems(MODES)
        cb_mode.setCurrentText(ent["mode"])
        cb_mode.currentTextChanged.connect(
            lambda t, e=ent: self._registry_update(e, "mode", t)
        )
        self.tbl_reg.setCellWidget(r, 6, cb_mode)

        sp_ch = QSpinBox()
        sp_ch.setRange(0, 3)
        sp_ch.setValue(ent["channel"])
        sp_ch.valueChanged.connect(
            lambda v, e=ent: self._registry_update(e, "channel", v)
        )
        self.tbl_reg.setCellWidget(r, 7, sp_ch)

        cb_sign = QComboBox()
        cb_sign.addItems(["+1", "-1"])
        cb_sign.setCurrentIndex(0 if ent["sign"] >= 0 else 1)
        cb_sign.currentIndexChanged.connect(
            lambda i, e=ent: self._registry_update(
                e, "sign", 1.0 if i == 0 else -1.0
            )
        )
        self.tbl_reg.setCellWidget(r, 8, cb_sign)

        sp_off = QDoubleSpinBox()
        sp_off.setRange(-12.5, 12.5)
        sp_off.setDecimals(4)
        sp_off.setSingleStep(0.01)
        sp_off.setValue(ent["offset"])
        sp_off.valueChanged.connect(
            lambda v, e=ent: self._registry_update(e, "offset", v)
        )
        self.tbl_reg.setCellWidget(r, 9, sp_off)

        status = QTableWidgetItem("—")
        status.setFlags(status.flags() & ~Qt.ItemIsEditable)
        self.tbl_reg.setItem(r, 10, status)

    def _registry_update(self, ent: dict, key: str, value) -> None:
        if value is None:
            return
        ent[key] = value

    def _registry_update_hex(self, ent: dict, key: str, edit: QLineEdit) -> None:
        text = edit.text().strip()
        try:
            value = int(text, 0)
            if not 0 < value <= 0x7FF:
                raise ValueError
            ent[key] = value
            edit.setText(f"0x{value:02X}")
        except ValueError:
            edit.setText(f"0x{int(ent.get(key, 1)):02X}")

    def _refresh_mit_motor_combo(self) -> None:
        if not hasattr(self, "cb_mit_motor"):
            return
        current = self.cb_mit_motor.currentData()
        self.cb_mit_motor.blockSignals(True)
        self.cb_mit_motor.clear()
        for ent in self.registry:
            self.cb_mit_motor.addItem(
                f"{ent['joint_name']} (0x{ent['can_id']:02X})",
                int(ent["can_id"]),
            )
        if current is not None:
            i = self.cb_mit_motor.findData(current)
            if i >= 0:
                self.cb_mit_motor.setCurrentIndex(i)
        self.cb_mit_motor.blockSignals(False)
        self._sync_mit_selection()

    def _on_row_enable(self, ent: dict, checked: bool) -> None:
        if self._reg_sync:
            return
        cid = int(ent["can_id"])
        if cid not in self.bridge.registered_can_ids():
            self._reg_sync = True
            try:
                row = self.registry.index(ent)
                chk = self.tbl_reg.cellWidget(row, 0)
                if chk is not None:
                    chk.setChecked(False)
            finally:
                self._reg_sync = False
            self.bridge.log(f"0x{cid:02X} 还没有应用注册表，先点『应用注册表』")
            return
        self.bridge.set_motor_enabled([cid], bool(checked))
        if not checked:
            row = self.registry.index(ent) if ent in self.registry else -1
            if row >= 0:
                chk = self.tbl_reg.cellWidget(row, 1)
                if chk is not None:
                    self._reg_sync = True
                    chk.setChecked(False)
                    self._reg_sync = False

    def _on_row_send(self, ent: dict, checked: bool) -> None:
        if self._reg_sync:
            return
        cid = int(ent["can_id"])
        if checked and not self.bridge.motor_enabled(cid):
            self._reg_sync = True
            row = self.registry.index(ent)
            chk = self.tbl_reg.cellWidget(row, 1)
            if chk is not None:
                chk.setChecked(False)
            self._reg_sync = False
            self.bridge.log(f"0x{cid:02X} 未使能，不能发送")
            return
        self.bridge.set_motor_sending([cid], bool(checked))

    def on_registry_add(self) -> None:
        used_joints = {e["joint_name"] for e in self.registry}
        used_ids = set()
        for e in self.registry:
            used_ids.update((int(e["can_id"]), int(e["mst_id"])))
        free_joint = next((j for j in JOINTS if j not in used_joints), None)
        free_can = next((c for c in range(1, 8) if c not in used_ids), None)
        if free_joint is None or free_can is None:
            QMessageBox.warning(self, "注册表", "6 个关节或 CAN ID 已用完")
            return
        self.registry.append(
            self._normalize_registry_entry(
                {
                    "joint_name": free_joint,
                    "can_id": free_can,
                    "mst_id": free_can,
                    "model": "4340",
                    "mode": "MIT",
                    "channel": 0,
                    "sign": 1.0,
                    "offset": 0.0,
                }
            )
        )
        self._rebuild_registry_table()

    def on_registry_delete(self) -> None:
        rows = sorted(
            {idx.row() for idx in self.tbl_reg.selectedIndexes()},
            reverse=True,
        )
        if not rows and self.tbl_reg.currentRow() >= 0:
            rows = [self.tbl_reg.currentRow()]
        for row in rows:
            if 0 <= row < len(self.registry):
                self.registry.pop(row)
        if rows:
            self._rebuild_registry_table()

    def _read_registry_entries(self):
        if not self.registry:
            raise ValueError("注册表是空的")
        if len(self.registry) > 6:
            raise ValueError("最多注册 6 台电机")
        seen_joint = set()
        seen_id = set()
        out = []
        for i, raw in enumerate(self.registry):
            ent = self._normalize_registry_entry(raw)
            jn = ent["joint_name"]
            if jn not in JOINTS:
                raise ValueError(f"第 {i + 1} 行关节名无效: {jn!r}")
            if jn in seen_joint:
                raise ValueError(f"关节重复: {jn}")
            seen_joint.add(jn)
            cid, mid, ch = ent["can_id"], ent["mst_id"], ent["channel"]
            if not (0 < cid <= 0x7FF) or not (0 < mid <= 0x7FF):
                raise ValueError(f"第 {i + 1} 行 CAN/MST ID 越界")
            if ent["model"] not in MODEL_NAMES:
                raise ValueError(f"第 {i + 1} 行型号无效: {ent['model']}")
            if ent["mode"] not in MODES:
                raise ValueError(f"第 {i + 1} 行模式无效: {ent['mode']}")
            if ch < 0:
                raise ValueError(f"第 {i + 1} 行通道无效")
            if ent["sign"] not in (-1.0, 1.0):
                raise ValueError(f"第 {i + 1} 行方向只能是 +1/-1")
            for ident in {cid, mid}:
                key = (ch, ident)
                if key in seen_id:
                    raise ValueError(
                        f"通道 {ch} 的 CAN/MST ID 0x{ident:02X} 冲突"
                    )
            seen_id.update({(ch, cid), (ch, mid)})
            out.append(ent)
        return out

    def on_registry_apply(self) -> None:
        if self._rec_active or self._replay_active:
            self.bridge.log("录制/回放中，不能应用电机注册表")
            return
        try:
            entries = self._read_registry_entries()
        except ValueError as exc:
            QMessageBox.warning(self, "注册表有误", str(exc))
            return
        self.bridge.configure_motors(
            entries, cb=lambda res: self.sig_registry.emit(res)
        )

    def _on_registry_applied(self, res) -> None:
        if not res.success:
            QMessageBox.warning(self, "注册表", res.message)
            self._append_log("注册表: " + res.message)
            return
        by_can = {int(e["can_id"]): e for e in self.registry}
        new = []
        for cid, jn in zip(res.can_ids, res.joint_names):
            ent = by_can.get(int(cid))
            if ent is None:
                continue
            ent["joint_name"] = str(jn)
            new.append(ent)
        if new:
            self.registry = new
        self._rebuild_registry_table()
        self._save_registry()
        self._append_log(res.message)
        self._driver_registry_synced = True

    def on_registry_read_driver(self) -> None:
        self._sync_registry_from_driver(force=True)

    def _sync_registry_from_driver(self, force: bool = False) -> None:
        bridge = self.bridge
        names = list(bridge.registry_names)
        if not names:
            if force:
                self.bridge.log("驱动还没有注册表（驱动未启动？）")
            return
        cans = list(bridge.registry_can_ids)
        msts = list(bridge.registry_mst_ids) or list(cans)
        models = list(bridge.registry_models)
        channels = list(bridge.registry_channels)
        signs = list(bridge.registry_signs)
        offsets = list(bridge.registry_offsets)
        entries = []
        for i, name in enumerate(names):
            entries.append(
                self._normalize_registry_entry(
                    {
                        "joint_name": name,
                        "can_id": cans[i] if i < len(cans) else 1,
                        "mst_id": msts[i] if i < len(msts) else 1,
                        "model": models[i] if i < len(models) else "4340",
                        "mode": "MIT",
                        "channel": channels[i] if i < len(channels) else 0,
                        "sign": signs[i] if i < len(signs) else 1.0,
                        "offset": offsets[i] if i < len(offsets) else 0.0,
                    }
                )
            )
        self.registry = entries
        self._rebuild_registry_table()
        self._save_registry()
        self._driver_registry_synced = True
        self._append_log("已从驱动读取注册表: " + ", ".join(names))

    def _save_registry(self) -> None:
        try:
            hmi_store.save(
                hmi_store.REGISTRY_NAME,
                {"motors": [dict(e) for e in self.registry]},
            )
        except Exception as exc:
            self._append_log(f"保存注册表失败: {exc}")

    def _update_registry_rows(self) -> None:
        if not hasattr(self, "tbl_reg"):
            return
        self._reg_sync = True
        try:
            for r, ent in enumerate(self.registry):
                if r >= self.tbl_reg.rowCount():
                    break
                cid = int(ent["can_id"])
                i = self.bridge._slot_for_can(cid)
                if i is None or not self.bridge.active_mask[i]:
                    enabled = sending = False
                    text = "未应用"
                else:
                    enabled = bool(self.bridge.enabled_each[i])
                    sending = bool(self.bridge.sending_each[i])
                    mode = (
                        self.bridge.modes[i]
                        if i < len(self.bridge.modes) else "?"
                    )
                    err = (
                        self.bridge.errs[i]
                        if i < len(self.bridge.errs) else -1
                    )
                    age = (
                        self.bridge.ages[i]
                        if i < len(self.bridge.ages) else 999.0
                    )
                    text = f"{mode} · ERR {err} · {age * 1000:.0f}ms"
                chk_en = self.tbl_reg.cellWidget(r, 0)
                chk_send = self.tbl_reg.cellWidget(r, 1)
                if chk_en is not None:
                    chk_en.setChecked(enabled)
                if chk_send is not None:
                    chk_send.setEnabled(enabled)
                    chk_send.setChecked(sending)
                item = self.tbl_reg.item(r, 10)
                if item is not None and item.text() != text:
                    item.setText(text)
        finally:
            self._reg_sync = False

    def _save_layout(self) -> None:
        if hmi_store.offscreen():
            return
        try:
            hmi_store.save(
                hmi_store.LAYOUT_NAME,
                {
                    "outer": list(self.outer.sizes()),
                    "inner": list(self.inner.sizes()),
                    "vsplit": list(self.vsplit.sizes()),
                    "window": [int(self.width()), int(self.height())],
                    "tab": int(self.tabs.currentIndex()),
                },
            )
        except Exception as exc:
            self._append_log(f"保存布局失败: {exc}")

    def _restore_layout(self) -> None:
        data = hmi_store.load(hmi_store.LAYOUT_NAME)
        if not data:
            return
        for key, splitter in (
            ("outer", self.outer),
            ("inner", self.inner),
            ("vsplit", self.vsplit),
        ):
            sizes = data.get(key)
            if (
                isinstance(sizes, list)
                and len(sizes) == splitter.count()
                and all(isinstance(v, (int, float)) for v in sizes)
            ):
                splitter.setSizes([int(v) for v in sizes])
        window = data.get("window")
        if (
            isinstance(window, list)
            and len(window) == 2
            and all(isinstance(v, (int, float)) for v in window)
        ):
            self.resize(max(800, int(window[0])), max(600, int(window[1])))
        tab = data.get("tab")
        if isinstance(tab, int) and 0 <= tab < self.tabs.count():
            self.tabs.setCurrentIndex(tab)

    def _all_registered(self) -> bool:
        return len({e["joint_name"] for e in self.registry}) == 6

    def _update_joint_availability(self) -> None:
        active = {e["joint_name"] for e in self.registry}
        if hasattr(self, "joint_targets"):
            for i, name in enumerate(JOINTS):
                if i < len(self.joint_targets):
                    self.joint_targets[i].setEnabled(name in active)
                item = self.tbl_joint.item(i, 0)
                if item is not None:
                    suffix = "" if name in active else "（未注册）"
                    item.setText(f"J{i + 1} {name}{suffix}")
        mask = np.array([name in active for name in JOINTS], dtype=bool)
        if hasattr(self, "preview"):
            try:
                self.preview.set_active_mask(mask)
            except Exception:
                pass
        if hasattr(self, "plot_panel"):
            try:
                self.plot_panel.set_active_mask(mask)
            except Exception:
                pass

    def _build_top_bar(self):
        box = QGroupBox("运行状态")
        lay = QHBoxLayout(box)
        self.btn_enable = QPushButton("全部使能")
        self.btn_enable.setObjectName("good")
        self.btn_send = QPushButton("全部开始发送")
        self.btn_send.setObjectName("primary")
        self.btn_disable_all = QPushButton("全部失能")
        self.btn_stop_all = QPushButton("全部停止发送")
        self.btn_home = QPushButton("回 fold")
        self.btn_home.setToolTip("6 轴全部注册、使能并开始发送后，平滑回到各关节 0 点")
        self.btn_home.setEnabled(False)
        self.btn_release = QPushButton("急停 / 失能")
        self.btn_release.setObjectName("danger")
        self.btn_enable.clicked.connect(self.on_enable_all)
        self.btn_send.clicked.connect(self.on_send_all)
        self.btn_disable_all.clicked.connect(self.on_disable_all)
        self.btn_stop_all.clicked.connect(self.on_stop_all)
        self.btn_home.clicked.connect(self.on_home)
        self.btn_release.clicked.connect(self.on_release)
        for b in (
            self.btn_enable, self.btn_send, self.btn_disable_all,
            self.btn_stop_all, self.btn_home, self.btn_release,
        ):
            lay.addWidget(b)

        lay.addSpacing(18)
        self.lbl_topic = QLabel("话题: —")
        self.lbl_motor = QLabel("电机反馈: —")
        self.lbl_state = QLabel("未使能 | 未发送")
        self.lbl_motion = QLabel("运动: 保持")
        for lab in (self.lbl_topic, self.lbl_motor, self.lbl_state, self.lbl_motion):
            lay.addWidget(lab)

        lay.addSpacing(18)
        self.btn_save_settings = QPushButton("保存参数")
        self.btn_save_settings.setToolTip(
            "把 6 轴 kp/kd、单轴 MIT kp/kd、重力/摩擦前馈、夹爪参数保存到\n"
            f"{hmi_store.BASE_DIR / hmi_store.SETTINGS_NAME}\n"
            "下次启动自动载入；夹爪驱动连接时会自动写回。"
        )
        self.btn_save_settings.clicked.connect(self.on_save_settings)
        lay.addWidget(self.btn_save_settings)

        lay.addSpacing(12)
        self.chk_gamepad = QCheckBox("手柄控制")
        self.chk_gamepad.setChecked(False)
        self.chk_gamepad.setToolTip(
            "按住 LB 才点动。X 记录当前关节为轨迹点，Y 执行轨迹，B 平滑停止。"
            "左摇杆 x/y，LT/RT z，右摇杆 yaw/pitch。"
        )
        self.lbl_gamepad = QLabel("手柄: —")
        self.chk_keyboard = QCheckBox("键盘控制")
        self.chk_keyboard.setChecked(False)
        self.chk_keyboard.setToolTip(
            "←→ 沿 X，↑↓ 沿 Y，空格上升，Shift 下降。"
            "需已使能并开始控制。速度 0.10 m/s。"
        )
        self.lbl_keyboard = QLabel("键盘: 关")
        lay.addWidget(self.chk_gamepad)
        lay.addWidget(self.lbl_gamepad)
        lay.addWidget(self.chk_keyboard)
        lay.addWidget(self.lbl_keyboard)
        lay.addStretch(1)
        return box

    def _build_tabs(self):
        self.tabs = QTabWidget()
        self.tabs.setMinimumWidth(470)
        self.tabs.setMaximumWidth(820)
        self.tabs.addTab(self._build_basic_tab(), "单轴")
        self.tabs.addTab(self._build_joint_tab(), "关节")
        self.tabs.addTab(self._build_waypoint_tab(), "轨迹")
        self.tabs.addTab(self._build_ik_tab(), "逆解")
        self.tabs.addTab(self._build_ident_tab(), "辨识")
        self.tabs.addTab(self._build_gripper_tab(), "夹爪")
        return self.tabs

    def _on_tab_changed(self, idx: int):
        # Splitter sizes are user-controlled and persisted; do not override.
        return

    # ---- 单轴页
    def _collapsible_group(self, title: str, expanded: bool = False):
        box = QGroupBox(title)
        box.setCheckable(True)
        box.setChecked(expanded)
        inner = QWidget()
        lay = QVBoxLayout(inner)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        outer = QVBoxLayout(box)
        outer.setContentsMargins(6, 6, 6, 6)
        outer.addWidget(inner)
        inner.setVisible(expanded)
        box.toggled.connect(inner.setVisible)
        return box, lay

    def _build_mit_group(self):
        box = QGroupBox("单轴 MIT（每轴独立）")
        lay = QGridLayout(box)
        self.cb_mit_motor = QComboBox()
        self.cb_mit_motor.currentIndexChanged.connect(self._sync_mit_selection)
        self.lbl_mit_state = QLabel("—")
        self.lbl_mit_state.setWordWrap(True)
        self.sp_mit_kp = self._dspin(0.0, 500.0, 5.0, 0.5, 2)
        self.sp_mit_kd = self._dspin(0.0, 20.0, 0.8, 0.1, 2)
        self.sp_mit_q = self._dspin(-12.5, 12.5, 0.0, 0.01, 4)
        self.sp_mit_dq = self._dspin(-30.0, 30.0, 0.0, 0.1, 3)
        self.sp_mit_tau = self._dspin(-30.0, 30.0, 0.0, 0.1, 3)

        lay.addWidget(QLabel("电机"), 0, 0)
        lay.addWidget(self.cb_mit_motor, 0, 1, 1, 3)
        lay.addWidget(self.lbl_mit_state, 0, 4, 1, 6)
        for col, (text, spin) in enumerate(
            (
                ("kp", self.sp_mit_kp),
                ("kd", self.sp_mit_kd),
                ("q", self.sp_mit_q),
                ("dq", self.sp_mit_dq),
                ("tau", self.sp_mit_tau),
            )
        ):
            lay.addWidget(QLabel(text), 1, col * 2)
            lay.addWidget(spin, 1, col * 2 + 1)

        self.btn_mit_enable = QPushButton("使能本轴")
        self.btn_mit_disable = QPushButton("失能本轴")
        self.btn_mit_send = QPushButton("发送数据")
        self.btn_mit_send.setCheckable(True)
        self.btn_mit_stop = QPushButton("停止发送")
        self.btn_mit_apply = QPushButton("发送 MIT")
        self.btn_mit_hold = QPushButton("保持")
        self.btn_mit_zero = QPushButton("零力矩")
        self.chk_mit_live = QCheckBox("实时下发 50 Hz")
        self.btn_mit_enable.clicked.connect(self.on_mit_enable)
        self.btn_mit_disable.clicked.connect(self.on_mit_disable)
        self.btn_mit_send.clicked.connect(self.on_mit_send_toggle)
        self.btn_mit_stop.clicked.connect(self.on_mit_stop)
        self.btn_mit_apply.clicked.connect(self.on_mit_apply)
        self.btn_mit_hold.clicked.connect(self.on_mit_hold)
        self.btn_mit_zero.clicked.connect(self.on_mit_zero)
        row = QHBoxLayout()
        for b in (
            self.btn_mit_enable, self.btn_mit_disable, self.btn_mit_send,
            self.btn_mit_stop, self.btn_mit_apply, self.btn_mit_hold,
            self.btn_mit_zero, self.chk_mit_live,
        ):
            row.addWidget(b)
        row.addStretch(1)
        lay.addLayout(row, 2, 0, 1, 10)
        hint = QLabel(
            "顺序：使能本轴 → 发送数据 → 发送 MIT。"
            "未发送时电机不出力也不回反馈；零力矩会让该轴自由拖动。"
        )
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        lay.addWidget(hint, 3, 0, 1, 10)
        self._refresh_mit_motor_combo()
        return box

    def _build_basic_tab(self):
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(8, 8, 8, 8)
        v.setSpacing(8)

        # 共享的「电机」下拉在单轴 MIT 组内，模式/参数都针对它
        v.addWidget(self._build_mit_group())

        settings, sl = self._collapsible_group(
            "电机设置（高级，点击展开）", expanded=False
        )
        mode = QGroupBox("控制模式（目标轴需失能且停止发送）")
        ml = QHBoxLayout(mode)
        self.cb_mode = QComboBox()
        self.cb_mode.addItems(MODES)
        self.btn_mode = QPushButton("应用模式")
        self.btn_mode.clicked.connect(self.on_apply_mode)
        self.lbl_mode_now = QLabel("当前: —")
        ml.addWidget(QLabel("对象: 上方『电机』"))
        ml.addWidget(self.cb_mode)
        ml.addWidget(self.btn_mode)
        ml.addWidget(self.lbl_mode_now)
        ml.addStretch(1)
        sl.addWidget(mode)

        param = QGroupBox("电机内部参数（RID）")
        pl = QGridLayout(param)
        self.cb_rid = QComboBox()
        self.cb_rid.setSizeAdjustPolicy(
            QComboBox.AdjustToMinimumContentsLengthWithIcon
        )
        self.cb_rid.setMinimumContentsLength(12)
        for label, rid, is_int in PARAM_REGS:
            self.cb_rid.addItem(f"{label} (RID {rid})", (rid, is_int))
        self.btn_param_read = QPushButton("读取")
        self.btn_param_write = QPushButton("写入")
        self.btn_param_save = QPushButton("保存到 Flash")
        self.btn_param_read.clicked.connect(self.on_param_read)
        self.btn_param_write.clicked.connect(self.on_param_write)
        self.btn_param_save.clicked.connect(self.on_param_save)
        self.lbl_param_result = QLineEdit("—")
        self.lbl_param_result.setReadOnly(True)
        self.ed_param_value = QLineEdit()
        self.ed_param_value.setPlaceholderText("写入值")
        self.ed_param_value.setMinimumWidth(80)
        pl.setColumnStretch(1, 1)
        pl.setColumnStretch(3, 1)
        pl.addWidget(QLabel("对象"), 0, 0)
        pl.addWidget(QLabel("上方『电机』下拉框"), 0, 1, 1, 2)
        pl.addWidget(QLabel("寄存器"), 1, 0)
        pl.addWidget(self.cb_rid, 1, 1, 1, 2)
        pl.addWidget(self.btn_param_read, 1, 3)
        pl.addWidget(QLabel("读回"), 2, 0)
        pl.addWidget(self.lbl_param_result, 2, 1, 1, 2)
        pl.addWidget(self.ed_param_value, 2, 3)
        pl.addWidget(self.btn_param_write, 2, 4)
        pl.addWidget(self.btn_param_save, 3, 4)
        hint = QLabel(
            "RID 10（控制模式）请用上面的「应用模式」；"
            "写入后需保存 Flash 才会掉电保留。"
        )
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        pl.addWidget(hint, 4, 0, 1, 4)
        sl.addWidget(param)
        v.addWidget(settings)
        settings.setChecked(False)

        traj, tl = self._collapsible_group(
            "6 轴轨迹（高级，仅全 6 轴；点击展开）", expanded=False
        )
        gain = QGroupBox("增益 / 限速（仅 6 轴轨迹，不影响单轴 MIT）")
        gl = QHBoxLayout(gain)
        self.sp_kp = self._dspin(0.0, 100.0, 5.0, 0.5, 2)
        self.sp_kd = self._dspin(0.0, 20.0, 0.8, 0.1, 2)
        self.sp_speed = self._dspin(0.05, 3.0, 0.4, 0.05, 2)
        gl.addWidget(QLabel("kp"))
        gl.addWidget(self.sp_kp)
        gl.addWidget(QLabel("kd"))
        gl.addWidget(self.sp_kd)
        gl.addWidget(QLabel("速度上限 rad/s"))
        gl.addWidget(self.sp_speed)
        gl.addStretch(1)
        self.sp_kp.valueChanged.connect(self._sync_gains)
        self.sp_kd.valueChanged.connect(self._sync_gains)
        tl.addWidget(gain)

        ff = QGroupBox("重力 / 摩擦前馈（仅 6 轴轨迹，不影响单轴 MIT）")
        self.grp_ff = ff
        fg = QGridLayout(ff)
        self.chk_gravity = QCheckBox("重力补偿")
        self.chk_gravity.setChecked(False)
        self.chk_gravity.setToolTip(
            "给 6 轴轨迹的 MIT 力矩加重力前馈。零点未标定前不要打开。"
        )
        self.sp_gravity_scale = self._dspin(0.0, 1.5, 1.0, 0.05, 2)
        self.chk_friction = QCheckBox("摩擦补偿")
        self.chk_friction.setChecked(False)
        self.sp_coulomb = self._dspin(0.0, 1.0, 0.10, 0.01, 2)
        self.sp_viscous = self._dspin(0.0, 1.0, 0.05, 0.01, 2)
        self.sp_fslope = self._dspin(10.0, 500.0, 100.0, 10.0, 0)
        self.sp_fslope.setToolTip(
            "tanh 斜率（1/(rad/s)）：越大越接近硬库仑摩擦；"
            "100 约对应 ±0.01 rad/s 的过渡带。"
        )
        self.lbl_ff = QLabel("前馈: —")
        self.lbl_ff.setWordWrap(True)
        fg.addWidget(self.chk_gravity, 0, 0)
        fg.addWidget(QLabel("scale"), 0, 1)
        fg.addWidget(self.sp_gravity_scale, 0, 2)
        fg.addWidget(self.chk_friction, 0, 3)
        fg.addWidget(QLabel("库仑"), 1, 0)
        fg.addWidget(self.sp_coulomb, 1, 1)
        fg.addWidget(QLabel("粘性"), 1, 2)
        fg.addWidget(self.sp_viscous, 1, 3)
        fg.addWidget(QLabel("slope"), 1, 4)
        fg.addWidget(self.sp_fslope, 1, 5)
        fg.addWidget(self.lbl_ff, 2, 0, 1, 4)
        self.btn_drag = QPushButton("拖动示教")
        self.btn_drag.setCheckable(True)
        self.btn_drag.setToolTip(
            "6 轴轨迹：期望角度、速度和重力前馈锁到当前关节状态。"
        )
        self.btn_drag.clicked.connect(self.on_drag_teach)
        fg.addWidget(self.btn_drag, 2, 4, 1, 2)
        hint = QLabel(
            "仅 6 轴轨迹使用。重力默认关闭；零位/方向未标定前不要打开，"
            "也不要回 fold 或用逆解。摩擦默认关闭。"
        )
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        fg.addWidget(hint, 3, 0, 1, 6)
        for ctrl in (self.chk_gravity, self.chk_friction):
            ctrl.toggled.connect(self._sync_ff)
        for sp in (
            self.sp_gravity_scale, self.sp_coulomb,
            self.sp_viscous, self.sp_fslope,
        ):
            sp.valueChanged.connect(self._sync_ff)
        tl.addWidget(ff)
        v.addWidget(traj)
        traj.setChecked(False)
        self._sync_ff()
        v.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(w)
        return scroll

    # ---- 关节页
    def _build_joint_tab(self):
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(8, 8, 8, 8)

        self.tbl_joint = QTableWidget(self.n, 5)
        self.tbl_joint.setHorizontalHeaderLabels(
            ["关节", "当前 q", "目标 q", "模式", "ERR"]
        )
        self.tbl_joint.verticalHeader().setVisible(False)
        self.tbl_joint.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tbl_joint.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.joint_targets = []
        for i, name in enumerate(JOINTS):
            self.tbl_joint.setItem(i, 0, QTableWidgetItem(f"J{i + 1} {name}"))
            self.tbl_joint.setItem(i, 1, QTableWidgetItem("—"))
            sp = self._dspin(-3.2, 3.2, 0.0, 0.01, 4)
            self.tbl_joint.setCellWidget(i, 2, sp)
            self.joint_targets.append(sp)
            self.tbl_joint.setItem(i, 3, QTableWidgetItem("—"))
            self.tbl_joint.setItem(i, 4, QTableWidgetItem("—"))
        v.addWidget(self.tbl_joint, 1)

        row = QHBoxLayout()
        b_read = QPushButton("读取当前到目标")
        b_go = QPushButton("运动到目标")
        b_stop = QPushButton("平滑停止")
        b_read.clicked.connect(self.on_joint_read)
        b_go.clicked.connect(self.on_joint_go)
        b_stop.clicked.connect(self.bridge.stop_motion)
        row.addWidget(b_read)
        row.addWidget(b_go)
        row.addWidget(b_stop)
        row.addStretch(1)
        self.lbl_tcp = QLabel("TCP: —")
        row.addWidget(self.lbl_tcp)
        v.addLayout(row)

        lim = QGroupBox("关节软件限位（URDF，度）")
        lg = QGridLayout(lim)
        self.sp_lim_lo = []
        self.sp_lim_hi = []
        for i in range(self.n):
            row_i, block = divmod(i, 2)
            lo = self._dspin(-360.0, 360.0, 0.0, 1.0, 1)
            hi = self._dspin(-360.0, 360.0, 0.0, 1.0, 1)
            for sp in (lo, hi):
                sp.setMaximumWidth(86)
            lo.setValue(float(np.degrees(self.joint_limits[i, 0])))
            hi.setValue(float(np.degrees(self.joint_limits[i, 1])))
            self.sp_lim_lo.append(lo)
            self.sp_lim_hi.append(hi)
            base = block * 4
            lg.addWidget(QLabel(f"J{i + 1}"), row_i, base)
            lg.addWidget(lo, row_i, base + 1)
            lg.addWidget(QLabel("~"), row_i, base + 2, Qt.AlignCenter)
            lg.addWidget(hi, row_i, base + 3)
        lrow = QHBoxLayout()
        b_model = QPushButton("读模型默认")
        b_apply = QPushButton("应用限位")
        b_model.clicked.connect(self.on_limits_model)
        b_apply.clicked.connect(self.on_limits_apply)
        lrow.addWidget(b_model)
        lrow.addWidget(b_apply)
        self.lbl_limits = QLabel("限位: —")
        self.lbl_limits.setObjectName("hint")
        self.lbl_limits.setWordWrap(True)
        lrow.addWidget(self.lbl_limits, 1)
        lg.addLayout(lrow, 3, 0, 1, 8)
        hint = QLabel(
            "软件限位只能收窄，不能超过模型/机械限位。单轴 MIT、关节目标、"
            "轨迹、逆解、手柄/键盘和辨识都会裁剪到该范围。"
        )
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        lg.addWidget(hint, 4, 0, 1, 8)
        v.addWidget(lim)
        self._sync_limit_spins()

        note = QLabel("按关节空间余弦插值运动；只按软件限位裁剪，不做碰撞检测。")
        note.setObjectName("warn")
        v.addWidget(note)
        return w

    # ---- 轨迹页
    def _build_waypoint_tab(self):
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(6, 6, 6, 6)
        note = QLabel(
            "3D 显示的是实机位姿快照，点「同步实机姿态」刷新。左键拖中心球 = 平移末端；"
            "拖红/绿/蓝圆环 = 绕对应轴旋转（红 X、绿 Y、蓝 Z）；左键拖空白或右键拖 = 转视角，"
            "中键拖 = 平移视角，滚轮 = 缩放；也可直接改 R/P/Y。姿态残差大时先把末端移离折叠位。"
            "勾选「实时跟随」后，失能手拖实机会让 3D 以 5 Hz 跟随。"
            "使能后的回 fold、关节运动和轨迹会让 3D 沿同一条平滑指令一起动。"
            "执行只做关节空间插值，不检查碰撞。"
        )
        note.setObjectName("warn")
        note.setWordWrap(True)
        v.addWidget(note)

        top = QHBoxLayout()
        self.lbl_preview = QLabel("实机快照")
        self.lbl_preview.setObjectName("hint")
        self.cb_camera = QComboBox()
        self.cb_camera.addItems(["overview", "front", "side", "top"])
        b_sync = QPushButton("同步实机姿态")
        b_sync.clicked.connect(self.on_preview_sync)
        b_reset_cam = QPushButton("相机复位")
        b_reset_cam.clicked.connect(self.on_reset_camera)
        top.addWidget(QLabel("3D"))
        top.addWidget(self.lbl_preview, 1)
        top.addWidget(QLabel("视角"))
        top.addWidget(self.cb_camera)
        top.addWidget(b_reset_cam)
        top.addWidget(b_sync)
        v.addLayout(top)

        pose = QHBoxLayout()
        self.sp_roll = self._dspin(-180.0, 180.0, 0.0, 1.0, 1)
        self.sp_pitch = self._dspin(-180.0, 180.0, 0.0, 1.0, 1)
        self.sp_yaw = self._dspin(-180.0, 180.0, 0.0, 1.0, 1)
        for sp in (self.sp_roll, self.sp_pitch, self.sp_yaw):
            sp.setMaximumWidth(78)
            sp.valueChanged.connect(self._on_pose_spins)
        pose.addWidget(QLabel("姿态 R/P/Y (°)"))
        pose.addWidget(self.sp_roll)
        pose.addWidget(self.sp_pitch)
        pose.addWidget(self.sp_yaw)
        self.chk_follow = QCheckBox("实时跟随（失能手拖）")
        self.chk_follow.setChecked(True)
        self.chk_follow.setToolTip(
            "失能手拖实机时，3D 以 5 Hz 跟随；使能+开始控制后自动暂停。"
        )
        pose.addWidget(self.chk_follow)
        pose.addStretch(1)
        v.addLayout(pose)

        self.preview = Preview3D(self.kin, self)
        self.preview.status.connect(self._set_preview_status)
        self.preview.poseChanged.connect(self._on_preview_pose)
        self.cb_camera.currentTextChanged.connect(self.preview.set_camera)
        v.addWidget(self.preview, 1)

        self.tbl_wp = QTableWidget(0, 7)
        self.tbl_wp.setHorizontalHeaderLabels(
            ["#"] + [f"J{i + 1}" for i in range(self.n)]
        )
        self.tbl_wp.verticalHeader().setVisible(False)
        self.tbl_wp.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tbl_wp.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tbl_wp.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tbl_wp.setMaximumHeight(130)
        v.addWidget(self.tbl_wp)

        row = QHBoxLayout()
        b_rec = QPushButton("记录当前点")
        b_del = QPushButton("删除选中")
        b_clear = QPushButton("清空")
        b_run = QPushButton("执行轨迹")
        b_stop = QPushButton("平滑停止")
        b_rec.clicked.connect(self.on_wp_record)
        b_del.clicked.connect(self.on_wp_delete)
        b_clear.clicked.connect(self.on_wp_clear)
        b_run.clicked.connect(lambda: self.on_wp_run(True))
        b_stop.clicked.connect(self.bridge.stop_motion)
        for b in (b_rec, b_del, b_clear, b_run, b_stop):
            row.addWidget(b)
        self.chk_dwell = QCheckBox("到点停顿")
        self.chk_dwell.setChecked(False)
        self.chk_dwell.setToolTip("不勾选时连续经过各点。勾选后，到达每个轨迹点先停住再走下一段。")
        self.sp_dwell = self._dspin(0.0, 30.0, 1.0, 0.1, 1)
        self.sp_dwell.setSuffix(" s")
        self.sp_dwell.setMaximumWidth(72)
        row.addWidget(self.chk_dwell)
        row.addWidget(self.sp_dwell)
        row.addStretch(1)
        v.addLayout(row)

        grp_rec = QGroupBox("拖动录制 / 轨迹库 / 复现（MIT 或 POS_VEL）")
        g = QGridLayout(grp_rec)
        self.btn_rec_start = QPushButton("开始拖动录制")
        self.btn_rec_stop = QPushButton("停止记录")
        self.btn_replay = QPushButton("轨迹复现")
        self.btn_replay_stop = QPushButton("停止复现")
        self.btn_rec_clear = QPushButton("清除录制")
        self.lbl_rec = QLabel("未录制")
        self.lbl_rec.setObjectName("hint")
        self.btn_rec_start.setToolTip(
            "进入拖动示教并开始 50 Hz 记录。需要先使能、开始发送、开启重力前馈。"
        )
        self.btn_rec_stop.setToolTip(
            "退出拖动示教并保持当前位姿，保存本次录制。"
        )
        self.btn_replay.setToolTip(
            "先运动到录制起点，再按录制时间回放整段轨迹。"
        )
        self.btn_replay_stop.setToolTip("停止回放并平滑保持。")
        self.btn_rec_clear.setToolTip("清除内存中的录制轨迹。")
        self.btn_rec_start.clicked.connect(self.on_rec_start)
        self.btn_rec_stop.clicked.connect(self.on_rec_stop)
        self.btn_replay.clicked.connect(self.on_replay_start)
        self.btn_replay_stop.clicked.connect(self.on_replay_stop)
        self.btn_rec_clear.clicked.connect(self.on_rec_clear)
        g.addWidget(self.btn_rec_start, 0, 0)
        g.addWidget(self.btn_rec_stop, 0, 1)
        g.addWidget(self.btn_replay, 0, 2)
        g.addWidget(self.btn_replay_stop, 0, 3)
        g.addWidget(self.btn_rec_clear, 0, 4)

        # ---- 轨迹库：录完自动存盘，下次启动可选 ----
        self._traj_list_guard = False
        self.cb_traj = QComboBox()
        self.cb_traj.setMinimumWidth(210)
        self.cb_traj.setToolTip(
            "已保存的轨迹（~/.config/zfy_arm/trajectories）。"
            "选中即载入内存，再点「轨迹复现」执行。"
        )
        self.cb_traj.currentIndexChanged.connect(self._on_traj_selected)
        self.btn_traj_refresh = QPushButton("刷新")
        self.btn_traj_save = QPushButton("保存为…")
        self.btn_traj_delete = QPushButton("删除")
        # 注意：clicked 会带一个 bool，不能直接连到带默认参数的槽
        self.btn_traj_refresh.clicked.connect(
            lambda _checked=False: self._refresh_traj_list()
        )
        self.btn_traj_save.clicked.connect(self.on_traj_save_as)
        self.btn_traj_delete.clicked.connect(self.on_traj_delete)
        g.addWidget(QLabel("已保存轨迹"), 1, 0)
        g.addWidget(self.cb_traj, 1, 1, 1, 3)
        g.addWidget(self.btn_traj_refresh, 1, 4)
        g.addWidget(self.btn_traj_save, 1, 5)
        g.addWidget(self.btn_traj_delete, 1, 6)

        # ---- 回放模式：MIT 或 POS_VEL ----
        self.cb_replay_mode = QComboBox()
        self.cb_replay_mode.addItem("MIT（力矩/位置伺服）", "MIT")
        self.cb_replay_mode.addItem("位置速度（POS_VEL）", "POS_VEL")
        self.cb_replay_mode.setToolTip(
            "MIT：和原来一样，按 kp/kd 做伺服，需要重力前馈。\n"
            "POS_VEL：向驱动器下发位置+速度，需要 6 轴先切到 POS_VEL 模式。"
        )
        self.btn_mode_switch = QPushButton("切换 6 轴模式")
        self.btn_mode_switch.setToolTip(
            "把全部 6 轴切到上面选定的模式。\n"
            "会先停止发送、失能、切模式，再自动使能并开始发送。"
        )
        self.btn_mode_switch.clicked.connect(self.on_switch_all_modes)
        self.chk_traj_autosave = QCheckBox("停止录制后自动保存")
        self.chk_traj_autosave.setChecked(True)
        g.addWidget(QLabel("回放模式"), 2, 0)
        g.addWidget(self.cb_replay_mode, 2, 1)
        g.addWidget(self.btn_mode_switch, 2, 2, 1, 2)
        g.addWidget(self.chk_traj_autosave, 2, 4, 1, 3)

        g.addWidget(self.lbl_rec, 3, 0, 1, 7)
        v.addWidget(grp_rec)
        self._refresh_traj_list()
        self._update_rec_buttons()
        return w

    def _set_preview_status(self, text: str):
        self.lbl_preview.setText(text)

    def _on_preview_pose(self, roll: float, pitch: float, yaw: float):
        self._pose_spin_guard = True
        try:
            self.sp_roll.setValue(roll)
            self.sp_pitch.setValue(pitch)
            self.sp_yaw.setValue(yaw)
        finally:
            self._pose_spin_guard = False

    def _on_pose_spins(self, _value=0.0):
        if self._pose_spin_guard:
            return
        self.preview.set_target_rpy(
            self.sp_roll.value(), self.sp_pitch.value(), self.sp_yaw.value()
        )

    def on_preview_sync(self):
        self.preview.reset_to_real(self.bridge.current_q())

    def on_reset_camera(self):
        self.preview.set_camera("overview")
        self.cb_camera.blockSignals(True)
        self.cb_camera.setCurrentText("overview")
        self.cb_camera.blockSignals(False)

    # ---- 逆解页
    def _build_ik_tab(self):
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(8, 8, 8, 8)
        self.lbl_ik_cur = QLabel("当前 TCP: —")
        v.addWidget(self.lbl_ik_cur)

        form = QFormLayout()
        self.sp_ik_x = self._dspin(-2.0, 2.0, -0.27, 0.01, 4)
        self.sp_ik_y = self._dspin(-2.0, 2.0, 0.0, 0.01, 4)
        self.sp_ik_z = self._dspin(0.0, 2.0, 0.16, 0.01, 4)
        form.addRow("目标 x/m", self.sp_ik_x)
        form.addRow("目标 y/m", self.sp_ik_y)
        form.addRow("目标 z/m", self.sp_ik_z)
        v.addLayout(form)

        row = QHBoxLayout()
        b_here = QPushButton("读取当前末端")
        b_solve = QPushButton("仅求解")
        b_run = QPushButton("求解并运动")
        b_here.clicked.connect(self.on_ik_read_current)
        b_solve.clicked.connect(self.on_ik_solve)
        b_run.clicked.connect(self.on_ik_run)
        for b in (b_here, b_solve, b_run):
            row.addWidget(b)
        row.addStretch(1)
        v.addLayout(row)

        self.lbl_ik_result = QLabel("解: —")
        self.lbl_ik_result.setWordWrap(True)
        v.addWidget(self.lbl_ik_result)
        note = QLabel(
            "位置逆解（TCP 坐标，不含姿态），用当前关节角作种子；"
            "解可能换到另一组肘部构型，且不做碰撞检测。误差 > 10 mm 时不要直接执行。"
        )
        note.setObjectName("warn")
        note.setWordWrap(True)
        v.addWidget(note)
        v.addStretch(1)
        return w

    # ---- 辨识页
    def _build_ident_tab(self):
        inner = QWidget()
        v = QVBoxLayout(inner)
        v.setContentsMargins(6, 6, 6, 6)
        v.setSpacing(6)

        note = QLabel(
            "辨识会让机械臂按激励轨迹自动运动。先标定零位/方向，确认周围无障碍、"
            "急停可触达。流程：驱动开始记录 CSV -> ident_excite 发激励 -> "
            "停止记录 -> identify_dynamics.py 用 MuJoCo 重力模型拟合。"
            "只拟合重力缩放/偏置与库仑-粘性摩擦，不拟合完整惯性参数。"
        )
        note.setObjectName("warn")
        note.setWordWrap(True)
        v.addWidget(note)

        exc = QGroupBox("激励设置")
        form = QGridLayout(exc)
        self.cb_ident_mode = QComboBox()
        for label, value in (
            ("静态位姿（重力 / 偏置）", "static"),
            ("单关节扫掠（摩擦）", "sweep"),
            ("多正弦（动态）", "fourier"),
        ):
            self.cb_ident_mode.addItem(label, value)
        self.sp_ident_freq = self._dspin(0.02, 2.0, 0.30, 0.05, 2)
        self.sp_ident_dur = self._dspin(1.0, 120.0, 20.0, 1.0, 1)
        self.sp_ident_dwell = self._dspin(0.0, 10.0, 1.0, 0.5, 1)
        self.sp_ident_harm = QSpinBox()
        self.sp_ident_harm.setRange(1, 10)
        self.sp_ident_harm.setValue(3)
        self.chk_ident_return = QCheckBox("结束回原位")
        self.chk_ident_return.setChecked(True)
        self.chk_ident_auto = QCheckBox("自动分析")
        self.chk_ident_auto.setChecked(True)
        form.addWidget(QLabel("模式"), 0, 0)
        form.addWidget(self.cb_ident_mode, 0, 1, 1, 3)
        form.addWidget(QLabel("频率 Hz"), 1, 0)
        form.addWidget(self.sp_ident_freq, 1, 1)
        form.addWidget(QLabel("时长 s"), 1, 2)
        form.addWidget(self.sp_ident_dur, 1, 3)
        form.addWidget(QLabel("保持 s"), 2, 0)
        form.addWidget(self.sp_ident_dwell, 2, 1)
        form.addWidget(QLabel("谐波"), 2, 2)
        form.addWidget(self.sp_ident_harm, 2, 3)
        form.addWidget(self.chk_ident_return, 3, 0, 1, 2)
        form.addWidget(self.chk_ident_auto, 3, 2, 1, 2)
        v.addWidget(exc)

        amp_box = QGroupBox("每关节幅值 rad（实机首轮建议 ≤ 0.10）")
        ag = QGridLayout(amp_box)
        self.sp_ident_amp = []
        for i in range(self.n):
            sp = self._dspin(0.01, 0.60, 0.08, 0.01, 2)
            sp.setMaximumWidth(90)
            self.sp_ident_amp.append(sp)
            ag.addWidget(QLabel(f"J{i + 1}"), 0, i)
            ag.addWidget(sp, 1, i)
        v.addWidget(amp_box)

        safe = QGroupBox("安全与运行")
        sl = QVBoxLayout(safe)
        self.chk_ident_safe = QCheckBox(
            "已确认：机械臂有支撑/低风险姿态、周围无障碍、急停可触达"
        )
        self.chk_ident_safe.setChecked(False)
        sl.addWidget(self.chk_ident_safe)
        row = QHBoxLayout()
        self.btn_ident_start = QPushButton("开始辨识")
        self.btn_ident_start.setObjectName("good")
        self.btn_ident_stop = QPushButton("停止辨识")
        self.btn_ident_stop.setObjectName("danger")
        self.btn_ident_stop.setEnabled(False)
        self.btn_ident_start.clicked.connect(self.on_ident_start)
        self.btn_ident_stop.clicked.connect(self.on_ident_stop)
        self.chk_ident_safe.toggled.connect(self._update_ident_buttons)
        row.addWidget(self.btn_ident_start)
        row.addWidget(self.btn_ident_stop)
        row.addStretch(1)
        sl.addLayout(row)
        self.lbl_ident = QLabel("辨识: 空闲")
        self.lbl_ident.setObjectName("hint")
        self.lbl_ident.setWordWrap(True)
        sl.addWidget(self.lbl_ident)
        self.lbl_ident_csv = QLabel("CSV: —")
        self.lbl_ident_csv.setObjectName("hint")
        self.lbl_ident_csv.setWordWrap(True)
        sl.addWidget(self.lbl_ident_csv)
        v.addWidget(safe)

        res = QGroupBox("拟合结果（URDF 帧）")
        rl = QVBoxLayout(res)
        self.lbl_ident_global = QLabel("全局重力缩放: —")
        self.lbl_ident_global.setWordWrap(True)
        rl.addWidget(self.lbl_ident_global)
        self.tbl_ident = QTableWidget(self.n, 5)
        self.tbl_ident.setHorizontalHeaderLabels(
            ["关节", "重力缩放", "偏置 N·m", "粘性", "库仑"]
        )
        self.tbl_ident.verticalHeader().setVisible(False)
        self.tbl_ident.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tbl_ident.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch
        )
        self.tbl_ident.setMaximumHeight(190)
        rl.addWidget(self.tbl_ident)
        arow = QHBoxLayout()
        self.btn_ident_apply = QPushButton("填入前馈（全局）")
        self.btn_ident_apply.setToolTip(
            "把全局重力缩放和平均摩擦填入「关节」页的前馈控件；"
            "不会自动勾选补偿。"
        )
        self.btn_ident_param = QPushButton("写入驱动重力增益")
        self.btn_ident_param.setToolTip(
            "把每关节重力缩放/偏置写到 /motor_driver 的 gravity_gains / "
            "gravity_biases 参数；同时把全局 scale 置 1，避免重复相乘。"
        )
        self.btn_ident_apply.setEnabled(False)
        self.btn_ident_param.setEnabled(False)
        self.btn_ident_apply.clicked.connect(self.on_ident_apply)
        self.btn_ident_param.clicked.connect(self.on_ident_write_params)
        arow.addWidget(self.btn_ident_apply)
        arow.addWidget(self.btn_ident_param)
        arow.addStretch(1)
        rl.addLayout(arow)
        hint = QLabel(
            "摩擦值目前只能以全局标量下发：填入前馈时取各关节平均。"
            "逐关节摩擦需要驱动侧把 _tau_ff 的 coulomb/viscous 改成 6 元数组。"
        )
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        rl.addWidget(hint)
        v.addWidget(res)

        self.ident_log_view = QPlainTextEdit()
        self.ident_log_view.setReadOnly(True)
        self.ident_log_view.setMaximumBlockCount(400)
        self.ident_log_view.setMaximumHeight(150)
        v.addWidget(self.ident_log_view)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(inner)
        return scroll

    # ---- 夹爪页
    def _build_gripper_tab(self):
        inner = QWidget()
        v = QVBoxLayout(inner)
        v.setContentsMargins(6, 6, 6, 6)
        v.setSpacing(6)

        note = QLabel(
            "夹爪由独立的 /gripper_driver 节点控制（MIT）。先按实物改 "
            "config/gripper.yaml 里的 q_min/q_max、open_q、close_q、"
            "close_direction 和 CAN ID/型号，再启动驱动。"
            "自适应抓取：低刚度缓慢闭合 -> 检测接触 -> 记录 hold_q -> "
            "自动切高刚度保持并保存。"
        )
        note.setObjectName("warn")
        note.setWordWrap(True)
        v.addWidget(note)

        drv = QGroupBox("夹爪驱动")
        dl = QVBoxLayout(drv)
        drow = QHBoxLayout()
        self.lbl_grip_driver = QLabel("驱动: 未连接")
        self.lbl_grip_driver.setObjectName("hint")
        self.btn_grip_start = QPushButton("启动夹爪驱动")
        self.btn_grip_stop = QPushButton("停止夹爪驱动")
        self.btn_grip_start.clicked.connect(self.on_grip_start_driver)
        self.btn_grip_stop.clicked.connect(self.on_grip_stop_driver)
        drow.addWidget(self.lbl_grip_driver, 1)
        drow.addWidget(self.btn_grip_start)
        drow.addWidget(self.btn_grip_stop)
        dl.addLayout(drow)
        self.lbl_grip_state = QLabel("状态: —")
        self.lbl_grip_state.setWordWrap(True)
        dl.addWidget(self.lbl_grip_state)
        v.addWidget(drv)

        run = QGroupBox("夹爪控制")
        rg = QGridLayout(run)
        self.btn_grip_enable = QPushButton("使能")
        self.btn_grip_disable = QPushButton("失能")
        self.btn_grip_send = QPushButton("开始发送")
        self.btn_grip_send_stop = QPushButton("停止发送")
        self.btn_grip_open = QPushButton("打开")
        self.btn_grip_grasp = QPushButton("自适应抓取")
        self.btn_grip_grasp.setObjectName("good")
        self.btn_grip_hold = QPushButton("高刚度保持")
        self.btn_grip_save = QPushButton("保存高刚度")
        self.btn_grip_zero = QPushButton("零力矩停止")
        self.btn_grip_zero.setObjectName("danger")
        self.btn_grip_enable.clicked.connect(
            lambda: self.bridge.gripper_enable(True)
        )
        self.btn_grip_disable.clicked.connect(
            lambda: self.bridge.gripper_enable(False)
        )
        self.btn_grip_send.clicked.connect(
            lambda: self.bridge.gripper_sending(True)
        )
        self.btn_grip_send_stop.clicked.connect(
            lambda: self.bridge.gripper_sending(False)
        )
        self.btn_grip_open.clicked.connect(self.bridge.gripper_open)
        self.btn_grip_grasp.clicked.connect(self.bridge.gripper_grasp)
        self.btn_grip_hold.clicked.connect(self.bridge.gripper_hold_high)
        self.btn_grip_save.clicked.connect(self.bridge.gripper_save)
        self.btn_grip_zero.clicked.connect(self.bridge.gripper_stop)
        for i, b in enumerate((
            self.btn_grip_enable, self.btn_grip_disable,
            self.btn_grip_send, self.btn_grip_send_stop,
            self.btn_grip_open, self.btn_grip_grasp,
            self.btn_grip_hold, self.btn_grip_save,
            self.btn_grip_zero,
        )):
            rg.addWidget(b, i // 3, i % 3)
        v.addWidget(run)

        st = QGroupBox("夹爪状态")
        sg = QGridLayout(st)
        self.lbl_grip_q = QLabel("q: —")
        self.lbl_grip_dq = QLabel("dq: —")
        self.lbl_grip_tau = QLabel("tau: —")
        self.lbl_grip_contact = QLabel("接触: —")
        sg.addWidget(self.lbl_grip_q, 0, 0)
        sg.addWidget(self.lbl_grip_dq, 0, 1)
        sg.addWidget(self.lbl_grip_tau, 1, 0)
        sg.addWidget(self.lbl_grip_contact, 1, 1)
        v.addWidget(st)

        par = QGroupBox("参数（写入 /gripper_driver，运行中可改）")
        pg = QGridLayout(par)
        self.sp_grip_kp_low = self._dspin(0.0, 500.0, 5.0, 0.5, 2)
        self.sp_grip_kd_low = self._dspin(0.0, 20.0, 0.3, 0.1, 2)
        self.sp_grip_kp_high = self._dspin(0.0, 500.0, 60.0, 1.0, 2)
        self.sp_grip_kd_high = self._dspin(0.0, 20.0, 2.0, 0.1, 2)
        self.sp_grip_tau = self._dspin(0.0, 50.0, 1.0, 0.1, 3)
        self.sp_grip_pos_err = self._dspin(0.0, 1.0, 0.05, 0.005, 4)
        self.sp_grip_speed = self._dspin(0.01, 2.0, 0.20, 0.01, 3)
        self.sp_grip_travel = self._dspin(0.01, 12.0, 1.0, 0.1, 3)
        self.sp_grip_timeout = self._dspin(0.5, 60.0, 6.0, 0.5, 1)
        self.sp_grip_open_q = self._dspin(-12.5, 12.5, 0.0, 0.01, 4)
        self.sp_grip_close_q = self._dspin(-12.5, 12.5, 1.0, 0.01, 4)
        self.sp_grip_qmin = self._dspin(-12.5, 12.5, -1.0, 0.01, 4)
        self.sp_grip_qmax = self._dspin(-12.5, 12.5, 1.0, 0.01, 4)
        self.cb_grip_dir = QComboBox()
        self.cb_grip_dir.addItem("+1（q 增大为闭合）", 1.0)
        self.cb_grip_dir.addItem("-1（q 减小为闭合）", -1.0)
        self.chk_grip_autosave = QCheckBox("抓取成功后自动保存")
        self.chk_grip_autosave.setChecked(True)
        fields = [
            ("低刚度 kp", self.sp_grip_kp_low),
            ("低刚度 kd", self.sp_grip_kd_low),
            ("高刚度 kp", self.sp_grip_kp_high),
            ("高刚度 kd", self.sp_grip_kd_high),
            ("接触力矩 N·m", self.sp_grip_tau),
            ("接触位置误差 rad", self.sp_grip_pos_err),
            ("闭合速度 rad/s", self.sp_grip_speed),
            ("最大行程 rad", self.sp_grip_travel),
            ("超时 s", self.sp_grip_timeout),
            ("打开位置 q", self.sp_grip_open_q),
            ("闭合位置 q", self.sp_grip_close_q),
            ("机械下限 q", self.sp_grip_qmin),
            ("机械上限 q", self.sp_grip_qmax),
        ]
        for i, (label, widget) in enumerate(fields):
            pg.addWidget(QLabel(label), i // 2, (i % 2) * 2)
            pg.addWidget(widget, i // 2, (i % 2) * 2 + 1)
        r = (len(fields) + 1) // 2
        pg.addWidget(QLabel("闭合方向"), r, 0)
        pg.addWidget(self.cb_grip_dir, r, 1)
        pg.addWidget(self.chk_grip_autosave, r + 1, 0, 1, 2)
        self.btn_grip_apply = QPushButton("应用参数")
        self.btn_grip_apply.clicked.connect(self.on_grip_apply_params)
        pg.addWidget(self.btn_grip_apply, r + 1, 2, 1, 2)
        v.addWidget(par)

        sav = QGroupBox("保存的高刚度")
        sl = QVBoxLayout(sav)
        self.lbl_grip_saved = QLabel("保存: —")
        self.lbl_grip_saved.setObjectName("hint")
        self.lbl_grip_saved.setWordWrap(True)
        sl.addWidget(self.lbl_grip_saved)
        v.addWidget(sav)

        self.grip_log_view = QPlainTextEdit()
        self.grip_log_view.setReadOnly(True)
        self.grip_log_view.setMaximumBlockCount(300)
        self.grip_log_view.setMaximumHeight(140)
        v.addWidget(self.grip_log_view)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(inner)
        return scroll

    @staticmethod
    def _dspin(lo, hi, value, step, decimals):
        sp = QDoubleSpinBox()
        sp.setRange(lo, hi)
        sp.setDecimals(decimals)
        sp.setSingleStep(step)
        sp.setValue(value)
        return sp

    # ------------------------------------------------------------------ 状态
    def _sync_gains(self):
        self.bridge._kp = float(self.sp_kp.value())
        self.bridge._kd = float(self.sp_kd.value())
        self.bridge.motion.set_gains(self.sp_kp.value(), self.sp_kd.value())
        self.bridge.publish_setup()

    def _sync_ff(self, _value=None):
        b = self.bridge
        b.gravity_enabled = self.chk_gravity.isChecked()
        b.gravity_scale = self.sp_gravity_scale.value()
        b.friction_enabled = self.chk_friction.isChecked()
        b.coulomb = self.sp_coulomb.value()
        b.viscous = self.sp_viscous.value()
        b.friction_slope = self.sp_fslope.value()
        b.publish_setup()

    # ------------------------------------------------------- 参数保存 / 载入
    def _collect_settings(self) -> dict:
        """All operator-tunable values that should survive a restart."""
        gripper = {name: float(spin.value())
                   for name, spin in self._grip_param_mapping()}
        gripper["close_direction"] = float(self.cb_grip_dir.currentData())
        gripper["auto_save"] = bool(self.chk_grip_autosave.isChecked())
        return {
            "version": 1,
            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            # 6 轴轨迹 / MIT 伺服
            "arm": {
                "kp": float(self.sp_kp.value()),
                "kd": float(self.sp_kd.value()),
                "speed": float(self.sp_speed.value()),
                # 单轴 MIT 页的 kp/kd
                "mit_kp": float(self.sp_mit_kp.value()),
                "mit_kd": float(self.sp_mit_kd.value()),
            },
            "feedforward": {
                "gravity": bool(self.chk_gravity.isChecked()),
                "gravity_scale": float(self.sp_gravity_scale.value()),
                "friction": bool(self.chk_friction.isChecked()),
                "coulomb": float(self.sp_coulomb.value()),
                "viscous": float(self.sp_viscous.value()),
                "slope": float(self.sp_fslope.value()),
            },
            "gripper": gripper,
        }

    def on_save_settings(self):
        data = self._collect_settings()
        try:
            hmi_store.save(hmi_store.SETTINGS_NAME, data)
        except Exception as exc:
            QMessageBox.warning(self, "保存参数", f"写入失败: {exc}")
            return
        # 让夹爪驱动下次连上时也用这份参数
        self._grip_restore = dict(data["gripper"])
        self.bridge.log(
            "参数已保存: "
            + str(hmi_store.BASE_DIR / hmi_store.SETTINGS_NAME)
        )

    _SPIN_SETTINGS = (
        # (settings group, key, widget)
        ("arm", "kp", "sp_kp"),
        ("arm", "kd", "sp_kd"),
        ("arm", "speed", "sp_speed"),
        ("arm", "mit_kp", "sp_mit_kp"),
        ("arm", "mit_kd", "sp_mit_kd"),
        ("feedforward", "gravity_scale", "sp_gravity_scale"),
        ("feedforward", "coulomb", "sp_coulomb"),
        ("feedforward", "viscous", "sp_viscous"),
        ("feedforward", "slope", "sp_fslope"),
    )

    @staticmethod
    def _set_spin_value(spin, value) -> None:
        try:
            value = float(value)
        except (TypeError, ValueError):
            return
        if not math.isfinite(value):
            return
        spin.blockSignals(True)
        try:
            spin.setValue(value)
        finally:
            spin.blockSignals(False)

    def _restore_settings(self) -> None:
        """Load hmi_settings.json into the widgets (defaults if absent)."""
        data = hmi_store.load(hmi_store.SETTINGS_NAME)
        if not isinstance(data, dict):
            # 没有保存过：保持默认值，仍然同步一次 bridge
            self._sync_gains()
            self._sync_ff()
            return

        for group, key, attr in self._SPIN_SETTINGS:
            sect = data.get(group)
            if isinstance(sect, dict) and key in sect:
                self._set_spin_value(getattr(self, attr), sect[key])

        ff = data.get("feedforward")
        if isinstance(ff, dict):
            if "gravity" in ff:
                self.chk_gravity.setChecked(bool(ff["gravity"]))
            if "friction" in ff:
                self.chk_friction.setChecked(bool(ff["friction"]))

        grip = data.get("gripper")
        if isinstance(grip, dict):
            restore: dict = {}
            for name, spin in self._grip_param_mapping():
                if name not in grip:
                    continue
                self._set_spin_value(spin, grip[name])
                try:
                    restore[name] = float(grip[name])
                except (TypeError, ValueError):
                    pass
            if grip.get("close_direction") is not None:
                try:
                    direction = float(grip["close_direction"])
                    restore["close_direction"] = direction
                    self.cb_grip_dir.setCurrentIndex(
                        0 if direction > 0.0 else 1
                    )
                except (TypeError, ValueError):
                    pass
            if "auto_save" in grip:
                auto_save = bool(grip["auto_save"])
                restore["auto_save"] = auto_save
                self.chk_grip_autosave.setChecked(auto_save)
            # 驱动连上后自动写回（驱动未就绪时保留，就绪后一次）
            self._grip_restore = restore

        # 复选框/数值是阻塞信号写入的，这里统一同步一次
        self._sync_gains()
        self._sync_ff()
        self.bridge.log(
            "已载入上次保存的参数"
            + (f"（{data.get('saved_at')}）" if data.get("saved_at") else "")
            + "；夹爪参数会在驱动连接后自动写回"
        )
        if self.chk_gravity.isChecked():
            self.bridge.log(
                "注意：上次保存时重力补偿为开启；使能并开始发送后才会出力"
            )

    # ------------------------------------------------------------ 软件限位
    def _load_joint_limits(self):
        model = np.asarray(self._model_limits, dtype=float)
        data = hmi_store.load(hmi_store.LIMITS_NAME)
        if isinstance(data, dict):
            try:
                lo = np.asarray(
                    [float(v) for v in data.get("lower", [])], dtype=float
                )
                hi = np.asarray(
                    [float(v) for v in data.get("upper", [])], dtype=float
                )
            except (TypeError, ValueError):
                lo = np.empty(0)
                hi = np.empty(0)
            if (lo.size == self.n and hi.size == self.n
                    and np.all(np.isfinite(lo))
                    and np.all(np.isfinite(hi))):
                lo = np.maximum(lo, model[:, 0])
                hi = np.minimum(hi, model[:, 1])
                if np.all(lo < hi):
                    return np.column_stack([lo, hi])
        return model.copy()

    def _q_within_limits(self, q, eps: float = 0.0):
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.size != self.n:
            return False, ["长度不符"]
        lo = self.joint_limits[:, 0]
        hi = self.joint_limits[:, 1]
        tol = 1e-6 + max(0.0, float(eps))
        bad = [
            f"J{i + 1}" for i in range(self.n)
            if not (lo[i] - tol <= q[i] <= hi[i] + tol)
        ]
        return (not bad), bad

    def _q_within_rec_limits(self, q):
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.size != self.n:
            return False, ["长度不符"]
        lo = self.joint_limits[:, 0]
        hi = self.joint_limits[:, 1]
        # J2/J3 的模型限位在 0，拖动/重力会让它们略微越界；
        # 录制/回放给这两轴更大的边界容差，其余关节保持严格。
        tol = np.array([0.01, 0.10, 0.10, 0.01, 0.01, 0.01])[:self.n]
        bad = [
            f"J{i + 1}" for i in range(self.n)
            if not (lo[i] - tol[i] <= q[i] <= hi[i] + tol[i])
        ]
        return (not bad), bad

    def _sync_limit_spins(self):
        if not hasattr(self, "sp_lim_lo"):
            return
        for i in range(self.n):
            self.sp_lim_lo[i].setValue(
                float(np.degrees(self.joint_limits[i, 0]))
            )
            self.sp_lim_hi[i].setValue(
                float(np.degrees(self.joint_limits[i, 1]))
            )
        self.lbl_limits.setText(
            "限位: " + ", ".join(
                f"J{i + 1} "
                f"{np.degrees(self.joint_limits[i, 0]):+.1f}"
                f"~{np.degrees(self.joint_limits[i, 1]):+.1f}°"
                for i in range(self.n)
            )
        )

    def on_limits_model(self):
        for i in range(self.n):
            self.sp_lim_lo[i].setValue(
                float(np.degrees(self._model_limits[i, 0]))
            )
            self.sp_lim_hi[i].setValue(
                float(np.degrees(self._model_limits[i, 1]))
            )
        self.bridge.log("已载入模型默认限位，点「应用限位」生效")

    def on_limits_apply(self):
        lo = np.radians(np.array([sp.value() for sp in self.sp_lim_lo]))
        hi = np.radians(np.array([sp.value() for sp in self.sp_lim_hi]))
        if np.any(lo >= hi):
            bad = [f"J{i + 1}" for i in range(self.n) if lo[i] >= hi[i]]
            QMessageBox.warning(
                self, "限位无效",
                "下限必须小于上限: " + ", ".join(bad),
            )
            return
        model = self._model_limits
        lo_c = np.maximum(lo, model[:, 0])
        hi_c = np.minimum(hi, model[:, 1])
        if np.any(lo_c >= hi_c):
            QMessageBox.warning(
                self, "限位无效",
                "输入范围与模型/机械限位没有交集。",
            )
            return
        clipped = not (np.allclose(lo, lo_c) and np.allclose(hi, hi_c))
        self.joint_limits = np.column_stack([lo_c, hi_c])
        self.kin.limits = self.joint_limits.copy()
        try:
            self.bridge.set_joint_limits(lo_c, hi_c)
        except Exception as exc:
            QMessageBox.warning(self, "限位", f"下发限位失败: {exc}")
            return
        self._update_limits_to_spins()
        self._sync_limit_spins()
        hmi_store.save(
            hmi_store.LIMITS_NAME,
            {"lower": lo_c.tolist(), "upper": hi_c.tolist()},
        )
        if clipped:
            self.bridge.log("输入限位超出模型/机械限位，已裁剪到允许范围")
        ok, bad = self._q_within_limits(self.bridge.current_q())
        if not ok:
            self.bridge.log(
                "当前实测角在限位外: " + ", ".join(bad)
                + "；下一次运动会被拉回范围内"
            )
        self.bridge.log(
            "软件限位已应用: " + ", ".join(
                f"J{i + 1}[{np.degrees(lo_c[i]):+.1f},"
                f"{np.degrees(hi_c[i]):+.1f}]°"
                for i in range(self.n)
            )
        )
        self._push_limits_to_driver(lo_c, hi_c)

    def _push_limits_to_driver(self, lo, hi) -> None:
        """Mirror the HMI limits into /motor_driver so other clients obey them."""
        for key, vals in (("joint_lower", lo), ("joint_upper", hi)):
            arg = "[" + ", ".join(f"{v:.6f}" for v in vals) + "]"
            try:
                res = subprocess.run(
                    ["ros2", "param", "set", "/motor_driver", key, arg],
                    capture_output=True, text=True, timeout=4.0,
                )
            except Exception as exc:
                self.bridge.log(f"驱动 {key} 同步失败: {exc}")
                return
            if res.returncode != 0:
                out = ((res.stdout or "") + (res.stderr or "")).strip()
                self.bridge.log(f"驱动 {key} 同步失败: {out}")
                return
        self.bridge.log("驱动侧软件限位已同步")

    def _update_limits_to_spins(self):
        for i in range(self.n):
            lo, hi = self.joint_limits[i]
            self.joint_targets[i].setRange(float(lo), float(hi))
        if hasattr(self, "cb_single") and hasattr(self, "sp_single_target"):
            j = self.cb_single.currentIndex()
            if j >= 0:
                lo, hi = self.joint_limits[j]
                self.sp_single_target.setRange(float(lo), float(hi))

    def _on_single_joint_changed(self):
        j = self.cb_single.currentIndex()
        if j < 0:
            return
        lo, hi = self.kin.limits[j]
        self.sp_single_target.setRange(float(lo), float(hi))
        q = self.bridge.current_q()
        self.sp_single_target.setValue(float(q[j]))
        self.lbl_single_cur.setText(f"当前 {q[j]:+.4f} rad")

    def _update_mode_label(self):
        if not hasattr(self, "lbl_mode_now") or not hasattr(self, "cb_mit_motor"):
            return
        cid = self._mit_cid()
        if not cid:
            self._set_text(self.lbl_mode_now, "当前: —")
            return
        i = self.bridge._slot_for_can(cid)
        mode = (
            self.bridge.modes[i]
            if i is not None and i < len(self.bridge.modes) else "?"
        )
        self._set_text(self.lbl_mode_now, f"当前: {mode}")

    @staticmethod
    def _set_text(widget, text: str):
        if widget.text() != text:
            widget.setText(text)

    @staticmethod
    def _set_item(item, text: str):
        if item.text() != text:
            item.setText(text)

    def _append_log(self, text: str):
        self.log_view.appendPlainText(text)

    def _set_param_text(self, text: str):
        self.lbl_param_result.setText(text)

    def _sync_preview_motion(self) -> None:
        if self._closing or not hasattr(self, "preview"):
            return
        q_cmd, active = self.bridge.motion.commanded()
        if not (self.bridge.enabled and self.bridge.sending):
            if self._seen_motion:
                self._seen_motion = False
                self.preview.finish_motion(q_cmd)
            return
        if active:
            self._seen_motion = True
        elif self._seen_motion:
            self._seen_motion = False
            self.preview.finish_motion(q_cmd)
        self.preview.show_motion(q_cmd)

    def _tick_ui(self):
        if self._closing:
            return
        self._tick_driver_state()
        if self._driver_sync_pending and self.bridge.registry_names:
            self._sync_registry_from_driver(force=True)
            self._driver_sync_pending = False
        if not self._driver_registry_synced and self.bridge.registry_names:
            if not self.registry:
                self._sync_registry_from_driver()
            else:
                self._driver_registry_synced = True
        q = self.bridge.current_q()
        age = self.bridge.feedback_age()
        motor_age = self.bridge.motor_age()
        if (
            not self._preview_initialized
            and hasattr(self, "preview")
            and age < 0.2
            and motor_age < 0.5
        ):
            self.preview.reset_to_real(q)
            self._preview_initialized = True
        now = time.monotonic()
        if (
            self.chk_follow.isChecked()
            and self.tabs.currentIndex() == 2
            and not (self.bridge.enabled and self.bridge.sending)
            and not self.preview.is_previewing()
            and not self.preview.dragging
            and motor_age < 0.5
            and now - self._last_follow >= 0.2
        ):
            self._last_follow = now
            mask = self.bridge.active_mask
            if mask.any() and np.max(
                np.abs((q - self.preview.q_preview())[mask])
            ) > 1e-4:
                self.preview.reset_to_real(q)
                self._preview_initialized = True
        self._set_text(
            self.lbl_topic,
            f"话题: {age * 1000:.0f} ms" if age < 9 else "话题: 无",
        )
        self._set_text(
            self.lbl_motor,
            f"电机反馈: {motor_age * 1000:.0f} ms" if motor_age < 9 else "电机反馈: 无",
        )
        active_slots = [
            i for i in range(self.n) if self.bridge.active_mask[i]
        ]
        n_active = len(active_slots)
        n_enabled = sum(
            1 for i in active_slots if self.bridge.enabled_each[i]
        )
        n_sending = sum(
            1 for i in active_slots if self.bridge.sending_each[i]
        )
        if n_enabled and n_sending:
            hmi_state = (
                f"使能 {n_enabled}/{n_active} · 发送 {n_sending}/{n_active}"
            )
        elif n_enabled:
            hmi_state = f"使能 {n_enabled}/{n_active} · 未发送"
        else:
            hmi_state = f"未使能 · 注册 {n_active}"
        if not self.bridge.driver_enabled:
            drv_state = "驱动未使能"
            color = "color: #8f98a5;"
        elif not self.bridge.driver_sending:
            drv_state = "驱动使能/未发送"
            color = "color: #ebcb8b;"
        else:
            drv_state = "驱动发送中"
            color = "color: #a3be8c;" if n_sending else "color: #ebcb8b;"
        state = f"{hmi_state} · {drv_state}"
        if self.lbl_state.styleSheet() != color:
            self.lbl_state.setStyleSheet(color)
        self._set_text(self.lbl_state, state)
        active, pending = self.bridge.motion.state()
        self._set_text(
            self.lbl_motion,
            f"运动: 执行中（剩余 {pending} 段）" if active else "运动: 保持",
        )
        tau_ff = np.asarray(self.bridge.last_tau_ff)
        if not (
            self.bridge.enabled
            and self.bridge.sending
            and self.bridge.driver_enabled
            and self.bridge.driver_sending
        ):
            ff_text = "前馈: 开始控制后生效"
        elif self.chk_gravity.isChecked() or self.chk_friction.isChecked():
            ff_text = "前馈: " + " ".join(
                f"{float(v):+.1f}" for v in tau_ff
            ) + " N·m"
        else:
            ff_text = "前馈: 关闭"
        self._set_text(self.lbl_ff, ff_text)

        self._set_text(self.btn_enable, "全部使能")
        self.btn_enable.setEnabled(n_active > 0 and n_enabled < n_active)
        self._set_text(self.btn_send, "全部开始发送")
        self.btn_send.setEnabled(n_enabled > 0 and n_sending < n_enabled)
        self.btn_disable_all.setEnabled(n_enabled > 0)
        self.btn_stop_all.setEnabled(n_sending > 0)
        all6_ready = (
            n_active == 6
            and n_enabled == 6
            and n_sending == 6
            and self.bridge.driver_enabled
            and self.bridge.driver_sending
        )
        self.btn_home.setEnabled(all6_ready)
        if (
            hasattr(self, "btn_drag")
            and self.btn_drag.isChecked()
            and not all6_ready
        ):
            self.btn_drag.blockSignals(True)
            self.btn_drag.setChecked(False)
            self.btn_drag.blockSignals(False)
            self.bridge.drag_teach(False)

        for i in range(self.n):
            if not self.bridge.active_mask[i]:
                self._set_item(self.tbl_joint.item(i, 1), "未注册")
                self._set_item(self.tbl_joint.item(i, 3), "未注册")
                self._set_item(self.tbl_joint.item(i, 4), "-")
                continue
            self._set_item(self.tbl_joint.item(i, 1), f"{q[i]:+.4f}")
            mode = self.bridge.modes[i] if i < len(self.bridge.modes) else "?"
            err = self.bridge.errs[i] if i < len(self.bridge.errs) else -1
            self._set_item(self.tbl_joint.item(i, 3), str(mode))
            self._set_item(self.tbl_joint.item(i, 4), str(err))

        self._update_mode_label()
        self._update_registry_rows()
        self._update_mit_state_label()

        try:
            p = self.kin.fk(q)
            text = f"TCP: x={p[0]:+.4f} y={p[1]:+.4f} z={p[2]:+.4f} m"
            self._set_text(self.lbl_tcp, text)
            self._set_text(self.lbl_ik_cur, "当前 " + text)
        except Exception:
            pass

    def _tick_plots(self):
        if (
            self._closing
            or self.isMinimized()
            or self.plot_panel.chk_pause.isChecked()
        ):
            return
        self.plot_panel.update_data(
            self.bridge.snapshot(), self.plot_panel.chk_cmd.isChecked()
        )

    # ------------------------------------------------------------------ 手柄
    def _keyboard_velocity(self) -> np.ndarray:
        if not self.chk_keyboard.isChecked() or self._key_suspend:
            return np.zeros(3)
        keys = self._keys
        return np.array([
            float(Qt.Key_Right in keys) - float(Qt.Key_Left in keys),
            float(Qt.Key_Up in keys) - float(Qt.Key_Down in keys),
            float(Qt.Key_Space in keys) - float(Qt.Key_Shift in keys),
        ], dtype=float)

    def _jog_ready(self) -> bool:
        return bool(
            not self._rec_active
            and not self._replay_active
            and self._all_registered()
            and self.bridge.enabled
            and self.bridge.sending
            and self.bridge.driver_enabled
            and self.bridge.driver_sending
            and self.bridge.motor_age() <= 0.5
            and self.bridge.feedback_age() <= 0.5
        )

    def _begin_jog(self, reason: str) -> None:
        q_now = self.bridge.current_q()
        self._jog_pos, self._jog_R = self.kin.tcp_pose(q_now)
        self._jog_q = q_now.copy()
        self._jog_t = time.monotonic()
        self.bridge.log(reason)

    def _step_jog(self, new_pos, new_R, now: float, prefix: str) -> bool:
        if not (np.all(np.isfinite(new_pos)) and np.all(np.isfinite(new_R))):
            return False
        # 增量点动以上一帧为种子，小步目标不需要 150 次迭代
        q, e_pos, e_rot, _ = self.kin.ik_pose(
            new_pos, new_R, self._jog_q, max_iter=60
        )
        if not (math.isfinite(e_pos) and math.isfinite(e_rot)):
            return False
        if e_pos > 0.005 or e_rot > 0.05:
            if now - self._pad_warn_t > 2.0:
                self._pad_warn_t = now
                self.bridge.log(
                    "%s: IK 误差 %.1f mm / %.1f°，保持"
                    % (prefix, e_pos * 1000.0, math.degrees(e_rot))
                )
            return False
        if np.max(np.abs(q - self._jog_q)) > 0.10:
            if now - self._pad_warn_t > 2.0:
                self._pad_warn_t = now
                self.bridge.log("%s: 关节跳变超过 0.10 rad，保持" % prefix)
            return False
        self._jog_pos = np.asarray(new_pos, dtype=float).copy()
        self._jog_R = np.asarray(new_R, dtype=float).copy()
        self._jog_q = q
        self.bridge.set_direct(q)
        return True

    def _update_pad_label(self, connected: bool, ready: bool):
        if self.gamepad.sock is None:
            text = "手柄: UDP 端口占用"
        elif not connected:
            text = "手柄: 未连接"
        elif not self.chk_gamepad.isChecked():
            text = "手柄: 已连接（未启用）"
        elif not ready:
            text = "手柄: 等待使能/发送"
        elif self._pad_stop_latch:
            text = "手柄: B 已停，松开 LB 再按"
        elif self._gamepad_active:
            text = "手柄: 控制中 (LB)"
        else:
            text = "手柄: 按住 LB 操作"
        self._set_text(self.lbl_gamepad, text)

    def _tick_gamepad(self):
        if self._closing:
            return
        self._sync_preview_motion()
        self.gamepad.poll()
        for src in self.gamepad.rejected_sources - self._pad_rejected_logged:
            self._pad_rejected_logged.add(src)
            self.bridge.log(
                f"手柄: 忽略来源 {src}（只接受本机或默认网关）"
            )
        if self.gamepad.source and self.gamepad.source != self._pad_source_logged:
            self._pad_source_logged = self.gamepad.source
            self.bridge.log(f"手柄 UDP 来源: {self.gamepad.source}")
        connected = self.gamepad.connected()
        ready = (
            self.bridge.enabled
            and self.bridge.sending
            and self.bridge.driver_enabled
            and self.bridge.driver_sending
        )
        self._handle_pad_waypoints(ready)
        lb = bool(self.gamepad.buttons & 0x0100)
        b_button = bool(self.gamepad.buttons & 0x2000)
        if not lb:
            self._pad_stop_latch = False
        if b_button and self._gamepad_active:
            self.bridge.stop_motion()
            self.bridge.log("手柄: B 平滑停止")
            self._gamepad_active = False
            self._pad_stop_latch = True
        key_dir = self._keyboard_velocity()
        key_on = bool(np.any(key_dir)) and self._jog_ready()
        if not self._keys:
            self._key_suspend = False
        active_now = (
            connected
            and self.chk_gamepad.isChecked()
            and ready
            and lb
            and not self._pad_stop_latch
            and self.bridge.motor_age() <= 0.5
            and self.bridge.feedback_age() <= 0.5
        )
        jog_on = (active_now or key_on) and not self.btn_drag.isChecked()
        if not jog_on:
            if self._gamepad_active:
                self._gamepad_active = False
                self.bridge.log("手柄: 松开/断开，保持当前目标")
            if self._keyboard_active:
                self._keyboard_active = False
                self.bridge.log("键盘: 松开，保持当前目标")
            self._update_pad_label(connected, ready)
            self._update_key_label(ready)
            return
        now = time.monotonic()
        if not (self._gamepad_active or self._keyboard_active):
            self._begin_jog("点动: 从当前末端开始")
        self._gamepad_active = active_now
        self._keyboard_active = key_on
        dt = min(max(now - self._jog_t, 0.0), 0.1)
        self._jog_t = now
        if dt <= 0.0:
            self._update_pad_label(connected, ready)
            self._update_key_label(ready)
            return
        lx, ly, rx, ry, lt, rt = self.gamepad.axes

        def dz(v):
            return 0.0 if abs(v) < 0.15 else v

        dpos = np.zeros(3)
        yaw = 0.0
        pitch = 0.0
        if active_now:
            dpos += np.array([-dz(lx), -dz(ly), (rt - lt)]) * 0.10 * dt
            yaw += -dz(rx) * 0.5 * dt
            pitch += dz(ry) * 0.5 * dt
        if key_on:
            dpos += key_dir * 0.10 * dt
        new_pos = np.clip(
            self._jog_pos + dpos, [-0.8, -0.8, 0.03], [0.8, 0.8, 1.2]
        )
        R = (
            _rot_axis([0.0, 0.0, 1.0], yaw)
            @ _rot_axis([0.0, 1.0, 0.0], pitch)
            @ self._jog_R
        )
        U, _S, Vt = np.linalg.svd(R)
        prefix = "键盘" if key_on and not active_now else "手柄"
        self._step_jog(new_pos, U @ Vt, now, prefix)
        self._update_pad_label(connected, ready)
        self._update_key_label(ready)

    def _update_key_label(self, ready: bool) -> None:
        if not self.chk_keyboard.isChecked():
            text = "键盘: 关"
        elif not ready:
            text = "键盘: 等待使能/发送"
        elif self._key_suspend:
            text = "键盘: 已暂停，松开按键"
        elif self._keyboard_active:
            text = "键盘: 控制中"
        else:
            text = "键盘: ←→X ↑↓Y 空格/Shift Z"
        self._set_text(self.lbl_keyboard, text)

    def eventFilter(self, obj, event):
        etype = event.type()
        if etype == QEvent.WindowDeactivate:
            self._keys.clear()
            return False
        if etype not in (QEvent.KeyPress, QEvent.KeyRelease):
            return False
        if not hasattr(self, "chk_keyboard") or not self.chk_keyboard.isChecked():
            return False
        # 编辑数值/文本时不要吞掉方向键、空格和 Shift
        focus = QApplication.focusWidget()
        if isinstance(focus, (QLineEdit, QSpinBox, QDoubleSpinBox,
                              QPlainTextEdit)):
            return False
        if isinstance(focus, QComboBox) and focus.isEditable():
            return False
        if QApplication.activeModalWidget() is not None:
            return False
        key = event.key()
        if key not in (
            Qt.Key_Left, Qt.Key_Right, Qt.Key_Up, Qt.Key_Down,
            Qt.Key_Space, Qt.Key_Shift,
        ):
            return False
        if event.isAutoRepeat():
            return True
        if etype == QEvent.KeyPress:
            self._keys.add(key)
        else:
            self._keys.discard(key)
        return True

    def _suspend_gamepad(self):
        self._gamepad_active = False
        self._keyboard_active = False
        self._pad_stop_latch = True
        self._key_suspend = True

    # ------------------------------------------------------------------ 操作
    def on_drag_teach(self):
        on = self.btn_drag.isChecked()
        if self._rec_active:
            if not on:
                self.on_rec_stop()
            return
        if self._replay_active:
            self.btn_drag.blockSignals(True)
            self.btn_drag.setChecked(False)
            self.btn_drag.blockSignals(False)
            self.bridge.log("回放中不能切换拖动示教")
            return
        if on and not self._check_motion_ready():
            self.btn_drag.blockSignals(True)
            self.btn_drag.setChecked(False)
            self.btn_drag.blockSignals(False)
            return
        if on:
            self._suspend_gamepad()
        self.bridge.drag_teach(on)

    # ------------------------------------------------------------ 全部 / 单轴
    def on_enable_all(self):
        self.bridge.set_motor_enabled([], True, all_motors=True)

    def on_disable_all(self):
        if self._rec_active:
            self.on_rec_stop()
        if self._replay_active:
            self._finish_replay("失能", abort=True)
        self._suspend_gamepad()
        self.bridge.set_motor_enabled([], False, all_motors=True)

    def on_send_all(self):
        self.bridge.set_motor_sending([], True, all_motors=True)

    def on_stop_all(self):
        if self._rec_active:
            self.on_rec_stop()
        if self._replay_active:
            self._finish_replay("停止发送", abort=True)
        self.bridge.set_motor_sending([], False, all_motors=True)

    # Backwards-compatible aliases (old callers)
    def on_enable(self):
        self.on_enable_all()

    def on_send_toggle(self):
        self.on_send_all()

    def _mit_cid(self):
        if not hasattr(self, "cb_mit_motor"):
            return None
        data = self.cb_mit_motor.currentData()
        return int(data) if data is not None else None

    def _sync_mit_selection(self):
        if not hasattr(self, "lbl_mit_state"):
            return
        cid = self._mit_cid()
        if cid is None:
            self.lbl_mit_state.setText("注册表为空")
            self._update_mode_label()
            return
        i = self.bridge._slot_for_can(cid)
        if i is None:
            self.lbl_mit_state.setText("未在驱动注册表里")
            self._update_mode_label()
            return
        q = float(self.bridge.current_q()[i])
        self.sp_mit_q.setValue(q)
        self._update_mit_state_label()
        self._update_mode_label()

    def _update_mit_state_label(self):
        if not hasattr(self, "lbl_mit_state"):
            return
        cid = self._mit_cid()
        if cid is None:
            self.lbl_mit_state.setText("注册表为空")
            return
        i = self.bridge._slot_for_can(cid)
        if i is None:
            self.lbl_mit_state.setText("未在驱动注册表里")
            return
        enabled = bool(
            self.bridge.enabled_each[i]
            if i < len(self.bridge.enabled_each) else False
        )
        sending = bool(
            self.bridge.sending_each[i]
            if i < len(self.bridge.sending_each) else False
        )
        mode = self.bridge.modes[i] if i < len(self.bridge.modes) else "?"
        err = self.bridge.errs[i] if i < len(self.bridge.errs) else -1
        age = self.bridge.ages[i] if i < len(self.bridge.ages) else 999.0
        self.lbl_mit_state.setText(
            f"使能 {'是' if enabled else '否'} · 发送 {'是' if sending else '否'} · "
            f"{mode} · ERR {err} · {age * 1000:.0f} ms"
        )
        self._reg_sync = True
        try:
            self.btn_mit_send.setChecked(sending)
            self.btn_mit_send.setText("停止发送" if sending else "发送数据")
        finally:
            self._reg_sync = False
        self.btn_mit_enable.setEnabled(not enabled)
        self.btn_mit_disable.setEnabled(enabled)
        self.btn_mit_apply.setEnabled(enabled and sending)
        self.btn_mit_hold.setEnabled(enabled and sending)
        self.btn_mit_zero.setEnabled(enabled and sending)

    def on_mit_enable(self):
        if self._rec_active or self._replay_active:
            self.bridge.log("录制/回放中，忽略单轴使能")
            return
        cid = self._mit_cid()
        if cid is None:
            QMessageBox.warning(self, "单轴", "注册表为空")
            return
        self.bridge.set_motor_enabled([cid], True)

    def on_mit_disable(self):
        cid = self._mit_cid()
        if cid is not None:
            self.bridge.set_motor_enabled([cid], False)

    def on_mit_send_toggle(self):
        if self._rec_active or self._replay_active:
            self.bridge.log("录制/回放中，忽略单轴发送切换")
            return
        if self._reg_sync:
            return
        cid = self._mit_cid()
        if cid is None:
            return
        on = self.btn_mit_send.isChecked()
        if on and not self.bridge.motor_enabled(cid):
            self._reg_sync = True
            self.btn_mit_send.setChecked(False)
            self._reg_sync = False
            QMessageBox.warning(self, "未使能", "先点「使能本轴」，再发送数据。")
            return
        self.bridge.set_motor_sending([cid], on)

    def on_mit_stop(self):
        if self._rec_active or self._replay_active:
            self.bridge.log("录制/回放中，忽略单轴停止发送；请用「停止复现」")
            return
        cid = self._mit_cid()
        if cid is not None:
            self.bridge.set_motor_sending([cid], False)

    def _mit_payload(self):
        cid = self._mit_cid()
        if cid is None:
            return None
        return (
            cid,
            self.sp_mit_kp.value(),
            self.sp_mit_kd.value(),
            self.sp_mit_q.value(),
            self.sp_mit_dq.value(),
            self.sp_mit_tau.value(),
        )

    def on_mit_apply(self):
        if self._rec_active or self._replay_active:
            self.bridge.log("录制/回放中，忽略单轴 MIT 指令")
            return
        cid = self._mit_cid()
        if cid is None:
            return
        if not (self.bridge.motor_enabled(cid)
                and self.bridge.motor_sending(cid)):
            QMessageBox.warning(self, "未就绪", "先使能本轴并点「发送数据」。")
            return
        payload = self._mit_payload()
        if payload is not None:
            self.bridge.publish_motor_mit(*payload, timeout=2.0)

    def on_mit_hold(self):
        if self._rec_active or self._replay_active:
            self.bridge.log("录制/回放中，忽略单轴保持")
            return
        cid = self._mit_cid()
        if cid is not None:
            self.bridge.hold_motor(cid)

    def on_mit_zero(self):
        if self._rec_active or self._replay_active:
            self.bridge.log("录制/回放中，忽略单轴零力矩")
            return
        cid = self._mit_cid()
        if cid is None:
            return
        if QMessageBox.question(
            self, "零力矩", "该轴会自由拖动，可能因重力下落。确认？"
        ) != QMessageBox.Yes:
            return
        self.bridge.zero_motor(cid)

    def _tick_mit_live(self):
        if not hasattr(self, "chk_mit_live") or not self.chk_mit_live.isChecked():
            return
        cid = self._mit_cid()
        if cid is None:
            return
        if not (self.bridge.motor_enabled(cid)
                and self.bridge.motor_sending(cid)):
            return
        payload = self._mit_payload()
        if payload is not None:
            self.bridge.publish_motor_mit(*payload, timeout=0.5)

    def on_home(self):
        if not self._check_motion_ready():
            return
        self._suspend_gamepad()
        q_now = self.bridge.current_q()
        target = np.zeros(self.n, dtype=float)
        ok, bad = self._q_within_limits(target)
        if not ok:
            QMessageBox.warning(
                self, "限位",
                "fold（全零）不在当前软件限位内: " + ", ".join(bad)
                + "。先调宽限位，或不要回 fold。",
            )
            return
        delta = float(np.max(np.abs(target - q_now)))
        if delta < 0.01:
            self.bridge.log("已经在 fold（全零）附近，不再运动")
            return
        answer = QMessageBox.question(
            self,
            "回 fold",
            f"机械臂将从当前实测角度平滑回到各关节 0 点（初始 fold），"
            f"最大关节变化 {delta:.2f} rad。\n"
            "不做碰撞检测，确认周围安全？",
        )
        if answer != QMessageBox.Yes:
            return
        self.preview.reset_to_real(q_now)
        duration = max(1.5, delta / max(0.05, self.sp_speed.value()))
        if self.bridge.move_to(target, duration):
            self.bridge.log(f"回 fold：{duration:.1f} s，从当前角度平滑回到全零")

    def on_release(self):
        answer = QMessageBox.question(
            self,
            "急停 / 失能",
            "失能后机械臂会失去保持力矩，可能因重力下落。确认失能？",
        )
        if answer == QMessageBox.Yes:
            if self._rec_active:
                self.on_rec_stop()
            if self._replay_active:
                self._finish_replay("急停/失能", abort=True)
            self._suspend_gamepad()
            self.bridge.release()

    # ------------------------------------------------------------ 参数辨识
    def _ident_log_line(self, text: str) -> None:
        if text:
            self.ident_log_view.appendPlainText(str(text))

    def _update_ident_buttons(self, _checked=None) -> None:
        running = self._ident_phase != "idle"
        self.btn_ident_start.setEnabled(
            (not running) and self.chk_ident_safe.isChecked()
        )
        self.btn_ident_stop.setEnabled(running)

    def _ident_mode_name(self) -> str:
        return str(self.cb_ident_mode.currentText())

    def _ident_lock_ui(self, lock: bool) -> None:
        widgets = (
            self.btn_home, self.btn_connect, self.cb_driver_config,
            self.chk_sim, self.chk_gamepad, self.chk_keyboard,
        )
        if lock:
            self._ident_ui_saved = {
                w: bool(w.isEnabled()) for w in widgets
            }
            for w in widgets:
                w.setEnabled(False)
            self._ident_tab_saved = [
                bool(self.tabs.isTabEnabled(i))
                for i in range(self.tabs.count())
            ]
            cur = self.tabs.currentIndex()
            for i in range(self.tabs.count()):
                self.tabs.setTabEnabled(i, i == cur)
        else:
            for w, enabled in self._ident_ui_saved.items():
                try:
                    w.setEnabled(enabled)
                except Exception:
                    pass
            self._ident_ui_saved = {}
            for i, enabled in enumerate(self._ident_tab_saved):
                if i < self.tabs.count():
                    self.tabs.setTabEnabled(i, enabled)
            self._ident_tab_saved = []

    def _ident_refresh_label(self) -> None:
        names = {"arming": "准备中", "running": "辨识中",
                 "stopping_log": "收尾中", "analyzing": "分析中"}
        phase = names.get(self._ident_phase, self._ident_phase)
        elapsed = (
            time.monotonic() - self._ident_t0
            if self._ident_t0 else 0.0
        )
        self.lbl_ident.setText(
            f"辨识: {phase} · {self._ident_mode_name()} · {elapsed:.0f} s"
        )
        if self._ident_csv:
            self.lbl_ident_csv.setText(
                "CSV: " + os.path.basename(self._ident_csv)
            )

    def _ident_read_new(self, path: str, offset: int):
        if not path:
            return "", offset
        try:
            with open(path, "r", encoding="utf-8", errors="replace") as fh:
                fh.seek(offset)
                text = fh.read()
                return text, fh.tell()
        except OSError:
            return "", offset

    def _ident_tail_logs(self) -> None:
        text, self._ident_log_offset = self._ident_read_new(
            self._ident_log_path, self._ident_log_offset
        )
        for line in text.splitlines():
            self._ident_log_line(line)
        text, self._ident_analysis_offset = self._ident_read_new(
            self._ident_analysis_log, self._ident_analysis_offset
        )
        for line in text.splitlines():
            self._ident_log_line(line)

    def _ident_on_log_started(self, res) -> None:
        # Runs in the ROS executor thread: only touch plain attributes here.
        if not res.success:
            self._ident_log_state = "error"
            self._ident_msg = "记录 CSV 失败: " + str(res.message)
            return
        msg = str(res.message)
        path = msg.split("logging:", 1)[1].strip() if "logging:" in msg else ""
        self._ident_csv = path
        self._ident_log_state = "on"
        self._ident_msg = "CSV: " + (path or msg)

    def _ident_on_log_stopped(self, res) -> None:
        self._ident_log_state = "off"
        self._ident_msg = "CSV 记录结束: " + str(res.message)

    def _ident_launch(self) -> bool:
        mode = self.cb_ident_mode.currentData()
        amps = ",".join(f"{sp.value():.3f}" for sp in self.sp_ident_amp)
        cmd = [
            "ros2", "run", IDENT_PKG, IDENT_EXCITE, "--ros-args",
            "-p", "confirm:=true",
            "-p", f"mode:={mode}",
            "-p", "auto_log:=false",
            "-p", f"amplitude:=[{amps}]",
            "-p", f"frequency:={self.sp_ident_freq.value():.3f}",
            "-p", f"duration:={self.sp_ident_dur.value():.1f}",
            "-p", f"dwell:={self.sp_ident_dwell.value():.2f}",
            "-p", f"harmonics:={int(self.sp_ident_harm.value())}",
            "-p", "return_home:=" + (
                "true" if self.chk_ident_return.isChecked() else "false"
            ),
            "-p", "rate:=200.0",
            "-p", f"kp:={self.sp_kp.value():.3f}",
            "-p", f"kd:={self.sp_kd.value():.3f}",
            "-p", "gravity:=false",
            "-p", "friction:=false",
            "-p", "q_min:=["
            + ", ".join(f"{v:.6f}" for v in self.joint_limits[:, 0])
            + "]",
            "-p", "q_max:=["
            + ", ".join(f"{v:.6f}" for v in self.joint_limits[:, 1])
            + "]",
        ]
        if self.sp_kp.value() < 10.0:
            self._ident_log_line(
                "警告: 辨识增益 kp=%.2f 较低，可能撑不住重力；请确认机械臂有支撑"
                % self.sp_kp.value()
            )
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self._ident_log_path = f"/tmp/zfy_hmi_ident_run_{stamp}.log"
        self._ident_log_offset = 0
        try:
            self._ident_log = open(
                self._ident_log_path, "wb", buffering=0
            )
            self._ident_proc = subprocess.Popen(
                cmd,
                stdout=self._ident_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception as exc:
            self._ident_proc = None
            self._ident_log_line(f"启动激励节点失败: {exc}")
            return False
        self._ident_log_line("激励节点: " + " ".join(cmd))
        return True

    def _ident_signal_stop(self) -> None:
        proc = self._ident_proc
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception:
            try:
                proc.send_signal(signal.SIGINT)
            except Exception:
                pass

    def _ident_close_log(self) -> None:
        log = self._ident_log
        self._ident_log = None
        if log is not None:
            try:
                log.close()
            except Exception:
                pass

    def _ident_find_csv(self) -> str:
        if self._ident_csv and os.path.isfile(self._ident_csv):
            return self._ident_csv
        best = ""
        best_t = 0.0
        for path in glob.glob("/tmp/zfy_ident*.csv"):
            try:
                stamp = os.path.getmtime(path)
            except OSError:
                continue
            if stamp < self._ident_wall_t0 - 10.0:
                continue
            if stamp > best_t:
                best, best_t = path, stamp
        return best

    def _ident_start_analysis(self) -> None:
        self._ident_csv = self._ident_find_csv()
        if not self._ident_csv:
            self._ident_log_line("没有找到本次辨识的 CSV，跳过分析")
            self._ident_finish("完成（未找到 CSV）")
            return
        if not os.path.isfile(IDENT_SCRIPT):
            self._ident_log_line(f"找不到分析脚本: {IDENT_SCRIPT}")
            self._ident_finish("完成（分析脚本缺失）")
            return
        self._ident_json = (
            os.path.splitext(self._ident_csv)[0] + ".ident.json"
        )
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self._ident_analysis_log = (
            f"/tmp/zfy_hmi_ident_analysis_{stamp}.log"
        )
        self._ident_analysis_offset = 0
        cmd = [
            sys.executable, IDENT_SCRIPT, self._ident_csv,
            "--out", self._ident_json,
        ]
        try:
            log = open(self._ident_analysis_log, "wb", buffering=0)
            self._ident_analysis_proc = subprocess.Popen(
                cmd, stdout=log, stderr=subprocess.STDOUT,
            )
            log.close()
        except Exception as exc:
            self._ident_analysis_proc = None
            self._ident_log_line(f"启动分析脚本失败: {exc}")
            self._ident_finish("完成（分析失败）")
            return
        self._ident_phase = "analyzing"
        self._ident_log_line("分析脚本: " + " ".join(cmd))

    def _ident_finish(self, reason: str, abort: bool = False) -> None:
        self._ident_phase = "idle"
        self._ident_stop_requested = False
        proc = self._ident_proc
        self._ident_proc = None
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
        self._ident_close_log()
        if abort and self._ident_log_state in ("starting", "on", "stopping"):
            self.bridge.set_logging(False, cb=self._ident_on_log_stopped)
        self._ident_unlock_ui()
        self.chk_gamepad.setChecked(bool(getattr(self, "_ident_prev_pad", False)))
        self.chk_keyboard.setChecked(bool(getattr(self, "_ident_prev_kb", False)))
        self.lbl_ident.setText("辨识: " + reason)
        self.bridge.log("辨识: " + reason)
        self._update_ident_buttons()

    def _ident_unlock_ui(self) -> None:
        self._ident_lock_ui(False)

    def on_ident_start(self):
        if self._ident_phase != "idle":
            return
        if not self.chk_ident_safe.isChecked():
            QMessageBox.warning(self, "辨识", "先勾选安全确认。")
            return
        if not os.path.isfile(IDENT_SCRIPT):
            QMessageBox.critical(
                self, "辨识", f"找不到分析脚本:\n{IDENT_SCRIPT}"
            )
            return
        if not self.bridge.cli_logging.service_is_ready():
            QMessageBox.warning(
                self, "辨识",
                "驱动没有 ~/set_logging 服务。请确认驱动已启动，"
                "并且是带辨识日志的 zfy_arm_sim2real 版本。",
            )
            return
        if not self._check_motion_ready():
            return
        ok, bad = self._q_within_limits(self.bridge.current_q())
        if not ok:
            QMessageBox.warning(
                self, "超出限位",
                "当前实测角超出软件限位: " + ", ".join(bad)
                + "。先把机械臂移到限位内，再开始辨识。",
            )
            return
        mode = str(self.cb_ident_mode.currentData())
        amps = ", ".join(f"{sp.value():.3f}" for sp in self.sp_ident_amp)
        text = (
            f"模式: {self._ident_mode_name()}\n"
            f"幅值: [{amps}] rad\n"
            f"频率: {self.sp_ident_freq.value():.2f} Hz，"
            f"时长: {self.sp_ident_dur.value():.1f} s，"
            f"保持: {self.sp_ident_dwell.value():.1f} s\n\n"
            "开始后机械臂会自动运动，其他页面将被锁定。确认继续？"
        )
        if QMessageBox.question(self, "开始参数辨识", text) != QMessageBox.Yes:
            return

        self._suspend_gamepad()
        self._ident_prev_pad = self.chk_gamepad.isChecked()
        self._ident_prev_kb = self.chk_keyboard.isChecked()
        self.chk_gamepad.setChecked(False)
        self.chk_keyboard.setChecked(False)
        self.bridge.stop_motion()

        self._ident_csv = ""
        self._ident_json = ""
        self._ident_result = None
        self._ident_stop_requested = False
        self._ident_phase = "arming"
        self._ident_log_state = "starting"
        self._ident_msg = ""
        self._ident_log_path = ""
        self._ident_log_offset = 0
        self._ident_analysis_log = ""
        self._ident_analysis_offset = 0
        self._ident_t0 = 0.0
        self._ident_wall_t0 = time.time()
        self._ident_deadline = time.monotonic() + 3.0
        self.ident_log_view.clear()
        self.tbl_ident.clearContents()
        self.lbl_ident_global.setText("全局重力缩放: —")
        self.lbl_ident_csv.setText("CSV: 正在开始记录…")
        self.btn_ident_apply.setEnabled(False)
        self.btn_ident_param.setEnabled(False)
        self._ident_lock_ui(True)
        self._update_ident_buttons()
        self.lbl_ident.setText("辨识: 正在开始记录 CSV…")
        self.bridge.log(f"辨识: {mode}，等待 CSV 记录开始")
        self.bridge.set_logging(True, cb=self._ident_on_log_started)

    def on_ident_stop(self):
        if self._ident_phase == "idle":
            return
        if self._ident_phase == "arming":
            self._ident_finish("已取消", abort=True)
            return
        if self._ident_phase == "running":
            self._ident_stop_requested = True
            self._ident_deadline = time.monotonic() + 4.0
            self._ident_signal_stop()
            self.bridge.stop_motion()
            self.lbl_ident.setText("辨识: 正在停止…")
            self.bridge.log("辨识: 请求停止激励")
            return
        if self._ident_phase == "stopping_log":
            self._ident_log_state = "off"
            return
        if self._ident_phase == "analyzing":
            proc = self._ident_analysis_proc
            self._ident_analysis_proc = None
            if proc is not None and proc.poll() is None:
                try:
                    proc.terminate()
                except Exception:
                    pass
            self._ident_finish("已取消分析")

    def _tick_ident(self):
        if self._closing:
            return
        self._ident_tail_logs()
        if self._ident_msg:
            self._ident_log_line(self._ident_msg)
            self._ident_msg = ""
        phase = self._ident_phase
        if phase == "idle":
            return
        now = time.monotonic()

        if phase == "arming":
            state = self._ident_log_state
            if state == "on":
                if self._ident_launch():
                    self._ident_phase = "running"
                    self._ident_t0 = now
                    self._ident_deadline = 0.0
                    self.bridge.log("辨识: 激励节点已启动")
                    self._ident_refresh_label()
                else:
                    self._ident_finish("激励节点启动失败", abort=True)
            elif state == "error" or now > self._ident_deadline:
                self._ident_finish("CSV 记录未开始", abort=True)
            else:
                self._ident_refresh_label()
            return

        if phase == "running":
            proc = self._ident_proc
            if proc is not None and proc.poll() is None:
                if (self._ident_stop_requested
                        and now > self._ident_deadline):
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except Exception:
                        pass
                self._ident_refresh_label()
                return
            rc = proc.returncode if proc is not None else None
            self._ident_proc = None
            self._ident_close_log()
            self._ident_phase = "stopping_log"
            self._ident_log_state = "stopping"
            self._ident_deadline = now + 3.0
            self._ident_log_line(
                f"激励节点结束 (rc={rc})，停止 CSV 记录"
            )
            self.bridge.set_logging(False, cb=self._ident_on_log_stopped)
            self._ident_refresh_label()
            return

        if phase == "stopping_log":
            if self._ident_log_state == "off" or now > self._ident_deadline:
                if self.chk_ident_auto.isChecked():
                    self._ident_start_analysis()
                else:
                    self._ident_finish("完成（未分析）")
                if self._ident_phase != "idle":
                    self._ident_refresh_label()
            else:
                self._ident_refresh_label()
            return

        if phase == "analyzing":
            proc = self._ident_analysis_proc
            if proc is not None and proc.poll() is None:
                self._ident_refresh_label()
                return
            rc = proc.returncode if proc is not None else None
            self._ident_analysis_proc = None
            self._ident_tail_logs()
            ok = (
                rc == 0 and self._ident_json
                and os.path.isfile(self._ident_json)
            )
            if ok:
                try:
                    with open(self._ident_json, "r", encoding="utf-8") as fh:
                        report = json.load(fh)
                except Exception as exc:
                    self._ident_log_line(f"读取结果失败: {exc}")
                    ok = False
                else:
                    self._ident_show_results(report)
            if ok:
                self._ident_finish("辨识完成")
            else:
                self._ident_finish("分析失败，见日志")

    @staticmethod
    def _fmt_ident(value) -> str:
        try:
            value = float(value)
        except (TypeError, ValueError):
            return "—"
        return f"{value:.4f}" if math.isfinite(value) else "—"

    def _ident_show_results(self, report: dict) -> None:
        self._ident_result = report
        per = report.get("per_joint") or {}
        scale = report.get("global_gravity_scale")
        text = "全局重力缩放: " + self._fmt_ident(scale)
        try:
            g = float(scale)
        except (TypeError, ValueError):
            g = float("nan")
        if math.isfinite(g) and not (0.5 <= g <= 1.5):
            text += "  ⚠ 明显偏离 1，先检查零位/方向/质量"
        self.lbl_ident_global.setText(text)
        for i, name in enumerate(JOINTS):
            row = per.get(name) or {}
            values = [
                name,
                self._fmt_ident(row.get("gravity_scale")),
                self._fmt_ident(row.get("offset")),
                self._fmt_ident(row.get("viscous")),
                self._fmt_ident(row.get("coulomb")),
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(str(value))
                item.setTextAlignment(Qt.AlignCenter)
                self.tbl_ident.setItem(i, col, item)
        self.btn_ident_apply.setEnabled(True)
        self.btn_ident_param.setEnabled(True)
        self._ident_log_line(
            f"结果: {self._ident_json}，CSV: {self._ident_csv}"
        )

    def on_ident_apply(self):
        report = self._ident_result
        if not report:
            return
        per = report.get("per_joint") or {}
        scale = report.get("global_gravity_scale")
        try:
            g = float(scale)
        except (TypeError, ValueError):
            g = float("nan")
        if math.isfinite(g) and g > 0.0:
            clipped = max(0.0, min(1.5, g))
            self.sp_gravity_scale.setValue(clipped)
            if abs(clipped - g) > 1e-6:
                self._ident_log_line(
                    f"全局缩放 {g:.3f} 超出控件范围，已截到 {clipped:.3f}"
                )

        def avg(key):
            vals = []
            for row in per.values():
                if not isinstance(row, dict):
                    continue
                try:
                    v = float(row.get(key))
                except (TypeError, ValueError):
                    continue
                if math.isfinite(v):
                    vals.append(v)
            return sum(vals) / len(vals) if vals else None

        coul = avg("coulomb")
        visc = avg("viscous")
        if coul is not None:
            self.sp_coulomb.setValue(max(0.0, min(1.0, coul)))
        if visc is not None:
            self.sp_viscous.setValue(max(0.0, min(1.0, visc)))
        self._sync_ff()
        self.bridge.log(
            "辨识: 已填入前馈控件（重力 %.3f，库仑 %s，粘性 %s）；"
            "勾选补偿后才会生效" % (
                self.sp_gravity_scale.value(),
                "—" if coul is None else f"{coul:.4f}",
                "—" if visc is None else f"{visc:.4f}",
            )
        )
        self._ident_log_line("已把全局结果填入前馈控件，未自动启用。")

    def on_ident_write_params(self):
        report = self._ident_result
        if not report:
            return
        per = report.get("per_joint") or {}
        gains = []
        biases = []
        missing = []
        for name in JOINTS:
            row = per.get(name) or {}
            try:
                g = float(row.get("gravity_scale"))
            except (TypeError, ValueError):
                g = float("nan")
            try:
                b = float(row.get("offset"))
            except (TypeError, ValueError):
                b = float("nan")
            if math.isfinite(g) and 0.05 < g < 5.0:
                gains.append(g)
            else:
                gains.append(1.0)
                missing.append(name)
            biases.append(b if math.isfinite(b) else 0.0)
        text = (
            "将写入 /motor_driver:\n"
            "gravity_gains = [" + ", ".join(f"{v:.3f}" for v in gains) + "]\n"
            "gravity_biases = [" + ", ".join(f"{v:+.3f}" for v in biases) + "]\n\n"
            "同时把全局前馈 scale 置 1，避免和每关节增益重复相乘。"
            "驱动重启后会恢复 YAML 默认值。确认写入？"
        )
        if missing:
            text += "\n\n这些关节没有有效的重力缩放，将写 1.0: " + ", ".join(missing)
        if QMessageBox.question(self, "写入驱动重力增益", text) != QMessageBox.Yes:
            return
        ok = True
        for key, vals in (
            ("gravity_gains", gains),
            ("gravity_biases", biases),
        ):
            arg = "[" + ", ".join(f"{v:.6f}" for v in vals) + "]"
            try:
                res = subprocess.run(
                    ["ros2", "param", "set", "/motor_driver", key, arg],
                    capture_output=True, text=True, timeout=5.0,
                )
            except Exception as exc:
                ok = False
                self._ident_log_line(f"{key} 写入失败: {exc}")
                continue
            out = (res.stdout or "") + (res.stderr or "")
            self._ident_log_line(f"{key}: {out.strip()}")
            if res.returncode != 0:
                ok = False
        if ok:
            self.sp_gravity_scale.setValue(1.0)
            self._sync_ff()
            self.bridge.log(
                "辨识: 已写入 gravity_gains/gravity_biases，全局 scale=1"
            )
            self._ident_log_line("驱动重力增益写入成功。")
        else:
            self._ident_log_line("驱动参数写入失败，见上方输出。")

    def _ident_shutdown(self) -> None:
        if hasattr(self, "ident_timer"):
            self.ident_timer.stop()
        if self._ident_phase != "idle":
            self._ident_finish("窗口关闭，已中止", abort=True)
        elif self._ident_log_state in ("starting", "on", "stopping"):
            try:
                self.bridge.cli_logging.call_async(SetBool.Request())
            except Exception:
                pass
            self._ident_log_state = "off"
        proc = self._ident_analysis_proc
        self._ident_analysis_proc = None
        if proc is not None and proc.poll() is None:
            try:
                proc.terminate()
            except Exception:
                pass
        self._ident_close_log()

    # ------------------------------------------------------------- 夹爪
    def _grip_log(self, text: str) -> None:
        if text:
            self.grip_log_view.appendPlainText(
                time.strftime("%H:%M:%S ") + str(text)
            )

    def on_grip_start_driver(self):
        if self._gripper_proc is not None:
            return
        if self.bridge.cli_grip_enable.service_is_ready():
            self._grip_log("已有外部夹爪驱动在运行，不重复启动")
            return
        # 优先用本工程的 WSL 单路配置（can0 / J7=0x17+0x07），
        # 老目录仅作兼容回退。
        local_cfg = os.path.join(
            PROJECT_ROOT, "config", "gripper_socketcan_wsl.yaml"
        )
        cfg = local_cfg if os.path.isfile(local_cfg) else os.path.expanduser(
            "~/zfy_arm/Zfy_Arm_real_control/zfy_arm_sim2real/"
            "config/gripper.yaml"
        )
        if not os.path.isfile(cfg):
            QMessageBox.critical(self, "夹爪", f"找不到配置:\n{cfg}")
            return
        inner = (
            "exec ros2 run zfy_arm_sim2real gripper_driver "
            f"--ros-args --params-file {cfg}"
        )
        self._gripper_log_path = "/tmp/zfy_hmi_gripper.log"
        try:
            self._gripper_log = open(
                self._gripper_log_path, "wb", buffering=0
            )
            self._gripper_proc = subprocess.Popen(
                ["bash", "-lc", inner],
                stdout=self._gripper_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception as exc:
            self._gripper_proc = None
            QMessageBox.critical(self, "夹爪", f"启动失败: {exc}")
            return
        self._grip_log("启动: " + inner)

    def on_grip_stop_driver(self):
        if self._gripper_proc is None:
            if self.bridge.cli_grip_enable.service_is_ready():
                self._grip_log("外部驱动，请在启动它的终端停止")
            return
        self._stop_gripper_process()
        self._grip_log("夹爪驱动已停止")

    def _stop_gripper_process(self):
        proc = self._gripper_proc
        self._gripper_proc = None
        if proc is not None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except Exception:
                pass
            try:
                proc.wait(timeout=2.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
        log = self._gripper_log
        self._gripper_log = None
        if log is not None:
            try:
                log.close()
            except Exception:
                pass

    def _grip_log_tail(self, n: int = 8) -> None:
        try:
            with open(
                self._gripper_log_path, "r", encoding="utf-8",
                errors="replace",
            ) as fh:
                lines = fh.readlines()[-n:]
        except Exception:
            return
        for line in lines:
            self._grip_log("日志: " + line.rstrip())

    def _grip_read_saved(self) -> None:
        path = os.path.expanduser(
            "~/.config/zfy_arm/gripper_grasp.json"
        )
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            self.lbl_grip_saved.setText("保存: —")
            return
        if mtime == self._grip_saved_mtime:
            return
        self._grip_saved_mtime = mtime
        try:
            with open(path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
        except Exception as exc:
            self.lbl_grip_saved.setText(f"保存: 读取失败 {exc}")
            return
        self.lbl_grip_saved.setText(
            "保存: q=%s kp=%s kd=%s 接触=%s 时间=%s\n%s"
            % (
                data.get("hold_q", "—"),
                data.get("kp_high", "—"),
                data.get("kd_high", "—"),
                data.get("contacted", "—"),
                data.get("saved_at", "—"),
                path,
            )
        )

    def _tick_gripper(self):
        if self._closing:
            return
        proc = self._gripper_proc
        if proc is not None and proc.poll() is not None:
            self._grip_log_tail()
            self._stop_gripper_process()
            self._grip_log("夹爪驱动进程已退出")
        ready = self.bridge.cli_grip_enable.service_is_ready()
        if ready:
            text = "驱动: 已连接（本 HMI 启动）" if proc is not None \
                else "驱动: 已连接（外部）"
            self.lbl_grip_driver.setText(text)
        else:
            self.lbl_grip_driver.setText("驱动: 未连接")
            self._grip_params_loaded = False
        self.btn_grip_start.setEnabled(not ready and proc is None)
        self.btn_grip_stop.setEnabled(proc is not None)
        controls = (
            self.btn_grip_enable, self.btn_grip_disable,
            self.btn_grip_send, self.btn_grip_send_stop,
            self.btn_grip_open, self.btn_grip_grasp,
            self.btn_grip_hold, self.btn_grip_save,
            self.btn_grip_zero, self.btn_grip_apply,
        )
        for b in controls:
            b.setEnabled(ready)
        if not ready:
            self.lbl_grip_state.setText("状态: —")
            self.lbl_grip_q.setText("q: —")
            self.lbl_grip_dq.setText("dq: —")
            self.lbl_grip_tau.setText("tau: —")
            self.lbl_grip_contact.setText("接触: —")
            self._grip_read_saved()
            return
        if not self._grip_params_loaded:
            self._grip_params_loaded = True
            if self._grip_restore:
                # 有「保存参数」记录的夹爪配置：先写回驱动，再读回核对
                QTimer.singleShot(0, self._grip_push_saved_params)
            else:
                QTimer.singleShot(0, self._grip_read_driver_params)
        state = int(self.bridge.grip_state)
        name = (
            GRIP_STATE_NAMES[state]
            if 0 <= state < len(GRIP_STATE_NAMES) else "?"
        )
        age = (
            time.monotonic() - self.bridge.grip_stamp
            if self.bridge.grip_stamp > 0.0 else 999.0
        )
        age_text = f"{age * 1000:.0f} ms" if age < 9.0 else "无"
        self.lbl_grip_state.setText(
            f"状态: {name} · 反馈 {age_text}"
        )
        self.lbl_grip_q.setText(f"q: {self.bridge.grip_q:+.4f} rad")
        self.lbl_grip_dq.setText(f"dq: {self.bridge.grip_dq:+.4f} rad/s")
        self.lbl_grip_tau.setText(f"tau: {self.bridge.grip_tau:+.4f} N·m")
        self.lbl_grip_contact.setText(
            "接触: " + ("是" if self.bridge.grip_contact else "否")
        )
        self._grip_read_saved()

    def _grip_param_mapping(self):
        return [
            ("kp_low", self.sp_grip_kp_low),
            ("kd_low", self.sp_grip_kd_low),
            ("kp_high", self.sp_grip_kp_high),
            ("kd_high", self.sp_grip_kd_high),
            ("contact_tau", self.sp_grip_tau),
            ("contact_pos_err", self.sp_grip_pos_err),
            ("close_speed", self.sp_grip_speed),
            ("max_travel", self.sp_grip_travel),
            ("timeout", self.sp_grip_timeout),
            ("open_q", self.sp_grip_open_q),
            ("close_q", self.sp_grip_close_q),
            ("q_min", self.sp_grip_qmin),
            ("q_max", self.sp_grip_qmax),
        ]

    def _grip_push_saved_params(self):
        """Write the saved gripper parameters into the driver on connect."""
        if not self._grip_restore:
            QTimer.singleShot(0, self._grip_read_driver_params)
            return
        if not self.bridge.cli_grip_set_params.service_is_ready():
            self._grip_params_loaded = False
            return
        valid = {name for name, _ in self._grip_param_mapping()}
        valid |= {"close_direction", "auto_save"}
        params = {
            key: value for key, value in self._grip_restore.items()
            if key in valid
        }
        if not params:
            QTimer.singleShot(0, self._grip_read_driver_params)
            return
        self._grip_pending_names = list(params.keys())
        self._grip_log("正在把上次保存的夹爪参数写入驱动…")
        if not self.bridge.gripper_set_parameters(
            params, cb=lambda res: self.sig_grip_set.emit(res)
        ):
            self._grip_params_loaded = False

    def _grip_read_driver_params(self):
        if not self.bridge.cli_grip_get_params.service_is_ready():
            self._grip_params_loaded = False
            return
        mapping = self._grip_param_mapping()
        names = [key for key, _ in mapping] + [
            "close_direction", "auto_save",
        ]
        if self.bridge.gripper_get_parameters(
            names, cb=lambda vals: self.sig_grip_params.emit(vals)
        ):
            self._grip_log("正在读取驱动参数…")

    def _on_grip_params(self, values):
        mapping = self._grip_param_mapping()
        names = [key for key, _ in mapping] + [
            "close_direction", "auto_save",
        ]
        by_name = {
            name: value for name, value in zip(names, values)
        }
        for key, spin in mapping:
            value = by_name.get(key)
            if value is None:
                continue
            if value.type == ParameterType.PARAMETER_DOUBLE:
                spin.setValue(float(value.double_value))
            elif value.type == ParameterType.PARAMETER_INTEGER:
                spin.setValue(float(value.integer_value))
        value = by_name.get("close_direction")
        if (value is not None
                and value.type == ParameterType.PARAMETER_DOUBLE):
            self.cb_grip_dir.setCurrentIndex(
                0 if value.double_value > 0.0 else 1
            )
        value = by_name.get("auto_save")
        if (value is not None
                and value.type == ParameterType.PARAMETER_BOOL):
            self.chk_grip_autosave.setChecked(bool(value.bool_value))
        self._grip_log("已从驱动读取夹爪参数")

    def on_grip_apply_params(self):
        if not self.bridge.cli_grip_set_params.service_is_ready():
            QMessageBox.warning(self, "夹爪", "夹爪驱动未连接。")
            return
        params = {
            "kp_low": self.sp_grip_kp_low.value(),
            "kd_low": self.sp_grip_kd_low.value(),
            "kp_high": self.sp_grip_kp_high.value(),
            "kd_high": self.sp_grip_kd_high.value(),
            "contact_tau": self.sp_grip_tau.value(),
            "contact_pos_err": self.sp_grip_pos_err.value(),
            "close_speed": self.sp_grip_speed.value(),
            "max_travel": self.sp_grip_travel.value(),
            "timeout": self.sp_grip_timeout.value(),
            "open_q": self.sp_grip_open_q.value(),
            "close_q": self.sp_grip_close_q.value(),
            "q_min": self.sp_grip_qmin.value(),
            "q_max": self.sp_grip_qmax.value(),
            "close_direction": float(self.cb_grip_dir.currentData()),
            "auto_save": bool(self.chk_grip_autosave.isChecked()),
        }
        self._grip_pending_names = list(params.keys())
        if self.bridge.gripper_set_parameters(
            params, cb=lambda res: self.sig_grip_set.emit(res)
        ):
            self._grip_log("正在应用参数…")

    def _on_grip_set_result(self, res):
        names = getattr(self, "_grip_pending_names", [])
        failed = []
        for i, result in enumerate(res.results):
            if not result.successful:
                name = names[i] if i < len(names) else "?"
                failed.append("%s: %s" % (name, result.reason))
        if failed:
            self._grip_log("应用失败: " + "; ".join(failed))
        else:
            self._grip_log("夹爪参数已应用")
            self._grip_read_driver_params()

    def _check_motion_ready(self) -> bool:
        if self._rec_active or self._replay_active:
            self.bridge.log("录制/回放中，先停止再执行其他运动")
            return False
        active = [i for i in range(self.n) if self.bridge.active_mask[i]]
        if len(active) != 6:
            QMessageBox.warning(
                self,
                "注册表不完整",
                "关节/轨迹/回 fold/逆解需要注册全部 6 轴；"
                "子集请用『单电机 MIT』。",
            )
            return False
        if not all(
            self.bridge.enabled_each[i] and self.bridge.sending_each[i]
            for i in active
        ):
            QMessageBox.warning(
                self,
                "未就绪",
                "先在注册表里让全部 6 轴『使能』并『开始发送』。",
            )
            return False
        if not (self.bridge.driver_enabled and self.bridge.driver_sending):
            QMessageBox.warning(
                self,
                "驱动未就绪",
                "驱动实际未使能或未发送；先检查驱动状态，再运动。",
            )
            return False
        if self.bridge.motor_age() > 0.5:
            QMessageBox.warning(
                self, "反馈过期", "电机反馈超过 0.5 s 未更新，拒绝运动。"
            )
            return False
        vel_joints = [
            f"J{i + 1}"
            for i, mode in enumerate(self.bridge.modes)
            if self.bridge.active_mask[i] and mode == "VEL"
        ]
        if vel_joints:
            QMessageBox.warning(
                self,
                "模式不匹配",
                "这些关节处于 VEL 模式，角度目标不适用: " + ", ".join(vel_joints),
            )
            return False
        return True

    def on_single_read(self):
        j = self.cb_single.currentIndex()
        q = self.bridge.current_q()
        self.sp_single_target.setValue(float(q[j]))

    def on_single_go(self):
        if not self._check_motion_ready():
            return
        self._suspend_gamepad()
        j = self.cb_single.currentIndex()
        mode = self.bridge.modes[j] if j < len(self.bridge.modes) else "?"
        if mode == "VEL":
            QMessageBox.warning(
                self, "VEL 模式", "速度模式不接受角度目标；请先切回 MIT/POS_VEL。"
            )
            return
        q = self.bridge.current_q()
        target = q.copy()
        raw = float(self.sp_single_target.value())
        target[j] = float(
            min(max(raw, self.joint_limits[j, 0]), self.joint_limits[j, 1])
        )
        if target[j] != raw:
            self.bridge.log(f"J{j + 1} 目标超出软件限位，已裁剪")
        delta = abs(target[j] - q[j])
        duration = max(0.8, delta / max(0.05, self.sp_speed.value()))
        self.bridge.move_to(target, duration)

    def on_apply_mode(self):
        cid = self._mit_cid()
        if not cid:
            QMessageBox.warning(self, "未注册", "上方『电机』下拉框没有注册电机。")
            return
        i = self.bridge._slot_for_can(cid)
        if i is not None and (
            self.bridge.enabled_each[i] or self.bridge.sending_each[i]
        ):
            QMessageBox.warning(
                self, "拒绝", "先失能该轴并停止发送，再切换模式。"
            )
            return
        self.bridge.set_mode(cid, self.cb_mode.currentText())

    def _param_can_id(self) -> int:
        cid = self._mit_cid()
        return int(cid) if cid else 0

    def on_param_read(self):
        cid = self._param_can_id()
        if not cid:
            QMessageBox.warning(self, "未注册", "该关节没有注册电机。")
            return
        rid, _is_int = self.cb_rid.currentData()
        self.lbl_param_result.setText("读取中…")

        def done(res):
            if res.success:
                value = res.value
                text = f"{int(round(value))} (0x{int(round(value)):X})" \
                    if res.is_integer else f"{value:.6f}"
            else:
                text = "失败"
            self.sig_param.emit(text)

        self.bridge.read_param(cid, rid, done)

    def on_param_write(self):
        cid = self._param_can_id()
        if not cid:
            QMessageBox.warning(self, "未注册", "该关节没有注册电机。")
            return
        rid, is_int = self.cb_rid.currentData()
        if rid == 10:
            QMessageBox.warning(
                self, "不能直接写", "CTRL_MODE 请用「应用模式」，否则驱动状态会不一致。"
            )
            return
        text = self.ed_param_value.text().strip()
        try:
            value = int(text, 0) if is_int else float(text)
        except ValueError:
            QMessageBox.warning(self, "输入有误", "写入值必须是数字。")
            return
        self.bridge.write_param(cid, rid, float(value), is_int)

    def on_param_save(self):
        cid = self._param_can_id()
        if not cid:
            QMessageBox.warning(self, "未注册", "该关节没有注册电机。")
            return
        i = self.bridge._slot_for_can(cid)
        if i is not None and (
            self.bridge.enabled_each[i] or self.bridge.sending_each[i]
        ):
            QMessageBox.warning(
                self, "先失能该轴", "保存 Flash 前先失能该轴并停止发送。"
            )
            return
        answer = QMessageBox.question(self, "保存", "把该电机参数写入 Flash？")
        if answer == QMessageBox.Yes:
            self.bridge.save_param(cid)

    def on_joint_read(self):
        q = self.bridge.current_q()
        for i in range(self.n):
            self.joint_targets[i].setValue(float(q[i]))

    def on_joint_go(self):
        if not self._check_motion_ready():
            return
        self._suspend_gamepad()
        q = self.bridge.current_q()
        raw = np.array([sp.value() for sp in self.joint_targets], dtype=float)
        target = np.clip(
            raw, self.joint_limits[:, 0], self.joint_limits[:, 1]
        )
        if not np.allclose(raw, target):
            bad = [
                f"J{i + 1}" for i in range(self.n)
                if abs(target[i] - raw[i]) > 1e-9
            ]
            self.bridge.log(
                "关节目标超出软件限位，已裁剪: " + ", ".join(bad)
            )
        max_delta = float(np.max(np.abs(target - q)))
        if max_delta > 0.5:
            answer = QMessageBox.question(
                self,
                "确认运动",
                f"本次最大关节变化 {max_delta:.2f} rad。确认执行？",
            )
            if answer != QMessageBox.Yes:
                return
        duration = max(1.0, max_delta / max(0.05, self.sp_speed.value()))
        self.bridge.move_to(target, duration)

    def on_wp_record(self):
        previewing = self.preview.is_previewing()
        if previewing:
            err_mm, err_deg = self.preview.ik_error()
            if err_mm > 5.0 or err_deg > 5.0:
                QMessageBox.warning(
                    self,
                    "IK 残差大",
                    f"位置 {err_mm:.1f} mm / 姿态 {err_deg:.1f}°；"
                    "先把末端移离限位或换一个可达位姿再记录。",
                )
                return
        if not previewing:
            if self.bridge.motor_age() > 0.5:
                QMessageBox.warning(self, "反馈过期", "反馈不新鲜，等驱动收到电机数据再记录。")
                return
            if np.max(np.abs(self.bridge.current_q() - self.preview.q_preview())) > 0.02:
                QMessageBox.warning(
                    self,
                    "快照过期",
                    "3D 快照与实机位姿不一致，先点「同步实机姿态」，或直接在 3D 里拖动。",
                )
                return
        q = self.preview.q_preview()
        if not self._add_waypoint(q, "预览点" if previewing else "快照点"):
            QMessageBox.warning(
                self, "超出限位", "该点超出软件限位，未记录。"
            )

    def _add_waypoint(self, q, tag: str) -> bool:
        q = np.asarray(q, dtype=float).copy()
        ok, bad = self._q_within_limits(q)
        if not ok:
            self.bridge.log(
                f"{tag}: {', '.join(bad)} 超出软件限位，未记录"
            )
            return False
        self.waypoints.append(q)
        row = self.tbl_wp.rowCount()
        self.tbl_wp.insertRow(row)
        self.tbl_wp.setItem(row, 0, QTableWidgetItem(str(row + 1)))
        for i in range(self.n):
            item = QTableWidgetItem(f"{q[i]:+.4f}")
            item.setTextAlignment(Qt.AlignCenter)
            self.tbl_wp.setItem(row, i + 1, item)
        self.bridge.log(
            f"记录{tag} {row + 1}: " + ", ".join(f"{v:+.3f}" for v in q)
        )
        return True

    def record_measured_waypoint(self) -> bool:
        if self.bridge.motor_age() > 0.5 or self.bridge.feedback_age() > 0.5:
            self.bridge.log("手柄 X：反馈不新鲜，未记录")
            return False
        return self._add_waypoint(self.bridge.current_q(), "手柄点")

    def _dwell_seconds(self) -> float:
        if not self.chk_dwell.isChecked():
            return 0.0
        return max(0.0, float(self.sp_dwell.value()))

    def _handle_pad_waypoints(self, ready: bool) -> None:
        if not self.chk_gamepad.isChecked():
            self._pad_buttons_prev = self.gamepad.buttons
            return
        buttons = int(self.gamepad.buttons)
        prev = int(self._pad_buttons_prev)
        self._pad_buttons_prev = buttons
        x_rise = bool(buttons & 0x4000) and not bool(prev & 0x4000)
        y_rise = bool(buttons & 0x8000) and not bool(prev & 0x8000)
        if x_rise:
            if not ready:
                self.bridge.log("手柄 X：先使能并开始控制，再记录中途点")
            else:
                self.record_measured_waypoint()
        if y_rise:
            self.on_wp_run(False)

    def on_wp_delete(self):
        rows = sorted({idx.row() for idx in self.tbl_wp.selectedIndexes()}, reverse=True)
        for row in rows:
            self.tbl_wp.removeRow(row)
            if 0 <= row < len(self.waypoints):
                self.waypoints.pop(row)
        for i in range(self.tbl_wp.rowCount()):
            self.tbl_wp.setItem(i, 0, QTableWidgetItem(str(i + 1)))

    def on_wp_clear(self):
        if not self.waypoints:
            return
        if QMessageBox.question(self, "清空", "删除全部轨迹点？") == QMessageBox.Yes:
            self.waypoints.clear()
            self.tbl_wp.setRowCount(0)

    def on_wp_run(self, confirm=True):
        if not self.waypoints:
            if confirm:
                QMessageBox.warning(self, "没有轨迹点", "先记录至少一个点。")
            else:
                self.bridge.log("手柄 Y：还没有轨迹点")
            return
        if not self._check_motion_ready():
            return
        bad_points = []
        for idx, wp in enumerate(self.waypoints):
            ok, bad = self._q_within_limits(wp)
            if not ok:
                bad_points.append(f"第{idx + 1}点 " + "/".join(bad))
        if bad_points:
            msg = "轨迹点超出当前软件限位: " + "; ".join(bad_points)
            if confirm:
                QMessageBox.warning(self, "超出限位", msg)
            else:
                self.bridge.log("手柄 Y：" + msg)
            return
        ok, bad = self._q_within_limits(self.bridge.current_q())
        if not ok:
            msg = "当前实测角在软件限位外: " + ", ".join(bad)
            if confirm:
                QMessageBox.warning(self, "超出限位", msg)
            else:
                self.bridge.log("手柄 Y：" + msg)
            return
        self._suspend_gamepad()
        dwell = self._dwell_seconds()
        if confirm:
            dwell_text = "连续经过各点" if dwell <= 0.0 else f"每点停顿 {dwell:.1f} s"
            answer = QMessageBox.question(
                self, "执行轨迹",
                f"将执行 {len(self.waypoints)} 个点（{dwell_text}）。\n"
                "不做碰撞检测，确认周围安全并开始？",
            )
            if answer != QMessageBox.Yes:
                return
        speed = max(0.05, self.sp_speed.value())
        q_now = self.bridge.current_q()
        self.preview.reset_to_real(q_now)
        durations = []
        prev = q_now
        for wp in self.waypoints:
            delta = float(np.max(np.abs(wp - prev)))
            durations.append(max(0.8, delta / speed))
            prev = wp
        self.bridge.move_to(self.waypoints[0], durations[0], dwell)
        for wp, dur in zip(self.waypoints[1:], durations[1:]):
            self.bridge.append_waypoint(wp, dur, dwell)
        dwell_text = "连续" if dwell <= 0.0 else f"每点停 {dwell:.1f} s"
        self.bridge.log(
            f"开始执行 {len(self.waypoints)} 个轨迹点（{dwell_text}）"
        )

    # ---------------------------------------------------- 拖动录制 / 轨迹复现
    def _update_rec_buttons(self):
        if not hasattr(self, "btn_rec_start"):
            return
        rec = bool(self._rec_active)
        rep = bool(self._replay_active)
        has = len(self._rec_q) >= 5 and not self._rec_bad
        self.btn_rec_start.setEnabled(not rec and not rep)
        self.btn_rec_stop.setEnabled(rec)
        self.btn_replay.setEnabled(not rec and not rep and has)
        self.btn_replay_stop.setEnabled(rep)
        self.btn_rec_clear.setEnabled(
            not rec and not rep and bool(self._rec_q))
        busy = rec or rep
        for name in (
            "cb_traj", "btn_traj_refresh", "btn_traj_delete",
            "cb_replay_mode", "btn_mode_switch",
        ):
            w = getattr(self, name, None)
            if w is not None:
                w.setEnabled(not busy)
        if hasattr(self, "btn_traj_save"):
            self.btn_traj_save.setEnabled(not busy and has)
        if hasattr(self, "btn_drag"):
            self.btn_drag.setEnabled(not rec and not rep)
        if hasattr(self, "grp_ff"):
            self.grp_ff.setEnabled(not rec and not rep)

    def _set_rec_status(self, text: str):
        self._set_text(self.lbl_rec, text)

    # ---------------------------------------------------------- 轨迹库（磁盘）
    @staticmethod
    def _safe_traj_name(name: str) -> str:
        text = "".join(
            ch for ch in str(name).strip()
            if ch.isalnum() or ch in "-_."
        )
        return text.strip("._") or "traj"

    @staticmethod
    def _traj_dir() -> Path:
        hmi_store.TRAJ_DIR.mkdir(parents=True, exist_ok=True)
        return hmi_store.TRAJ_DIR

    def _traj_files(self):
        try:
            files = [
                p for p in self._traj_dir().glob("*.json") if p.is_file()
            ]
        except OSError:
            return []
        return sorted(
            files, key=lambda p: p.stat().st_mtime, reverse=True
        )

    def _refresh_traj_list(self, select=None):
        """Rebuild the saved-trajectory combo (newest first)."""
        if not hasattr(self, "cb_traj"):
            return
        if select is None:
            select = self.cb_traj.currentData()
        files = self._traj_files()
        self._traj_list_guard = True
        try:
            self.cb_traj.clear()
            for path in files:
                meta = hmi_store.load_path(path) or {}
                label = str(meta.get("name") or path.stem)
                duration = float(meta.get("duration", 0.0) or 0.0)
                count = len(meta.get("samples") or [])
                self.cb_traj.addItem(
                    f"{label}  ({duration:.1f} s / {count} 点)", str(path)
                )
            idx = self.cb_traj.findData(str(select)) if select else -1
            self.cb_traj.setCurrentIndex(idx)
        finally:
            self._traj_list_guard = False

    def _compute_velocities(self, times, qs):
        """Central-difference dq for each sample, lightly smoothed."""
        n = len(times)
        if n < 2:
            return [np.zeros(self.n) for _ in range(n)]
        vel = []
        for i in range(n):
            i0 = max(0, i - 1)
            i1 = min(n - 1, i + 1)
            dt = float(times[i1] - times[i0])
            if dt <= 1e-6:
                vel.append(np.zeros(self.n))
            else:
                vel.append(
                    (np.asarray(qs[i1]) - np.asarray(qs[i0])) / dt
                )
        if n >= 3:
            smoothed = []
            for i in range(n):
                a = vel[max(0, i - 1)]
                b = vel[i]
                c = vel[min(n - 1, i + 1)]
                smoothed.append((a + b + c) / 3.0)
            vel = smoothed
        return [np.asarray(v, dtype=float) for v in vel]

    def _traj_payload(self, name: str) -> dict:
        if len(self._rec_q) < 5:
            raise ValueError("轨迹点太少")
        t0 = float(self._rec_t[0])
        samples = []
        for t, q in zip(self._rec_t, self._rec_q):
            q = np.asarray(q, dtype=float).reshape(-1)[: self.n]
            samples.append(
                [round(float(t) - t0, 6)]
                + [round(float(v), 6) for v in q]
            )
        return {
            "version": 1,
            "name": str(name),
            "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "joint_names": list(JOINTS[: self.n]),
            "duration": round(float(self._rec_t[-1] - t0), 6),
            "samples": samples,
        }

    def _save_trajectory(self, name=None, auto: bool = False) -> str:
        name = name or time.strftime("drag-%Y%m%d-%H%M%S")
        safe = self._safe_traj_name(name)
        folder = self._traj_dir()
        path = folder / f"{safe}.json"
        k = 2
        while path.exists():
            path = folder / f"{safe}-{k}.json"
            k += 1
        data = self._traj_payload(path.stem)
        hmi_store.save_path(path, data)
        self._rec_name = str(data["name"])
        self._rec_path = str(path)
        self._refresh_traj_list(select=str(path))
        self.bridge.log(
            ("已自动保存轨迹: " if auto else "已保存轨迹: ") + str(path)
        )
        return str(path)

    def on_traj_save_as(self):
        if len(self._rec_q) < 5:
            QMessageBox.warning(self, "保存轨迹", "当前没有可保存的轨迹。")
            return
        default = self._rec_name or time.strftime("drag-%Y%m%d-%H%M%S")
        text, ok = QInputDialog.getText(
            self, "保存轨迹", "轨迹名称:", text=default
        )
        if not ok or not str(text).strip():
            return
        try:
            self._save_trajectory(str(text).strip())
        except Exception as exc:
            QMessageBox.warning(self, "保存轨迹", f"写入失败: {exc}")

    def _load_trajectory(self, path) -> bool:
        data = hmi_store.load_path(path)
        if not isinstance(data, dict):
            QMessageBox.warning(self, "载入轨迹", f"文件无法解析:\n{path}")
            return False
        samples = data.get("samples")
        if not isinstance(samples, list) or len(samples) < 5:
            QMessageBox.warning(self, "载入轨迹", "轨迹为空或点太少。")
            return False
        ts, qs = [], []
        for row in samples:
            try:
                vals = [float(v) for v in row]
            except (TypeError, ValueError):
                continue
            if (len(vals) < self.n + 1
                    or not all(math.isfinite(v) for v in vals)):
                continue
            ts.append(vals[0])
            qs.append(np.asarray(vals[1: 1 + self.n], dtype=float))
        if len(ts) < 5:
            QMessageBox.warning(self, "载入轨迹", "有效点太少，文件已损坏。")
            return False
        order = np.argsort(np.asarray(ts))
        ts = [float(ts[i]) for i in order]
        qs = [qs[i] for i in order]
        t0 = ts[0]
        ts = [max(0.0, t - t0) for t in ts]
        self._rec_q = qs
        self._rec_t = ts
        self._rec_vel = self._compute_velocities(ts, qs)
        self._rec_bad = []
        self._rec_name = str(data.get("name") or Path(path).stem)
        self._rec_path = str(path)
        self._set_rec_status(
            "已载入: %s · %d 点 / %.2f s"
            % (self._rec_name, len(ts), ts[-1])
        )
        self.bridge.log(f"已载入轨迹: {self._rec_name} ({path})")
        self._update_rec_buttons()
        return True

    def _on_traj_selected(self, idx: int):
        if getattr(self, "_traj_list_guard", False) or idx < 0:
            return
        if self._rec_active or self._replay_active:
            self.bridge.log("录制/回放中，暂不切换轨迹")
            return
        path = self.cb_traj.itemData(idx)
        if path:
            self._load_trajectory(path)

    def on_traj_delete(self):
        path = self.cb_traj.currentData()
        if not path:
            QMessageBox.warning(self, "删除轨迹", "先在列表里选中一条轨迹。")
            return
        label = self.cb_traj.currentText()
        if QMessageBox.question(
            self, "删除轨迹", f"删除这条轨迹？\n{label}"
        ) != QMessageBox.Yes:
            return
        try:
            os.remove(path)
        except OSError as exc:
            QMessageBox.warning(self, "删除轨迹", f"删除失败: {exc}")
            return
        if getattr(self, "_rec_path", "") == path:
            self._rec_q, self._rec_t, self._rec_vel = [], [], []
            self._rec_bad, self._rec_name, self._rec_path = [], "", ""
            self._set_rec_status("未录制")
        self._refresh_traj_list()
        self._update_rec_buttons()
        self.bridge.log(f"已删除轨迹: {path}")

    def on_rec_start(self):
        if self._rec_active or self._replay_active:
            return
        if getattr(self, "_ident_phase", "idle") != "idle":
            QMessageBox.warning(
                self, "辨识进行中", "先停止辨识，再开始拖动录制。")
            return
        if not self._check_motion_ready():
            return
        if not (self.chk_gravity.isChecked()
                and self.bridge.gravity_enabled):
            QMessageBox.warning(
                self, "需要重力前馈",
                "拖动录制和停止后的保持需要重力补偿。\n"
                "请先在「关节」页勾选「重力补偿」并应用，再开始。",
            )
            return
        active, pending = self.bridge.motion.state()
        if active or pending:
            QMessageBox.warning(
                self, "正在运动", "先等待当前轨迹结束，再开始录制。")
            return
        q = self.bridge.current_q()
        if not np.all(np.isfinite(q)):
            QMessageBox.warning(
                self, "反馈异常", "当前关节角无效，不能录制。")
            return
        self._suspend_gamepad()
        self._rec_q = []
        self._rec_t = []
        self._rec_vel = []
        self._rec_bad = []
        self._rec_name = ""
        self._rec_path = ""
        self._rec_stale = 0
        self._rec_t0 = time.monotonic()
        self._rec_last_warn = 0.0
        self._rec_active = True
        self.btn_drag.blockSignals(True)
        self.btn_drag.setChecked(True)
        self.btn_drag.blockSignals(False)
        self.bridge.drag_teach(True)
        self.rec_timer.start()
        self._update_rec_buttons()
        self._set_rec_status("录制中: 0.0 s / 0 点")
        self.bridge.log(
            "开始拖动录制：拖动机械臂，完成后点「停止记录」。")

    def _rec_tick(self):
        if not self._rec_active:
            self.rec_timer.stop()
            return
        if not (self.bridge.enabled and self.bridge.sending
                and self.bridge.driver_enabled
                and self.bridge.driver_sending):
            self.bridge.log("录制停止：驱动未使能或未发送")
            self.on_rec_stop()
            return
        if (self.bridge.feedback_age() > 0.5
                or self.bridge.motor_age() > 0.5):
            self._rec_stale += 1
            if (self._rec_stale == 1
                    or time.monotonic() - self._rec_last_warn > 1.0):
                self._rec_last_warn = time.monotonic()
                self.bridge.log("录制中：反馈不新鲜，暂停记录该帧")
            if self._rec_stale > 50:
                self.bridge.log("录制停止：反馈持续超时")
                self.on_rec_stop()
            return
        self._rec_stale = 0
        q = self.bridge.current_q().copy()
        if not np.all(np.isfinite(q)):
            return
        t = time.monotonic() - self._rec_t0
        self._rec_t.append(t)
        self._rec_q.append(q)
        self._set_rec_status(
            "录制中: %.1f s / %d 点" % (t, len(self._rec_q)))
        if t > 300.0:
            self.bridge.log("录制超过 300 s，自动停止")
            self.on_rec_stop()

    def on_rec_stop(self):
        if not self._rec_active:
            return
        self.rec_timer.stop()
        self._rec_active = False
        self.bridge.drag_teach(False)
        self.btn_drag.blockSignals(True)
        self.btn_drag.setChecked(False)
        self.btn_drag.blockSignals(False)
        self.btn_drag.setEnabled(True)
        n = len(self._rec_q)
        if n < 5:
            self._rec_q = []
            self._rec_t = []
            self._rec_vel = []
            self._rec_bad = []
            self._rec_name = ""
            self._rec_path = ""
            self._set_rec_status("录制无效：点太少（%d）" % n)
            self.bridge.log("录制无效：只有 %d 点" % n)
        else:
            # 手动拖动时 J2/J3 在模型限位 0 附近会因重力下垂/反馈噪声
            # 越过一点。J2/J3 给 0.10 rad 容差，其余关节 0.01 rad；
            # 容差内的点夹回软件限位，避免整段回放因边界采样点被禁用。
            tol = np.array([0.01, 0.10, 0.10, 0.01, 0.01, 0.01])[:self.n]
            self._rec_bad = []
            repaired = 0
            for idx, q in enumerate(self._rec_q):
                q_arr = np.asarray(q, dtype=float)
                lo = self.joint_limits[:, 0]
                hi = self.joint_limits[:, 1]
                q_cl = np.clip(q_arr, lo, hi)
                over = np.abs(q_arr - q_cl)
                bad = [
                    f"J{i + 1}" for i in range(self.n)
                    if over[i] > tol[i] + 1e-6
                ]
                if not bad:
                    if np.any(over > 1e-6):
                        self._rec_q[idx] = q_cl
                        repaired += 1
                    continue
                self._rec_bad.append((idx, bad, float(over.max())))
            span = max(0.0, self._rec_t[-1] - self._rec_t[0])
            if repaired:
                self.bridge.log(
                    "录制边界点已夹回限位：%d 个（J2/J3 容差 0.10 rad，其余 0.01 rad）"
                    % repaired)
            if self._rec_bad:
                self._rec_vel = []
                self._rec_name = ""
                self._rec_path = ""
                idx, bad, over = self._rec_bad[0]
                self._set_rec_status(
                    "录制完成：%d 点，含超限点（回放禁用）" % n)
                self.bridge.log(
                    "录制完成但有点位超限：第 %d 点 %s，最大超出 %.3f rad；回放已禁用"
                    % (idx + 1, "/".join(bad), over))
            else:
                self._rec_vel = self._compute_velocities(
                    self._rec_t, self._rec_q
                )
                self._rec_name = time.strftime("drag-%Y%m%d-%H%M%S")
                self._rec_path = ""
                status = "录制完成：%d 点 / %.2f s" % (n, span)
                if repaired:
                    status += "（修正 %d 个边界点）" % repaired
                if (hasattr(self, "chk_traj_autosave")
                        and self.chk_traj_autosave.isChecked()):
                    try:
                        path = self._save_trajectory(self._rec_name, auto=True)
                        status += " · 已保存 %s" % os.path.basename(path)
                    except Exception as exc:
                        self.bridge.log(f"自动保存轨迹失败: {exc}")
                self._set_rec_status(status)
                self.bridge.log(status)
        self._update_rec_buttons()

    def on_rec_clear(self):
        if self._rec_active:
            self.on_rec_stop()
        if self._replay_active:
            self._finish_replay("清除录制", abort=True)
        self._rec_q = []
        self._rec_t = []
        self._rec_vel = []
        self._rec_bad = []
        self._rec_name = ""
        self._rec_path = ""
        self._set_rec_status("未录制")
        self._update_rec_buttons()

    def on_replay_start(self):
        if self._rec_active or self._replay_active:
            return
        if len(self._rec_q) < 5:
            QMessageBox.warning(
                self, "没有录制", "先拖动录制一段轨迹，再复现。")
            return
        if self._rec_bad:
            QMessageBox.warning(
                self, "轨迹超限", "录制轨迹含超限点，不能复现。")
            return
        bad = []
        for idx, q in enumerate(self._rec_q):
            ok, badj = self._q_within_limits(q)
            if not ok:
                bad.append((idx, badj))
        self._rec_bad = bad
        if bad:
            idx, badj = bad[0]
            QMessageBox.warning(
                self, "轨迹超限",
                "按当前软件限位，第 %d 点超限: %s\n"
                "请调整限位或重新录制。" % (idx + 1, "/".join(badj)),
            )
            self._update_rec_buttons()
            return
        if getattr(self, "_ident_phase", "idle") != "idle":
            QMessageBox.warning(
                self, "辨识进行中", "先停止辨识，再开始轨迹复现。")
            return
        if not self._check_motion_ready():
            return
        ok, bad_now = self._q_within_rec_limits(self.bridge.current_q())
        if not ok:
            QMessageBox.warning(
                self, "当前位姿超限",
                "当前实测角在软件限位外: " + ", ".join(bad_now),
            )
            return
        replay_mode = str(self.cb_replay_mode.currentData() or "MIT")
        active_slots = [
            i for i in range(self.n) if self.bridge.active_mask[i]
        ]
        wrong = [
            f"J{i + 1}:{self.bridge.modes[i]}"
            for i in active_slots
            if self.bridge.modes[i] != replay_mode
        ]
        if wrong:
            QMessageBox.warning(
                self, "模式不匹配",
                f"回放模式是 {replay_mode}，但这些轴不是：\n"
                + ", ".join(wrong)
                + "\n先点「切换 6 轴模式」，或把回放模式改成与驱动一致。",
            )
            return
        if replay_mode == "MIT" and not (
                self.chk_gravity.isChecked()
                and self.bridge.gravity_enabled):
            QMessageBox.warning(
                self, "需要重力前馈",
                "MIT 轨迹复现需要重力补偿。\n"
                "请先在「关节」页勾选「重力补偿」并应用。\n"
                "（POS_VEL 模式不需要重力前馈。）",
            )
            return
        active, pending = self.bridge.motion.state()
        if active or pending:
            QMessageBox.warning(
                self, "正在运动", "先等待当前轨迹结束，再开始复现。")
            return
        span = max(0.0, self._rec_t[-1] - self._rec_t[0])
        answer = QMessageBox.question(
            self,
            "轨迹复现",
            f"将先运动到录制起点，再以 {replay_mode} 模式回放 "
            f"{span:.2f} s 的轨迹。\n"
            "不做碰撞检测，确认周围安全并继续？",
        )
        if answer != QMessageBox.Yes:
            return
        self._suspend_gamepad()
        self._replay_mode = replay_mode
        t0 = self._rec_t[0]
        self._replay_t = [max(0.0, t - t0) for t in self._rec_t]
        self._replay_q = [np.asarray(q).copy() for q in self._rec_q]
        if len(self._rec_vel) != len(self._rec_q):
            self._rec_vel = self._compute_velocities(
                self._rec_t, self._rec_q
            )
        self._replay_vel = (
            [np.asarray(v, dtype=float).copy() for v in self._rec_vel]
            if replay_mode == "POS_VEL" else []
        )
        self._replay_target0 = self._replay_q[0].copy()
        q_now = self.bridge.current_q().copy()
        delta = float(np.max(np.abs(self._replay_target0 - q_now)))
        approach_dur = max(1.0, delta / 0.3)
        self._replay_approach_dur = approach_dur
        self._replay_approach_t0 = time.monotonic()
        self._replay_seen_motion = False
        self._replay_i = 0
        self._replay_t0 = 0.0
        self._replay_phase = "approach"
        self._replay_active = True
        self.btn_drag.setEnabled(False)
        self._update_rec_buttons()
        self._set_rec_status("正在运动到录制起点…")
        self.bridge.log(
            "轨迹复现：先运动到录制起点（%.1f s）" % approach_dur)
        if not self.bridge.move_to(self._replay_target0, approach_dur):
            self._finish_replay("预移动被拒绝", abort=True)
            return
        self.replay_timer.start()

    def on_replay_stop(self):
        if self._replay_active:
            self._finish_replay("用户停止", abort=True)

    def on_switch_all_modes(self):
        """把全部注册轴切到回放模式（停止发送 → 失能 → 切模式 → 恢复）。"""
        if self._rec_active or self._replay_active:
            self.bridge.log("录制/回放中，不能切换模式")
            return
        if not self.btn_mode_switch.isEnabled():
            return   # 上一次切换还没走完
        mode = str(self.cb_replay_mode.currentData() or "MIT")
        cids = [int(c) for c in self.bridge.registered_can_ids()]
        if not cids:
            QMessageBox.warning(self, "切换模式", "电机注册表为空。")
            return
        slots = [i for i in range(self.n) if self.bridge.active_mask[i]]
        if slots and all(self.bridge.modes[i] == mode for i in slots):
            self.bridge.log(f"已注册的轴都已是 {mode}，无需切换")
            return
        if not self.bridge.cli_mode.service_is_ready():
            QMessageBox.warning(self, "切换模式", "驱动未连接。")
            return
        if QMessageBox.question(
            self, "切换 6 轴模式",
            f"将把 {len(cids)} 轴全部切到 {mode}。\n\n"
            "流程：停止发送 → 失能 → 切模式 → 重新使能 → 开始发送。\n"
            "失能期间没有保持力矩，机械臂可能因重力下落，请先扶稳。\n\n"
            "继续？",
        ) != QMessageBox.Yes:
            return
        self.btn_mode_switch.setEnabled(False)
        self.bridge.log(f"开始切换 {len(cids)} 轴到 {mode}…")
        b = self.bridge

        def unlock():
            if not self.btn_mode_switch.isEnabled():
                self.btn_mode_switch.setEnabled(True)
                self.bridge.log("模式切换超时或中断，请检查驱动状态")

        # 兜底：任何一步没回调都把按钮放出来
        QTimer.singleShot(15000, unlock)

        state = {"pending": len(cids), "failed": []}

        def on_all_modes():
            if state["failed"]:
                self.bridge.log(
                    "切模式失败: " + "; ".join(state["failed"])
                    + "；保持失能，请检查驱动日志"
                )
                self.btn_mode_switch.setEnabled(True)
                return
            self.bridge.log(f"已切到 {mode}，正在重新使能…")

            def on_enabled(res):
                if not res.success:
                    self.bridge.log("重新使能失败: " + str(res.message))
                    self.btn_mode_switch.setEnabled(True)
                    return

                def on_sending(res2):
                    if res2.success:
                        self.bridge.modes = [mode] * len(self.bridge.modes)
                        self.bridge.publish_setup()
                        self.bridge.log("已重新使能并开始发送")
                    else:
                        self.bridge.log(
                            "开始发送失败: " + str(res2.message))
                    self.btn_mode_switch.setEnabled(True)

                b.set_motor_sending(cids, True, all_motors=True, cb=on_sending)

            b.set_motor_enabled(cids, True, all_motors=True, cb=on_enabled)

        def on_mode(res):
            if not res.success:
                state["failed"].append(str(res.message))
            state["pending"] -= 1
            if state["pending"] <= 0:
                on_all_modes()

        def on_disabled(res):
            if not res.success:
                self.bridge.log("失能失败: " + str(res.message))
                self.btn_mode_switch.setEnabled(True)
                return
            state["pending"] = len(cids)
            for cid in cids:
                b.set_mode(cid, mode, cb=on_mode)

        def on_stopped(res):
            b.set_motor_enabled(cids, False, all_motors=True, cb=on_disabled)

        b.set_motor_sending(cids, False, all_motors=True, cb=on_stopped)

    def _finish_replay(self, reason: str, abort: bool = False):
        if not self._replay_active:
            return
        self.replay_timer.stop()
        self._replay_active = False
        self._replay_phase = "idle"
        self._replay_q = []
        self._replay_t = []
        self._replay_vel = []
        self._replay_i = 0
        self.btn_drag.setEnabled(True)
        self._update_rec_buttons()
        if abort:
            self.bridge.stop_motion()
        self._set_rec_status(
            "复现%s：%s" % ("中止" if abort else "完成", reason))
        self.bridge.log(
            "轨迹复现%s：%s" % ("中止" if abort else "完成", reason))

    def _replay_tick(self):
        if not self._replay_active:
            self.replay_timer.stop()
            return
        if not (self.bridge.enabled and self.bridge.sending
                and self.bridge.driver_enabled
                and self.bridge.driver_sending):
            self._finish_replay("驱动未使能或未发送", abort=True)
            return
        if (self.bridge.feedback_age() > 0.5
                or self.bridge.motor_age() > 0.5):
            self._finish_replay("反馈超时", abort=True)
            return
        if self._replay_phase == "approach":
            active, pending = self.bridge.motion.state()
            if active:
                self._replay_seen_motion = True
            q_now = self.bridge.current_q()
            elapsed = time.monotonic() - self._replay_approach_t0
            close = float(np.max(
                np.abs(q_now - self._replay_target0))) <= 0.10
            if (self._replay_seen_motion and not active
                    and pending == 0 and close):
                self._replay_phase = "play"
                self._replay_t0 = time.monotonic()
                self._replay_i = 0
                self.bridge.log("到达录制起点，开始回放")
                self._set_rec_status(
                    "回放中: 0.0 / %.2f s" % self._replay_t[-1])
            elif elapsed > self._replay_approach_dur + 4.0:
                self._finish_replay("未到达录制起点", abort=True)
            return
        if self._replay_phase != "play":
            return
        active, pending = self.bridge.motion.state()
        if active or pending:
            self._finish_replay("检测到其他运动", abort=True)
            return
        pos_vel = self._replay_mode == "POS_VEL"
        zero_dq = np.zeros(self.n)
        elapsed = time.monotonic() - self._replay_t0
        if elapsed >= self._replay_t[-1]:
            self.bridge.set_direct(
                self._replay_q[-1].copy(), zero_dq if pos_vel else None
            )
            self._finish_replay("完成", abort=False)
            return
        while (self._replay_i + 1 < len(self._replay_t)
               and self._replay_t[self._replay_i + 1] <= elapsed):
            self._replay_i += 1
        i = self._replay_i
        dq = zero_dq if pos_vel else None
        if i + 1 >= len(self._replay_t):
            q = self._replay_q[-1].copy()
        else:
            ta = self._replay_t[i]
            tb = self._replay_t[i + 1]
            u = 0.0 if tb <= ta else (elapsed - ta) / (tb - ta)
            u = min(max(u, 0.0), 1.0)
            q = self._replay_q[i] * (1.0 - u) + self._replay_q[i + 1] * u
            if pos_vel and i + 1 < len(self._replay_vel):
                dq = (
                    self._replay_vel[i] * (1.0 - u)
                    + self._replay_vel[i + 1] * u
                )
        self.bridge.set_direct(q, dq)
        self._set_rec_status(
            "回放中: %.1f / %.2f s" % (elapsed, self._replay_t[-1]))

    def on_ik_read_current(self):
        p = self.kin.fk(self.bridge.current_q())
        self.sp_ik_x.setValue(float(p[0]))
        self.sp_ik_y.setValue(float(p[1]))
        self.sp_ik_z.setValue(float(p[2]))

    def _solve_ik(self):
        target = np.array(
            [self.sp_ik_x.value(), self.sp_ik_y.value(), self.sp_ik_z.value()]
        )
        q, err, iters = self.kin.ik(target, self.bridge.current_q())
        self.last_ik = (q, err)
        self.lbl_ik_result.setText(
            "解: "
            + ", ".join(f"{v:+.3f}" for v in q)
            + f"   误差 {err * 1000:.1f} mm / {iters} 次"
        )
        if err > 0.01:
            self.lbl_ik_result.setStyleSheet("color: #ebcb8b;")
        else:
            self.lbl_ik_result.setStyleSheet("color: #a3be8c;")
        return q, err

    def on_ik_solve(self):
        self._solve_ik()

    def on_ik_run(self):
        if not self._check_motion_ready():
            return
        self._suspend_gamepad()
        q, err = self._solve_ik()
        if err > 0.01:
            answer = QMessageBox.question(
                self,
                "逆解误差大",
                f"末端误差 {err * 1000:.1f} mm，仍要执行吗？",
            )
            if answer != QMessageBox.Yes:
                return
        q_now = self.bridge.current_q()
        delta = float(np.max(np.abs(q - q_now)))
        if delta > 0.5:
            answer = QMessageBox.question(
                self,
                "确认运动",
                f"逆解最大关节变化 {delta:.2f} rad。确认执行？",
            )
            if answer != QMessageBox.Yes:
                return
        duration = max(1.0, delta / max(0.05, self.sp_speed.value()))
        self.bridge.move_to(q, duration)

    # --------------------------------------------------------------- 生命周期
    def _release_on_exit(self) -> None:
        """Ask the driver to drop torque before the GUI process goes away."""
        bridge = self.bridge
        if not (
            bridge.enabled or bridge.sending
            or bridge.driver_enabled or bridge.driver_sending
        ):
            return
        if not bridge.cli_release.service_is_ready():
            bridge.log("关闭时驱动不在，未能自动失能")
            return
        fut = bridge.cli_release.call_async(Trigger.Request())
        deadline = time.monotonic() + 1.5
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not fut.done():
            bridge.log("关闭时失能超时，请确认驱动或按物理急停")
            return
        try:
            res = fut.result()
        except Exception as exc:
            bridge.log(f"关闭时失能失败: {exc}")
            return
        if res is not None and res.success:
            bridge.enabled = False
            bridge.sending = False
            bridge.driver_enabled = False
            bridge.driver_sending = False
            bridge.log("关闭窗口，已失能")
        else:
            bridge.log("关闭时失能被拒绝")

    def shutdown_ros(self):
        if self._closing:
            return
        self._closing = True
        self.bridge.stopping = True
        self.ui_timer.stop()
        self.plot_timer.stop()
        self.pad_timer.stop()
        self._gamepad_active = False
        self.gamepad.close()
        if hasattr(self, "preview"):
            try:
                self.preview.close()
            except Exception:
                pass
        self._ident_shutdown()
        self._stop_gripper_process()
        self._release_on_exit()
        self._stop_driver_process()
        try:
            self.executor.shutdown(timeout_sec=0.5)
        except Exception:
            pass
        try:
            self._spin_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.bridge.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def closeEvent(self, event):
        self.rec_timer.stop()
        self.replay_timer.stop()
        if self._rec_active:
            self.on_rec_stop()
        if self._replay_active:
            self._finish_replay("关闭窗口", abort=True)
        self._save_layout()
        self._save_registry()
        self.shutdown_ros()
        event.accept()


def _acquire_single_instance():
    try:
        lock = open("/tmp/zfy_arm_hmi.lock", "w")
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print("另一个 ZFY HMI 已在运行，退出")
        return None
    lock.write(str(os.getpid()))
    lock.flush()
    return lock


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", default=DEFAULT_SCENE)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--tab", type=int, default=0, help="启动时打开的页（0=基础）")
    args, ros_args = parser.parse_known_args()

    lock = _acquire_single_instance()
    if lock is None:
        return 1

    rclpy.init(args=ros_args)
    app = QApplication([sys.argv[0]])
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)
    families = load_cjk_font(app)
    print("字体:", families if families else "无中文字体（中文会显示为方块）")

    try:
        kin = ArmKinematics(args.scene)
    except Exception as exc:
        print(f"加载 MuJoCo 模型失败: {exc}")
        rclpy.shutdown()
        return 2

    window = MainWindow(kin, rate=args.rate)
    window.tabs.setCurrentIndex(max(0, min(5, int(args.tab))))
    window.show()

    signal.signal(signal.SIGINT, lambda *_a: app.quit())
    tick = QTimer()
    tick.start(200)
    tick.timeout.connect(lambda: None)

    code = app.exec_()
    window.shutdown_ros()
    return code


if __name__ == "__main__":
    sys.exit(main())
