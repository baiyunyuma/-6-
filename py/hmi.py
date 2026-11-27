#!/usr/bin/env python3
"""ZFY 达妙（DM）电机调试上位机 —— Linux 版。

界面与行为对齐 DM_DJI_TOOL_V1_src 的达妙部分：

  顶部        设备连接（设备类型 / 通信模式 / 设备索引 / 名义与数据波特率 /
              下发频率 / 扫描 / 连接 / 断开 / 释放设备 / 保存参数）
  页1 参数配置  电机配置（CAN ID / MST ID / 型号 / 模式 / 通道）
              参数读写（读取 / 写入 / 保存 Flash）
  页2 调试与反馈 ┌ 左（可滚动）                 右 ┐
              电机控制 / 信号源 / 前馈补偿 / 电机状态 | 四联反馈曲线

与旧版的关键差别：
  * 采样在 SDK 接收回调里按**真实反馈率**完成（core -> on_sample），
    界面只按自己的节拍拉取渲染 —— 丢的只是渲染帧，不会丢采样点；
  * 曲线拆成 位置 / 速度 / 力矩 / 前馈力矩 四个子图，峰值保持降采样；
  * 每台电机的设定值、前馈、限位、信号源、使能/发送状态互相独立；
  * 配置保存到同目录 config.json / layout.json。

必须用 ./run_hmi.sh 启动（它设 LD_LIBRARY_PATH）。
"""
import argparse
import math
import os
import sys
import threading
import time
from contextlib import contextmanager

import numpy as np
from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QFontDatabase
from PyQt5.QtWidgets import (
    QAbstractItemView, QApplication, QCheckBox, QComboBox, QDoubleSpinBox,
    QFileDialog, QGridLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel,
    QLineEdit, QMainWindow, QMessageBox, QPushButton, QScrollArea, QSpinBox,
    QSplitter, QTableWidget, QTableWidgetItem, QTabWidget, QTextEdit,
    QVBoxLayout, QWidget,
)

import config_store
import feedforward as ffmod
from hmi_plots import (
    CURVE_POINTS_MAX, CURVE_POINTS_MIN, CURVE_POINTS_PER_PX,
    RENDER_ADAPT_COOLDOWN_S, RENDER_INTERVAL_MAX_MS, RENDER_INTERVAL_MS,
    PlotPanel,
)
from motor_config import DEFAULT_CHANNEL, MODELS, MotorSpec
from telemetry import TELEM_CHANNELS, TelemetryBuffer

MODES = ["MIT", "POS_VEL", "VEL", "POS_FORCE"]

#: 每种模式下真正参与控制的指令字段
MODE_FIELDS = {
    "MIT":       ("kp", "kd", "q", "dq", "tau"),
    "POS_VEL":   ("pos", "vel"),
    "VEL":       ("vel",),
    "POS_FORCE": ("pos", "tau"),
}
FIELD_LABELS = {"kp": "kp", "kd": "kd", "q": "位置 q", "dq": "速度 dq",
                "tau": "力矩 tau", "pos": "位置 pos", "vel": "速度 vel"}
#: (下限, 上限, 默认, 步长)
FIELD_RANGES = {"kp": (0, 500, 0.0, 0.5), "kd": (0, 5, 0.0, 0.05),
                "q": (-12.5, 12.5, 0.0, 0.01), "dq": (-50, 50, 0.0, 0.01),
                "tau": (-200, 200, 0.0, 0.01), "pos": (-12.5, 12.5, 0.0, 0.01),
                "vel": (-50, 50, 0.0, 0.01)}

PARAM_REGS = [
    ("CAN ID (ESC_ID)", 8), ("回传 ID (MST_ID)", 7), ("加速度 (ACC)", 4),
    ("减速度 (DEC)", 5), ("速度环 KP (KP_ASR)", 25), ("速度环 KI (KI_ASR)", 26),
    ("位置环 KP (KP_APR)", 27), ("位置环 KI (KI_APR)", 28),
    ("控制模式 (CTRL_MODE)", 10), ("CAN 波特率 (can_br)", 35),
    ("超时 (TIMEOUT)", 9), ("最大速度 (MAX_SPD)", 6),
]

FF_GROUPS = [
    ("gravity", "重力补偿", [("gain", "增益", 1.0, -10, 10),
                            ("mass", "质量(kg)", 0.5, 0, 100),
                            ("com", "质心(m)", 0.1, 0, 10),
                            ("offset", "偏角(rad)", 0.0, -3.1416, 3.1416)]),
    ("friction", "摩擦补偿", [("coulomb", "库仑(N·m)", 0.1, 0, 100),
                             ("viscous", "粘性(N·m·s/rad)", 0.01, 0, 100),
                             ("slope", "tanh斜率", 100.0, 0, 10000)]),
    ("inertia", "惯性补偿", [("inertia", "惯量(kg·m²)", 0.01, 0, 10),
                            ("alpha", "滤波系数α", 0.2, 0, 1)]),
]

SIGNAL_FIELDS = {
    "sine": [("amplitude", "幅值(rad)", 0.5, 0, 12.5),
             ("freq", "频率(Hz)", 0.2, 0.001, 100),
             ("offset", "偏移(rad)", 0.0, -12.5, 12.5)],
    "ramp": [("start", "起点(rad)", 0.0, -12.5, 12.5),
             ("end", "终点(rad)", 1.5, -12.5, 12.5),
             ("duration", "时间(s)", 1.5, 0.01, 1000)],
}
CUSTOM_DEFAULT_EXPR = "2*x+10"

CJK_FONTS = ["/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
             "/mnt/c/Windows/Fonts/msyh.ttc", "/mnt/c/Windows/Fonts/simhei.ttf"]


def load_cjk_font(app):
    fams = []
    for p in CJK_FONTS:
        if os.path.exists(p):
            fid = QFontDatabase.addApplicationFont(p)
            if fid >= 0:
                fams.extend(QFontDatabase.applicationFontFamilies(fid)[:1])
    f = QFont()
    f.setFamilies(fams + ["DejaVu Sans"] if fams else ["DejaVu Sans"])
    f.setPointSize(10)
    app.setFont(f)
    return fams


# ---------------------------------------------------------------------------
# 每电机的默认状态
def _default_sp():
    return {k: float(FIELD_RANGES[k][2]) for k in FIELD_RANGES}


def _default_ff():
    out = {}
    for key, _label, fields in FF_GROUPS:
        out[key] = {"enabled": False}
        for fname, _flabel, dflt, _lo, _hi in fields:
            out[key][fname] = float(dflt)
    return out


def _default_limit():
    return {"enabled": False, "lo": -10.0, "hi": 10.0}


def _default_sig():
    return {"enabled": False, "type": "sine",
            "amplitude": 0.5, "freq": 0.2, "offset": 0.0,
            "start": 0.0, "end": 1.5, "duration": 1.5,
            "expr": CUSTOM_DEFAULT_EXPR}


def _default_latest():
    return {"pos": 0.0, "vel": 0.0, "tau": 0.0, "ff_tau": 0.0,
            "cmd_q": 0.0, "cmd_dq": 0.0, "cmd_tau": 0.0,
            "t": 0.0, "interval": 0.0, "err": 0,
            "mode": "MIT", "enabled": False, "sending": False}


# ---------------------------------------------------------------------------
class MotorWorker(QThread):
    logMsg = pyqtSignal(str)
    failed = pyqtSignal(str)
    paramResult = pyqtSignal(int, int, object)

    def __init__(self, specs, nom, dat, canfd, hz, sim,
                 device_index=0, device_type="single", backend="vendor",
                 can_channel="can0", can_bitrate=1_000_000):
        super().__init__()
        self.specs = list(specs)
        self.nom, self.dat, self.canfd = nom, dat, canfd
        self.hz = max(1, int(hz))
        self.sim = sim
        self.device_index = int(device_index)
        self.device_type = device_type
        self.backend = str(backend or "vendor").strip().lower()
        self.can_channel = str(can_channel or "can0")
        self.can_bitrate = int(can_bitrate or 1_000_000)
        self.bus = None

        self._sample_lock = threading.RLock()
        self._state_lock = threading.Lock()

        cids = [s.can_id for s in self.specs]
        self._sp = {cid: _default_sp() for cid in cids}
        self._ff = {cid: _default_ff() for cid in cids}
        self._zero = {cid: False for cid in cids}
        self._limits = {cid: _default_limit() for cid in cids}
        self._sig = {cid: _default_sig() for cid in cids}
        self._sig_t0 = {cid: time.monotonic() for cid in cids}
        self._modes = {s.can_id: s.mode for s in self.specs}
        self._pending = []
        self._enabled = set()
        self._sending = set()
        self._recording = True
        self._filters = {}
        self._last_cmd = {}
        self._last_ff = {}
        self._stop = False

        # 反馈缓冲与最近值：先建好，保证 on_sample 第一次进来时就有地方写
        self._telem = {cid: TelemetryBuffer(TELEM_CHANNELS) for cid in cids}
        self._latest = {cid: _default_latest() for cid in cids}

        self._rx_count = 0
        self._rx_t0 = time.monotonic()
        self._rx_hz = 0.0
        self._send_count = 0
        self._send_t0 = time.monotonic()
        self._send_hz = 0.0
        self._sample_errors = 0
        self._log_throttle = {}

    # ---------------------------------------------------------- GUI 侧接口
    def update_sp(self, can_id, **kw):
        with self._state_lock:
            self._sp.setdefault(can_id, _default_sp()).update(kw)

    def set_ff(self, can_id, params):
        with self._state_lock:
            self._ff[can_id] = params

    def set_zero_force(self, can_id, on):
        with self._state_lock:
            self._zero[can_id] = bool(on)

    def set_limits(self, can_id, enabled, lo, hi):
        with self._state_lock:
            self._limits[can_id] = {"enabled": bool(enabled),
                                    "lo": float(lo), "hi": float(hi)}

    def set_sig(self, can_id, params):
        with self._state_lock:
            self._sig[can_id] = dict(params)
            # 每次改信号源都重开相位，所见即所得
            self._sig_t0[can_id] = time.monotonic()

    def set_recording(self, on):
        with self._state_lock:
            self._recording = bool(on)

    def request(self, cmd, can_id, arg=None):
        with self._state_lock:
            self._pending.append((cmd, can_id, arg))

    def stop(self):
        self._stop = True

    def clear_telemetry(self):
        with self._sample_lock:
            for buf in self._telem.values():
                buf.clear()
            for cid in self._latest:
                self._latest[cid] = _default_latest()
            self._last_cmd.clear()
            self._last_ff.clear()
            self._rx_count = 0
            self._rx_t0 = time.monotonic()
            self._sample_errors = 0

    def telemetry(self, max_points=None, tail=None, include=True):
        with self._sample_lock:
            samples = {}
            if include:
                for cid, buf in self._telem.items():
                    if buf.n:
                        samples[cid] = buf.view(max_points, tail)
            latest = {cid: dict(d) for cid, d in self._latest.items()}
        with self._state_lock:
            enabled = set(self._enabled)
            sending = set(self._sending)
        for cid, d in latest.items():
            d["enabled"] = cid in enabled
            d["sending"] = cid in sending
            try:
                d["mode"] = self.bus.mode_of(cid) if self.bus is not None \
                    else self._modes.get(cid, "MIT")
            except Exception:
                d["mode"] = self._modes.get(cid, "MIT")
        return {
            "samples": samples,
            "latest": latest,
            "rates": {
                "rx_hz": float(self._rx_hz),
                "send_hz": float(self._send_hz),
                "target_send_hz": float(self.hz),
                "sample_errors": int(self._sample_errors),
            },
        }

    # ------------------------------------------------------------ SDK 回调
    def _on_sample(self, can_id, t, q, dq, tau, err):
        """SDK 接收线程调用（持 SDK 锁）。只做内存写入。"""
        try:
            with self._sample_lock:
                buf = self._telem.get(can_id)
                if buf is None:
                    return
                self._rx_count += 1
                cmd = self._last_cmd.get(can_id)
                if cmd is None:
                    cq = cdq = ctau = 0.0
                else:
                    cq, cdq, ctau = cmd
                ff = float(self._last_ff.get(can_id, 0.0))
                if self._recording:
                    buf.push(t, (q, dq, tau, ff, cq, cdq, ctau))
                d = self._latest.setdefault(can_id, _default_latest())
                prev = d.get("t") or 0.0
                d.update(pos=q, vel=dq, tau=tau, ff_tau=ff,
                         cmd_q=cq, cmd_dq=cdq, cmd_tau=ctau,
                         t=float(t), interval=(float(t) - prev) if prev else 0.0,
                         err=int(err))
        except Exception:
            self._sample_errors += 1

    # -------------------------------------------------------------- 线程主体
    def run(self):
        try:
            if self.backend == "socketcan":
                from socketcan_bus import SocketCanBus
                self.bus = SocketCanBus(
                    self.specs, sim=self.sim, on_sample=self._on_sample,
                    channel=self.can_channel, bitrate=self.can_bitrate)
            else:
                from core import MotorBus
                self.bus = MotorBus(
                    self.specs, nom_baud=self.nom, dat_baud=self.dat,
                    sim=self.sim, canfd=self.canfd,
                    device_index=self.device_index, device_type=self.device_type,
                    on_sample=self._on_sample)
        except Exception as exc:
            self.failed.emit("打开总线失败: %s" % exc)
            return

        self.logMsg.emit("总线已打开（%s，后端 %s，%s）" % (
            "模拟" if self.sim else "实机",
            self.backend,
            ("SocketCAN " + self.can_channel) if self.backend == "socketcan"
            else ("CAN FD" if self.canfd else "标准 CAN")))

        if self.backend == "socketcan":
            for _ch, st in sorted(
                    (getattr(self.bus, "channel_status", {}) or {}).items()):
                self.logMsg.emit(
                    "CAN 接口 %s: operstate=%s driver=%s" % (
                        self.can_channel, st.get("operstate", "unknown"),
                        st.get("driver") or "未知"))
        else:
            # 通道波特率回读：SDK 只把这条警告打到 stderr，界面上看不到
            for ch, st in sorted(
                    (getattr(self.bus, "channel_status", {}) or {}).items()):
                if st.get("ok") is False:
                    self.logMsg.emit(
                        "⚠ 通道 %d 波特率回读不符（canfd=%s nom=%s data=%s），"
                        "请核对设备类型与通道号" % (
                            ch, st.get("canfd"), st.get("nom"), st.get("dat")))

        # 逐台读一次 CTRL_MODE：通道或 CAN ID 填错时，连接后立刻就能看见
        alive = []
        for s in self.specs:
            try:
                v = self.bus.read_param(s.can_id, 10, wait=0.15)
            except Exception:
                v = None
            alive.append((s.can_id, v is not None))
        if alive:
            self.logMsg.emit("连接探测: " + ", ".join(
                "0x%02X %s" % (cid, "应答" if ok else "无应答")
                for cid, ok in alive))

        dt = 1.0 / self.hz
        next_t = time.perf_counter()
        while not self._stop:
            with self._state_lock:
                pend, self._pending = self._pending, []

            for cmd, cid, arg in pend:
                self._handle(cmd, cid, arg)

            now = time.monotonic()
            try:
                self._send_all(now)
                if self.sim:
                    self.bus.sample_sim(now)
            except Exception as exc:
                self._err_log("控制循环异常: %s" % exc)

            self._update_rates(now)

            next_t += dt
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.perf_counter()

        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
        self.logMsg.emit("总线已关闭")

    # ------------------------------------------------------------ 命令执行
    def _handle(self, cmd, cid, arg):
        try:
            if cmd == "enable":
                self.bus.enable(cid, bool(arg))
                with self._state_lock:
                    if arg:
                        self._enabled.add(cid)
                    else:
                        self._enabled.discard(cid)
                        self._sending.discard(cid)
                self.logMsg.emit("%s CAN 0x%02X" % ("使能" if arg else "失能", cid))
            elif cmd == "sending":
                ok = True
                with self._state_lock:
                    if arg:
                        if cid in self._enabled:
                            self._sending.add(cid)
                        else:
                            ok = False
                    else:
                        self._sending.discard(cid)
                if ok:
                    self.logMsg.emit("%s发送 CAN 0x%02X" % ("开始" if arg else "停止", cid))
                else:
                    self.logMsg.emit("CAN 0x%02X 未使能，拒绝发送" % cid)
            elif cmd == "mode":
                ok = self.bus.set_mode(cid, arg)
                with self._state_lock:
                    if ok:
                        self._modes[cid] = arg
                        was_enabled = cid in self._enabled
                    else:
                        was_enabled = False
                if ok and was_enabled:
                    self.bus.enable(cid, True)
                self.logMsg.emit("CAN 0x%02X → %s%s" % (
                    cid, arg, "" if ok else "（驱动器未确认）"))
            elif cmd == "zero":
                self.bus.zero(cid)
                self.logMsg.emit("CAN 0x%02X 设零位" % cid)
            elif cmd == "readparam":
                val = self.bus.read_param(cid, int(arg), wait=0.2)
                self.paramResult.emit(cid, int(arg), val)
            elif cmd == "writeparam":
                rid, val = arg
                ok = self.bus.write_param(cid, int(rid), val)
                self.logMsg.emit("写 CAN 0x%02X RID %d ← %s%s" % (
                    cid, rid, val,
                    "（未收到应答）" if ok is False else ""))
            elif cmd == "saveflash":
                with self._state_lock:
                    was_enabled = cid in self._enabled
                ok = self.bus.save(cid)
                if ok is False:
                    self.logMsg.emit(
                        "CAN 0x%02X 存 Flash 未收到 0xAA 应答" % cid)
                    return
                if was_enabled:
                    # SDK 保存前会先发 0xFD，这里补一次使能，否则界面说「已使能」
                    # 而驱动器其实是失能的
                    self.bus.enable(cid, True)
                    self.logMsg.emit("存 Flash 完成，已重新使能 CAN 0x%02X" % cid)
                else:
                    self.logMsg.emit("CAN 0x%02X 存 Flash 完成" % cid)
        except Exception as exc:
            self.logMsg.emit("命令 %s 失败: %s" % (cmd, exc))

    # -------------------------------------------------------------- 发送循环
    def _send_all(self, now):
        with self._state_lock:
            sending = list(self._sending)
            sp_map = {cid: dict(v) for cid, v in self._sp.items()}
            ff_map = {cid: v for cid, v in self._ff.items()}
            zero_map = dict(self._zero)
            lim_map = {cid: dict(v) for cid, v in self._limits.items()}
            modes = dict(self._modes)
            sig_map = {cid: dict(v) for cid, v in self._sig.items()}

        for cid in sending:
            sp = sp_map.get(cid) or _default_sp()
            mode = modes.get(cid, "MIT")
            lim = lim_map.get(cid) or _default_limit()

            with self._sample_lock:
                d = self._latest.get(cid) or {}
                pos = float(d.get("pos", 0.0))
                vel = float(d.get("vel", 0.0))

            tau_ff = self._compute_ff(cid, pos, vel, ff_map.get(cid) or _default_ff())
            sig_q = self._signal_target(cid, now, sig_map.get(cid) or _default_sig())
            q_ref = sp["q"] if sig_q is None else sig_q
            if lim["enabled"]:
                q_ref = min(max(q_ref, lim["lo"]), lim["hi"])

            try:
                if mode == "MIT":
                    if zero_map.get(cid):
                        cq, cdq, ctau = pos, vel, tau_ff
                    else:
                        cq = q_ref
                        cdq = sp["dq"]
                        ctau = sp["tau"] + tau_ff
                    self.bus.mit(cid, sp["kp"], sp["kd"], cq, cdq, ctau)
                elif mode == "POS_VEL":
                    cq, cdq, ctau = q_ref, sp["vel"], 0.0
                    self.bus.pos_vel(cid, cq, cdq)
                elif mode == "VEL":
                    cq, cdq, ctau = 0.0, sp["vel"], 0.0
                    self.bus.vel(cid, cdq)
                else:  # POS_FORCE
                    cq, cdq, ctau = q_ref, 0.0, sp["tau"]
                    self.bus.pos_force(cid, cq, ctau)

                with self._sample_lock:
                    self._last_cmd[cid] = (cq, cdq, ctau)
                    self._last_ff[cid] = tau_ff if mode == "MIT" else 0.0
            except Exception as exc:
                self._err_log("发送 CAN 0x%02X 失败: %s" % (cid, exc))

        # 发送率按「控制循环节拍」统计（与参考工具一致），不是所有电机的帧数总和
        if sending:
            self._send_count += 1

    def _compute_ff(self, cid, pos, vel, ff):
        """前馈力矩。速度滤波无论是否启用都要更新，否则启用瞬间会从旧状态开始。"""
        i = ff.get("inertia") or {}
        vf = self._filters.setdefault(cid, ffmod.VelocityFilter())
        vf.set_alpha(i.get("alpha", 1.0))
        accel = vf.update(vel)

        tau = 0.0
        g = ff.get("gravity") or {}
        if g.get("enabled"):
            tau += ffmod.gravity_torque(g.get("mass", 0.0), g.get("com", 0.0),
                                        pos, g.get("offset", 0.0), g.get("gain", 1.0))
        f = ff.get("friction") or {}
        if f.get("enabled"):
            tau += ffmod.friction_torque(vel, f.get("coulomb", 0.0),
                                         f.get("viscous", 0.0), f.get("slope", 100.0))
        if i.get("enabled"):
            tau += ffmod.inertia_torque(accel, i.get("inertia", 0.0))
        return tau

    def _signal_target(self, cid, now, sig):
        """信号源目标位置；未启用返回 None。相位起点在 set_sig 里刷新。"""
        if not sig.get("enabled"):
            return None
        t0 = self._sig_t0.get(cid, now)
        t = max(0.0, now - t0)
        stype = sig.get("type", "sine")
        if stype == "sine":
            return (sig.get("offset", 0.0)
                    + sig.get("amplitude", 0.0)
                    * math.sin(2.0 * math.pi * sig.get("freq", 0.0) * t))
        if stype == "custom":
            duration = sig.get("duration", 1.0)
            x = 1.0 if duration <= 0 else min(1.0, t / duration)
            return self._eval_expr(sig.get("expr", "0"), x)
        start = sig.get("start", 0.0)
        end = sig.get("end", 0.0)
        duration = sig.get("duration", 1.0)
        if duration <= 0:
            return end
        progress = min(1.0, t / duration)
        return start + (end - start) * progress

    @staticmethod
    def _eval_expr(expr, x):
        namespace = {
            "x": x, "sin": math.sin, "cos": math.cos, "tan": math.tan,
            "pi": math.pi, "e": math.e, "sqrt": math.sqrt,
            "abs": abs, "pow": pow, "exp": math.exp, "log": math.log,
        }
        try:
            return float(eval(expr, {"__builtins__": {}}, namespace))
        except Exception:
            return 0.0

    # ---------------------------------------------------------------- 统计
    def _update_rates(self, now):
        if now - self._rx_t0 >= 1.0:
            with self._sample_lock:
                n = self._rx_count
                self._rx_count = 0
            # 计数是「所有电机的反馈帧总和」，换算成每台电机的采样率 ——
            # 曲线是按每台电机一条建的，tail 也按单台点数算。
            self._rx_hz = (n / max(1, len(self.specs))
                           / max(1e-6, now - self._rx_t0))
            self._rx_t0 = now
        if now - self._send_t0 >= 1.0:
            self._send_hz = self._send_count / max(1e-6, now - self._send_t0)
            self._send_count = 0
            self._send_t0 = now

    def _err_log(self, msg):
        now = time.monotonic()
        cnt, t0 = self._log_throttle.get(msg, (0, now))
        if now - t0 >= 2.0:
            self.logMsg.emit(msg + ("（2 秒内重复 %d 次）" % (cnt + 1) if cnt else ""))
            self._log_throttle[msg] = (0, now)
        else:
            self._log_throttle[msg] = (cnt + 1, t0)


# ---------------------------------------------------------------------------
class Hmi(QMainWindow):
    def __init__(self, sim=False, backend="vendor",
                 can_channel="can0", can_bitrate=1_000_000):
        super().__init__()
        # 标题栏由 WSLg/窗口管理器绘制，部分环境下中文会显示成方框；
        # 这里用 ASCII，界面内的中文仍由 Microsoft YaHei 正常渲染。
        self.setWindowTitle("ZFY DM Motor Tool")
        self.resize(1500, 950)
        self.worker = None
        self.specs = []

        self.setpoints = {}
        self.ff_params = {}
        self.zero_force = {}
        self.pos_limits = {}
        self.signal_source = {}
        self.sending = {}
        self.motor_enabled = {}
        self.motor_modes = {}

        self._last_rates = {}
        self._render_tick = 0
        self._render_prev_t = 0.0
        self._render_last_ms = 0.0
        self._render_t0 = time.monotonic()
        self._log_throttle = {}

        self._build_ui()
        self.chk_sim.setChecked(sim)
        self._load_config()
        if str(backend).strip().lower() == "socketcan":
            idx = self.cb_backend.findData("socketcan")
            if idx >= 0:
                self.cb_backend.setCurrentIndex(idx)
            self.ed_can_iface.setText(str(can_channel or "can0"))
            self.sp_can_bitrate.setValue(int(can_bitrate or 1_000_000))
            self._on_backend_changed()
        if self.motor_table.rowCount() == 0:
            self._add_motor_row(0x01, 0x01, "4340", "MIT", DEFAULT_CHANNEL)
        self._refresh_motor_combos()

        self._render_timer = QTimer(self)
        self._render_timer.setTimerType(Qt.PreciseTimer)
        self._render_timer.setInterval(RENDER_INTERVAL_MS)
        self._render_timer.timeout.connect(self._on_render_tick)
        self._render_timer.start()

        self._load_layout()
        if not sim:
            self._log("提示：先用 ./run_hmi.sh 启动；无硬件可用 --sim 或勾选「仿真」")

    # ================================================================== UI
    def _build_ui(self):
        central = QWidget()
        root = QVBoxLayout(central)
        root.addWidget(self._build_connection_group())

        self.tabs = QTabWidget()

        tab1 = QWidget()
        t1 = QVBoxLayout(tab1)
        t1.addWidget(self._build_motor_config_group(), 1)
        t1.addWidget(self._build_param_group(), 1)
        self.tabs.addTab(tab1, "参数配置")

        tab2 = QWidget()
        t2 = QHBoxLayout(tab2)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 0, 0)
        lv.addWidget(self._build_control_group())
        lv.addWidget(self._build_signal_group())
        lv.addWidget(self._build_feedforward_group())
        lv.addWidget(self._build_status_group())
        lv.addStretch(1)
        scroll.setWidget(left)

        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(0, 0, 0, 0)
        self.plot_panel = PlotPanel()
        self.plot_panel.record_toggled.connect(self._on_record_toggled)
        self.plot_panel.export_csv_requested.connect(self.on_export_csv)
        self.plot_panel.export_png_requested.connect(self.on_export_png)
        rv.addWidget(self.plot_panel, 1)
        self.lbl_rate = QLabel("采样率: --")
        self.lbl_rate.setStyleSheet(
            "color: #888; font-family: monospace; font-size: 11px;")
        self.lbl_rate.setToolTip(
            "反馈=实际接收到的电机反馈帧率（也是曲线采样率）；\n"
            "发送=实际下发的控制帧率。MIT 模式下电机收到一帧回一帧，\n"
            "所以「暂停发送」时两条都会归零，曲线停住属于正常现象。")
        rv.addWidget(self.lbl_rate)

        self.splitter = QSplitter(Qt.Horizontal)
        self.splitter.addWidget(scroll)
        self.splitter.addWidget(right)
        self.splitter.setStretchFactor(0, 875)
        self.splitter.setStretchFactor(1, 785)
        self.splitter.setSizes([875, 785])
        t2.addWidget(self.splitter)
        self.tabs.addTab(tab2, "调试与反馈")

        root.addWidget(self.tabs, 1)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setFixedHeight(100)
        self.log.setFont(QFont("DejaVu Sans Mono", 9))
        root.addWidget(QLabel("日志"))
        root.addWidget(self.log)
        self.setCentralWidget(central)

    # ---- 设备连接
    def _build_connection_group(self):
        box = QGroupBox("设备连接")
        lay = QHBoxLayout(box)

        lay.addWidget(QLabel("设备类型"))
        self.cb_device_type = QComboBox()
        self.cb_device_type.addItem("单路 CANFD", "single")
        self.cb_device_type.addItem("双路 CANFD", "dual")
        lay.addWidget(self.cb_device_type)

        lay.addWidget(QLabel("后端"))
        self.cb_backend = QComboBox()
        self.cb_backend.addItem("厂商 USB SDK", "vendor")
        self.cb_backend.addItem("SocketCAN", "socketcan")
        self.cb_backend.currentIndexChanged.connect(self._on_backend_changed)
        lay.addWidget(self.cb_backend)

        lay.addWidget(QLabel("CAN 接口"))
        self.ed_can_iface = QLineEdit("can0")
        self.ed_can_iface.setFixedWidth(64)
        lay.addWidget(self.ed_can_iface)

        lay.addWidget(QLabel("CAN 速率"))
        self.sp_can_bitrate = QSpinBox()
        self.sp_can_bitrate.setRange(10000, 5000000)
        self.sp_can_bitrate.setSingleStep(100000)
        self.sp_can_bitrate.setValue(1000000)
        lay.addWidget(self.sp_can_bitrate)

        lay.addWidget(QLabel("通信模式"))
        self.cb_comm = QComboBox()
        self.cb_comm.addItem("标准 CAN", False)
        self.cb_comm.addItem("FDCAN", True)
        lay.addWidget(self.cb_comm)

        lay.addWidget(QLabel("设备索引"))
        self.sp_index = QSpinBox()
        self.sp_index.setRange(0, 15)
        lay.addWidget(self.sp_index)

        lay.addWidget(QLabel("名义波特率"))
        self.sp_nom = QSpinBox()
        self.sp_nom.setRange(10000, 8000000)
        self.sp_nom.setSingleStep(100000)
        self.sp_nom.setValue(1000000)
        lay.addWidget(self.sp_nom)

        lay.addWidget(QLabel("数据波特率"))
        self.sp_dat = QSpinBox()
        self.sp_dat.setRange(100000, 10000000)
        self.sp_dat.setSingleStep(100000)
        self.sp_dat.setValue(1000000)
        lay.addWidget(self.sp_dat)

        lay.addWidget(QLabel("下发频率"))
        self.cb_hz = QComboBox()
        self.cb_hz.addItems(["100", "250", "500", "1000"])
        self.cb_hz.setCurrentText("250")
        lay.addWidget(self.cb_hz)

        self.chk_sim = QCheckBox("仿真")
        lay.addWidget(self.chk_sim)

        self.btn_scan = QPushButton("扫描设备")
        self.btn_scan.clicked.connect(self.on_scan)
        lay.addWidget(self.btn_scan)

        self.btn_connect = QPushButton("连接")
        self.btn_connect.clicked.connect(self.on_connect)
        lay.addWidget(self.btn_connect)

        self.btn_disconnect = QPushButton("断开")
        self.btn_disconnect.setEnabled(False)
        self.btn_disconnect.clicked.connect(self.on_disconnect)
        lay.addWidget(self.btn_disconnect)

        self.btn_release = QPushButton("释放设备")
        self.btn_release.setToolTip(
            "真正关闭 USB 设备并销毁 SDK context（「断开」只失能、不释放）。\n"
            "换波特率/设备类型前、或怀疑设备被本程序占用时点它。")
        self.btn_release.clicked.connect(self.on_release)
        lay.addWidget(self.btn_release)

        self.btn_save_config = QPushButton("保存参数")
        self.btn_save_config.setStyleSheet("background-color: #FFC107; color: black;")
        self.btn_save_config.clicked.connect(self.on_save_config)
        lay.addWidget(self.btn_save_config)

        lay.addStretch(1)
        self.lbl_status = QLabel("未连接")
        self.lbl_status.setStyleSheet("color:#888")
        lay.addWidget(self.lbl_status)
        self._on_backend_changed()
        return box

    def _on_backend_changed(self, *_):
        """SocketCAN 只需要接口名/速率；厂商 SDK 才看设备类型等字段。"""
        sc = self.cb_backend.currentData() == "socketcan"
        self.ed_can_iface.setEnabled(sc)
        self.sp_can_bitrate.setEnabled(sc)
        self.cb_device_type.setEnabled(not sc)
        self.cb_comm.setEnabled(not sc)
        self.sp_index.setEnabled(not sc)
        self.sp_nom.setEnabled(not sc)
        self.sp_dat.setEnabled(not sc)
        self.btn_scan.setEnabled(not sc)

    # ---- 电机注册表
    def _build_motor_config_group(self):
        box = QGroupBox("电机配置")
        lay = QVBoxLayout(box)
        self.motor_table = QTableWidget(0, 5)
        self.motor_table.setHorizontalHeaderLabels(
            ["CAN ID", "MST ID", "型号", "模式", "通道"])
        self.motor_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.motor_table.verticalHeader().setVisible(False)
        self.motor_table.itemSelectionChanged.connect(self._on_motor_row_selected)
        lay.addWidget(self.motor_table)

        row = QHBoxLayout()
        self.btn_add = QPushButton("添加电机")
        self.btn_add.clicked.connect(self.on_add_motor)
        self.btn_del = QPushButton("删除选中")
        self.btn_del.clicked.connect(self.on_del_motor)
        row.addWidget(self.btn_add)
        row.addWidget(self.btn_del)
        row.addStretch(1)
        lbl = QLabel("型号: " + ", ".join(MODELS))
        lbl.setStyleSheet("color:#888")
        row.addWidget(lbl)
        lay.addLayout(row)
        return box

    # ---- 参数读写
    def _build_param_group(self):
        box = QGroupBox("参数读写")
        lay = QVBoxLayout(box)
        row = QHBoxLayout()
        row.addWidget(QLabel("选择电机"))
        self.cb_pmotor = QComboBox()
        row.addWidget(self.cb_pmotor, 1)
        row.addWidget(QLabel("寄存器"))
        self.cb_reg = QComboBox()
        for label, rid in PARAM_REGS:
            self.cb_reg.addItem("%s (RID %d)" % (label, rid), rid)
        row.addWidget(self.cb_reg, 2)
        lay.addLayout(row)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("写入值"))
        self.ed_pval = QLineEdit()
        self.ed_pval.setPlaceholderText("写入值")
        row2.addWidget(self.ed_pval, 1)
        row2.addWidget(QLabel("读回"))
        self.lbl_param_result = QLineEdit("—")
        self.lbl_param_result.setReadOnly(True)
        row2.addWidget(self.lbl_param_result, 1)
        lay.addLayout(row2)

        row3 = QHBoxLayout()
        b1 = QPushButton("读取")
        b1.clicked.connect(self.on_read_param)
        b2 = QPushButton("写入")
        b2.clicked.connect(self.on_write_param)
        b3 = QPushButton("保存到Flash")
        b3.clicked.connect(self.on_save_flash)
        for b in (b1, b2, b3):
            row3.addWidget(b)
        row3.addStretch(1)
        lay.addLayout(row3)
        lay.addWidget(QLabel("提示：改完 ID / 波特率 / 控制模式后必须「保存到Flash」，"
                            "否则断电丢失"))
        return box

    # ---- 电机控制
    def _build_control_group(self):
        box = QGroupBox("电机控制")
        lay = QVBoxLayout(box)

        top = QHBoxLayout()
        top.addWidget(QLabel("选择电机"))
        self.cb_motor = QComboBox()
        self.cb_motor.currentIndexChanged.connect(self.on_motor_selected)
        top.addWidget(self.cb_motor, 1)
        top.addWidget(QLabel("当前模式"))
        self.lbl_mode = QLabel("—")
        top.addWidget(self.lbl_mode)
        lay.addLayout(top)

        btns = QHBoxLayout()
        self.btn_enable = QPushButton("已失能")
        self.btn_enable.clicked.connect(self.on_enable_toggled)
        self.btn_zero = QPushButton("设零位")
        self.btn_zero.clicked.connect(self.on_zero)
        self.btn_send = QPushButton("发送数据")
        self.btn_send.setCheckable(True)
        self.btn_send.setToolTip(
            "独立于「使能」。未使能时拒绝发送；\n"
            "MIT 模式下停止发送后电机不再回帧，曲线会停住，属正常现象。")
        self.btn_send.toggled.connect(self.on_send_toggled)
        btns.addWidget(self.btn_enable)
        btns.addWidget(self.btn_zero)
        btns.addWidget(self.btn_send)
        btns.addStretch(1)
        lay.addLayout(btns)

        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("切换模式"))
        self.cb_mode = QComboBox()
        self.cb_mode.addItems(MODES)
        mode_row.addWidget(self.cb_mode)
        self.btn_apply_mode = QPushButton("应用模式")
        self.btn_apply_mode.clicked.connect(self.on_apply_mode)
        mode_row.addWidget(self.btn_apply_mode)
        mode_row.addStretch(1)
        lay.addLayout(mode_row)

        lay.addWidget(QLabel("指令设定（按当前模式显示对应项）"))
        grid = QGridLayout()
        self.sp_fields = {}
        self.field_labels = {}
        for r, key in enumerate(("kp", "kd", "q", "dq", "tau", "pos", "vel")):
            lab = QLabel(FIELD_LABELS[key])
            sp = QDoubleSpinBox()
            lo, hi, dflt, step = FIELD_RANGES[key]
            sp.setRange(lo, hi)
            sp.setDecimals(4)
            sp.setSingleStep(step)
            sp.setValue(dflt)
            sp.valueChanged.connect(lambda _v, k=key: self._on_setpoint_changed(k))
            grid.addWidget(lab, r // 2, (r % 2) * 2)
            grid.addWidget(sp, r // 2, (r % 2) * 2 + 1)
            self.sp_fields[key] = sp
            self.field_labels[key] = lab
        lay.addLayout(grid)

        lim_row = QHBoxLayout()
        self.chk_pos_limit = QCheckBox("位置限制")
        self.chk_pos_limit.toggled.connect(self._on_pos_limit_changed)
        self.sp_pos_min = QDoubleSpinBox()
        self.sp_pos_min.setRange(-1000, 1000)
        self.sp_pos_min.setValue(-10.0)
        self.sp_pos_min.valueChanged.connect(self._on_pos_limit_changed)
        self.sp_pos_max = QDoubleSpinBox()
        self.sp_pos_max.setRange(-1000, 1000)
        self.sp_pos_max.setValue(10.0)
        self.sp_pos_max.valueChanged.connect(self._on_pos_limit_changed)
        lim_row.addWidget(self.chk_pos_limit)
        lim_row.addWidget(QLabel("下限"))
        lim_row.addWidget(self.sp_pos_min)
        lim_row.addWidget(QLabel("上限"))
        lim_row.addWidget(self.sp_pos_max)
        lim_row.addStretch(1)
        lay.addLayout(lim_row)

        self._apply_mode_keys("MIT")
        return box

    # ---- 信号源
    def _build_signal_group(self):
        box = QGroupBox("信号源（目标位置）")
        v = QVBoxLayout(box)

        row1 = QHBoxLayout()
        self.chk_signal = QCheckBox("启用信号源")
        self.chk_signal.toggled.connect(self._push_signal)
        row1.addWidget(self.chk_signal)
        self.cb_signal_type = QComboBox()
        self.cb_signal_type.addItem("正弦", "sine")
        self.cb_signal_type.addItem("斜坡轨迹", "ramp")
        self.cb_signal_type.addItem("自定义函数", "custom")
        self.cb_signal_type.currentIndexChanged.connect(self._on_signal_type_changed)
        row1.addWidget(self.cb_signal_type)
        row1.addStretch(1)
        v.addLayout(row1)

        grid = QGridLayout()
        self.sig_fields = {}
        self.sig_labels = {}
        for stype, fields in SIGNAL_FIELDS.items():
            self.sig_fields[stype] = {}
            self.sig_labels[stype] = {}
            for c, (fname, flabel, dflt, lo, hi) in enumerate(fields):
                lab = QLabel(flabel)
                sp = QDoubleSpinBox()
                sp.setRange(lo, hi)
                sp.setDecimals(4)
                sp.setValue(dflt)
                sp.valueChanged.connect(self._push_signal)
                grid.addWidget(lab, 0, c * 2)
                grid.addWidget(sp, 0, c * 2 + 1)
                self.sig_fields[stype][fname] = sp
                self.sig_labels[stype][fname] = lab
        v.addLayout(grid)

        custom_row = QHBoxLayout()
        self.lbl_custom_expr = QLabel("函数 f(x)")
        self.le_expr = QLineEdit(CUSTOM_DEFAULT_EXPR)
        self.le_expr.setToolTip("x 为归一化时间(0→1)。示例：2*x+10、cos(x)*x、1.5*x*x")
        self.le_expr.textChanged.connect(self._push_signal)
        self.lbl_custom_dur = QLabel("时间(s)")
        self.sp_custom_dur = QDoubleSpinBox()
        self.sp_custom_dur.setRange(0.01, 1000.0)
        self.sp_custom_dur.setDecimals(3)
        self.sp_custom_dur.setValue(1.5)
        self.sp_custom_dur.valueChanged.connect(self._push_signal)
        for w in (self.lbl_custom_expr, self.le_expr,
                  self.lbl_custom_dur, self.sp_custom_dur):
            custom_row.addWidget(w)
        custom_row.addStretch(1)
        v.addLayout(custom_row)

        self.lbl_custom_hint = QLabel(
            "x 为归一化时间(0→1)。支持：+ - * / **；"
            "sin cos tan exp log sqrt abs pow；常量 pi、e。")
        self.lbl_custom_hint.setWordWrap(True)
        self.lbl_custom_hint.setStyleSheet("color: gray; font-size: 11px;")
        v.addWidget(self.lbl_custom_hint)

        self._update_signal_fields()
        return box

    # ---- 前馈补偿
    def _build_feedforward_group(self):
        box = QGroupBox("前馈补偿（加到 MIT 前馈力矩）")
        self.ff_group = box
        v = QVBoxLayout(box)
        self.chk_zero_force = QCheckBox("零力补偿（目标位置/速度跟随实时值）")
        self.chk_zero_force.toggled.connect(self._push_ff)
        v.addWidget(self.chk_zero_force)

        grid = QGridLayout()
        self.ff_chk, self.ff_sp = {}, {}
        for row, (key, label, fields) in enumerate(FF_GROUPS):
            chk = QCheckBox(label)
            chk.toggled.connect(self._push_ff)
            self.ff_chk[key] = chk
            grid.addWidget(chk, row, 0)
            self.ff_sp[key] = {}
            col = 1
            for fname, flabel, dflt, lo, hi in fields:
                grid.addWidget(QLabel(flabel), row, col)
                sp = QDoubleSpinBox()
                sp.setRange(lo, hi)
                sp.setDecimals(4)
                sp.setValue(dflt)
                sp.valueChanged.connect(self._push_ff)
                grid.addWidget(sp, row, col + 1)
                self.ff_sp[key][fname] = sp
                col += 2
        v.addLayout(grid)
        return box

    # ---- 电机状态
    def _build_status_group(self):
        box = QGroupBox("电机状态")
        lay = QVBoxLayout(box)
        self.status_table = QTableWidget(0, 8)
        self.status_table.setHorizontalHeaderLabels(
            ["CAN ID", "模式", "位置", "速度", "力矩", "错误", "使能", "周期(s)"])
        self.status_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.status_table.verticalHeader().setVisible(False)
        self.status_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        lay.addWidget(self.status_table)
        return box

    # ================================================================== 工具
    @contextmanager
    def _blocking(self, *widgets):
        for w in widgets:
            w.blockSignals(True)
        try:
            yield
        finally:
            for w in widgets:
                w.blockSignals(False)

    def _log(self, s):
        self.log.append("%s  %s" % (time.strftime("%H:%M:%S"), s))

    def _log_throttled(self, msg):
        now = time.monotonic()
        cnt, t0 = self._log_throttle.get(msg, (0, now))
        if now - t0 >= 2.0:
            self._log(msg + ("（2 秒内重复 %d 次）" % (cnt + 1) if cnt else ""))
            self._log_throttle[msg] = (0, now)
        else:
            self._log_throttle[msg] = (cnt + 1, t0)

    # ============================================================ 电机注册表
    @staticmethod
    def _hex_spin(value, maxv=0x7FF):
        sp = QSpinBox()
        sp.setRange(0, maxv)
        sp.setDisplayIntegerBase(16)
        sp.setPrefix("0x")
        sp.setValue(int(value))
        return sp

    def _add_motor_row(self, can, mst, model, mode, ch):
        r = self.motor_table.rowCount()
        self.motor_table.insertRow(r)
        self.motor_table.setCellWidget(r, 0, self._hex_spin(can))
        self.motor_table.setCellWidget(r, 1, self._hex_spin(mst))

        cb_model = QComboBox()
        cb_model.addItems(list(MODELS))
        if model in MODELS:
            cb_model.setCurrentText(model)
        self.motor_table.setCellWidget(r, 2, cb_model)

        cb_mode = QComboBox()
        cb_mode.addItems(MODES)
        if mode in MODES:
            cb_mode.setCurrentText(mode)
        self.motor_table.setCellWidget(r, 3, cb_mode)

        sp_ch = QSpinBox()
        sp_ch.setRange(0, 3)
        sp_ch.setValue(int(ch))
        self.motor_table.setCellWidget(r, 4, sp_ch)
        self._refresh_motor_combos()

    def on_add_motor(self):
        can = 0x01 + self.motor_table.rowCount()
        self._add_motor_row(can, can, "4340", "MIT", DEFAULT_CHANNEL)

    def on_del_motor(self):
        r = self.motor_table.currentRow()
        if r >= 0:
            self.motor_table.removeRow(r)
            self._refresh_motor_combos()

    def _on_motor_row_selected(self):
        pass

    def _read_specs(self):
        out = []
        for r in range(self.motor_table.rowCount()):
            try:
                can = int(self.motor_table.cellWidget(r, 0).value())
                mst = int(self.motor_table.cellWidget(r, 1).value())
                model = self.motor_table.cellWidget(r, 2).currentText()
                mode = self.motor_table.cellWidget(r, 3).currentText()
                ch = int(self.motor_table.cellWidget(r, 4).value())
            except Exception:
                raise ValueError("第 %d 行格式不对" % (r + 1))
            if model not in MODELS:
                raise ValueError("第 %d 行型号未知: %s" % (r + 1, model))
            if mode not in MODES:
                raise ValueError("第 %d 行模式未知: %s" % (r + 1, mode))
            out.append(MotorSpec("m%02X" % can, can, mst, model, ch, mode))
        if not out:
            raise ValueError("注册表是空的")
        if len({s.can_id for s in out}) != len(out):
            raise ValueError("CAN ID 不能重复")
        return out

    def _refresh_motor_combos(self):
        cur_m = self.cb_motor.currentData() if hasattr(self, "cb_motor") else None
        cur_p = self.cb_pmotor.currentData() if hasattr(self, "cb_pmotor") else None
        for cb, fmt in ((getattr(self, "cb_motor", None), "CAN 0x%02X (%s)"),
                        (getattr(self, "cb_pmotor", None), "CAN 0x%02X")):
            if cb is None:
                continue
            cb.blockSignals(True)
            cb.clear()
            for r in range(self.motor_table.rowCount()):
                can = self.motor_table.cellWidget(r, 0).value()
                if fmt.endswith("(%s)"):
                    label = fmt % (can, self.motor_table.cellWidget(r, 2).currentText())
                else:
                    label = fmt % can
                cb.addItem(label, can)
            cb.blockSignals(False)
        if hasattr(self, "cb_motor"):
            i = self.cb_motor.findData(cur_m)
            self.cb_motor.setCurrentIndex(i if i >= 0 else 0)
        if hasattr(self, "cb_pmotor"):
            i = self.cb_pmotor.findData(cur_p)
            self.cb_pmotor.setCurrentIndex(i if i >= 0 else 0)
        # 保证注册表里每一台都有独立的默认状态（即使还没选中过）
        for r in range(self.motor_table.rowCount()):
            self._ensure_state(self.motor_table.cellWidget(r, 0).value())
        if hasattr(self, "cb_motor") and self.cb_motor.count():
            self.on_motor_selected()

    # ============================================================ 每电机状态
    def _ensure_state(self, cid):
        if cid not in self.setpoints:
            self.setpoints[cid] = _default_sp()
        if cid not in self.ff_params:
            self.ff_params[cid] = _default_ff()
        if cid not in self.zero_force:
            self.zero_force[cid] = False
        if cid not in self.pos_limits:
            self.pos_limits[cid] = _default_limit()
        if cid not in self.signal_source:
            self.signal_source[cid] = _default_sig()
        if cid not in self.sending:
            self.sending[cid] = False
        if cid not in self.motor_enabled:
            self.motor_enabled[cid] = False
        if cid not in self.motor_modes:
            spec = next((s for s in self.specs if s.can_id == cid), None)
            if spec is not None:
                self.motor_modes[cid] = spec.mode
            else:
                # 连接前 self.specs 是空的，直接读注册表那一行
                mode = "MIT"
                for r in range(self.motor_table.rowCount()):
                    if self.motor_table.cellWidget(r, 0).value() == cid:
                        mode = self.motor_table.cellWidget(r, 3).currentText()
                        break
                self.motor_modes[cid] = mode

    def _selected_cid(self):
        return self.cb_motor.currentData()

    def on_motor_selected(self):
        cid = self._selected_cid()
        if cid is None:
            return
        self._ensure_state(cid)

        sp = self.setpoints[cid]
        lim = self.pos_limits[cid]
        ff = self.ff_params[cid]
        sig = self.signal_source[cid]
        mode = self.motor_modes.get(cid, "MIT")

        widgets = list(self.sp_fields.values()) + [
            self.cb_mode, self.chk_pos_limit, self.sp_pos_min, self.sp_pos_max,
            self.chk_signal, self.cb_signal_type, self.chk_zero_force,
            self.btn_send,
        ]
        widgets += list(self.ff_chk.values())
        for sps in self.ff_sp.values():
            widgets += list(sps.values())
        for d in self.sig_fields.values():
            widgets += list(d.values())
        widgets += [self.le_expr, self.sp_custom_dur]

        with self._blocking(*widgets):
            for key, spin in self.sp_fields.items():
                spin.setValue(float(sp.get(key, FIELD_RANGES[key][2])))
            # 模式下拉跟随该电机记住的模式（标签以驱动器实际模式为准）
            i = self.cb_mode.findText(mode)
            self.cb_mode.setCurrentIndex(i if i >= 0 else 0)

            self.chk_pos_limit.setChecked(bool(lim["enabled"]))
            self.sp_pos_min.setValue(float(lim["lo"]))
            self.sp_pos_max.setValue(float(lim["hi"]))

            self.chk_zero_force.setChecked(bool(self.zero_force.get(cid)))
            for key, fields in ff.items():
                if key in self.ff_chk:
                    self.ff_chk[key].setChecked(bool(fields.get("enabled")))
                    for fname, val in fields.items():
                        if fname in self.ff_sp[key]:
                            self.ff_sp[key][fname].setValue(float(val))

            self.chk_signal.setChecked(bool(sig.get("enabled")))
            stype = sig.get("type", "sine")
            i = self.cb_signal_type.findData(stype)
            self.cb_signal_type.setCurrentIndex(i if i >= 0 else 0)
            for st, fields in self.sig_fields.items():
                for fname, spin in fields.items():
                    if fname in sig:
                        spin.setValue(float(sig[fname]))
            self.le_expr.setText(str(sig.get("expr", CUSTOM_DEFAULT_EXPR)))
            self.sp_custom_dur.setValue(float(sig.get("duration", 1.5)))

            self.btn_send.setChecked(bool(self.sending.get(cid)))

        self._apply_mode_keys(mode)
        self.lbl_mode.setText(mode)
        self._update_enable_button(cid)
        self._apply_zero_force_lock(cid)
        self._update_signal_fields()
        self._update_ff_visibility(mode)

    def _apply_mode_keys(self, mode):
        keys = set(MODE_FIELDS.get(mode, ()))
        for key, spin in self.sp_fields.items():
            show = key in keys
            spin.setVisible(show)
            self.field_labels[key].setVisible(show)

    def _update_ff_visibility(self, mode):
        self.ff_group.setVisible(mode == "MIT")

    def _update_enable_button(self, cid):
        on = bool(self.motor_enabled.get(cid))
        self.btn_enable.setText("已使能" if on else "已失能")
        self.btn_enable.setStyleSheet(
            "background-color: #4CAF50; color: white;" if on else
            "background-color: #F44336; color: white;")

    def _apply_zero_force_lock(self, cid):
        on = bool(self.zero_force.get(cid))
        for key in ("q", "dq", "tau"):
            self.sp_fields[key].setEnabled(not on)
        if not on:
            # 零力期间这三个框显示的是实时值；关掉后恢复成真正的设定值，
            # 否则界面显示的和 worker 实际使用的对不上。
            sp = self.setpoints.get(cid)
            if sp:
                with self._blocking(self.sp_fields["q"], self.sp_fields["dq"],
                                    self.sp_fields["tau"]):
                    for key in ("q", "dq", "tau"):
                        self.sp_fields[key].setValue(float(sp.get(key, 0.0)))

    # ---- 指令设定
    def _on_setpoint_changed(self, key):
        cid = self._selected_cid()
        if cid is None:
            return
        self._ensure_state(cid)
        val = float(self.sp_fields[key].value())
        self.setpoints[cid][key] = val
        if self.worker is not None:
            self.worker.update_sp(cid, **{key: val})

    def _on_pos_limit_changed(self, *_):
        cid = self._selected_cid()
        if cid is None:
            return
        self._ensure_state(cid)
        lim = {"enabled": self.chk_pos_limit.isChecked(),
               "lo": float(self.sp_pos_min.value()),
               "hi": float(self.sp_pos_max.value())}
        self.pos_limits[cid] = lim
        if self.worker is not None:
            self.worker.set_limits(cid, **lim)

    # ---- 信号源
    def _on_signal_type_changed(self, *_):
        self._update_signal_fields()
        self._push_signal()

    def _update_signal_fields(self):
        stype = self.cb_signal_type.currentData()
        for st, fields in self.sig_fields.items():
            show = (st == stype)
            for fname, spin in fields.items():
                spin.setVisible(show)
                self.sig_labels[st][fname].setVisible(show)
        custom = (stype == "custom")
        for w in (self.lbl_custom_expr, self.le_expr,
                  self.lbl_custom_dur, self.sp_custom_dur, self.lbl_custom_hint):
            w.setVisible(custom)

    def _push_signal(self, *_):
        cid = self._selected_cid()
        if cid is None:
            return
        stype = self.cb_signal_type.currentData()
        params = {"enabled": self.chk_signal.isChecked(), "type": stype}
        for fname, spin in self.sig_fields[stype].items():
            params[fname] = float(spin.value())
        if stype == "custom":
            # 只有自定义函数用 f(x) 与它自己的时长；斜坡的 duration 是上面的独立字段，
            # 不能用自定义框的值覆盖。
            params["expr"] = self.le_expr.text()
            params["duration"] = float(self.sp_custom_dur.value())
        self._ensure_state(cid)
        self.signal_source[cid] = params
        if self.worker is not None:
            self.worker.set_sig(cid, params)

    # ---- 前馈
    def _push_ff(self, *_):
        cid = self._selected_cid()
        if cid is None:
            return
        params = {}
        for key, _label, _fields in FF_GROUPS:
            params[key] = {"enabled": self.ff_chk[key].isChecked()}
            for fname, spin in self.ff_sp[key].items():
                params[key][fname] = float(spin.value())
        self._ensure_state(cid)
        self.ff_params[cid] = params
        self.zero_force[cid] = self.chk_zero_force.isChecked()
        if self.worker is not None:
            self.worker.set_ff(cid, params)
            self.worker.set_zero_force(cid, self.zero_force[cid])
        self._apply_zero_force_lock(cid)

    # ================================================================ 连接
    def on_scan(self):
        if self.cb_backend.currentData() == "socketcan":
            ch = self.ed_can_iface.text().strip() or "can0"
            self._log("SocketCAN 模式不需要扫描设备；请先确认 %s 已配置并 up" % ch)
            return
        try:
            # 必须先 import dm_backend：它会预载 bundled 的新版 libstdc++/libusb
            import dm_backend  # noqa: F401
            from dmcan import DmCanContext, dmcan_device_type
            name = ("USB2CANFD_DUAL" if self.cb_device_type.currentData() == "dual"
                    else "USB2CANFD")
            dev_type = getattr(dmcan_device_type, name, dmcan_device_type.USB2CANFD)
            ctx = DmCanContext()
            n = 0
            safe_to_destroy = True
            try:
                n = ctx.find_devices(dev_type)
                # ⚠ 厂商 libdm_device.so 的坑（Linux 版实测 139）：
                # 枚举到设备但**没 open 过**时调用 dmcan_context_destroy() 会段错误。
                # 先逐个 open + close 再 destroy 就安全；任何一台没打开成功就
                # 放弃销毁（把 _ctx 置空，避免 __del__ 再触发一次崩溃），
                # 泄漏的 context 由进程退出时 OS 回收。
                for dev in list(getattr(ctx, "_devices", None) or []):
                    opened = False
                    try:
                        opened = bool(dev.open())
                    except Exception:
                        opened = False
                    if opened:
                        try:
                            dev.close()
                        except Exception:
                            safe_to_destroy = False
                    else:
                        safe_to_destroy = False
            finally:
                if safe_to_destroy:
                    try:
                        ctx.destroy()
                    except Exception:
                        pass
                else:
                    ctx._ctx = None
            self._log("扫描到 %d 个 %s 设备" % (n, name))
        except Exception as exc:
            self._log("扫描失败: %s" % exc)

    def _set_connected(self, on):
        self.btn_connect.setEnabled(not on)
        self.btn_disconnect.setEnabled(on)
        self.btn_release.setEnabled(not on)
        # 连接期间不要扫描：扫描要临时 open 设备，会和正在运行的会话抢接口
        self.btn_scan.setEnabled(not on)
        self.motor_table.setEnabled(not on)
        self.btn_add.setEnabled(not on)
        self.btn_del.setEnabled(not on)
        for w in (self.cb_device_type, self.cb_comm, self.sp_index,
                  self.sp_nom, self.sp_dat, self.cb_hz, self.chk_sim):
            w.setEnabled(not on)
        if on:
            self.lbl_status.setText("● 已连接")
            self.lbl_status.setStyleSheet("color:#2a8a2a")
        else:
            self.lbl_status.setText("未连接")
            self.lbl_status.setStyleSheet("color:#888")

    def on_connect(self):
        try:
            specs = self._read_specs()
        except ValueError as exc:
            QMessageBox.warning(self, "注册表有误", str(exc))
            return
        self.specs = specs
        for s in specs:
            self._ensure_state(s.can_id)
            self.motor_modes[s.can_id] = s.mode

        self.plot_panel.set_motors([s.can_id for s in specs])
        self._rebuild_status_table()

        self.worker = MotorWorker(
            specs, self.sp_nom.value(), self.sp_dat.value(),
            self.cb_comm.currentData(), int(self.cb_hz.currentText()),
            self.chk_sim.isChecked(),
            device_index=self.sp_index.value(),
            device_type=self.cb_device_type.currentData(),
            backend=self.cb_backend.currentData(),
            can_channel=self.ed_can_iface.text().strip() or "can0",
            can_bitrate=self.sp_can_bitrate.value())
        self.worker.logMsg.connect(self._log)
        self.worker.failed.connect(self._on_worker_failed)
        self.worker.paramResult.connect(self._on_param_result)
        self._push_all_to_worker()
        # 连接前用户可能已经关掉「记录数据」，要带过去
        self.worker.set_recording(self.plot_panel.chk_record.isChecked())
        self.worker.start()
        self._set_connected(True)
        self.on_motor_selected()          # 按当前电机刷新字段显隐/零力锁
        self._log("正在连接 %d 台电机（%s）" % (
            len(specs), "仿真" if self.chk_sim.isChecked() else "实机"))

    def _on_worker_failed(self, msg):
        self._log("✘ " + msg)
        QMessageBox.critical(self, "连接失败", msg)
        self.on_disconnect()

    def on_disconnect(self):
        if self.worker is not None:
            self.worker.stop()
            if not self.worker.wait(3000):
                self._log("⚠ 工作线程未在 3 秒内退出")
            self.worker = None
        self._set_connected(False)
        for cid in self.motor_enabled:
            self.motor_enabled[cid] = False
        for cid in self.sending:
            self.sending[cid] = False
        self.btn_send.setChecked(False)
        self._update_enable_button(self._selected_cid())

    def on_release(self):
        self.on_disconnect()
        if self.cb_backend.currentData() == "socketcan":
            self._log("设备已释放（SocketCAN 接口已关闭）")
        else:
            self._log("设备已释放（SDK context 已销毁）")

    def _push_all_to_worker(self):
        if self.worker is None:
            return
        for s in self.specs:
            cid = s.can_id
            self._ensure_state(cid)
            self.worker.update_sp(cid, **self.setpoints[cid])
            self.worker.set_ff(cid, self.ff_params[cid])
            self.worker.set_zero_force(cid, self.zero_force[cid])
            lim = self.pos_limits[cid]
            self.worker.set_limits(cid, lim["enabled"], lim["lo"], lim["hi"])
            self.worker.set_sig(cid, self.signal_source[cid])

    # ---- 控制按钮
    def on_enable_toggled(self):
        cid = self._selected_cid()
        if cid is None or self.worker is None:
            return
        self._ensure_state(cid)
        want = not self.motor_enabled.get(cid, False)
        if want:
            self.worker.request("enable", cid, True)
            self.motor_enabled[cid] = True
        else:
            self.worker.request("enable", cid, False)
            self.motor_enabled[cid] = False
            self.sending[cid] = False
            with self._blocking(self.btn_send):
                self.btn_send.setChecked(False)
        self._update_enable_button(cid)

    def on_send_toggled(self, checked):
        cid = self._selected_cid()
        if cid is None or self.worker is None:
            with self._blocking(self.btn_send):
                self.btn_send.setChecked(False)
            return
        self._ensure_state(cid)
        if checked and not self.motor_enabled.get(cid, False):
            self._log("CAN 0x%02X 未使能，不能发送" % cid)
            with self._blocking(self.btn_send):
                self.btn_send.setChecked(False)
            return
        self.sending[cid] = bool(checked)
        self.worker.request("sending", cid, bool(checked))

    def on_apply_mode(self):
        cid = self._selected_cid()
        if cid is None or self.worker is None:
            return
        mode = self.cb_mode.currentText()
        self.motor_modes[cid] = mode
        self._apply_mode_keys(mode)
        self._update_ff_visibility(mode)
        self.worker.request("mode", cid, mode)

    def on_zero(self):
        cid = self._selected_cid()
        if cid is None or self.worker is None:
            return
        if QMessageBox.question(self, "确认", "把 CAN 0x%02X 当前位置设为零点？" % cid) \
                == QMessageBox.Yes:
            self.worker.request("zero", cid)

    # ---- 参数
    def on_read_param(self):
        cid = self.cb_pmotor.currentData()
        rid = self.cb_reg.currentData()
        if cid is not None and rid is not None and self.worker is not None:
            self.lbl_param_result.setText("读取中…")
            self.worker.request("readparam", cid, rid)

    def _on_param_result(self, cid, rid, value):
        if value is None:
            self.lbl_param_result.setText("超时/无回复")
        elif isinstance(value, int):
            self.lbl_param_result.setText("%d (0x%X)" % (value, value))
        else:
            self.lbl_param_result.setText("%.6f" % float(value))

    def on_write_param(self):
        cid = self.cb_pmotor.currentData()
        rid = self.cb_reg.currentData()
        if cid is None or rid is None or self.worker is None:
            return
        text = self.ed_pval.text().strip()
        # 与 SDK 的 Motor_Control.is_in_ranges 一致：这些 RID 是整数寄存器，
        # 其余（ACC/DEC/MAX_SPD/KP_ASR/KI_ASR/KP_APR/KI_APR）存的是 float。
        is_int_reg = (7 <= rid <= 10) or (13 <= rid <= 16) or (35 <= rid <= 36)
        try:
            value = int(text, 0) if is_int_reg else float(text)
        except ValueError:
            QMessageBox.warning(self, "输入有误", "写入值必须是数字")
            return
        self.worker.request("writeparam", cid, (rid, value))

    def on_save_flash(self):
        cid = self.cb_pmotor.currentData()
        if cid is not None and self.worker is not None:
            self.worker.request("saveflash", cid)

    # ============================================================ 渲染节拍
    def _curve_pull_args(self):
        rate = (self._last_rates.get("rx_hz")
                or self._last_rates.get("send_hz")
                or float(self.cb_hz.currentText()))
        tail = int(self.plot_panel.x_window * rate * 1.25) + 200
        tail = max(2000, tail)
        tail = 1 << (tail - 1).bit_length()   # 向上量化到 2 的幂
        px = self.plot_panel.plot_px()
        per_curve = int(px * CURVE_POINTS_PER_PX / self.plot_panel.n_curves())
        per_curve = max(CURVE_POINTS_MIN, min(CURVE_POINTS_MAX, per_curve))
        return tail, per_curve

    def _adapt_render_interval(self, period_ms):
        self._render_last_ms = period_ms
        if period_ms <= 0.0:
            return
        now = time.monotonic()
        if now - self._render_t0 < RENDER_ADAPT_COOLDOWN_S:
            return
        cur_ms = float(self._render_timer.interval())
        target = float(RENDER_INTERVAL_MS)
        if period_ms > cur_ms * 1.5:
            new_ms = min(float(RENDER_INTERVAL_MAX_MS), max(cur_ms, target) * 1.5)
        elif period_ms < cur_ms * 1.15:
            new_ms = max(target, cur_ms / 1.5)
        else:
            return
        new_ms = int(round(new_ms))
        if new_ms != int(cur_ms):
            self._render_timer.setInterval(new_ms)
            self._render_t0 = now

    def _on_render_tick(self):
        now = time.monotonic()
        period_ms = ((now - self._render_prev_t) * 1e3
                     if self._render_prev_t else 0.0)
        self._render_prev_t = now

        if self.worker is None:
            self._adapt_render_interval(period_ms)
            return

        include = self.plot_panel.isVisible()
        tail = max_points = None
        if include:
            tail, max_points = self._curve_pull_args()

        try:
            tel = self.worker.telemetry(max_points=max_points, tail=tail,
                                        include=include)
        except Exception as exc:
            self._log_throttled("[遥测] %s: %s" % (type(exc).__name__, exc))
            self._adapt_render_interval(period_ms)
            return

        self._last_rates = tel["rates"]
        if include:
            self.plot_panel.render(tel["samples"])

        self._render_tick += 1
        if self._render_tick % 3 == 0:
            self._update_status_table(tel["latest"])
            self._update_zero_force_display(tel["latest"])
            self._update_rate_label()
            self._sync_mode_label(tel["latest"])
        self._adapt_render_interval(period_ms)

    def _update_rate_label(self):
        r = self._last_rates or {}
        text = "采样率: 反馈 %.0f Hz | 发送 %.0f Hz (设定 %s Hz)" % (
            r.get("rx_hz", 0.0), r.get("send_hz", 0.0), self.cb_hz.currentText())
        err = int(r.get("sample_errors", 0))
        if err:
            text += " | 采样异常 %d" % err
        if self.worker is not None and not any(self.sending.values()):
            text += " | 未发送：MIT 模式下没有下发就没有反馈帧，曲线会停住"
        try:
            iv = max(1, int(self._render_timer.interval()))
            text += " | 渲染 %.0f Hz（实际节拍 %.0f ms）" % (
                1000.0 / iv, self._render_last_ms)
        except Exception:
            pass
        self.lbl_rate.setText(text)

    def _sync_mode_label(self, latest):
        cid = self._selected_cid()
        if cid is None or cid not in latest:
            return
        mode = latest[cid].get("mode", "MIT")
        if self.lbl_mode.text() != mode:
            self.lbl_mode.setText(mode)
            # 以驱动器实际模式为准刷新字段显隐：模式被拒时界面不会继续假装成功
            self._apply_mode_keys(mode)
            self._update_ff_visibility(mode)

    def _update_status_table(self, latest):
        for r, s in enumerate(self.specs):
            if r >= self.status_table.rowCount():
                break
            d = latest.get(s.can_id)
            if not d:
                continue
            err = int(d.get("err", 0))
            # USB 传输会把多帧批量交上来，回调时间戳可能只差几微秒。
            # 这时「周期」用最近一秒的实测平均帧率换算，比显示 0.0000 有意义。
            dt = float(d.get("interval", 0.0))
            if dt < 1e-4:
                rate = float(self._last_rates.get("rx_hz", 0.0) or 0.0)
                dt = (1.0 / rate) if rate > 0 else 0.0
            vals = ["0x%02X" % s.can_id, d.get("mode", "—"),
                    "%+.4f" % d.get("pos", 0.0), "%+.4f" % d.get("vel", 0.0),
                    "%+.3f" % d.get("tau", 0.0), str(err),
                    "是" if d.get("enabled") else "否",
                    "%.4f" % dt]
            for c, val in enumerate(vals):
                item = self.status_table.item(r, c)
                if item is None:
                    item = QTableWidgetItem()
                    self.status_table.setItem(r, c, item)
                item.setText(val)

    def _rebuild_status_table(self):
        self.status_table.setRowCount(len(self.specs))
        for r, s in enumerate(self.specs):
            self.status_table.setItem(r, 0, QTableWidgetItem("0x%02X" % s.can_id))
            for c in range(1, 8):
                self.status_table.setItem(r, c, QTableWidgetItem("—"))

    def _update_zero_force_display(self, latest):
        cid = self._selected_cid()
        if cid is None or not self.zero_force.get(cid):
            return
        d = latest.get(cid)
        if not d:
            return
        with self._blocking(self.sp_fields["q"], self.sp_fields["dq"],
                            self.sp_fields["tau"]):
            self.sp_fields["q"].setValue(d.get("pos", 0.0))
            self.sp_fields["dq"].setValue(d.get("vel", 0.0))
            self.sp_fields["tau"].setValue(d.get("ff_tau", 0.0))

    def _on_record_toggled(self, on):
        if self.worker is not None:
            self.worker.set_recording(bool(on))
        self._log("已开始记录曲线数据" if on else "已暂停记录（已有数据保留）")

    # ================================================================ 导出
    def on_export_csv(self):
        if self.worker is None:
            self._log("未连接，没有可导出的数据")
            return
        try:
            tel = self.worker.telemetry(include=True)   # 全量，不降采样
        except Exception as exc:
            self._log("取数据失败: %s" % exc)
            return
        samples = tel["samples"]
        if not samples:
            self._log("还没有采样数据")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "导出 CSV",
            "feedback_%s.csv" % time.strftime("%Y%m%d-%H%M%S"),
            "CSV 文件 (*.csv)")
        if not path:
            return

        cids = sorted(samples)
        t_ref = samples[cids[0]][0]
        n = len(t_ref)
        if n == 0:
            self._log("采样缓冲是空的")
            return
        t0 = float(t_ref[0])
        t_rel = t_ref - t0

        cols = [t_rel]
        header = ["t(s)"]
        for cid in cids:
            t_arr, chans = samples[cid]
            if len(t_arr) == n and np.array_equal(t_arr, t_ref):
                idx = np.arange(n)
            else:
                idx = np.searchsorted(t_arr, t_ref, side="left")
                np.clip(idx, 0, max(0, len(t_arr) - 1), out=idx)
                prev = np.maximum(idx - 1, 0)
                better = np.abs(t_arr[prev] - t_ref) < np.abs(t_arr[idx] - t_ref)
                idx = np.where(better, prev, idx)
            for ch in ("pos", "vel", "tau", "ff_tau", "cmd_q", "cmd_dq", "cmd_tau"):
                cols.append(chans[ch][idx].astype(np.float64))
                header.append("0x%X_%s" % (cid, ch))

        out = np.column_stack(cols)
        np.savetxt(path, out, delimiter=",", fmt="%.6f",
                   header=",".join(header), comments="")
        self._log("已导出 %d 行到 %s（全量未降采样）" % (n, path))

    def on_export_png(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "导出 PNG",
            "curves_%s.png" % time.strftime("%Y%m%d-%H%M%S"),
            "PNG 图片 (*.png)")
        if not path:
            return
        try:
            ok = self.plot_panel.export_png(path)
            self._log("已导出 " + path if ok else "导出失败（grab 返回空）")
        except Exception as exc:
            self._log("导出失败: %s" % exc)

    # ================================================================ 配置
    def _collect_config(self):
        # 注册表无效时直接抛错：宁可这次不保存，也不要用空列表覆盖掉好配置
        motors = [s.to_dict() for s in self._read_specs()]
        return {
            "motors": motors,
            "setpoints": {str(k): v for k, v in self.setpoints.items()},
            "ff_params": {str(k): v for k, v in self.ff_params.items()},
            "zero_force": {str(k): bool(v) for k, v in self.zero_force.items()},
            "pos_limits": {str(k): v for k, v in self.pos_limits.items()},
            "signal_source": {str(k): v for k, v in self.signal_source.items()},
            "splitter_h": list(self.splitter.sizes()),
            "window": [self.width(), self.height()],
            "tab_index": int(self.tabs.currentIndex()),
        }

    def on_save_config(self):
        try:
            path = config_store.save_config(self._collect_config())
            self._log("参数已保存到 " + path)
        except Exception as exc:
            self._log("保存失败: %s" % exc)

    def _load_config(self):
        cfg = config_store.load_json(config_store.config_path())
        if not cfg:
            return False
        motors = cfg.get("motors") or []
        if motors:
            self.motor_table.setRowCount(0)
            for m in motors:
                try:
                    self._add_motor_row(
                        int(m.get("can_id", 1)), int(m.get("mst_id", 1)),
                        m.get("model", "4340"), m.get("mode", "MIT"),
                        int(m.get("channel", DEFAULT_CHANNEL)))
                except Exception:
                    continue

        def _int_keys(d):
            return {int(k): v for k, v in (d or {}).items()}

        self.setpoints = {}
        for cid, v in _int_keys(cfg.get("setpoints")).items():
            sp = _default_sp()
            sp.update({k: float(x) for k, x in v.items() if k in sp})
            self.setpoints[cid] = sp
        self.ff_params = {}
        for cid, v in _int_keys(cfg.get("ff_params")).items():
            ff = _default_ff()
            for key, val in (v or {}).items():
                if key in ff and isinstance(val, dict):
                    ff[key].update(val)
            self.ff_params[cid] = ff
        self.zero_force = {cid: bool(v)
                           for cid, v in _int_keys(cfg.get("zero_force")).items()}
        self.pos_limits = {}
        for cid, v in _int_keys(cfg.get("pos_limits")).items():
            lim = _default_limit()
            lim.update({k: v[k] for k in ("enabled", "lo", "hi") if k in v})
            self.pos_limits[cid] = lim
        self.signal_source = {}
        for cid, v in _int_keys(cfg.get("signal_source")).items():
            sig = _default_sig()
            sig.update(v or {})
            self.signal_source[cid] = sig
        return True

    def _load_layout(self):
        lay = config_store.load_json(config_store.layout_path())
        if not lay:
            return
        try:
            sizes = lay.get("splitter_h")
            if isinstance(sizes, list) and len(sizes) == 2:
                self.splitter.setSizes([int(x) for x in sizes])
            win = lay.get("window")
            if isinstance(win, list) and len(win) == 2:
                w = max(800, min(int(win[0]), self.screen().availableGeometry().width()))
                h = max(600, min(int(win[1]), self.screen().availableGeometry().height()))
                self.resize(w, h)
            idx = int(lay.get("tab_index", 0))
            if 0 <= idx < self.tabs.count():
                self.tabs.setCurrentIndex(idx)
        except Exception:
            pass

    def _save_layout(self):
        if config_store.offscreen():
            return
        try:
            config_store.save_layout({
                "splitter_h": list(self.splitter.sizes()),
                "window": [self.width(), self.height()],
                "tab_index": int(self.tabs.currentIndex()),
            })
        except Exception:
            pass

    def closeEvent(self, ev):
        self._save_layout()
        if not config_store.offscreen():
            try:
                config_store.save_config(self._collect_config())
            except Exception:
                pass
        self.on_disconnect()
        super().closeEvent(ev)


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--backend", choices=["vendor", "socketcan"], default="vendor",
                    help="电机后端：vendor = 厂商 USB SDK；socketcan = can0")
    ap.add_argument("--can-channel", default="can0",
                    help="SocketCAN 接口名（默认 can0）")
    ap.add_argument("--can-bitrate", type=int, default=1_000_000,
                    help="SocketCAN 接口速率（仅显示/记录，界面仍需 ip link 配置）")
    args = ap.parse_args()

    app = QApplication(sys.argv)
    fams = load_cjk_font(app)
    print("字体:", fams if fams else "无中文字体")
    w = Hmi(sim=args.sim, backend=args.backend,
            can_channel=args.can_channel, can_bitrate=args.can_bitrate)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
