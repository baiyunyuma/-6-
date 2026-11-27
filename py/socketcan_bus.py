"""SocketCAN backend for the ZFY arm motor driver.

Drop-in replacement for ``core.MotorBus`` when the USB2CANFD module runs the
vendor **gs_usb** firmware (Linux ``gs_usb`` driver -> ``can0``).

Why: the vendor user-space SDK (``libdm_device.so``) keeps a fixed ring of 16
USB receive transfers and frees a transfer permanently when it completes with
ERROR/CANCELLED.  Under WSL/usbip that ring drains after a handful of frames
and feedback stops.  The kernel ``gs_usb`` driver resubmits URBs on its own and
exposes a standard SocketCAN device, so this module talks to ``can0`` instead.

The public surface mirrors ``core.MotorBus`` closely enough for
``zfy_arm_sim2real/motor_driver.py`` to use either backend unchanged:

    specs, _open_channels, channel_status, states, mode_of
    enable / release / mit / pos_vel / vel / pos_force
    read_param / write_param / save / zero / set_mode
    register / unregister / close / sample_sim
    mc facade: getMotor(ch, cid), refresh_motor_status(motor),
               control_cmd(id, cmd, ch), rx_hz

Frame encoding follows ``py/dm_backend/damiao.py`` (classic CAN, 11-bit ID,
8 data bytes).  ``sim=True`` keeps the same first-order model as MotorBus so
the ROS driver and HMI can be exercised without any hardware.
"""
from __future__ import annotations

import math
import os
import socket
import struct
import threading
import time
from typing import Callable, Dict, List, Optional

from motor_config import MODELS, MODE_CODES, MotorSpec

#: Control-frame CAN-ID offset (this is not the RID 10 value).
#:
#: Control frames go to the motor's ESC_ID (``MotorSpec.can_id``, RID 8) plus
#: this offset.  Feedback comes back on MST_ID (``MotorSpec.mst_id``, RID 7).
#: The two IDs may differ (e.g. control 0x11, feedback 0x01), so they are
#: tracked independently throughout this module.
MODE_OFFSET = {"MIT": 0x000, "POS_VEL": 0x100, "VEL": 0x200, "POS_FORCE": 0x300}

#: RID 10 = CTRL_MODE in the Damiao register map
CTRL_MODE_RID = 10

#: how long a 0x33/0x55/0xAA frame may still be treated as a parameter reply
PARAM_REPLY_WINDOW = 0.5

#: keep swallowing parameter-looking frames briefly after an exchange, so a
#: late reply is not decoded as MIT feedback
PARAM_DRAIN_WINDOW = 0.3

#: register value -> mode name (the inverse of MODE_CODES)
MODE_BY_CODE = {int(v): k for k, v in MODE_CODES.items()}

_CAN_FRAME = struct.Struct("=IB3x8s")
_SFF_MASK = 0x7FF
_EFF_MASK = 0x1FFFFFFF
_ERR_FLAG = 0x20000000
_RTR_FLAG = 0x40000000
_EFF_FLAG = 0x80000000
#: 非阻塞发送标志：队列满时 send 立即抛 BlockingIOError，
#: 而不是把 250 Hz 控制环卡在等队列可写上。
_MSG_DONTWAIT = getattr(socket, "MSG_DONTWAIT", 0)

#: RIDs whose payload is a little-endian uint32 instead of float32
def _is_int_rid(rid: int) -> bool:
    rid = int(rid)
    return (7 <= rid <= 10) or (13 <= rid <= 16) or (35 <= rid <= 36)


def _float_to_uint(value: float, lo: float, hi: float, bits: int) -> int:
    value = max(lo, min(hi, float(value)))
    return int((value - lo) / (hi - lo) * ((1 << bits) - 1))


def _uint_to_float(value: int, lo: float, hi: float, bits: int) -> float:
    return (float(value) / ((1 << bits) - 1)) * (hi - lo) + lo


def _limits(spec: MotorSpec):
    """(q_max, dq_max, tau_max) for the model."""
    return MODELS[spec.model][1:]


def encode_mit(spec: MotorSpec, kp: float, kd: float, q: float,
               dq: float, tau: float) -> bytes:
    """Pack one MIT command exactly like the vendor SDK."""
    q_max, dq_max, tau_max = _limits(spec)
    kp_uint = _float_to_uint(kp, 0.0, 500.0, 12)
    kd_uint = _float_to_uint(kd, 0.0, 5.0, 12)
    q_uint = _float_to_uint(q, -q_max, q_max, 16)
    dq_uint = _float_to_uint(dq, -dq_max, dq_max, 12)
    tau_uint = _float_to_uint(tau, -tau_max, tau_max, 12)
    data = bytearray(8)
    data[0] = (q_uint >> 8) & 0xFF
    data[1] = q_uint & 0xFF
    data[2] = (dq_uint >> 4) & 0xFF
    data[3] = ((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)
    data[4] = kp_uint & 0xFF
    data[5] = (kd_uint >> 4) & 0xFF
    data[6] = ((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)
    data[7] = tau_uint & 0xFF
    return bytes(data)


def decode_feedback(spec: MotorSpec, data: bytes):
    """Return (err, q, dq, tau) from an 8-byte MIT feedback frame."""
    if len(data) < 6:
        raise ValueError("feedback frame too short")
    err = (data[0] >> 4) & 0x0F
    q_uint = (data[1] << 8) | data[2]
    dq_uint = (data[3] << 4) | (data[4] >> 4)
    tau_uint = ((data[4] & 0x0F) << 8) | data[5]
    q_max, dq_max, tau_max = _limits(spec)
    return (
        err,
        _uint_to_float(q_uint, -q_max, q_max, 16),
        _uint_to_float(dq_uint, -dq_max, dq_max, 12),
        _uint_to_float(tau_uint, -tau_max, tau_max, 12),
    )


class _SocketCanMotor:
    """Small Motor-like object for the driver's ``mc.getMotor`` usage."""

    def __init__(self, bus: "SocketCanBus", spec: MotorSpec):
        self._bus = bus
        self.spec = spec
        self.state_q = 0.0
        self.state_dq = 0.0
        self.state_tau = 0.0
        self.state_err = 0
        self.param_map: Dict[int, float] = {}

    def GetCanId(self) -> int:
        return int(self.spec.can_id)

    def GetMasterId(self) -> int:
        return int(self.spec.mst_id)

    def GetChannel(self) -> int:
        return int(self.spec.channel)

    def GetMotorMode(self) -> int:
        mode = self._bus._modes.get(int(self.spec.can_id), str(self.spec.mode))
        return MODE_OFFSET.get(mode, 0)

    def get_limit_param(self):
        return _limits(self.spec)

    def receive_data(self, q: float, dq: float, tau: float, err: int = 0):
        self.state_q = float(q)
        self.state_dq = float(dq)
        self.state_tau = float(tau)
        self.state_err = int(err)

    def set_param(self, rid: int, value):
        self.param_map[int(rid)] = value

    def get_param_as_uint32(self, rid: int) -> int:
        v = self.param_map.get(int(rid), 0)
        try:
            return int(v)
        except Exception:
            return 0

    def get_param_as_float(self, rid: int) -> float:
        try:
            return float(self.param_map.get(int(rid), 0.0))
        except Exception:
            return 0.0

    def Get_Position(self) -> float:
        return self.state_q

    def Get_Velocity(self) -> float:
        return self.state_dq

    def Get_tau(self) -> float:
        return self.state_tau

    def Get_Err(self) -> int:
        return self.state_err


class _McFacade:
    """Compatibility shim for code that reaches through ``bus.mc``."""

    def __init__(self, bus: "SocketCanBus"):
        self._bus = bus

    def getMotor(self, *args):
        if len(args) == 1:
            cid = int(args[0])
        elif len(args) == 2:
            cid = int(args[1])
        else:
            raise TypeError("getMotor expects id or (channel, id)")
        return self._bus._motors_by_id.get(cid)

    def refresh_motor_status(self, motor) -> bool:
        cid = int(motor.GetCanId())
        ch = int(motor.GetChannel())
        payload = bytes([cid & 0xFF, (cid >> 8) & 0xFF, 0xCC, 0x00])
        return self._bus._send_frame(ch, 0x7FF, payload)

    def control_cmd(self, can_id: int, cmd: int, ch: int = 0) -> bool:
        return self._bus._send_frame(
            int(ch), int(can_id), bytes([0xFF] * 7 + [int(cmd) & 0xFF]))

    def read_motor_param(self, motor, rid: int, timeout: float = 1.0):
        return self._bus.read_param(int(motor.GetCanId()), rid, wait=timeout)

    def change_motor_param(self, motor, rid: int, value) -> bool:
        return bool(self._bus.write_param(int(motor.GetCanId()), rid, value))

    def save_motor_param(self, motor):
        return self._bus.save(int(motor.GetCanId()))

    @property
    def rx_hz(self) -> float:
        return self._bus.rx_hz


class SocketCanBus:
    """MotorBus-compatible SocketCAN transport."""

    def __init__(
        self,
        specs: List[MotorSpec],
        nom_baud: int = 1_000_000,
        dat_baud: int = 1_000_000,
        sim: bool = False,
        canfd: bool = False,
        set_baudrate: bool = True,
        device_index: int = 0,
        device_type: str = "single",
        on_sample: Optional[Callable] = None,
        channel: str = "can0",
        bitrate: Optional[int] = None,
        require_driver: Optional[str] = "gs_usb",
        channels: Optional[List[str]] = None,
    ):
        self.specs = list(specs)
        self.sim = bool(sim)
        self.channel_name = str(channel or "can0")
        self.bitrate = int(bitrate or nom_baud or 1_000_000)
        self.on_sample = on_sample
        #: expected kernel driver for the interface.  ``None`` disables the
        #: check (used by the vcan regression test).  On a Jetson the on-board
        #: ``can0`` is ``mttcan``, not the USB2CANFD's ``gs_usb``.
        self.require_driver = str(require_driver) if require_driver else None
        #: last transport-level error (kept for diagnostics / HMI logs)
        self.last_error: Optional[str] = None
        #: TX statistics.  send() is non-blocking, so a full queue drops the
        #: frame instead of stalling the control loop.
        self.tx_frames = 0
        self.tx_dropped = 0
        self.last_tx_error: Optional[str] = None

        #: channel index -> Linux interface name.  The dual USB2CANFD exposes
        #: two independent buses; ``channels[0]`` serves MotorSpec.channel 0,
        #: ``channels[1]`` serves MotorSpec.channel 1, etc.
        if channels:
            self._ifaces = {
                i: str(name)
                for i, name in enumerate(channels)
                if str(name or "").strip()
            }
        else:
            self._ifaces = {0: self.channel_name}
        if 0 not in self._ifaces:
            self._ifaces[0] = self.channel_name
        self._map_channels = {int(s.channel) for s in self.specs} or {0}
        missing = sorted(self._map_channels - set(self._ifaces))
        if missing:
            raise ValueError(
                "注册表用到 channel %s，但没有配置对应接口；"
                "请给 SocketCanBus 传 channels=[接口0, 接口1, ...]"
                % missing
            )

        self._lock = threading.RLock()
        self._latest: Dict[int, tuple] = {}
        self._latest_t: Dict[int, float] = {}
        self._modes: Dict[int, str] = {}
        self._motors_by_id: Dict[int, _SocketCanMotor] = {}
        self._by_can: Dict[int, MotorSpec] = {}
        self._by_mst: Dict[int, MotorSpec] = {}
        self._by_key: Dict[tuple, MotorSpec] = {}
        self._by_mst_key: Dict[tuple, MotorSpec] = {}
        self._open_channels = {int(s.channel) for s in self.specs} or {0}
        self._rebuild()

        # parameter gate: only frames that arrive while a 0x33/0x55 exchange is
        # pending are treated as parameter replies.
        self._param_lock = threading.Lock()
        self._param_event = threading.Event()
        self._param_expected: Optional[tuple] = None
        self._param_result = None
        self._param_active = False
        self._param_active_t = 0.0
        self._param_drain_until = 0.0

        self.rx_hz = 0.0
        self._rx_count = 0
        self._rx_t0 = time.monotonic()
        self._rx_lock = threading.Lock()

        self.channel_status: Dict[int, dict] = {}

        self._socks: Dict[int, socket.socket] = {}
        self._threads: Dict[int, threading.Thread] = {}
        self._stop = threading.Event()

        # simulation state (same one-pole model as MotorBus)
        self._sim_q = {int(s.can_id): 0.0 for s in self.specs}
        self._sim_target = {int(s.can_id): 0.0 for s in self.specs}
        self._sim_state: Dict[int, dict] = {}
        self._sim_enabled = set()
        self._sim_last = time.monotonic()

        self.mc = _McFacade(self)

        if self.sim:
            for s in self.specs:
                self._sim_state[s.can_id] = {
                    "q": 0.0, "dq": 0.0, "tau": 0.0, "err": 0,
                }
            self.channel_status = {
                ch: {"ok": True, "canfd": False, "nom": self.bitrate,
                     "dat": 0, "iface": self._ifaces.get(ch, "")}
                for ch in sorted(self._open_channels)
            } or {0: {"ok": True, "canfd": False, "nom": self.bitrate,
                      "dat": 0, "iface": self._ifaces.get(0, "")}}
            return

        self._open_socket()

    # ------------------------------------------------------------------ setup
    @staticmethod
    def _read_sysfs(path: str) -> Optional[str]:
        try:
            with open(path) as fh:
                return fh.read().strip()
        except Exception:
            return None

    def _interface_driver(self, iface: str) -> Optional[str]:
        link = "/sys/class/net/%s/device/driver" % iface
        try:
            if not os.path.exists(link):
                return None
            return os.path.basename(os.path.realpath(link))
        except Exception:
            return None

    def _check_interface(self, ch: int) -> None:
        """Fail fast if one interface is down or has the wrong driver."""
        ch = int(ch)
        iface = self._ifaces[ch]
        state = self._read_sysfs(
            "/sys/class/net/%s/operstate" % iface) or "unknown"
        driver = self._interface_driver(iface)
        is_vcan = iface.startswith("vcan")
        if self.require_driver and not is_vcan and driver != self.require_driver:
            raise RuntimeError(
                "接口 %r（channel %d）的驱动是 %r，不是 %r。Jetson 上 can0 "
                "可能是板载 mttcan；USB2CANFD 刷 gs_usb 固件后一般会新建 "
                "can1/can2。先用 `ip -d link show` 确认，再改 can_channels。"
                % (iface, ch, driver or "未知", self.require_driver)
            )
        if state == "down":
            raise RuntimeError(
                "接口 %r（channel %d）处于 DOWN。先执行 `sudo ip link set "
                "%s type can bitrate %d` 和 `sudo ip link set %s up`"
                % (iface, ch, iface, self.bitrate, iface)
            )
        self.channel_status[ch] = {
            "ok": True,
            "canfd": False,
            "nom": self.bitrate,
            "dat": 0,
            "operstate": state,
            "driver": driver,
            "iface": iface,
        }

    def _check_all(self) -> None:
        for ch in sorted(self._open_channels):
            self._check_interface(ch)

    def _bind_socket(self, ch: int) -> socket.socket:
        iface = self._ifaces[int(ch)]
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            sock.bind((iface,))
        except OSError as exc:
            try:
                sock.close()
            except Exception:
                pass
            raise RuntimeError(
                "无法绑定 SocketCAN 接口 %r（channel %d）: %s；先执行 "
                "`sudo ip link set %s type can bitrate %d` 和 "
                "`sudo ip link set %s up`"
                % (iface, int(ch), exc, iface, self.bitrate, iface)
            ) from exc
        sock.settimeout(0.2)
        return sock

    def _reopen(self, ch: int) -> bool:
        """(Re)create one interface's socket. Called from its RX thread."""
        ch = int(ch)
        if self._socks.get(ch) is not None:
            return True
        try:
            self._check_interface(ch)
            self._socks[ch] = self._bind_socket(ch)
            self.last_error = None
            return True
        except Exception as exc:
            self.last_error = "channel %d 重连失败: %s" % (ch, exc)
            sock = self._socks.pop(ch, None)
            try:
                if sock is not None:
                    sock.close()
            except Exception:
                pass
            st = self.channel_status.get(ch)
            if st is not None:
                st["ok"] = False
                st["err"] = self.last_error
            return False

    def _open_socket(self) -> None:
        self._check_all()
        for ch in sorted(self._open_channels):
            self._socks[ch] = self._bind_socket(ch)
            th = threading.Thread(
                target=self._rx_loop, args=(ch,),
                name="socketcan-rx-%d" % ch, daemon=True,
            )
            self._threads[ch] = th
            th.start()

    # -------------------------------------------------------------- registry
    def _rebuild(self) -> None:
        self._by_can = {}
        self._by_mst = {}
        self._by_key = {}
        self._by_mst_key = {}
        self._motors_by_id = {}
        for s in self.specs:
            cid, mid, ch = int(s.can_id), int(s.mst_id), int(s.channel)
            self._by_can[cid] = s
            self._by_mst[mid] = s
            self._by_key[(ch, cid)] = s
            self._by_mst_key[(ch, mid)] = s
            self._motors_by_id[cid] = _SocketCanMotor(self, s)
            self._modes[cid] = str(s.mode)

    def register(self, spec: MotorSpec) -> None:
        if not (0 < int(spec.can_id) <= 0x7FF) or not (0 < int(spec.mst_id) <= 0x7FF):
            raise ValueError("CAN/MST ID 必须在 1..0x7FF")
        if spec.model not in MODELS:
            raise ValueError("未知型号 %r" % spec.model)
        if spec.mode not in MODE_CODES:
            raise ValueError("未知模式 %r" % spec.mode)
        if int(spec.channel) not in self._open_channels:
            raise ValueError(
                "通道 %d 未在驱动启动时打开；换通道需要失能后重启驱动"
                % int(spec.channel)
            )
        with self._lock:
            used = set()
            for s in self.specs:
                if int(s.channel) == int(spec.channel):
                    used.update((int(s.can_id), int(s.mst_id)))
            if int(spec.can_id) in used or int(spec.mst_id) in used:
                raise ValueError(
                    "CAN/MST ID 冲突: CAN 0x%02X, MST 0x%02X"
                    % (spec.can_id, spec.mst_id)
                )
            self.specs.append(spec)
            cid, mid, ch = int(spec.can_id), int(spec.mst_id), int(spec.channel)
            self._by_can[cid] = spec
            self._by_mst[mid] = spec
            self._by_key[(ch, cid)] = spec
            self._by_mst_key[(ch, mid)] = spec
            self._motors_by_id[cid] = _SocketCanMotor(self, spec)
            self._modes[cid] = str(spec.mode)
            self._sim_q.setdefault(cid, 0.0)
            self._sim_target.setdefault(cid, 0.0)
            self._sim_state.setdefault(cid, {"q": 0.0, "dq": 0.0, "tau": 0.0, "err": 0})

    def unregister(self, can_id: int) -> None:
        cid = int(can_id)
        with self._lock:
            spec = self._by_can.get(cid)
            if spec is None:
                return
            self.specs = [s for s in self.specs if int(s.can_id) != cid]
            self._by_can.pop(cid, None)
            if self._by_mst.get(int(spec.mst_id)) is spec:
                self._by_mst.pop(int(spec.mst_id), None)
            self._by_key.pop((int(spec.channel), cid), None)
            if self._by_mst_key.get((int(spec.channel), int(spec.mst_id))) is spec:
                self._by_mst_key.pop((int(spec.channel), int(spec.mst_id)), None)
            self._motors_by_id.pop(cid, None)
            self._modes.pop(cid, None)
            self._latest.pop(cid, None)
            self._latest_t.pop(cid, None)

    # ------------------------------------------------------------------ RX
    def _rx_loop(self, ch: int) -> None:
        ch = int(ch)
        sock = self._socks.get(ch)
        while not self._stop.is_set():
            if sock is None:
                if not self._reopen(ch):
                    time.sleep(0.5)
                    continue
                sock = self._socks.get(ch)
                continue
            try:
                buf = sock.recv(16)
            except socket.timeout:
                continue
            except OSError as exc:
                # Interface went down / USB unplugged.  Do not die silently:
                # keep the error visible and retry in the background.
                self.last_error = "channel %d 接收失败: %s" % (ch, exc)
                st = self.channel_status.get(ch)
                if st is not None:
                    st["ok"] = False
                    st["err"] = self.last_error
                try:
                    sock.close()
                except Exception:
                    pass
                if self._socks.get(ch) is sock:
                    self._socks.pop(ch, None)
                sock = None
                time.sleep(0.2)
                continue
            if len(buf) < 16:
                continue
            raw_id, dlc, payload = _CAN_FRAME.unpack(buf)
            if raw_id & _ERR_FLAG:
                continue
            if raw_id & _RTR_FLAG:
                continue
            can_id = raw_id & (_EFF_MASK if raw_id & _EFF_FLAG else _SFF_MASK)
            data = payload[: max(0, min(int(dlc), 8))]
            now = time.monotonic()
            with self._rx_lock:
                self._rx_count += 1
                if now - self._rx_t0 >= 1.0:
                    self.rx_hz = self._rx_count / (now - self._rx_t0)
                    self._rx_count = 0
                    self._rx_t0 = now
            try:
                self._handle_frame(ch, can_id, data, now)
            except Exception:
                # never let a malformed frame kill the receive thread
                pass

    def _handle_frame(self, ch: int, can_id: int, data: bytes, now: float) -> None:
        # ---- parameter reply (0x33 read / 0x55 write ack / 0xAA save ack)
        # Bytes 0..1 carry a motor ID (ESC_ID or MST_ID).  A genuine MIT feedback frame
        # puts ERR in the high nibble of byte 0 and q in bytes 1..2, so near
        # q = -q_max it can look exactly like a 0x33/0x55/0xAA reply.  Only
        # treat such a frame as a reply while an exchange is pending (or in
        # the short drain window after it); otherwise let it fall through to
        # the normal feedback decoder.
        with self._param_lock:
            active = (
                self._param_active
                and (now - self._param_active_t) < PARAM_REPLY_WINDOW
            )
            expected = self._param_expected
        if ((active or now < self._param_drain_until) and len(data) >= 8
                and data[2] in (0x33, 0x55, 0xAA)):
            cid = int(data[0]) | (int(data[1]) << 8)
            with self._lock:
                # 参数回复里的 ID 可能是控制 ID（ESC_ID/can_id），也可能是
                # 回传 ID（MST_ID/mst_id）。两种都接受，与厂商 SDK 一致。
                spec = (self._by_key.get((int(ch), cid))
                        or self._by_mst_key.get((int(ch), cid)))
                if spec is None:
                    cand = self._by_can.get(cid) or self._by_mst.get(cid)
                    if cand is not None and int(cand.channel) == int(ch):
                        spec = cand
            if spec is not None:
                # expected = (channel, control_id/can_id, rid)。回复里的 ID
                # 可能和请求不同，所以按解析出的 spec 判断是不是在等的那台。
                same_motor = (
                    expected is None
                    or int(spec.can_id) == int(expected[1])
                )
                if active and same_motor:
                    rid = int(data[3])
                    if data[2] in (0x33, 0x55):
                        if expected is None or rid == expected[2]:
                            value = self._parse_param(rid, data[4:8])
                            motor = self._motors_by_id.get(int(spec.can_id))
                            if motor is not None and value is not None:
                                motor.set_param(rid, value)
                                if rid == CTRL_MODE_RID:
                                    self._apply_mode_code(
                                        int(spec.can_id), value)
                            with self._param_lock:
                                self._param_result = value
                                self._param_active = False
                                self._param_expected = None
                            self._param_event.set()
                    else:
                        # 0xAA save ack: accept by motor, RID byte may differ
                        with self._param_lock:
                            self._param_active = False
                            self._param_expected = None
                        self._param_event.set()
                return

        # ---- normal MIT feedback
        if can_id == 0x7FF or len(data) < 6:
            return
        with self._lock:
            # 反馈回传 ID = MST_ID (mst_id)；控制 ID = ESC_ID (can_id)。
            # 优先按回传 ID 匹配；同时接受控制 ID，兼容少数在 ESC_ID 上
            # 回帧的固件。
            spec = (self._by_mst_key.get((int(ch), can_id))
                    or self._by_key.get((int(ch), can_id)))
            if spec is None:
                cand = self._by_mst.get(can_id) or self._by_can.get(can_id)
                if cand is not None and int(cand.channel) == int(ch):
                    spec = cand
        if spec is None:
            return
        err, q, dq, tau = decode_feedback(spec, data)
        cid = int(spec.can_id)
        motor = self._motors_by_id.get(cid)
        if motor is not None:
            motor.receive_data(q, dq, tau, err)
        with self._lock:
            self._latest[cid] = (q, dq, tau, err)
            self._latest_t[cid] = now
        cb = self.on_sample
        if cb is not None:
            try:
                cb(cid, now, q, dq, tau, err)
            except Exception:
                pass

    @staticmethod
    def _parse_param(rid: int, raw: bytes):
        if len(raw) < 4:
            return None
        if _is_int_rid(rid):
            return int.from_bytes(raw[:4], byteorder="little", signed=False)
        return struct.unpack("<f", raw[:4])[0]

    def _apply_mode_code(self, can_id: int, value) -> None:
        """Keep the local mode in sync after a RID 10 read/write reply.

        ``write_param``/``read_param`` go through raw registers, unlike
        ``set_mode``; without this the next ``enable()`` would use a stale
        command-ID offset and ``mode_of()`` would lie.
        """
        try:
            name = MODE_BY_CODE.get(int(value))
        except Exception:
            return
        if name is None:
            return
        cid = int(can_id)
        self._modes[cid] = name
        spec = self._by_can.get(cid)
        if spec is not None:
            spec.mode = name

    # --------------------------------------------------------------- transmit
    def _channel_of(self, can_id: int) -> int:
        spec = self._by_can.get(int(can_id))
        return int(spec.channel) if spec is not None else 0

    def _send_frame(self, ch: int, can_id: int, payload,
                    dlc: Optional[int] = None) -> bool:
        sock = self._socks.get(int(ch))
        if sock is None or self.sim:
            return False
        data = bytes(payload)
        n = len(data) if dlc is None else int(dlc)
        if n > 8:
            n = 8
        try:
            buf = _CAN_FRAME.pack(int(can_id) & _SFF_MASK, n, data[:n].ljust(8, b"\0"))
            # 非阻塞：发送队列满时立即丢帧，绝不把控制环卡在 send 上。
            sock.send(buf, _MSG_DONTWAIT)
            self.tx_frames += 1
            return True
        except OSError as exc:
            self.tx_dropped += 1
            self.last_tx_error = str(exc)
            return False

    def _offset(self, cid: int) -> int:
        spec = self._by_can.get(int(cid))
        mode = self._modes.get(int(cid), spec.mode if spec else "MIT")
        return MODE_OFFSET.get(mode, 0)

    # ---------------------------------------------------------------- control
    def enable(self, can_id: int, on: bool, switch_mode: bool = True) -> None:
        cid = int(can_id)
        if self.sim:
            if on:
                self._sim_enabled.add(cid)
            else:
                self._sim_enabled.discard(cid)
            return
        spec = self._by_can.get(cid)
        if spec is None:
            return
        ch = int(spec.channel)
        mode = self._modes.get(cid, spec.mode)
        if on and switch_mode:
            code = MODE_CODES.get(mode)
            if code is not None:
                if not self._write_param_raw(
                        cid, 10, bytes([code, 0, 0, 0]), wait=0.1, ch=ch):
                    raise RuntimeError(
                        "CAN 0x%02X 切换控制模式 %s 未收到应答" % (cid, mode)
                    )
            time.sleep(0.002)
        offset = MODE_OFFSET.get(mode, 0)
        for _ in range(5):
            self.mc.control_cmd(cid + offset, 0xFC if on else 0xFD, ch)
            time.sleep(0.002)

    def set_mode(self, can_id: int, mode: str) -> bool:
        cid = int(can_id)
        mode = str(mode).strip().upper()
        if mode not in MODE_CODES:
            return False
        if self.sim:
            self._modes[cid] = mode
            spec = self._by_can.get(cid)
            if spec is not None:
                spec.mode = mode
            return True
        # 只有电机回了 0x55 才更新本地模式；否则后续控制帧会发到错误偏移。
        ok = self._write_param_raw(cid, 10, bytes([MODE_CODES[mode], 0, 0, 0]),
                                   wait=0.1, ch=self._channel_of(cid))
        if ok:
            self._modes[cid] = mode
            spec = self._by_can.get(cid)
            if spec is not None:
                spec.mode = mode
        time.sleep(0.05)
        return ok

    def mit(self, can_id: int, kp: float, kd: float, q: float,
            dq: float, tau: float) -> None:
        cid = int(can_id)
        if self.sim:
            self._sim_target[cid] = float(q)
            return
        spec = self._by_can.get(cid)
        if spec is None:
            return
        # 控制帧发到 ESC_ID（can_id）+ 模式偏移；反馈从 MST_ID（mst_id）回。
        self._send_frame(
            int(spec.channel),
            cid + self._offset(cid),
            encode_mit(spec, kp, kd, q, dq, tau),
        )

    def pos_vel(self, can_id: int, pos: float, vel: float) -> None:
        cid = int(can_id)
        if self.sim:
            self._sim_target[cid] = float(pos)
            return
        if cid not in self._by_can:
            return
        self._send_frame(self._channel_of(cid), cid + 0x100,
                         struct.pack("<ff", float(pos), float(vel)))

    def vel(self, can_id: int, vel: float) -> None:
        cid = int(can_id)
        if self.sim:
            self._sim_target[cid] = self._sim_q.get(cid, 0.0) + float(vel) * 0.5
            return
        if cid not in self._by_can:
            return
        self._send_frame(self._channel_of(cid), cid + 0x200,
                         struct.pack("<f", float(vel)))

    def pos_force(self, can_id: int, pos: float, tau: float) -> None:
        cid = int(can_id)
        if self.sim:
            self._sim_target[cid] = float(pos)
            return
        if cid not in self._by_can:
            return
        self._send_frame(self._channel_of(cid), cid + 0x300,
                         struct.pack("<ff", float(pos), float(tau)))

    def release(self, can_id: int) -> None:
        self.mit(can_id, 0.0, 0.0, 0.0, 0.0, 0.0)

    # -------------------------------------------------------------- parameters
    def _write_param_raw(self, can_id: int, rid: int, data: bytes,
                         wait: float = 0.05, ch: Optional[int] = None) -> bool:
        cid = int(can_id)
        ch = self._channel_of(cid) if ch is None else int(ch)
        data = bytes(data)
        if len(data) != 4:
            raise ValueError("data must contain exactly 4 bytes")
        now = time.monotonic()
        with self._param_lock:
            self._param_result = None
            self._param_expected = (ch, cid, int(rid))
            self._param_active = True
            self._param_active_t = now
            self._param_drain_until = now + PARAM_DRAIN_WINDOW
            self._param_event.clear()
        payload = bytes([cid & 0xFF, (cid >> 8) & 0xFF, 0x55, int(rid)]) + data
        sent = self._send_frame(ch, 0x7FF, payload)
        acked = False
        if wait and sent:
            acked = self._param_event.wait(float(wait))
        with self._param_lock:
            self._param_active = False
            self._param_expected = None
        # 只有收到电机的 0x55 应答才算写入成功；仅“帧发出去了”不算。
        return bool(sent and acked)

    def read_param(self, can_id: int, rid: int, wait: float = 0.2):
        cid = int(can_id)
        rid = int(rid)
        if self.sim:
            return MODE_CODES.get("MIT", 1) if rid == 10 else 0
        ch = self._channel_of(cid)
        now = time.monotonic()
        with self._param_lock:
            self._param_result = None
            self._param_expected = (ch, cid, rid)
            self._param_active = True
            self._param_active_t = now
            self._param_drain_until = now + PARAM_DRAIN_WINDOW
            self._param_event.clear()
        payload = bytes([cid & 0xFF, (cid >> 8) & 0xFF, 0x33, rid, 0, 0, 0, 0])
        ok = self._send_frame(ch, 0x7FF, payload)
        value = None
        if ok and wait and self._param_event.wait(float(wait)):
            with self._param_lock:
                value = self._param_result
        with self._param_lock:
            self._param_active = False
            self._param_expected = None
        return value

    def write_param(self, can_id: int, rid: int, value) -> bool:
        cid = int(can_id)
        rid = int(rid)
        if self.sim:
            if rid == 10:
                for name, code in MODE_CODES.items():
                    if code == int(value):
                        self._modes[cid] = name
            return True
        if _is_int_rid(rid):
            data = int(round(float(value))).to_bytes(4, "little", signed=False)
        else:
            data = struct.pack("<f", float(value))
        return self._write_param_raw(cid, rid, data, wait=0.05)

    def save(self, can_id: int) -> bool:
        cid = int(can_id)
        if self.sim:
            return True
        ch = self._channel_of(cid)
        self.mc.control_cmd(cid + self._offset(cid), 0xFD, ch)
        time.sleep(0.01)
        # The 0xAA ack must be consumed as a parameter frame; otherwise it
        # would be parsed as MIT feedback and corrupt q/dq/tau/err.
        now = time.monotonic()
        with self._param_lock:
            self._param_expected = (ch, cid, 0x01)
            self._param_active = True
            self._param_active_t = now
            self._param_drain_until = now + PARAM_DRAIN_WINDOW
            self._param_event.clear()
        payload = bytes([cid & 0xFF, (cid >> 8) & 0xFF, 0xAA, 0x01, 0, 0, 0, 0])
        sent = self._send_frame(ch, 0x7FF, payload)
        # 保存 Flash 的应答可能比普通参数写稍慢，给到 0.1 s。
        acked = self._param_event.wait(0.1) if sent else False
        with self._param_lock:
            self._param_active = False
            self._param_expected = None
        time.sleep(0.1)
        return bool(sent and acked)

    def zero(self, can_id: int) -> None:
        cid = int(can_id)
        if self.sim:
            return
        self.mc.control_cmd(cid + self._offset(cid), 0xFE,
                            self._channel_of(cid))
        time.sleep(0.002)

    # ------------------------------------------------------------------ state
    def states(self) -> Dict[int, dict]:
        if self.sim:
            return {cid: dict(st) for cid, st in self._sim_state.items()}
        out = {}
        with self._lock:
            for cid in self._by_can:
                st = self._latest.get(cid)
                if st is None:
                    out[cid] = {"q": 0.0, "dq": 0.0, "tau": 0.0, "err": 0}
                else:
                    out[cid] = {
                        "q": st[0], "dq": st[1], "tau": st[2], "err": st[3],
                    }
        return out

    def mode_of(self, can_id: int) -> str:
        spec = self._by_can.get(int(can_id))
        return self._modes.get(int(can_id), spec.mode if spec else "MIT")

    def has_motor(self, can_id: int) -> bool:
        return int(can_id) in self._by_can

    def _motor(self, can_id: int):
        return self._motors_by_id.get(int(can_id))

    def sample_sim(self, now: Optional[float] = None) -> None:
        if not self.sim:
            return
        now = time.monotonic() if now is None else float(now)
        dt = min(max(now - self._sim_last, 0.0), 0.1)
        self._sim_last = now
        for s in self.specs:
            cid = int(s.can_id)
            cur = self._sim_q.get(cid, 0.0)
            tgt = self._sim_target.get(cid, 0.0)
            cur += (tgt - cur) * (1.0 - math.exp(-dt / 0.15))
            self._sim_q[cid] = cur
            dq = (tgt - cur) * 6.0
            tau = 0.4 * (tgt - cur)
            err = 1 if cid in self._sim_enabled else 0
            st = {"q": cur, "dq": dq, "tau": tau, "err": err}
            self._sim_state[cid] = st
            with self._lock:
                self._latest[cid] = (cur, dq, tau, err)
                self._latest_t[cid] = now
            cb = self.on_sample
            if cb is not None:
                try:
                    cb(cid, now, cur, dq, tau, err)
                except Exception:
                    pass

    def close(self) -> None:
        # Match MotorBus.close(): never leave a motor enabled after the owner
        # disappears.  The ROS driver disables explicitly first; the Qt HMI
        # only calls close(), so this is the last line of defense.
        if not self.sim and self._socks:
            for spec in list(self.specs):
                cid = int(spec.can_id)
                ch = int(spec.channel)
                offset = MODE_OFFSET.get(
                    self._modes.get(cid, str(spec.mode)), 0)
                for _ in range(3):
                    try:
                        self._send_frame(
                            ch, cid + offset, bytes([0xFF] * 7 + [0xFD]))
                    except Exception:
                        pass
                    time.sleep(0.002)
        self._stop.set()
        for th in list(self._threads.values()):
            if th is not None and th.is_alive():
                th.join(timeout=0.6)
        self._threads.clear()
        for sock in list(self._socks.values()):
            try:
                sock.close()
            except Exception:
                pass
        self._socks.clear()


__all__ = ["SocketCanBus", "MODE_OFFSET"]
