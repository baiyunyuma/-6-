"""
电机总线封装 —— 基于 DM Device SDK（dm_backend）。

和旧版 libu2canfd 的关键差别：
  * 可以显式选择「经典 CAN 帧」（canfd=False）—— 旧 SDK 只会发 FD 帧
  * 电机按 (channel, can_id) 注册，channel 必须与实物接线一致（实测是 1）
  * 反馈帧的 channel 字段恒为 0，dm_backend/damiao.py 里已加按 can_id 兜底
  * **每解析出一帧反馈就回调 on_sample**，采样率 = 反馈率，不丢帧

sim=True 时不碰硬件，用一阶响应生成数据，方便调界面。
"""

import math
import time
from typing import Callable, Dict, List, Optional

from motor_config import MODE_BY_VALUE, MODE_ENUM_NAMES, MODELS, MODE_CODES, MotorSpec

#: 反馈回调签名：(can_id, t, q, dq, tau, err) -> None
SampleCallback = Callable[[int, float, float, float, float, int], None]


class MotorBus:
    def __init__(self, specs: List[MotorSpec], nom_baud: int = 1_000_000,
                 dat_baud: int = 1_000_000, sim: bool = False,
                 canfd: bool = False, set_baudrate: bool = True,
                 device_index: int = 0, device_type: str = "single",
                 on_sample: Optional[SampleCallback] = None):
        self.specs = list(specs)
        self.sim = sim
        self.canfd = canfd
        self.on_sample = on_sample
        self.mc = None

        self._by_can = {s.can_id: s for s in self.specs}
        self._by_key = {(s.channel, s.can_id): s for s in self.specs}
        self._by_mst = {s.mst_id: s for s in self.specs}
        self._by_mst_key = {(s.channel, s.mst_id): s for s in self.specs}
        #: 启动时已打开/配置过的通道；运行时注册只允许这些通道
        self._open_channels = {s.channel for s in self.specs}
        self._motors = {}            # can_id -> SDK Motor
        #: ch -> 波特率回读结果（真实模式才有内容）
        self.channel_status = {}
        #: 上层认为电机当前应处的模式（使能前会用它校准驱动器）
        self._modes = {s.can_id: s.mode for s in self.specs}

        # 仿真状态
        self._sim_q = {s.can_id: 0.0 for s in self.specs}
        self._sim_target = {s.can_id: 0.0 for s in self.specs}
        self._sim_state = {}
        self._sim_enabled = set()
        self._sim_last = time.monotonic()

        if not sim:
            import dm_backend as dm
            from dmcan import dmcan_device_type

            dtype_name = ("USB2CANFD_DUAL" if str(device_type).lower() == "dual"
                          else "USB2CANFD")
            dtype = getattr(dmcan_device_type, dtype_name, None)
            if dtype is None:          # 老 SDK 没有双路类型时退回单路
                dtype = dmcan_device_type.USB2CANFD

            init = []
            for s in self.specs:
                mode_enum = getattr(dm.Control_Mode, MODE_ENUM_NAMES.get(s.mode, "MIT_MODE"))
                init.append(dm.DmActData(
                    motorType=getattr(dm.DM_Motor_Type, MODELS[s.model][0]),
                    mode=mode_enum,
                    can_id=s.can_id,
                    mst_id=s.mst_id,
                    channel=s.channel,
                ))
            self.mc = dm.Motor_Control(
                nom_baud, dat_baud, "", init,
                device_index=int(device_index),
                device_type=dtype,
                auto_enable=False,     # 由调用方显式使能
                set_baudrate=set_baudrate,
                canfd=canfd,
                brs=canfd,
            )
            # 每帧反馈的钩子：SDK 接收线程里同步调用，必须快
            self.mc.on_sample = self._dispatch_sample
            for s in self.specs:
                m = self.mc.getMotor(s.channel, s.can_id)
                if m is not None:
                    self._motors[s.can_id] = m

            # 回读各通道的实际波特率配置。单通道模块上把电机注册到通道 1 时，
            # SDK 只会打印一行 warn 到 stderr；这里显式记下来给界面提示。
            self.channel_status = {}
            for ch in sorted({s.channel for s in self.specs}):
                st = {"ok": None, "canfd": None, "nom": None, "dat": None}
                try:
                    info = self.mc.device.get_channel_baudrate(ch)
                    st.update(canfd=bool(info.canfd),
                              nom=int(info.can_baudrate),
                              dat=int(info.canfd_baudrate))
                    st["ok"] = (st["nom"] == int(nom_baud)
                                and st["canfd"] == bool(canfd)
                                and (not canfd or st["dat"] == int(dat_baud)))
                except Exception as exc:
                    st["err"] = str(exc)
                self.channel_status[ch] = st

    # ---------------------------------------------------------- 运行时注册
    def registered_specs(self) -> List[MotorSpec]:
        return list(self.specs)

    def _rebuild_id_aliases_locked(self) -> None:
        """Rebuild SDK _motors_by_id after add/remove.

        addMotor uses setdefault, so a removed+re-added motor can leave a
        stale can_id/mst_id alias pointing at the old object.
        """
        mc = self.mc
        if mc is None:
            return
        mc._motors_by_id.clear()
        for motor in mc._unique_motors():
            mc._motors_by_id.setdefault(motor.GetCanId(), motor)
            mc._motors_by_id.setdefault(motor.GetMasterId(), motor)

    def register(self, spec: MotorSpec) -> None:
        """Register one motor on an already-open channel (no USB reopen)."""
        if not (0 < int(spec.can_id) <= 0x7FF) or not (0 < int(spec.mst_id) <= 0x7FF):
            raise ValueError("CAN/MST ID 必须在 1..0x7FF")
        if spec.model not in MODELS:
            raise ValueError(f"未知型号 {spec.model!r}")
        if spec.mode not in MODE_CODES:
            raise ValueError(f"未知模式 {spec.mode!r}")
        if int(spec.channel) not in self._open_channels:
            raise ValueError(
                f"通道 {spec.channel} 未在驱动启动时打开；"
                "换通道需要全部失能后重启驱动"
            )
        used = set()
        for s in self.specs:
            if int(s.channel) == int(spec.channel):
                used.update((int(s.can_id), int(s.mst_id)))
        if int(spec.can_id) in used or int(spec.mst_id) in used:
            raise ValueError(
                f"CAN/MST ID 冲突: CAN 0x{spec.can_id:02X}, "
                f"MST 0x{spec.mst_id:02X}"
            )

        if self.sim:
            self._sim_q[spec.can_id] = 0.0
            self._sim_target[spec.can_id] = 0.0
            self._sim_state[spec.can_id] = {
                "q": 0.0, "dq": 0.0, "tau": 0.0, "err": 0,
            }
        else:
            import dm_backend as dm
            from dm_backend.damiao import Motor as DmMotor
            motor = DmMotor(
                getattr(dm.DM_Motor_Type, MODELS[spec.model][0]),
                getattr(dm.Control_Mode, MODE_ENUM_NAMES.get(spec.mode, "MIT_MODE")),
                int(spec.can_id), int(spec.mst_id), int(spec.channel),
            )
            with self.mc._lock:
                self.mc.addMotor(motor)
                self._rebuild_id_aliases_locked()
            self._motors[spec.can_id] = motor

        self.specs.append(spec)
        self._by_can[spec.can_id] = spec
        self._by_key[(spec.channel, spec.can_id)] = spec
        self._by_mst[spec.mst_id] = spec
        self._by_mst_key[(spec.channel, spec.mst_id)] = spec
        self._modes[spec.can_id] = spec.mode

    def unregister(self, can_id: int) -> None:
        """Remove a motor from the live bus. Caller must disable it first."""
        spec = self._by_can.get(int(can_id))
        if spec is None:
            return
        if not self.sim and self.mc is not None:
            motor = self._motors.get(spec.can_id)
            with self.mc._lock:
                if motor is not None:
                    for key, m in list(self.mc.motors.items()):
                        if m is motor:
                            del self.mc.motors[key]
                self._rebuild_id_aliases_locked()
        self.specs = [s for s in self.specs if s.can_id != spec.can_id]
        self._by_can.pop(spec.can_id, None)
        self._by_key.pop((spec.channel, spec.can_id), None)
        if self._by_mst.get(spec.mst_id) is spec:
            self._by_mst.pop(spec.mst_id, None)
        if self._by_mst_key.get((spec.channel, spec.mst_id)) is spec:
            self._by_mst_key.pop((spec.channel, spec.mst_id), None)
        self._motors.pop(spec.can_id, None)
        self._modes.pop(spec.can_id, None)
        self._sim_q.pop(spec.can_id, None)
        self._sim_target.pop(spec.can_id, None)
        self._sim_state.pop(spec.can_id, None)
        self._sim_enabled.discard(spec.can_id)

    # ------------------------------------------------------------------ 采样
    def _dispatch_sample(self, channel, can_id, t, q, dq, tau, err=None):
        """SDK 接收线程里调用（持着 SDK 锁）。只做查找 + 转发，不抛异常。"""
        try:
            cid = int(can_id)
            # 反馈帧的仲裁 ID 可能是 CAN_ID 或 MST_ID，channel 字段也可能恒为 0
            spec = (
                self._by_key.get((channel, cid))
                or self._by_mst_key.get((channel, cid))
                or self._by_can.get(cid)
                or self._by_mst.get(cid)
            )
            if spec is None:
                return
            if err is None:
                m = self._motors.get(spec.can_id)
                err = 0
                if m is not None:
                    try:
                        err = int(m.Get_Err())
                    except Exception:
                        err = 0
            cb = self.on_sample
            if cb is not None:
                cb(spec.can_id, float(t), float(q), float(dq), float(tau), err)
        except Exception:
            pass

    def sample_sim(self, now: Optional[float] = None) -> None:
        """仿真：积分一阶模型，然后走与实机相同的采样回调。"""
        if not self.sim:
            return
        now = time.monotonic() if now is None else now
        dt = min(max(now - self._sim_last, 0.0), 0.1)
        self._sim_last = now
        for s in self.specs:
            cur = self._sim_q[s.can_id]
            tgt = self._sim_target[s.can_id]
            cur += (tgt - cur) * (1.0 - math.exp(-dt / 0.15))
            self._sim_q[s.can_id] = cur
            dq = (tgt - cur) * 6.0
            tau = 0.4 * (tgt - cur)
            err = 1 if s.can_id in self._sim_enabled else 0
            self._sim_state[s.can_id] = {"q": cur, "dq": dq, "tau": tau, "err": err}
            self._dispatch_sample(s.channel, s.can_id, now, cur, dq, tau, err)

    def states(self) -> Dict[int, Dict[str, float]]:
        """can_id -> {q, dq, tau, err}（最近一帧的缓存值，不触发采样）。"""
        if self.sim:
            return {cid: dict(st) for cid, st in self._sim_state.items()}
        out = {}
        for s in self.specs:
            m = self._motors.get(s.can_id)
            if m is None:
                continue
            out[s.can_id] = {"q": m.state_q, "dq": m.state_dq, "tau": m.state_tau,
                             "err": getattr(m, "state_err", 0)}
        return out

    def mode_of(self, can_id: int) -> str:
        """驱动器**实际**的控制模式（不是界面下拉框）。"""
        if self.sim:
            return self._modes.get(can_id, "MIT")
        m = self._motors.get(can_id)
        if m is None:
            return "MIT"
        try:
            return MODE_BY_VALUE.get(int(m.GetMotorMode()), "MIT")
        except Exception:
            return "MIT"

    def has_motor(self, can_id: int) -> bool:
        return self.sim or can_id in self._motors

    # ------------------------------------------------------------------ 控制
    def _motor(self, can_id: int):
        if self.mc is None:
            return None
        s = self._by_can.get(can_id)
        if s is None:
            return None
        return self.mc.getMotor(s.channel, can_id)

    def enable(self, can_id: int, on: bool, switch_mode: bool = True) -> None:
        """使能/失能。系统命令的 CAN ID = 电机 ID + 当前模式的偏移量。

        switch_mode=False 时跳过 CTRL_MODE 写入，用于已经确认处于目标模式的电机。
        """
        if self.sim:
            if on:
                self._sim_enabled.add(can_id)
            else:
                self._sim_enabled.discard(can_id)
            return
        m = self._motor(can_id)
        if m is None:
            return
        if on and switch_mode:
            # 使能前先把控制模式校准到上层认为的模式（与参考工具一致）
            try:
                spec = self._by_can.get(can_id)
                mode = self._modes.get(can_id, spec.mode if spec else "MIT")
                self.mc.switchControlMode(m, MODE_CODES[mode])
            except Exception:
                pass
            time.sleep(0.002)
        cid = int(m.GetCanId()) + int(m.GetMotorMode())
        ch = int(m.GetChannel())     # 必须用电机所在通道（默认形参是 0，会发错通道）
        for _ in range(5):           # 与官方例程一致：连发 5 次，容忍丢帧
            self.mc.control_cmd(cid, 0xFC if on else 0xFD, ch)
            time.sleep(0.002)

    def set_mode(self, can_id: int, mode: str) -> bool:
        """写 CTRL_MODE。返回驱动器是否确认。"""
        if self.sim:
            self._modes[can_id] = mode
            return True
        m = self._motor(can_id)
        if m is None:
            return False
        import dm_backend as dm
        try:
            ok = bool(self.mc.switchControlMode(m, getattr(dm.Control_Mode_Code, mode)))
        except Exception:
            ok = False
        if ok:
            self._modes[can_id] = mode
        time.sleep(0.05)
        return ok

    def mit(self, can_id: int, kp: float, kd: float, q: float,
            dq: float, tau: float) -> None:
        if self.sim:
            self._sim_target[can_id] = q
            return
        m = self._motor(can_id)
        if m is None:
            return
        self.mc.control_mit(m, kp, kd, q, dq, tau)

    def pos_vel(self, can_id: int, pos: float, vel: float) -> None:
        if self.sim:
            self._sim_target[can_id] = pos
            return
        m = self._motor(can_id)
        if m is not None:
            self.mc.control_pos_vel(m, pos, vel)

    def vel(self, can_id: int, vel: float) -> None:
        if self.sim:
            self._sim_target[can_id] = self._sim_q[can_id] + vel * 0.5
            return
        m = self._motor(can_id)
        if m is not None:
            self.mc.control_vel(m, vel)

    def pos_force(self, can_id: int, pos: float, tau: float) -> None:
        if self.sim:
            self._sim_target[can_id] = pos
            return
        m = self._motor(can_id)
        if m is not None:
            self.mc.control_pos_force(m, pos, tau)

    def release(self, can_id: int) -> None:
        self.mit(can_id, 0.0, 0.0, 0.0, 0.0, 0.0)

    # -------------------------------------------------------------- 参数
    def read_param(self, can_id: int, rid: int, wait: float = 0.2):
        """读寄存器。返回 int / float / None（超时）。

        用 SDK 自己的 read_motor_param：它会按寄存器类型选择 uint32 还是 float，
        不能一律 get_param_as_uint32 —— 浮点寄存器那样读会永远是 0。
        """
        if self.sim:
            return MODE_CODES["MIT"] if int(rid) == 10 else 0
        m = self._motor(can_id)
        if m is None:
            return None
        return self.mc.read_motor_param(m, int(rid), timeout=max(0.05, float(wait)))

    def write_param(self, can_id: int, rid: int, value: float) -> None:
        if self.sim:
            return
        m = self._motor(can_id)
        if m is not None:
            self.mc.change_motor_param(m, rid, value)
            time.sleep(0.02)

    def save(self, can_id: int) -> None:
        if self.sim:
            return
        m = self._motor(can_id)
        if m is not None:
            self.mc.save_motor_param(m)

    def zero(self, can_id: int) -> None:
        if self.sim:
            return
        m = self._motor(can_id)
        if m is not None:
            self.mc.set_zero_position(m)
            time.sleep(0.02)

    def close(self) -> None:
        if self.mc is not None:
            try:
                self.mc.on_sample = None      # 先摘钩子，避免拆除时还在回调
            except Exception:
                pass
            try:
                self.mc.close()
            except Exception:
                pass
            self.mc = None
        self._motors.clear()
