#!/usr/bin/env python3
"""ZFY 夹爪单电机驱动：SocketCAN + 达妙 MIT + 自适应抓取。

设计成独立节点，不并入 6 轴 ``motor_driver``：

* 6 轴臂的 ``JointGoal``、伺服插值、重力模型都按 6 个关节写死；
* 夹爪是独立机构，只需要一个 MIT 轴，自己使能/发送/抓取；
* 两个节点可以绑同一个 ``can0``（CAN_RAW 允许多个 socket），也可以
  给夹爪单独一路 ``can1``；互不干扰。

服务（``~/`` = ``/gripper_driver/``）：
  ~/enable          std_srvs/SetBool   使能/失能（MIT 模式）
  ~/sending         std_srvs/SetBool   开始/停止发送 MIT
  ~/open            std_srvs/Trigger   低速张开到 open_q
  ~/adaptive_grasp  std_srvs/Trigger   低刚度闭合，检测接触后转高刚度保持
  ~/hold_high       std_srvs/Trigger   以当前实测位置 + 高刚度保持
  ~/save_grasp      std_srvs/Trigger   保存 hold_q / kp_high / kd_high
  ~/stop            std_srvs/Trigger   零力矩并停止发送
话题：
  ~/state           sensor_msgs/JointState  q / dq / tau
  ~/status          std_msgs/Int32          状态机编号
  ~/contact         std_msgs/Bool           是否已接触物体

自适应抓取流程：
  从当前 q 出发，以 ``close_speed`` 向 ``close_q`` 缓慢闭合，全程用
  ``kp_low / kd_low``。接触判据：

  * 力矩：``|tau| >= contact_tau``（始终启用）；
  * 位置：``|q - q_des| >= contact_pos_err``，或速度很低时
    ``|q - q_des| >= 0.5 * contact_pos_err``。

  判据必须**连续成立** ``contact_hold_time`` 秒才判定接触，单帧
  瞬时超阈值不算数。``contact_pos_err <= 0`` 表示关闭位置误差判据，
  只用力矩判据。判定后记录 ``hold_q = q``，等待 ``settle_time`` 后
  切到 ``kp_high / kd_high`` 保持，并可选自动保存。
"""
from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32
from std_srvs.srv import SetBool, Trigger

IDLE = 0
ENABLED = 1
OPENING = 2
ADAPTIVE = 3
CONTACT = 4
HOLD_HIGH = 5
HOLDING = 6
FAULT = 7
STATE_NAMES = {
    IDLE: "IDLE",
    ENABLED: "ENABLED",
    OPENING: "OPENING",
    ADAPTIVE: "ADAPTIVE",
    CONTACT: "CONTACT",
    HOLD_HIGH: "HOLD_HIGH",
    HOLDING: "HOLDING",
    FAULT: "FAULT",
}


def _find_py_dir(explicit: str):
    """Locate the HMI ``py`` dir holding motor_config.py / socketcan_bus.py."""
    candidates = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    here = Path(__file__).resolve()
    for base in (here.parent, *here.parents):
        candidates.append(base / "py")
        candidates.append(base)
    for cand in candidates:
        try:
            cand = cand.resolve()
        except OSError:
            continue
        if ((cand / "motor_config.py").is_file()
                and (cand / "socketcan_bus.py").is_file()):
            return cand
    return None


class GripperDriver(Node):
    def __init__(self):
        super().__init__("gripper_driver")

        # ---------------------------------------------------------- parameters
        self.declare_parameter("sim", False)
        self.declare_parameter("hmi_py_path", "")
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("can_bitrate", 1_000_000)
        self.declare_parameter("name", "gripper")
        self.declare_parameter("can_id", 7)
        self.declare_parameter("mst_id", 7)
        self.declare_parameter("model", "4310")
        self.declare_parameter("channel", 0)

        #: 夹爪机械行程（电机原始弧度）。必须按实物改，默认只是电机量程。
        self.declare_parameter("q_min", -12.5)
        self.declare_parameter("q_max", 12.5)
        self.declare_parameter("open_q", 0.0)
        self.declare_parameter("close_q", 1.0)
        self.declare_parameter("close_direction", 1.0)   # +1 / -1

        #: 低刚度 = 自适应闭合阶段；高刚度 = 夹住后保持
        self.declare_parameter("kp_low", 5.0)
        self.declare_parameter("kd_low", 0.3)
        self.declare_parameter("kp_high", 60.0)
        self.declare_parameter("kd_high", 2.0)

        self.declare_parameter("close_speed", 0.20)      # rad/s
        self.declare_parameter("max_travel", 1.0)        # rad
        self.declare_parameter("min_travel", 0.02)       # 触发接触前至少走过
        self.declare_parameter("contact_tau", 1.0)       # N*m
        #: 位置误差判据阈值；<= 0 表示关闭该判据，只保留力矩判据。
        self.declare_parameter("contact_pos_err", 0.05)  # rad
        self.declare_parameter("contact_vel", 0.02)      # rad/s
        #: 判据需要连续成立这么久才算接触。
        self.declare_parameter("contact_hold_time", 0.15)
        self.declare_parameter("settle_time", 0.20)
        self.declare_parameter("timeout", 5.0)
        self.declare_parameter("position_tolerance", 0.01)
        self.declare_parameter("control_rate", 200.0)
        self.declare_parameter("feedback_timeout", 0.5)
        self.declare_parameter("auto_save", True)
        self.declare_parameter(
            "save_path", "~/.config/zfy_arm/gripper_grasp.json"
        )

        self.driver_name = str(self.get_parameter("name").value)
        self.sim = bool(self.get_parameter("sim").value)
        self.can_channel = str(self.get_parameter("can_channel").value)
        self.can_bitrate = int(self.get_parameter("can_bitrate").value)
        self.can_id = int(self.get_parameter("can_id").value)
        self.mst_id = int(self.get_parameter("mst_id").value)
        self.model = str(self.get_parameter("model").value)
        self.channel = int(self.get_parameter("channel").value)

        self.q_min = float(self.get_parameter("q_min").value)
        self.q_max = float(self.get_parameter("q_max").value)
        self.open_q = float(self.get_parameter("open_q").value)
        self.close_q = float(self.get_parameter("close_q").value)
        self.close_direction = float(
            self.get_parameter("close_direction").value
        )
        self.kp_low = float(self.get_parameter("kp_low").value)
        self.kd_low = float(self.get_parameter("kd_low").value)
        self.kp_high = float(self.get_parameter("kp_high").value)
        self.kd_high = float(self.get_parameter("kd_high").value)

        self.close_speed = float(self.get_parameter("close_speed").value)
        self.max_travel = float(self.get_parameter("max_travel").value)
        self.min_travel = float(self.get_parameter("min_travel").value)
        self.contact_tau = float(self.get_parameter("contact_tau").value)
        self.contact_pos_err = float(
            self.get_parameter("contact_pos_err").value
        )
        self.contact_vel = float(self.get_parameter("contact_vel").value)
        self.contact_hold_time = float(
            self.get_parameter("contact_hold_time").value
        )
        self.settle_time = float(self.get_parameter("settle_time").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.position_tolerance = float(
            self.get_parameter("position_tolerance").value
        )
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.feedback_timeout = float(
            self.get_parameter("feedback_timeout").value
        )
        self.auto_save = bool(self.get_parameter("auto_save").value)
        self.save_path = os.path.expanduser(
            str(self.get_parameter("save_path").value)
        )
        self._validate_static_params()

        # --------------------------------------------------------------- bus
        py_dir = _find_py_dir(str(self.get_parameter("hmi_py_path").value))
        if py_dir is None:
            raise RuntimeError(
                "找不到 py/motor_config.py + socketcan_bus.py；"
                "用 hmi_py_path 指定 Zfy_Arm_real_control/py"
            )
        if str(py_dir) not in sys.path:
            sys.path.insert(0, str(py_dir))
        from motor_config import MODELS, MotorSpec  # noqa: E402
        from socketcan_bus import SocketCanBus  # noqa: E402

        if self.model not in MODELS:
            raise RuntimeError(
                "未知型号 %r，可选: %s" % (self.model, ", ".join(MODELS))
            )
        self.spec = MotorSpec(
            self.driver_name, self.can_id, self.mst_id,
            self.model, self.channel, "MIT",
        )
        try:
            self.bus = SocketCanBus(
                [self.spec],
                sim=self.sim,
                nom_baud=self.can_bitrate,
                canfd=False,
                set_baudrate=False,
                on_sample=self._on_sample,
                channel=self.can_channel,
                channels=[self.can_channel],
                bitrate=self.can_bitrate,
                require_driver=None if self.sim else "gs_usb",
            )
        except Exception as exc:
            raise RuntimeError("打开夹爪 CAN 失败: %s" % exc) from exc

        # ------------------------------------------------------------- state
        self._lock = threading.Lock()
        self._q = 0.0
        self._dq = 0.0
        self._tau = 0.0
        self._err = 0
        self._stamp = 0.0
        self._enabled = False
        self._sending = False
        self._state = IDLE
        self._contact = False
        self._hold_q = 0.0
        self._q_des = 0.0
        self._q_start = 0.0
        self._state_t = 0.0
        self._contact_t0 = 0.0
        self._adaptive_t0 = 0.0
        self._last_tick = 0.0
        self._last_tau_contact = 0.0
        self._saved = {}
        self._warn_t = {}
        self._load_saved()

        # --------------------------------------------------------- interfaces
        self.state_pub = self.create_publisher(JointState, "~/state", 10)
        self.status_pub = self.create_publisher(Int32, "~/status", 10)
        self.contact_pub = self.create_publisher(Bool, "~/contact", 10)

        self.create_service(SetBool, "~/enable", self._srv_enable)
        self.create_service(SetBool, "~/sending", self._srv_sending)
        self.create_service(Trigger, "~/open", self._srv_open)
        self.create_service(Trigger, "~/adaptive_grasp",
                            self._srv_adaptive_grasp)
        self.create_service(Trigger, "~/hold_high", self._srv_hold_high)
        self.create_service(Trigger, "~/save_grasp", self._srv_save_grasp)
        self.create_service(Trigger, "~/stop", self._srv_stop)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.create_timer(1.0 / max(20.0, self.control_rate), self._tick)
        self.create_timer(0.05, self._publish_state)

        self.get_logger().info(
            "夹爪驱动已启动：%s，CAN %s，ID 0x%02X，型号 %s，%s"
            % (
                "仿真" if self.sim else "实机",
                self.can_channel, self.can_id, self.model,
                "MIT",
            )
        )
        self.get_logger().info(
            "行程 q=[%.4f, %.4f]，open=%.4f close=%.4f dir=%+d；"
            "低刚度 kp/kd=%.2f/%.2f，高刚度 %.2f/%.2f"
            % (
                self.q_min, self.q_max, self.open_q, self.close_q,
                int(self.close_direction), self.kp_low, self.kd_low,
                self.kp_high, self.kd_high,
            )
        )

    # ------------------------------------------------------------- validation
    def _validate_static_params(self) -> None:
        if not (math.isfinite(self.q_min) and math.isfinite(self.q_max)):
            raise RuntimeError("q_min / q_max 必须是有限数")
        if self.q_min >= self.q_max:
            raise RuntimeError("q_min 必须小于 q_max")
        if not (self.q_min <= self.open_q <= self.q_max):
            raise RuntimeError("open_q 必须在 [q_min, q_max] 内")
        if not (self.q_min <= self.close_q <= self.q_max):
            raise RuntimeError("close_q 必须在 [q_min, q_max] 内")
        if self.close_direction not in (-1.0, 1.0):
            raise RuntimeError("close_direction 只能是 +1 或 -1")
        if self.control_rate < 20.0 or self.control_rate > 1000.0:
            raise RuntimeError("control_rate 必须在 20..1000 Hz")
        if self.close_speed <= 0.0 or self.max_travel <= 0.0:
            raise RuntimeError("close_speed / max_travel 必须为正")
        if self.kp_low < 0.0 or self.kd_low < 0.0:
            raise RuntimeError("kp_low / kd_low 必须非负")
        if self.kp_high < 0.0 or self.kd_high < 0.0:
            raise RuntimeError("kp_high / kd_high 必须非负")

    def _on_set_parameters(self, params):
        numeric = {
            "kp_low", "kd_low", "kp_high", "kd_high",
            "close_speed", "max_travel", "min_travel",
            "contact_tau", "contact_pos_err", "contact_vel",
            "contact_hold_time", "settle_time", "timeout",
            "position_tolerance", "open_q", "close_q",
            "q_min", "q_max", "close_direction",
        }
        nonneg = {
            "kp_low", "kd_low", "kp_high", "kd_high",
            "contact_tau", "contact_pos_err", "contact_vel",
            "contact_hold_time", "settle_time", "timeout",
            "position_tolerance", "close_speed", "max_travel", "min_travel",
        }
        proposed = {}
        for p in params:
            name = p.name
            if name == "auto_save":
                proposed[name] = bool(p.value)
                continue
            if name not in numeric:
                continue
            try:
                value = float(p.value)
            except (TypeError, ValueError) as exc:
                return SetParametersResult(
                    successful=False, reason="%s: %s" % (name, exc)
                )
            if not math.isfinite(value):
                return SetParametersResult(
                    successful=False, reason="%s 必须是有限数" % name
                )
            if name in nonneg and value < 0.0:
                return SetParametersResult(
                    successful=False, reason="%s 不能为负" % name
                )
            if name == "close_direction" and value not in (-1.0, 1.0):
                return SetParametersResult(
                    successful=False, reason="close_direction 只能是 +1/-1"
                )
            proposed[name] = value
        # 先按“全部提交后的最终值”验证，避免逐个赋值造成的瞬时范围冲突
        lo = proposed.get("q_min", self.q_min)
        hi = proposed.get("q_max", self.q_max)
        open_q = proposed.get("open_q", self.open_q)
        close_q = proposed.get("close_q", self.close_q)
        if lo >= hi:
            return SetParametersResult(
                successful=False, reason="q_min 必须小于 q_max"
            )
        if not (lo <= open_q <= hi):
            return SetParametersResult(
                successful=False, reason="open_q 不在 [q_min,q_max]"
            )
        if not (lo <= close_q <= hi):
            return SetParametersResult(
                successful=False, reason="close_q 不在 [q_min,q_max]"
            )
        for name, value in proposed.items():
            setattr(self, name, value)
        self.get_logger().info(
            "参数更新: low %.2f/%.2f high %.2f/%.2f contact tau %.3f"
            % (self.kp_low, self.kd_low, self.kp_high, self.kd_high,
               self.contact_tau)
        )
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ state
    def _on_sample(self, can_id, stamp, q, dq, tau, err):
        if int(can_id) != self.can_id:
            return
        with self._lock:
            self._q = float(q)
            self._dq = float(dq)
            self._tau = float(tau)
            self._err = int(err)
            self._stamp = float(stamp)

    def _snapshot(self):
        with self._lock:
            return (
                self._q, self._dq, self._tau, self._err, self._stamp,
                self._enabled, self._sending, self._state,
                self._contact, self._hold_q,
            )

    def _fresh(self) -> bool:
        with self._lock:
            return self._stamp > 0.0 and (
                time.monotonic() - self._stamp <= self.feedback_timeout
            )

    def _warn(self, key: str, text: str, period: float = 2.0) -> None:
        now = time.monotonic()
        if now - self._warn_t.get(key, 0.0) < period:
            return
        self._warn_t[key] = now
        self.get_logger().warn(text)

    def _clamp_q(self, q: float) -> float:
        return min(max(float(q), self.q_min), self.q_max)

    def _send_mit(self, kp: float, kd: float, q: float,
                  dq: float = 0.0, tau: float = 0.0) -> None:
        try:
            self.bus.mit(
                self.can_id, float(kp), float(kd),
                self._clamp_q(q), float(dq), float(tau),
            )
        except Exception as exc:
            self._warn("mit", "发送 MIT 失败: %s" % exc, 2.0)

    def _set_state(self, state: int) -> None:
        if self._state != state:
            self._state = state
            self.get_logger().info("状态 -> " + STATE_NAMES.get(state, str(state)))

    # ------------------------------------------------------------- control
    def _tick(self):
        if self.sim:
            try:
                self.bus.sample_sim()
            except Exception as exc:
                self._warn("sim", "仿真采样失败: %s" % exc, 2.0)
        enabled, sending, state = self._enabled, self._sending, self._state
        if not enabled or not sending:
            return
        if not self._fresh():
            if state != FAULT:
                self.get_logger().error(
                    "夹爪反馈超过 %.1f s 未更新，零力矩并停止发送"
                    % self.feedback_timeout
                )
            self._send_mit(0.0, 0.0, self._q)
            self._sending = False
            self._set_state(FAULT)
            return

        now = time.monotonic()
        dt = min(max(now - self._last_tick, 0.0), 0.1)
        self._last_tick = now

        if state == ADAPTIVE:
            self._tick_adaptive(dt, now)
        elif state == OPENING:
            self._tick_open(dt, now)
        elif state == CONTACT:
            self._send_mit(self.kp_low, self.kd_low, self._hold_q)
            if now - self._state_t >= self.settle_time:
                self._state_t = now
                self._set_state(HOLD_HIGH)
                self.get_logger().info(
                    "切换到高刚度保持: q=%.4f kp=%.2f kd=%.2f"
                    % (self._hold_q, self.kp_high, self.kd_high)
                )
                if self.auto_save:
                    self._save_grasp(contacted=self._contact)
        elif state == HOLD_HIGH:
            self._send_mit(self.kp_high, self.kd_high, self._hold_q)
        elif state == HOLDING:
            self._send_mit(self.kp_low, self.kd_low, self._hold_q)
        else:
            self._send_mit(self.kp_low, self.kd_low, self._hold_q)

    def _tick_adaptive(self, dt: float, now: float) -> None:
        self._q_des += self.close_direction * self.close_speed * dt
        # close_q 和 max_travel 谁先到就用谁作为本次闭合的终点
        if self.close_direction > 0.0:
            limit = min(self.close_q, self._q_start + self.max_travel)
            limit = self._clamp_q(limit)
            self._q_des = min(self._q_des, limit)
        else:
            limit = max(self.close_q, self._q_start - self.max_travel)
            limit = self._clamp_q(limit)
            self._q_des = max(self._q_des, limit)
        self._q_des = self._clamp_q(self._q_des)
        self._send_mit(self.kp_low, self.kd_low, self._q_des)

        moved = abs(self._q_des - self._q_start)
        err = abs(self._q - self._q_des)
        tau_abs = abs(self._tau)
        vel_abs = abs(self._dq)

        # 力矩判据始终启用；contact_pos_err <= 0 时关闭位置误差判据。
        hit_tau = tau_abs >= self.contact_tau
        hit_pos = False
        if self.contact_pos_err > 0.0:
            hit_pos = (
                err >= self.contact_pos_err
                or (vel_abs <= self.contact_vel
                    and err >= 0.5 * self.contact_pos_err)
            )
        candidate = moved >= self.min_travel and (hit_tau or hit_pos)
        if candidate:
            # 判据必须连续成立 contact_hold_time 秒才算接触，单帧不算。
            if self._contact_t0 <= 0.0:
                self._contact_t0 = now
            elif now - self._contact_t0 >= self.contact_hold_time:
                self._last_tau_contact = self._tau
                trigger = (
                    "tau %.2f>=%.2f" % (tau_abs, self.contact_tau)
                    if hit_tau else "err %.3f" % err
                )
                self._enter_contact(
                    now, True,
                    "检测到接触(%s 持续 %.2f s)"
                    % (trigger, now - self._contact_t0),
                )
                return
        else:
            self._contact_t0 = 0.0

        at_end = (
            (self.close_direction > 0.0 and self._q_des >= limit - 1e-6)
            or (self.close_direction < 0.0 and self._q_des <= limit + 1e-6)
        )
        if at_end:
            self._enter_contact(now, False, "到闭合位")
        elif now - self._adaptive_t0 >= self.timeout:
            self._enter_contact(now, False, "自适应抓取超时")

    def _enter_contact(self, now: float, contacted: bool, reason: str) -> None:
        self._hold_q = self._clamp_q(self._q)
        self._q_des = self._hold_q
        self._contact = bool(contacted)
        self._state_t = now
        self._set_state(CONTACT)
        self.get_logger().info(
            "%s: q=%.4f tau=%.3f；%.2f s 后切高刚度%s"
            % (
                reason, self._hold_q, self._tau, self.settle_time,
                "" if contacted else "（未检测到接触）",
            )
        )

    def _tick_open(self, dt: float, now: float) -> None:
        step = self.close_speed * dt
        delta = self.open_q - self._q_des
        if abs(delta) <= step:
            self._q_des = self.open_q
        else:
            self._q_des += math.copysign(step, delta)
        self._q_des = self._clamp_q(self._q_des)
        self._send_mit(self.kp_low, self.kd_low, self._q_des)
        if (abs(self._q - self.open_q) <= self.position_tolerance
                and abs(self._dq) <= self.contact_vel):
            self._hold_q = self.open_q
            self._q_des = self.open_q
            self._contact = False
            self._set_state(HOLDING)
            self.get_logger().info("已张开到 %.4f" % self.open_q)

    # ------------------------------------------------------------- services
    def _srv_enable(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            try:
                self.bus.enable(self.can_id, True, switch_mode=True)
            except Exception as exc:
                response.success = False
                response.message = "使能失败: %s" % exc
                return response
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline and not self._fresh():
                # MIT 模式下电机收到控制帧才会持续回反馈；kp=kd=tau=0 是零力矩探测。
                probe = self._q if self._stamp > 0.0 else 0.0
                self._send_mit(0.0, 0.0, probe)
                time.sleep(0.02)
            if not self._fresh():
                try:
                    self.bus.enable(self.can_id, False)
                except Exception:
                    pass
                response.success = False
                response.message = "使能后未收到反馈"
                return response
            self._hold_q = self._q
            self._q_des = self._q
            self._enabled = True
            self._sending = False
            self._set_state(ENABLED)
            response.success = True
            response.message = "夹爪已使能，q=%.4f" % self._hold_q
            return response
        self._sending = False
        try:
            self.bus.enable(self.can_id, False)
        except Exception as exc:
            response.success = False
            response.message = "失能失败: %s" % exc
            return response
        self._enabled = False
        self._contact = False
        self._set_state(IDLE)
        response.success = True
        response.message = "夹爪已失能"
        return response

    def _srv_sending(self, request: SetBool.Request,
                     response: SetBool.Response):
        if request.data:
            if not self._enabled:
                response.success = False
                response.message = "先使能夹爪"
                return response
            # DM motors only stream feedback in response to MIT frames.  If
            # "开始发送" is clicked more than feedback_timeout after "使能",
            # the freshness check in _tick() would trip before the first
            # command reaches the motor.  Send zero-torque probes first.
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline and not self._fresh():
                self._send_mit(
                    0.0, 0.0, self._q if self._stamp > 0.0 else 0.0)
                time.sleep(0.02)
            if not self._fresh():
                response.success = False
                response.message = "开始发送前未收到夹爪反馈"
                return response
            self._hold_q = self._q
            self._q_des = self._q
            self._last_tick = time.monotonic()
            self._sending = True
            self._set_state(HOLDING)
            response.success = True
            response.message = "开始发送 MIT"
            return response
        if self._sending:
            self._send_mit(self.kp_high, self.kd_high, self._hold_q)
            time.sleep(0.01)
        self._sending = False
        self._set_state(ENABLED)
        response.success = True
        response.message = "停止发送"
        return response

    def _require_ready(self):
        if not self._enabled:
            return False, "先使能夹爪"
        if not self._sending:
            return False, "先开始发送"
        if not self._fresh():
            return False, "夹爪反馈过期"
        return True, ""

    def _srv_open(self, _request, response):
        ok, msg = self._require_ready()
        if not ok:
            response.success = False
            response.message = msg
            return response
        self._q_des = self._q
        self._contact = False
        self._state_t = time.monotonic()
        self._last_tick = self._state_t
        self._set_state(OPENING)
        response.success = True
        response.message = "开始张开到 %.4f" % self.open_q
        return response

    def _srv_adaptive_grasp(self, _request, response):
        ok, msg = self._require_ready()
        if not ok:
            response.success = False
            response.message = msg
            return response
        now = time.monotonic()
        self._q_start = self._q
        self._q_des = self._q
        self._hold_q = self._q
        self._contact = False
        self._contact_t0 = 0.0
        self._adaptive_t0 = now
        self._state_t = now
        self._last_tick = now
        self._set_state(ADAPTIVE)
        response.success = True
        response.message = (
            "开始自适应抓取: q=%.4f dir=%+d speed=%.3f"
            % (self._q_start, int(self.close_direction), self.close_speed)
        )
        return response

    def _srv_hold_high(self, _request, response):
        ok, msg = self._require_ready()
        if not ok:
            response.success = False
            response.message = msg
            return response
        self._hold_q = self._clamp_q(self._q)
        self._q_des = self._hold_q
        self._state_t = time.monotonic()
        self._set_state(HOLD_HIGH)
        response.success = True
        response.message = (
            "高刚度保持 q=%.4f kp=%.2f kd=%.2f"
            % (self._hold_q, self.kp_high, self.kd_high)
        )
        return response

    def _srv_save_grasp(self, _request, response):
        try:
            path = self._save_grasp(contacted=self._contact)
        except Exception as exc:
            response.success = False
            response.message = "保存失败: %s" % exc
            return response
        response.success = True
        response.message = "已保存: " + path
        return response

    def _srv_stop(self, _request, response):
        self._send_mit(0.0, 0.0, self._q)
        self._sending = False
        self._contact = False
        self._set_state(ENABLED if self._enabled else IDLE)
        response.success = True
        response.message = "已零力矩并停止发送"
        return response

    # ------------------------------------------------------------- persistence
    def _load_saved(self) -> None:
        try:
            with open(self.save_path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
        except FileNotFoundError:
            return
        except Exception as exc:
            self.get_logger().warn("读取抓取参数失败: %s" % exc)
            return
        if not isinstance(data, dict):
            return
        self._saved = data
        try:
            if "kp_high" in data:
                self.kp_high = float(data["kp_high"])
            if "kd_high" in data:
                self.kd_high = float(data["kd_high"])
        except (TypeError, ValueError):
            pass
        self.get_logger().info(
            "已载入抓到参数: %s（hold_q=%s）"
            % (self.save_path, data.get("hold_q"))
        )

    def _save_grasp(self, contacted: bool) -> str:
        path = self.save_path
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        data = {
            "hold_q": float(self._hold_q),
            "kp_high": float(self.kp_high),
            "kd_high": float(self.kd_high),
            "contacted": bool(contacted),
            "contact_tau": float(self._last_tau_contact),
            "q_start": float(self._q_start),
            "close_direction": float(self.close_direction),
            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(data, fh, ensure_ascii=False, indent=2)
        os.replace(tmp, path)
        self._saved = data
        self.get_logger().info(
            "保存抓取: q=%.4f kp=%.2f kd=%.2f contact=%s"
            % (self._hold_q, self.kp_high, self.kd_high, contacted)
        )
        return path

    # ------------------------------------------------------------- publishing
    def _publish_state(self) -> None:
        q, dq, tau, err, stamp, enabled, sending, state, contact, hold_q = (
            self._snapshot()
        )
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.driver_name]
        msg.position = [q]
        msg.velocity = [dq]
        msg.effort = [tau]
        self.state_pub.publish(msg)
        self.status_pub.publish(Int32(data=int(state)))
        self.contact_pub.publish(Bool(data=bool(contact)))

    # ------------------------------------------------------------- lifecycle
    def shutdown(self) -> None:
        try:
            if self._enabled:
                self._send_mit(0.0, 0.0, self._q)
                self.bus.enable(self.can_id, False)
        except Exception:
            pass
        try:
            self.bus.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = GripperDriver()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
