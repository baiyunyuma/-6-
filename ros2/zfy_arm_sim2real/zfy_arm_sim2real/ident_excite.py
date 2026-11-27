#!/usr/bin/env python3
"""Identification excitation trajectories for the ZFY arm.

The node does NOT enable or start the motors.  Before running it:

    ros2 service call /motor_driver/set_motor_enable ...
    ros2 service call /motor_driver/set_motor_sending ...

Then, with a CSV path configured in motor_driver, run e.g.:

    ros2 run zfy_arm_sim2real ident_excite --ros-args \
      -p confirm:=true -p mode:=static

Modes:
  static   hold a series of poses (gravity/offset identification)
  sweep    one joint at a time, slow sinusoid (friction identification)
  fourier  multisine on all joints (dynamic parameter identification)

`auto_log:=true` (default) toggles /motor_driver/set_logging around the trial,
so the driver writes a timestamped CSV in its `log_path` directory.
"""
from __future__ import annotations

import math
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32
from std_srvs.srv import SetBool

from zfy_arm_interfaces.msg import JointGoal
from zfy_arm_interfaces.srv import GetMotorStatus

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class IdentExcite(Node):
    def __init__(self):
        super().__init__("zfy_ident_excite")
        self.declare_parameter("joint_names", JOINTS)
        self.declare_parameter("mode", "static")
        self.declare_parameter("confirm", False)
        self.declare_parameter("command_topic", "/arm_joint_goal")
        self.declare_parameter("amplitude", [0.10] * 6)
        self.declare_parameter("frequency", 0.30)
        self.declare_parameter("harmonics", 3)
        self.declare_parameter("duration", 10.0)
        self.declare_parameter("dwell", 1.0)
        self.declare_parameter("settle", 0.4)
        self.declare_parameter("rate", 200.0)
        self.declare_parameter("kp", 5.0)
        self.declare_parameter("kd", 0.8)
        self.declare_parameter("gravity", False)
        self.declare_parameter("gravity_scale", 1.0)
        self.declare_parameter("friction", False)
        self.declare_parameter("coulomb", 0.10)
        self.declare_parameter("viscous", 0.05)
        self.declare_parameter("friction_slope", 100.0)
        self.declare_parameter("auto_log", True)
        self.declare_parameter("return_home", True)
        #: 机械/模型关节限位（URDF 弧度）。HMI 会用自己的软件限位覆盖。
        self.declare_parameter(
            "q_min",
            [-2.6179939, 0.0, -3.1415927, -1.5707963, -1.5707963, -6.2831853],
        )
        self.declare_parameter(
            "q_max",
            [2.6179939, 3.0543262, 0.0, 1.5707963, 1.5707963, 6.2831853],
        )
        self.declare_parameter("status_timeout", 5.0)

        self.joint_names = [str(v) for v in self.get_parameter("joint_names").value]
        self.mode = str(self.get_parameter("mode").value).lower()
        self.confirm = bool(self.get_parameter("confirm").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.amp = [float(v) for v in self.get_parameter("amplitude").value]
        self.freq = max(0.02, float(self.get_parameter("frequency").value))
        self.harmonics = max(1, int(self.get_parameter("harmonics").value))
        self.duration = max(1.0, float(self.get_parameter("duration").value))
        self.dwell = max(0.0, float(self.get_parameter("dwell").value))
        self.settle = max(0.0, float(self.get_parameter("settle").value))
        self.rate = max(20.0, float(self.get_parameter("rate").value))
        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)
        self.gravity = bool(self.get_parameter("gravity").value)
        self.gravity_scale = float(self.get_parameter("gravity_scale").value)
        self.friction = bool(self.get_parameter("friction").value)
        self.coulomb = float(self.get_parameter("coulomb").value)
        self.viscous = float(self.get_parameter("viscous").value)
        self.friction_slope = float(self.get_parameter("friction_slope").value)
        self.auto_log = bool(self.get_parameter("auto_log").value)
        self.return_home = bool(self.get_parameter("return_home").value)
        self.q_min = [float(v) for v in self.get_parameter("q_min").value]
        self.q_max = [float(v) for v in self.get_parameter("q_max").value]
        self.status_timeout = float(self.get_parameter("status_timeout").value)

        n = len(self.joint_names)
        if len(self.amp) != n or len(self.q_min) != n or len(self.q_max) != n:
            raise RuntimeError("amplitude/q_min/q_max 长度必须等于关节数")
        if any(lo >= hi for lo, hi in zip(self.q_min, self.q_max)):
            raise RuntimeError("q_min 必须小于 q_max")
        if self.mode not in ("static", "sweep", "fourier"):
            raise RuntimeError("mode 必须是 static / sweep / fourier")

        self._q = None
        self._q_t = 0.0
        self._active = False
        self._pending = 0
        self._state = "wait_state"
        self._done = False
        self._shutdown_at = 0.0
        self._shutdown_called = False
        self._log_started = False
        self._status_future = None
        self._status_t0 = 0.0
        self._log_future = None

        # static
        self._poses = []
        self._pose_i = 0
        self._pose_phase = "move"
        self._pose_t0 = 0.0
        self._pose_started = False

        # sweep/fourier
        self._seg_i = 0
        self._seg_t0 = 0.0
        self._run_t0 = 0.0
        self._done_t0 = 0.0

        self.goal_pub = self.create_publisher(JointGoal, self.command_topic, 10)
        self.create_subscription(JointState, "/joint_states", self._on_state, 20)
        self.create_subscription(
            Bool, "/motor_driver/motion_active", self._on_active, 10)
        self.create_subscription(
            Int32, "/motor_driver/motion_pending", self._on_pending, 10)
        self.status_client = self.create_client(
            GetMotorStatus, "/motor_driver/status")
        self.log_client = self.create_client(
            SetBool, "/motor_driver/set_logging")
        self.create_timer(1.0 / self.rate, self._tick)

    # ------------------------------------------------------------- callbacks
    def _on_state(self, msg: JointState):
        index = {n: i for i, n in enumerate(msg.name)}
        if not all(n in index for n in self.joint_names):
            return
        self._q = [float(msg.position[index[n]]) for n in self.joint_names]
        self._q_t = time.monotonic()

    def _on_active(self, msg: Bool):
        self._active = bool(msg.data)

    def _on_pending(self, msg: Int32):
        self._pending = int(msg.data)

    def _publish(self, command, position=None, duration=0.0):
        msg = JointGoal()
        msg.command = int(command)
        msg.position = [] if position is None else [float(v) for v in position]
        msg.duration = float(duration)
        msg.kp = self.kp
        msg.kd = self.kd
        msg.gravity = self.gravity
        msg.gravity_scale = self.gravity_scale
        msg.friction = self.friction
        msg.coulomb = self.coulomb
        msg.viscous = self.viscous
        msg.friction_slope = self.friction_slope
        self.goal_pub.publish(msg)

    # ----------------------------------------------------------- state machine
    def _tick(self):
        if self._done:
            if (not self._shutdown_called
                    and time.monotonic() >= self._shutdown_at):
                self._shutdown_called = True
                try:
                    rclpy.shutdown()
                except Exception:
                    pass
            return
        now = time.monotonic()
        if self._state == "wait_state":
            if self._q is not None and now - self._q_t < 0.5:
                self._state = "wait_status"
                self._status_t0 = now
                self.get_logger().info("已收到关节状态，检查驱动状态")
            return

        if self._state == "wait_status":
            if self._status_future is None:
                if not self.status_client.service_is_ready():
                    if now - self._status_t0 > self.status_timeout:
                        self._finish(False, "/motor_driver/status 不可用")
                    return
                self._status_future = self.status_client.call_async(
                    GetMotorStatus.Request())
                self._status_future.add_done_callback(self._on_status)
            return

        if self._state == "start_log":
            if self.auto_log:
                if self._log_future is None:
                    if not self.log_client.service_is_ready():
                        self._finish(False, "/motor_driver/set_logging 不可用")
                        return
                    req = SetBool.Request()
                    req.data = True
                    self._log_future = self.log_client.call_async(req)
                    self._log_future.add_done_callback(self._on_log_started)
                return
            self._begin()

        if self._state == "running":
            if self.mode == "static":
                self._tick_static(now)
            elif self.mode == "sweep":
                self._tick_sweep(now)
            else:
                self._tick_fourier(now)

    def _on_status(self, fut):
        try:
            res = fut.result()
        except Exception as exc:  # noqa: BLE001
            self._finish(False, "读取驱动状态失败: %s" % exc)
            return
        self._status_future = None
        if res is None:
            self._finish(False, "驱动状态无应答")
            return
        if not (res.enabled and res.sending):
            self._finish(False, "驱动未使能或未开始发送")
            return
        if res.enabled_each and not all(res.enabled_each):
            self._finish(False, "有电机未使能")
            return
        if res.sending_each and not all(res.sending_each):
            self._finish(False, "有电机未开始发送")
            return
        self._state = "start_log"

    def _on_log_started(self, fut):
        self._log_future = None
        try:
            res = fut.result()
        except Exception as exc:  # noqa: BLE001
            self._finish(False, "启动日志失败: %s" % exc)
            return
        if res is None or not res.success:
            self._finish(False, "启动日志失败: %s" % (
                res.message if res is not None else "无应答"))
            return
        self._log_started = True
        self.get_logger().info("日志: " + res.message)
        self._begin()

    def _begin(self):
        q0 = list(self._q)
        self._publish(JointGoal.GAINS)
        if self.mode == "static":
            self._poses = [q0]
            for i in range(len(self.joint_names)):
                for s in (1.0, -1.0):
                    q = list(q0)
                    q[i] = min(max(q0[i] + s * self.amp[i],
                                   self.q_min[i]), self.q_max[i])
                    if max(abs(a - b) for a, b in zip(q, q0)) > 0.01:
                        self._poses.append(q)
            if self.return_home:
                self._poses.append(list(q0))
            self._pose_i = 0
            self._pose_phase = "move"
            self._pose_started = False
            self._pose_t0 = time.monotonic()
            self.get_logger().info(
                "static: %d 个位姿，每个保持 %.1f s" % (
                    len(self._poses), self.dwell))
        elif self.mode == "sweep":
            self._seg_i = 0
            self._seg_t0 = time.monotonic()
            self.get_logger().info(
                "sweep: 每关节 %.1f s @ %.2f Hz，幅值 %s" % (
                    self.duration, self.freq, self.amp))
        else:
            self._run_t0 = time.monotonic()
            self.get_logger().info(
                "fourier: %.1f s，基频 %.2f Hz，%d 次谐波" % (
                    self.duration, self.freq, self.harmonics))
        self._state = "running"

    # ----------------------------------------------------------------- static
    def _tick_static(self, now):
        if self._pose_i >= len(self._poses):
            self._finish(True, "static 完成")
            return
        target = self._poses[self._pose_i]
        if self._pose_phase == "move":
            if not self._pose_started:
                delta = max(abs(a - b) for a, b in zip(target, self._q))
                self._publish(
                    JointGoal.MOVE, target,
                    duration=max(1.0, delta / 0.4),
                )
                self._pose_started = True
                self._pose_t0 = now
                self.get_logger().info(
                    "pose %d/%d" % (self._pose_i + 1, len(self._poses)))
            elif self._active:
                self._pose_phase = "wait_done"
            elif now - self._pose_t0 > 2.0:
                self._pose_phase = "dwell"
                self._pose_t0 = now
        elif self._pose_phase == "wait_done":
            if not self._active and self._pending == 0:
                self._pose_phase = "dwell"
                self._pose_t0 = now
        elif self._pose_phase == "dwell":
            if now - self._pose_t0 >= self.dwell:
                self._pose_i += 1
                self._pose_phase = "move"
                self._pose_started = False

    # ------------------------------------------------------------------ sweep
    def _sweep_target(self, i, t):
        q = list(self._q0)
        q[i] = q[i] + self.amp[i] * math.sin(2 * math.pi * self.freq * t)
        return [min(max(v, self.q_min[j]), self.q_max[j])
                for j, v in enumerate(q)]

    def _tick_sweep(self, now):
        if self._seg_i >= len(self.joint_names):
            self._finish(True, "sweep 完成")
            return
        if not hasattr(self, "_q0"):
            self._q0 = list(self._q)
        t = now - self._seg_t0
        if t >= self.duration:
            self._seg_i += 1
            self._seg_t0 = now
            if self._seg_i >= len(self.joint_names):
                self._finish(True, "sweep 完成")
            return
        self._publish(JointGoal.DIRECT, self._sweep_target(self._seg_i, t))

    # ---------------------------------------------------------------- fourier
    def _fourier_target(self, t):
        q = list(self._q0)
        for i in range(len(q)):
            s = 0.0
            for k in range(1, self.harmonics + 1):
                f = self.freq * k
                phase = 0.7 * k * (i + 1)
                s += math.sin(2 * math.pi * f * t + phase) / self.harmonics
            q[i] += self.amp[i] * s
        return [min(max(v, self.q_min[j]), self.q_max[j])
                for j, v in enumerate(q)]

    def _tick_fourier(self, now):
        if not hasattr(self, "_q0"):
            self._q0 = list(self._q)
        t = now - self._run_t0
        if t >= self.duration:
            self._finish(True, "fourier 完成")
            return
        self._publish(JointGoal.DIRECT, self._fourier_target(t))

    # ----------------------------------------------------------------- finish
    def _finish(self, ok, reason):
        if self._done:
            return
        self._done = True
        self._shutdown_at = time.monotonic() + 0.8
        try:
            self._publish(JointGoal.STOP)
        except Exception:
            pass
        if self._log_started and self.log_client.service_is_ready():
            req = SetBool.Request()
            req.data = False
            self.log_client.call_async(req)
        self.get_logger().info(
            ("完成: " if ok else "中止: ") + str(reason))


def main(args=None):
    rclpy.init(args=args)
    node = IdentExcite()
    if not node.confirm:
        node.get_logger().error(
            "拒绝运动：确认周围安全后加 -p confirm:=true")
        node.destroy_node()
        rclpy.shutdown()
        return
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
