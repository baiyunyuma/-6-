#!/usr/bin/env python3
"""Drive the real ZFY arm from MoveIt's FollowJointTrajectory action.

MoveIt's simple controller manager sends goals to
``/zfy_arm_controller/follow_joint_trajectory``.  This node is that action
server for the real arm: it converts each trajectory into a queue of
``JointGoal`` MOVE/APPEND commands for ``/motor_driver``, so the existing
MIT servo, gravity and friction feed-forward are reused unchanged.

Safety rules:

* Never enables or starts sending by itself.  The driver must already be
  enabled + sending, and /joint_states must be fresh, or the goal is rejected.
* On cancel, timeout, stale feedback or final-position error it sends
  ``JointGoal.STOP`` (smooth stop) before returning.
* Do not run this together with ``zfy_mujoco_driver``: both would answer the
  same action name and publish conflicting /joint_states.
"""
from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32

from zfy_arm_interfaces.msg import JointGoal
from zfy_arm_interfaces.srv import GetMotorStatus

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
ACTION = "/zfy_arm_controller/follow_joint_trajectory"
ABORT_CODE = -1


class TrajBridge(Node):
    def __init__(self):
        super().__init__("zfy_arm_traj_bridge")

        self.declare_parameter("joint_names", JOINTS)
        self.declare_parameter(
            "joint_lower",
            [-2.6179939, 0.0, -3.1415927, -1.5707963, -1.5707963, -6.2831853],
        )
        self.declare_parameter(
            "joint_upper",
            [2.6179939, 3.0543262, 0.0, 1.5707963, 1.5707963, 6.2831853],
        )
        self.declare_parameter("command_topic", "/arm_joint_goal")
        self.declare_parameter("kp", 5.0)
        self.declare_parameter("kd", 0.8)
        self.declare_parameter("gravity", False)
        self.declare_parameter("gravity_scale", 1.0)
        self.declare_parameter("friction", False)
        self.declare_parameter("coulomb", 0.10)
        self.declare_parameter("viscous", 0.05)
        self.declare_parameter("friction_slope", 100.0)
        #: >1 slows the trajectory down (safer first tests)
        self.declare_parameter("velocity_scale", 1.0)
        self.declare_parameter("feedback_rate", 10.0)
        self.declare_parameter("feedback_timeout", 0.5)
        self.declare_parameter("completion_timeout", 5.0)
        self.declare_parameter("position_tolerance", 0.05)

        self.joint_names = [
            str(v) for v in self.get_parameter("joint_names").value
        ]
        self.joint_lower = [
            float(v) for v in self.get_parameter("joint_lower").value
        ]
        self.joint_upper = [
            float(v) for v in self.get_parameter("joint_upper").value
        ]
        n = len(self.joint_names)
        if len(self.joint_lower) != n or len(self.joint_upper) != n:
            raise RuntimeError("joint_lower / joint_upper 长度必须等于关节数")
        if any(lo >= hi for lo, hi in zip(self.joint_lower, self.joint_upper)):
            raise RuntimeError("joint_lower 必须小于 joint_upper")
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)
        self.gravity = bool(self.get_parameter("gravity").value)
        self.gravity_scale = float(self.get_parameter("gravity_scale").value)
        self.friction = bool(self.get_parameter("friction").value)
        self.coulomb = float(self.get_parameter("coulomb").value)
        self.viscous = float(self.get_parameter("viscous").value)
        self.friction_slope = float(self.get_parameter("friction_slope").value)
        self.velocity_scale = max(0.05, float(
            self.get_parameter("velocity_scale").value))
        self.feedback_rate = max(1.0, float(
            self.get_parameter("feedback_rate").value))
        self.feedback_timeout = max(0.1, float(
            self.get_parameter("feedback_timeout").value))
        self.completion_timeout = max(1.0, float(
            self.get_parameter("completion_timeout").value))
        self.position_tolerance = max(0.005, float(
            self.get_parameter("position_tolerance").value))

        self._lock = threading.Lock()
        self._q = None
        self._dq = None
        self._q_t = 0.0
        self._active = False
        self._pending = 0

        self._goal_lock = threading.Lock()
        self._goal_pending = False
        self._cancel = threading.Event()
        self._cb_group = ReentrantCallbackGroup()

        self.goal_pub = self.create_publisher(
            JointGoal, self.command_topic, 10)
        self.create_subscription(
            JointState, "/joint_states", self._on_state, 20,
            callback_group=self._cb_group)
        self.create_subscription(
            Bool, "/motor_driver/motion_active", self._on_active, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            Int32, "/motor_driver/motion_pending", self._on_pending, 10,
            callback_group=self._cb_group)
        self.status_client = self.create_client(
            GetMotorStatus, "/motor_driver/status",
            callback_group=self._cb_group)

        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            ACTION,
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            "实机轨迹桥已启动: %s -> %s（不会自动使能）"
            % (ACTION, self.command_topic))

    # ------------------------------------------------------------- callbacks
    def _on_state(self, msg: JointState):
        index = {n: i for i, n in enumerate(msg.name)}
        if not all(n in index for n in self.joint_names):
            return
        with self._lock:
            self._q = [float(msg.position[index[n]]) for n in self.joint_names]
            self._dq = (
                [float(msg.velocity[index[n]]) for n in self.joint_names]
                if len(msg.velocity) == len(msg.name) else None
            )
            self._q_t = time.monotonic()

    def _on_active(self, msg: Bool):
        with self._lock:
            self._active = bool(msg.data)

    def _on_pending(self, msg: Int32):
        with self._lock:
            self._pending = int(msg.data)

    def _on_cancel(self, _goal_handle):
        self._cancel.set()
        return CancelResponse.ACCEPT

    def _on_goal(self, goal_request):
        err = self._validate(goal_request.trajectory)
        if err is not None:
            self.get_logger().warn("拒绝轨迹: " + err)
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._goal_pending:
                self.get_logger().warn("拒绝轨迹: 上一条还在执行")
                return GoalResponse.REJECT
            self._goal_pending = True
        try:
            status = self._get_status()
            if status is None:
                raise RuntimeError("读不到 /motor_driver/status")
            if not (status.enabled and status.sending):
                raise RuntimeError("驱动未使能或未开始发送")
            if status.enabled_each and not all(status.enabled_each):
                raise RuntimeError("有电机未使能")
            if status.sending_each and not all(status.sending_each):
                raise RuntimeError("有电机未开始发送")
            with self._lock:
                q_age = (
                    time.monotonic() - self._q_t
                    if self._q is not None else 1e9
                )
            if q_age > self.feedback_timeout:
                raise RuntimeError("反馈超过 %.1f s 未更新" % q_age)
        except Exception as exc:
            with self._goal_lock:
                self._goal_pending = False
            self.get_logger().warn("拒绝轨迹: " + str(exc))
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # --------------------------------------------------------------- helpers
    def _validate(self, traj):
        if not traj.points:
            return "轨迹为空"
        names = list(traj.joint_names) or list(self.joint_names)
        missing = [n for n in self.joint_names if n not in names]
        if missing:
            return "轨迹缺少关节 " + ", ".join(missing)
        idx = [names.index(n) for n in self.joint_names]
        prev = -1.0
        for i, pt in enumerate(traj.points):
            if len(pt.positions) != len(names):
                return "第 %d 点位置长度不对" % i
            if not all(math.isfinite(v) for v in pt.positions):
                return "第 %d 点含非有限值" % i
            q = [float(pt.positions[k]) for k in idx]
            for j, value in enumerate(q):
                if not (self.joint_lower[j] - 1e-6
                        <= value <= self.joint_upper[j] + 1e-6):
                    return "第 %d 点 %s 超出软件限位" % (
                        i, self.joint_names[j]
                    )
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            if t < prev - 1e-9:
                return "时间戳非单调"
            prev = t
        return None

    def _ordered_points(self, traj):
        names = list(traj.joint_names) or list(self.joint_names)
        idx = [names.index(n) for n in self.joint_names]
        out = []
        for pt in traj.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            q = [float(pt.positions[i]) for i in idx]
            out.append((max(0.0, t), q))
        return out

    def _get_status(self, timeout: float = 1.5):
        if not self.status_client.service_is_ready():
            if not self.status_client.wait_for_service(timeout_sec=timeout):
                return None
        event = threading.Event()
        box = {}

        def done(fut):
            try:
                box["res"] = fut.result()
            except Exception as exc:  # noqa: BLE001
                box["err"] = exc
            finally:
                event.set()

        self.status_client.call_async(
            GetMotorStatus.Request()).add_done_callback(done)
        if not event.wait(timeout):
            return None
        return box.get("res")

    # --------------------------------------------------------------- commands
    def _publish(self, command, position=None, duration=0.0):
        msg = JointGoal()
        msg.command = int(command)
        msg.duration = float(duration)
        msg.position = [] if position is None else [float(v) for v in position]
        msg.kp = self.kp
        msg.kd = self.kd
        msg.gravity = self.gravity
        msg.gravity_scale = self.gravity_scale
        msg.friction = self.friction
        msg.coulomb = self.coulomb
        msg.viscous = self.viscous
        msg.friction_slope = self.friction_slope
        self.goal_pub.publish(msg)

    def _send_gains(self):
        self._publish(JointGoal.GAINS)

    def _send_stop(self):
        self._publish(JointGoal.STOP)

    def _send_trajectory(self, points):
        prev_t = 0.0
        for i, (t, q) in enumerate(points):
            dt = max(0.02, (t - prev_t) * self.velocity_scale)
            self._publish(
                JointGoal.MOVE if i == 0 else JointGoal.APPEND,
                position=q, duration=dt,
            )
            prev_t = t
            time.sleep(0.002)

    # -------------------------------------------------------------- execution
    def _result(self, code, text):
        res = FollowJointTrajectory.Result()
        res.error_code = int(code)
        res.error_string = str(text)
        return res

    def _abort(self, goal_handle, text):
        self._send_stop()
        goal_handle.abort()
        self.get_logger().error("轨迹中止: " + text)
        return self._result(ABORT_CODE, text)

    def _execute(self, goal_handle):
        self._cancel.clear()
        try:
            traj = goal_handle.request.trajectory
            points = self._ordered_points(traj)
            if not points:
                return self._abort(goal_handle, "轨迹为空")
            self._send_gains()
            time.sleep(0.02)
            self._send_trajectory(points)
            return self._wait(goal_handle, points)
        except Exception as exc:  # noqa: BLE001
            return self._abort(goal_handle, "执行异常: %s" % exc)
        finally:
            with self._goal_lock:
                self._goal_pending = False

    def _wait(self, goal_handle, points):
        final_q = points[-1][1]
        total = max(
            sum(max(0.02, (t - (points[i - 1][0] if i else 0.0))
                    * self.velocity_scale)
                for i, (t, _q) in enumerate(points)),
            0.05,
        )
        start = time.monotonic()
        deadline = start + total * 1.5 + self.completion_timeout + 2.0
        started = False
        last_fb = 0.0
        fb_period = 1.0 / self.feedback_rate

        while time.monotonic() < deadline:
            if self._cancel.is_set():
                self._send_stop()
                goal_handle.canceled()
                self.get_logger().warn("轨迹被取消，已请求平滑停止")
                return self._result(ABORT_CODE, "cancelled")
            now = time.monotonic()
            with self._lock:
                q = None if self._q is None else list(self._q)
                dq = None if self._dq is None else list(self._dq)
                q_age = now - self._q_t if self._q is not None else 1e9
                active = self._active
                pending = self._pending
            if q is None or q_age > self.feedback_timeout:
                return self._abort(goal_handle, "反馈超时 %.1f s" % q_age)
            if active:
                started = True
            if now - last_fb >= fb_period:
                last_fb = now
                fb = FollowJointTrajectory.Feedback()
                fb.header.stamp = self.get_clock().now().to_msg()
                fb.joint_names = list(self.joint_names)
                fb.actual.positions = list(q)
                if dq is not None:
                    fb.actual.velocities = list(dq)
                goal_handle.publish_feedback(fb)
            if started and not active and pending == 0:
                err = max(abs(a - b) for a, b in zip(q, final_q))
                if err > self.position_tolerance:
                    return self._abort(
                        goal_handle,
                        "到达误差 %.3f rad > %.3f" % (
                            err, self.position_tolerance),
                    )
                goal_handle.succeed()
                self.get_logger().info("轨迹完成（误差 %.3f rad）" % err)
                return self._result(FollowJointTrajectory.Result.SUCCESSFUL, "")
            if (not started and pending == 0
                    and now - start > 1.5):
                err = max(abs(a - b) for a, b in zip(q, final_q))
                if err <= self.position_tolerance:
                    goal_handle.succeed()
                    return self._result(
                        FollowJointTrajectory.Result.SUCCESSFUL, "")
        return self._abort(goal_handle, "轨迹执行超时")


def main(args=None):
    rclpy.init(args=args)
    node = TrajBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
