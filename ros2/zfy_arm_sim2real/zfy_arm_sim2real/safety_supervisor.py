"""Independent safety supervisor for the ZFY real arm.

It does not command motion.  It watches:
  * /joint_states freshness              (topic)
  * /arm_hmi/heartbeat                   (the HMI control process)
  * /motor_driver/status                 (enabled/sending, ERR, feedback age)

Actions:
  * critical driver ERR (8..14: voltage/current/temperature/comm/overload)
    -> call /motor_driver/release once and log loudly.
  * stale /joint_states or missing HMI heartbeat while the driver is enabled
    -> repeat a warning every 5 s.  It does NOT release on these, because a
       release drops the gravity-compensated hold; the physical E-stop is the
       final authority.
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

from zfy_arm_interfaces.srv import GetMotorStatus

CRITICAL_ERR = {8, 9, 10, 11, 12, 13, 14}


class SafetySupervisor(Node):
    def __init__(self):
        super().__init__("safety_supervisor")
        self.declare_parameter("state_timeout", 1.0)
        self.declare_parameter("hmi_timeout", 1.5)
        self.declare_parameter("status_period", 1.0)
        self.state_timeout = float(self.get_parameter("state_timeout").value)
        self.hmi_timeout = float(self.get_parameter("hmi_timeout").value)
        status_period = float(self.get_parameter("status_period").value)

        self._state_stamp = 0.0
        self._hb_stamp = 0.0
        self._hb_seen = False
        self._status = None
        self._status_stamp = 0.0
        self._status_busy = False
        self._released = False
        self._last_warn = 0.0

        self.create_subscription(JointState, "/joint_states", self._on_state, 20)
        self.create_subscription(Empty, "/arm_hmi/heartbeat", self._on_hb, 10)
        self.cli_status = self.create_client(
            GetMotorStatus, "/motor_driver/status"
        )
        self.cli_release = self.create_client(Trigger, "/motor_driver/release")
        self.create_timer(status_period, self._poll_status)
        self.create_timer(0.5, self._watchdog)
        self.get_logger().info(
            "安全监督已启动：state_timeout=%.1f s, hmi_timeout=%.1f s"
            % (self.state_timeout, self.hmi_timeout)
        )

    def _on_state(self, _msg):
        self._state_stamp = time.monotonic()

    def _on_hb(self, _msg):
        self._hb_stamp = time.monotonic()
        self._hb_seen = True

    def _poll_status(self):
        if self._status_busy or not self.cli_status.service_is_ready():
            return
        self._status_busy = True
        future = self.cli_status.call_async(GetMotorStatus.Request())
        future.add_done_callback(self._on_status)

    def _on_status(self, future):
        self._status_busy = False
        try:
            res = future.result()
        except Exception:
            return
        if res is None or not res.success:
            return
        self._status = res
        self._status_stamp = time.monotonic()
        if not bool(res.enabled):
            self._released = False

    def _watchdog(self):
        now = time.monotonic()
        st = self._status
        if st is None:
            return
        if not bool(st.enabled):
            self._released = False
            return

        errs = [int(e) for e in st.err]
        bad = [i + 1 for i, e in enumerate(errs) if e in CRITICAL_ERR]
        if bad and not self._released:
            self._released = True
            self.get_logger().error(
                "检测到严重 ERR %s（关节 %s），自动 release"
                % (bad, ", ".join("J%d" % i for i in bad))
            )
            self._release()

        if now - self._state_stamp > self.state_timeout:
            self._warn(
                "超过 %.1f s 没有 /joint_states；检查驱动/反馈"
                % self.state_timeout
            )
        if self._hb_seen and now - self._hb_stamp > self.hmi_timeout:
            self._warn(
                "HMI 心跳超过 %.1f s 未更新；驱动会 hold 最后目标，"
                "需要人工确认或物理急停" % self.hmi_timeout
            )
        if bool(st.sending):
            ages = [float(a) for a in st.age]
            if ages and max(ages) > 1.0:
                self._warn(
                    "电机反馈超过 1 s 未更新（最大 %.2f s）" % max(ages)
                )

    def _warn(self, text):
        now = time.monotonic()
        if now - self._last_warn > 5.0:
            self._last_warn = now
            self.get_logger().warn(text)

    def _release(self):
        if not self.cli_release.service_is_ready():
            self.get_logger().error("release 服务不可用，无法自动失能")
            self._released = False
            return
        future = self.cli_release.call_async(Trigger.Request())
        future.add_done_callback(self._on_release)

    def _on_release(self, future):
        try:
            res = future.result()
        except Exception as exc:
            self.get_logger().error("release 调用异常: %s" % exc)
            self._released = False
            return
        if res is None or not res.success:
            self.get_logger().error("release 失败，将在下一周期重试")
            self._released = False
        else:
            self.get_logger().warn("安全 release: " + res.message)


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
