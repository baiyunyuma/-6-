"""单关节正弦测试：给 motor_driver 发 MIT 指令，验证 ROS 2 控制链路。

会先使能并开始发送 MIT，其他关节按 --hold-kp/--hold-kd 保持，再让目标关节
做慢速正弦，结束后回到中点、保持 0.5 s、零力矩失能。实机运行必须先加 --confirm。
"""

from __future__ import annotations

import argparse
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--confirm", action="store_true", help="确认机械臂周围安全、允许其运动"
    )
    parser.add_argument("--joint", type=int, default=4, choices=range(1, 7), metavar="1-6")
    parser.add_argument("--amplitude", type=float, default=0.10, help="rad")
    parser.add_argument("--period", type=float, default=6.0, help="s")
    parser.add_argument("--seconds", type=float, default=12.0, help="s")
    parser.add_argument("--kp", type=float, default=5.0, help="目标关节 kp")
    parser.add_argument("--kd", type=float, default=0.3, help="目标关节 kd")
    parser.add_argument("--hold-kp", type=float, default=5.0, help="其余关节保持 kp")
    parser.add_argument("--hold-kd", type=float, default=0.3, help="其余关节保持 kd")
    parser.add_argument(
        "--no-release", action="store_true", help="结束时保持使能（默认零力矩失能）"
    )
    return parser.parse_args()


class MitDemo(Node):
    def __init__(self):
        super().__init__("mit_demo")
        self.latest: Optional[JointState] = None
        self.count = 0
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        self.pub = self.create_publisher(Float64MultiArray, "/mit_command", 10)
        self.enable_cli = self.create_client(SetBool, "/motor_driver/enable")
        self.sending_cli = self.create_client(SetBool, "/motor_driver/sending")
        self.release_cli = self.create_client(Trigger, "/motor_driver/release")

    def _on_state(self, msg):
        self.latest = msg
        self.count += 1

    def wait_for_state(self, timeout=5.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest is not None:
                return True
        return False

    def wait_for_new_state(self, previous_count, timeout=2.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.count > previous_count:
                return True
        return False

    def call(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            future.cancel()
            return None
        return future.result()


def main(args=None):
    opts = _parse_args()
    if not opts.confirm:
        print("拒绝运行：这可能驱动真机。确认周围安全后加 --confirm 再执行。")
        return 2
    if opts.period <= 0.0 or opts.seconds <= 0.0 or opts.amplitude < 0.0:
        print("--period / --seconds 必须为正，--amplitude 不能为负")
        return 2

    rclpy.init(args=[])
    node = MitDemo()
    try:
        if not node.wait_for_state():
            print("5 s 内没收到 /joint_states，motor_driver 启动了吗？")
            return 1

        response = node.call(node.enable_cli, SetBool.Request(data=True))
        if response is None:
            print("调用 /motor_driver/enable 失败（服务不可用或超时）")
            return 1
        if not response.success:
            print(f"使能失败: {response.message}")
            return 1
        print(f"使能: {response.message}")

        response = node.call(node.sending_cli, SetBool.Request(data=True))
        if response is None:
            print("调用 /motor_driver/sending 失败（服务不可用或超时）")
            return 1
        if not response.success:
            print(f"开始发送失败: {response.message}")
            return 1
        print(f"发送: {response.message}")

        count = node.count
        if not node.wait_for_new_state(count):
            print("开始发送后没有新的 /joint_states，中止")
            return 1
        positions = dict(zip(node.latest.name, node.latest.position))
        missing = [name for name in JOINT_NAMES if name not in positions]
        if missing:
            print(f"/joint_states 缺少 {missing}，中止")
            return 1

        hold_q = [float(positions[name]) for name in JOINT_NAMES]
        index = opts.joint - 1
        q0 = hold_q[index]
        n = len(JOINT_NAMES)
        print(
            f"关节 {opts.joint} 起点 {q0:+.3f} rad，"
            f"幅度 ±{opts.amplitude:.3f} rad，周期 {opts.period:.1f} s，"
            f"持续 {opts.seconds:.1f} s"
        )
        print("Ctrl+C 中止；正常结束后会先保持中点再释放电机")

        publish = Float64MultiArray()
        start = time.monotonic()
        next_t = start
        try:
            while rclpy.ok():
                elapsed = time.monotonic() - start
                if elapsed >= opts.seconds:
                    break
                kp = [opts.hold_kp] * n
                kd = [opts.hold_kd] * n
                q = list(hold_q)
                kp[index] = opts.kp
                kd[index] = opts.kd
                q[index] = q0 + opts.amplitude * math.sin(
                    2.0 * math.pi * elapsed / opts.period
                )
                publish.data = [*kp, *kd, *q, *([0.0] * n), *([0.0] * n)]
                node.pub.publish(publish)
                rclpy.spin_once(node, timeout_sec=0.0)
                next_t += 0.01
                delay = next_t - time.monotonic()
                if delay > 0.0:
                    time.sleep(delay)
                else:
                    next_t = time.monotonic()
        except KeyboardInterrupt:
            print("收到 Ctrl+C，停止运动")
        finally:
            # 无论正常结束还是中断，先回中点保持 0.5 s，避免直接掉落
            kp = [opts.hold_kp] * n
            kd = [opts.hold_kd] * n
            kp[index] = opts.kp
            kd[index] = opts.kd
            publish.data = [*kp, *kd, *hold_q, *([0.0] * n), *([0.0] * n)]
            try:
                for _ in range(50):
                    node.pub.publish(publish)
                    rclpy.spin_once(node, timeout_sec=0.0)
                    time.sleep(0.01)
            except KeyboardInterrupt:
                pass
    finally:
        if not opts.no_release:
            response = node.call(node.release_cli, Trigger.Request(), timeout=5.0)
            if response is not None and response.success:
                print(f"已释放: {response.message}")
            else:
                print("释放失败或超时，请手动停止 driver 或断开电机电源")
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
