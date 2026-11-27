#!/usr/bin/env python3
"""SocketCAN 后端回归测试（无需真实电机）。

用法：
    python3 test_socketcan_vcan.py              # 纯逻辑 + vcan0 收发
    python3 test_socketcan_vcan.py --no-can     # 只跑纯逻辑
    python3 test_socketcan_vcan.py --iface vcan1

vcan 测试前先建接口：
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set vcan0 up

设计原则：不依赖厂商 SDK（x86 专用），所有参考编码都在本文件里重新
实现一遍，逐字节与 dm_backend/damiao.py 的 control_mit 对照。
"""
from __future__ import annotations

import argparse
import os
import socket
import struct
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from motor_config import MotorSpec, MODELS  # noqa: E402
from socketcan_bus import (  # noqa: E402
    MODE_OFFSET,
    SocketCanBus,
    encode_mit,
)

_CAN_FRAME = struct.Struct("=IB3x8s")
_SPEC = MotorSpec("joint1", 0x01, 0x01, "4340", 0, "MIT")
_SPEC4 = MotorSpec("joint4", 0x04, 0x04, "4310", 0, "MIT")


def fresh_spec(cid=1, model="4340"):
    """每个用例独立一份，避免模式同步测试改到共享对象。"""
    return MotorSpec("joint%d" % cid, cid, cid, model, 0, "MIT")


# --------------------------------------------------------------------------
# 参考实现（与 py/dm_backend/damiao.py 的 control_mit 一致）
# --------------------------------------------------------------------------
def _f2u(value, lo, hi, bits):
    value = max(lo, min(hi, float(value)))
    return int((value - lo) / (hi - lo) * ((1 << bits) - 1))


def _u2f(value, lo, hi, bits):
    return (float(value) / ((1 << bits) - 1)) * (hi - lo) + lo


def _limits(spec):
    return MODELS[spec.model][1:]


def ref_encode(spec, kp, kd, q, dq, tau):
    q_max, dq_max, tau_max = _limits(spec)
    kp_uint = _f2u(kp, 0.0, 500.0, 12)
    kd_uint = _f2u(kd, 0.0, 5.0, 12)
    q_uint = _f2u(q, -q_max, q_max, 16)
    dq_uint = _f2u(dq, -dq_max, dq_max, 12)
    tau_uint = _f2u(tau, -tau_max, tau_max, 12)
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


def pack_feedback(spec, cid, err, q, dq, tau):
    """Pack an MIT feedback frame exactly as the DM driver sends it."""
    q_max, dq_max, tau_max = _limits(spec)
    q_uint = _f2u(q, -q_max, q_max, 16)
    dq_uint = _f2u(dq, -dq_max, dq_max, 12)
    tau_uint = _f2u(tau, -tau_max, tau_max, 12)
    return bytes([
        ((int(err) & 0xF) << 4) | (int(cid) & 0xF),
        (q_uint >> 8) & 0xFF,
        q_uint & 0xFF,
        (dq_uint >> 4) & 0xFF,
        ((dq_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF),
        tau_uint & 0xFF,
        0,
        0,
    ])


def pack_param(cid, cmd, rid, value, as_int=True):
    payload = (struct.pack("<I", int(value)) if as_int
               else struct.pack("<f", float(value)))
    return bytes([cid & 0xFF, (cid >> 8) & 0xFF, cmd, rid]) + payload


def pack_can(can_id, data):
    data = bytes(data)
    return _CAN_FRAME.pack(int(can_id) & 0x7FF, len(data),
                           data.ljust(8, b"\0"))


def unpack_can(buf):
    raw_id, dlc, payload = _CAN_FRAME.unpack(buf)
    return raw_id & 0x7FF, payload[: min(dlc, 8)]


# --------------------------------------------------------------------------
# 纯逻辑测试（不需要 CAN 接口）
# --------------------------------------------------------------------------
def test_encode_matches_vendor():
    for spec in (_SPEC, _SPEC4):
        for args in (
            (80.0, 5.0, 0.3, -0.2, 1.5),
            (0.0, 0.0, 0.0, 0.0, 0.0),
            (500.0, 5.0, 12.5, 10.0, 28.0),
            (80.0, 5.0, -12.5, -10.0, -28.0),
        ):
            got = encode_mit(spec, *args)
            want = ref_encode(spec, *args)
            assert got == want, (
                "%s %r\n got  %s\n want %s"
                % (spec.model, args, got.hex(), want.hex())
            )


def test_decode_feedback():
    bus = SocketCanBus([fresh_spec()], sim=True)
    err, q, dq, tau = 3, 0.3, -0.2, 1.5
    bus._handle_frame(0, 0x01, pack_feedback(_SPEC, 1, err, q, dq, tau),
                      time.monotonic())
    assert 1 in bus._latest, "反馈没有被解码"
    got_q, got_dq, got_tau, got_err = bus._latest[1]
    assert got_err == err, (got_err, err)
    assert abs(got_q - q) < 2.5e-3, (got_q, q)
    assert abs(got_dq - dq) < 0.02, (got_dq, dq)
    assert abs(got_tau - tau) < 0.05, (got_tau, tau)


def test_param_collision_is_feedback():
    """q 接近 -q_max 时，byte0=id, byte1=0, byte2=0x55 的反馈不能被吞掉。"""
    bus = SocketCanBus([fresh_spec()], sim=True)
    q_max, dq_max, tau_max = _limits(_SPEC)
    q_uint = 0x0055
    dq_uint = 0x800
    tau_uint = 0x800
    frame = bytes([
        0x01,  # err=0, id=1
        0x00,  # q high byte
        0x55,  # q low byte == a parameter-command byte
        (dq_uint >> 4) & 0xFF,
        ((dq_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF),
        tau_uint & 0xFF,
        0,
        0,
    ])
    bus._handle_frame(0, 0x01, frame, time.monotonic())
    assert 1 in bus._latest, "碰撞带反馈被当成参数帧吞掉了（回归）"
    q = _u2f(q_uint, -q_max, q_max, 16)
    assert abs(bus._latest[1][0] - q) < 1e-3
    assert not bus._param_event.is_set()


def test_param_reply_updates_mode():
    bus = SocketCanBus([fresh_spec()], sim=True)
    now = time.monotonic()
    bus._param_active = True
    bus._param_active_t = now
    bus._param_expected = (0, 1, 10)
    bus._param_event.clear()
    bus._handle_frame(0, 0x7FF, pack_param(1, 0x33, 10, 2), now)
    assert bus._param_event.is_set(), "参数回复没有唤醒等待者"
    assert bus._param_result == 2, bus._param_result
    assert bus.mode_of(1) == "POS_VEL", bus.mode_of(1)


def test_unexpected_param_reply_ignored():
    bus = SocketCanBus([fresh_spec()], sim=True)
    bus._param_event.clear()
    bus._handle_frame(0, 0x7FF, pack_param(1, 0x33, 10, 2), time.monotonic())
    assert not bus._param_event.is_set()
    assert bus.mode_of(1) == "MIT", bus.mode_of(1)


def test_split_control_and_feedback_ids():
    """控制 ID=0x11、反馈 ID=0x01：发送走 0x11，反馈按 0x01 收。"""
    spec = MotorSpec("joint1", 0x11, 0x01, "4340", 0, "MIT")
    bus = SocketCanBus([spec], sim=True)

    # 反馈在 MST_ID=0x01 上回来，状态仍归到控制 ID=0x11
    q = 0.25
    bus._handle_frame(
        0, 0x01, pack_feedback(spec, 0x11, 0, q, 0, 0), time.monotonic()
    )
    assert 0x11 in bus._latest, "反馈没有被归到控制 ID"
    assert abs(bus._latest[0x11][0] - q) < 2.5e-3

    # 控制帧发到 ESC_ID=0x11（MIT 偏移 0）
    sent = []
    bus.sim = False  # 只测组帧；_send_frame 用桩函数，不碰真实 socket
    bus._send_frame = (
        lambda ch, cid, payload, dlc=None:
        sent.append((int(ch), int(cid), bytes(payload))) or True
    )
    bus.mit(0x11, 0.0, 0.0, 0.1, 0.0, 0.0)
    assert sent[-1][1] == 0x11, sent[-1]

    # 参数帧的 0x7FF 负载里用控制 ID 寻址
    sent.clear()
    bus._write_param_raw(0x11, 8, bytes([0x11, 0, 0, 0]), wait=0.0)
    assert sent[-1][1] == 0x7FF, sent[-1]
    assert sent[-1][2][:2] == bytes([0x11, 0x00]), sent[-1][2].hex()

    # 参数回复即使带 MST_ID=0x01，也要能匹配回同一台电机
    now = time.monotonic()
    with bus._param_lock:
        bus._param_active = True
        bus._param_active_t = now
        bus._param_expected = (0, 0x11, 10)
        bus._param_result = None
        bus._param_event.clear()
    bus._handle_frame(0, 0x7FF, pack_param(0x01, 0x33, 10, 2), now)
    assert bus._param_event.is_set(), "带 MST_ID 的参数回复没有被接受"
    assert bus._param_result == 2, bus._param_result
    assert bus.mode_of(0x11) == "POS_VEL", bus.mode_of(0x11)


def test_send_is_nonblocking_and_counts_drop():
    """发送必须用 MSG_DONTWAIT；队列满时丢帧计数，而不是阻塞控制环。"""
    spec = MotorSpec("joint1", 0x11, 0x01, "4340", 0, "MIT")
    bus = SocketCanBus([spec], sim=True)
    bus.sim = False

    calls = []

    class OkSock:
        def send(self, buf, flags=0):
            calls.append(flags)
            return len(buf)

    bus._socks[0] = OkSock()
    assert bus._send_frame(0, 0x11, b"\0" * 8) is True
    assert calls and calls[-1] == socket.MSG_DONTWAIT
    assert bus.tx_frames == 1

    class FullSock:
        def send(self, buf, flags=0):
            raise BlockingIOError(11, "Resource temporarily unavailable")

    bus._socks[0] = FullSock()
    before = bus.tx_dropped
    assert bus._send_frame(0, 0x11, b"\0" * 8) is False
    assert bus.tx_dropped == before + 1


def test_set_mode_requires_motor_ack():
    """没有 0x55 应答时不能谎报切换成功，也不能改本地模式。"""
    spec = MotorSpec("joint1", 0x11, 0x01, "4340", 0, "MIT")
    bus = SocketCanBus([spec], sim=True)
    bus.sim = False
    sent = []
    bus._send_frame = (
        lambda ch, cid, payload, dlc=None:
        (sent.append(int(cid)), True)[1]
    )
    assert bus.set_mode(0x11, "POS_VEL") is False
    assert bus.mode_of(0x11) == "MIT", bus.mode_of(0x11)
    assert sent and sent[-1] == 0x7FF


def test_enable_mode_switch_requires_ack():
    """使能时如果 RID 10 没应答，必须失败，不能盲发 0xFC 到错误偏移。"""
    spec = MotorSpec("joint1", 0x11, 0x01, "4340", 0, "MIT")
    bus = SocketCanBus([spec], sim=True)
    bus.sim = False
    bus._modes[0x11] = "POS_VEL"
    bus._send_frame = lambda ch, cid, payload, dlc=None: True
    try:
        bus.enable(0x11, True, switch_mode=True)
    except RuntimeError as exc:
        assert "未收到应答" in str(exc), str(exc)
    else:
        raise AssertionError("没有 0x55 应答时 enable 不应成功")


def test_save_requires_ack():
    """保存 Flash 必须等 0xAA 应答，不能只发不等。"""
    spec = MotorSpec("joint1", 0x11, 0x01, "4340", 0, "MIT")
    bus = SocketCanBus([spec], sim=True)
    bus.sim = False
    bus._send_frame = lambda ch, cid, payload, dlc=None: True
    assert bus.save(0x11) is False


def test_multi_channel_rejected():
    specs = [
        MotorSpec("a", 1, 1, "4340", 0),
        MotorSpec("b", 2, 2, "4340", 1),
    ]
    try:
        SocketCanBus(specs, sim=True)
    except ValueError as exc:
        assert "channel" in str(exc)
    else:
        raise AssertionError("多通道注册没有被拒绝")


def test_driver_guard():
    """非 gs_usb、非 vcan 的接口必须在绑定前被拒绝。"""
    try:
        SocketCanBus([fresh_spec()], sim=False, channel="lo", require_driver="gs_usb")
    except RuntimeError as exc:
        assert "驱动" in str(exc), str(exc)
    else:
        raise AssertionError("驱动校验没有生效")


def test_bad_interface():
    try:
        SocketCanBus([fresh_spec()], sim=False, channel="no_such_can",
                     require_driver=None)
    except RuntimeError as exc:
        assert "绑定" in str(exc) or "SocketCAN" in str(exc), str(exc)
    else:
        raise AssertionError("不存在的接口没有被拒绝")


def dual_specs():
    """J1-J3 on channel 0, J4-J6 on channel 1 (the real wiring)."""
    out = []
    for i in range(1, 7):
        out.append(MotorSpec(
            "joint%d" % i, i, i,
            "4340" if i <= 3 else "4310",
            0 if i <= 3 else 1,
            "MIT",
        ))
    return out


def test_dual_sim_channel_status():
    bus = SocketCanBus(dual_specs(), sim=True,
                       channels=["vcan_tx0", "vcan_tx1"])
    assert set(bus.channel_status) == {0, 1}, bus.channel_status
    assert bus.channel_status[0]["iface"] == "vcan_tx0"
    assert bus.channel_status[1]["iface"] == "vcan_tx1"
    bus.mit(1, 0, 0, 0.5, 0, 0)
    bus.mit(4, 0, 0, -0.5, 0, 0)
    assert bus._sim_target[1] == 0.5
    assert bus._sim_target[4] == -0.5


def test_dual_interface_routing(iface0, iface1):
    specs = dual_specs()
    bus = SocketCanBus(specs, sim=False, channels=[iface0, iface1],
                       bitrate=1_000_000, require_driver=None)
    obs0 = Observer(iface0)
    obs1 = Observer(iface1)
    try:
        obs0.frames(0.05)
        obs1.frames(0.05)

        # J1 (channel 0) must only appear on iface0
        bus.enable(1, True, switch_mode=False)
        got0 = [d for cid, d in obs0.frames(0.3) if cid == 1]
        got1 = [d for cid, d in obs1.frames(0.3) if cid == 1]
        assert len(got0) >= 5, "channel 0 没有发出使能帧"
        assert not got1, "channel 0 的帧串到 channel 1"

        # J4 (channel 1) must only appear on iface1
        obs0.frames(0.05)
        obs1.frames(0.05)
        bus.enable(4, True, switch_mode=False)
        got0 = [d for cid, d in obs0.frames(0.3) if cid == 4]
        got1 = [d for cid, d in obs1.frames(0.3) if cid == 4]
        assert len(got1) >= 5, "channel 1 没有发出使能帧"
        assert not got0, "channel 1 的帧串到 channel 0"

        # MIT commands route by MotorSpec.channel
        obs0.frames(0.05)
        obs1.frames(0.05)
        bus.mit(1, 0, 0, 0.0, 0.0, 0.0)
        bus.mit(4, 0, 0, 0.0, 0.0, 0.0)
        c0 = [cid for cid, _ in obs0.frames(0.2)]
        c1 = [cid for cid, _ in obs1.frames(0.2)]
        assert 1 in c0 and 4 not in c0, c0
        assert 4 in c1 and 1 not in c1, c1

        # Feedback is matched per channel; same ID on the wrong bus is ignored
        bus._handle_frame(0, 1, pack_feedback(specs[0], 1, 0, 0.25, 0, 0),
                          time.monotonic())
        bus._handle_frame(1, 1, pack_feedback(specs[0], 1, 0, -0.25, 0, 0),
                          time.monotonic())
        assert abs(bus._latest[1][0] - 0.25) < 1e-3, bus._latest[1]
        bus._handle_frame(1, 4, pack_feedback(specs[3], 4, 0, -0.3, 0, 0),
                          time.monotonic())
        assert abs(bus._latest[4][0] + 0.3) < 1e-3, bus._latest[4]

        # Parameter writes must leave on the motor's own channel
        obs0.frames(0.05)
        obs1.frames(0.05)
        bus.write_param(4, 10, 2)
        p0 = [(cid, d) for cid, d in obs0.frames(0.2) if cid == 0x7FF]
        p1 = [(cid, d) for cid, d in obs1.frames(0.2) if cid == 0x7FF]
        assert any(d[2] == 0x55 and d[3] == 10 for _, d in p1), p1
        assert not any(d[2] == 0x55 and d[3] == 10 for _, d in p0), p0
    finally:
        obs0.close()
        obs1.close()
        bus.close()


# --------------------------------------------------------------------------
# vcan 收发测试
# --------------------------------------------------------------------------
class Observer:
    def __init__(self, iface):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW,
                                  socket.CAN_RAW)
        self.sock.bind((iface,))
        self.sock.settimeout(0.2)

    def frames(self, duration=0.4):
        out = []
        end = time.monotonic() + duration
        while time.monotonic() < end:
            try:
                buf = self.sock.recv(16)
            except socket.timeout:
                continue
            if len(buf) == 16:
                out.append(unpack_can(buf))
        return out

    def send(self, can_id, data):
        self.sock.send(pack_can(can_id, data))

    def close(self):
        self.sock.close()


def _wait_frames(observer, predicate, timeout=1.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            buf = observer.sock.recv(16)
        except socket.timeout:
            continue
        if len(buf) != 16:
            continue
        can_id, data = unpack_can(buf)
        if predicate(can_id, data):
            return can_id, data
    return None


def test_enable_disable_frames(iface):
    bus = SocketCanBus([fresh_spec()], sim=False, channel=iface,
                       bitrate=1_000_000, require_driver=None)
    obs = Observer(iface)
    try:
        obs.frames(0.05)  # 清空
        bus.enable(1, True, switch_mode=False)
        got = [d for cid, d in obs.frames(0.3) if cid == 1]
        assert len(got) >= 5, "使能帧不足: %d" % len(got)
        assert all(d == b"\xff" * 7 + b"\xfc" for d in got), got

        obs.frames(0.05)
        bus.enable(1, False)
        got = [d for cid, d in obs.frames(0.3) if cid == 1]
        assert got and all(d == b"\xff" * 7 + b"\xfd" for d in got), got
    finally:
        obs.close()
        bus.close()


def test_param_roundtrip(iface):
    bus = SocketCanBus([fresh_spec()], sim=False, channel=iface,
                       bitrate=1_000_000, require_driver=None)
    obs = Observer(iface)
    stop = threading.Event()

    def responder():
        # 只回复一次读、一次写，避免看到自己的回复后死循环
        read_done = write_done = False
        while not stop.is_set():
            try:
                buf = obs.sock.recv(16)
            except socket.timeout:
                continue
            if len(buf) != 16:
                continue
            can_id, data = unpack_can(buf)
            if can_id != 0x7FF or len(data) < 8:
                continue
            cid = data[0] | (data[1] << 8)
            if cid != 1:
                continue
            if data[2] == 0x33 and data[3] == 10 and not read_done:
                read_done = True
                obs.send(0x7FF, pack_param(1, 0x33, 10, 3))
            elif (data[2] == 0x55 and data[3] == 10
                  and data[4:8] == struct.pack("<I", 2) and not write_done):
                write_done = True
                obs.send(0x7FF, pack_param(1, 0x55, 10, 2))

    th = threading.Thread(target=responder, daemon=True)
    th.start()
    try:
        value = bus.read_param(1, 10, wait=0.6)
        assert value == 3, "read_param 返回 %r" % (value,)

        bus.write_param(1, 10, 2)
        assert bus.mode_of(1) == "POS_VEL", bus.mode_of(1)

        # 先停掉应答线程，否则它会和主线程抢同一个 observer socket
        stop.set()
        th.join(timeout=0.5)

        obs.frames(0.05)
        bus.enable(1, True, switch_mode=False)
        got = [(cid, d) for cid, d in obs.frames(0.3) if cid == 0x101]
        assert len(got) >= 5, "POS_VEL 偏移帧不足: %d" % len(got)
        assert all(d == b"\xff" * 7 + b"\xfc" for _, d in got), got
        bus.enable(1, False)
    finally:
        stop.set()
        obs.close()
        bus.close()


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="vcan0")
    ap.add_argument("--iface2", default="vcan1",
                    help="双路测试的第二个接口（J4-J6）")
    ap.add_argument("--no-can", action="store_true",
                    help="只跑纯逻辑测试，不碰 CAN 接口")
    args = ap.parse_args()

    tests = [
        test_encode_matches_vendor,
        test_decode_feedback,
        test_param_collision_is_feedback,
        test_param_reply_updates_mode,
        test_unexpected_param_reply_ignored,
        test_split_control_and_feedback_ids,
        test_send_is_nonblocking_and_counts_drop,
        test_set_mode_requires_motor_ack,
        test_enable_mode_switch_requires_ack,
        test_save_requires_ack,
        test_multi_channel_rejected,
        test_dual_sim_channel_status,
        test_driver_guard,
        test_bad_interface,
    ]
    if not args.no_can:
        have0 = os.path.exists("/sys/class/net/%s" % args.iface)
        have1 = os.path.exists("/sys/class/net/%s" % args.iface2)
        if not have0:
            print("跳过 vcan 测试：接口 %s 不存在。先执行：" % args.iface)
            print("  sudo modprobe vcan")
            print("  sudo ip link add dev %s type vcan && sudo ip link set %s up"
                  % (args.iface, args.iface))
            print("  sudo ip link add dev %s type vcan && sudo ip link set %s up"
                  % (args.iface2, args.iface2))
        else:
            def vcan_enable():
                test_enable_disable_frames(args.iface)

            def vcan_param():
                test_param_roundtrip(args.iface)

            tests += [vcan_enable, vcan_param]
            if have1:
                def vcan_dual():
                    test_dual_interface_routing(args.iface, args.iface2)

                tests += [vcan_dual]
            else:
                print("跳过双路测试：接口 %s 不存在" % args.iface2)

    failed = 0
    for fn in tests:
        name = getattr(fn, "__name__", "vcan_test")
        try:
            fn()
            print("PASS  %s" % name)
        except Exception as exc:
            failed += 1
            print("FAIL  %s: %s" % (name, exc))
    print("\n%d passed, %d failed" % (len(tests) - failed, failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
