#!/usr/bin/env python3
"""
实机测试：使能 → MIT 控制（kd=2，位置 -1.6 rad）→ 运动完成后失能

时序：
  1. 打开总线（经典 CAN，1 Mbps，通道 1）
  2. 切 MIT 模式 → 使能
  3. 使能后**重读**起始位置（电机未使能时不回反馈，之前读到的是假 0）
  4. 目标从起始位置按斜坡推到 -1.6 rad（不阶跃，避免力矩顶限幅）
  5. 保持到稳定（|误差| < 0.03 rad 且 |速度| < 0.1 rad/s 连续 0.5 s）
  6. 失能 → 关闭

    python3 test_move.py
    python3 test_move.py --kp 20 --kd 2 --target -1.6
"""
import argparse
import sys
import time

from motor_config import MotorSpec
from core import MotorBus

CAN_ID = 0x01
CHANNEL = 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kp", type=float, default=10.0)
    ap.add_argument("--kd", type=float, default=2.0)
    ap.add_argument("--target", type=float, default=-1.6)
    ap.add_argument("--ramp", type=float, default=3.0, help="斜坡时长 [s]")
    ap.add_argument("--rate", type=float, default=250.0, help="控制频率 [Hz]")
    args = ap.parse_args()

    spec = [MotorSpec("motor1", CAN_ID, CAN_ID, "4340", channel=CHANNEL)]

    print("=== 1. 打开总线（经典 CAN 1M，通道 %d）===" % CHANNEL, flush=True)
    bus = MotorBus(spec, nom_baud=1_000_000, dat_baud=1_000_000,
                   sim=False, canfd=False)
    time.sleep(0.3)

    # 诊断：回读通道配置
    try:
        info = bus.mc.device.get_channel_baudrate(CHANNEL)
        print("  ch%d 回读: canfd=%s nom=%s data=%s"
              % (CHANNEL, info.canfd, info.can_baudrate, info.canfd_baudrate),
              flush=True)
    except Exception as exc:
        print("  回读失败:", exc, flush=True)

    def state():
        return bus.states().get(CAN_ID)

    print("\n=== 2. 切 MIT 模式 ===", flush=True)
    bus.set_mode(CAN_ID, "MIT")
    time.sleep(0.2)

    print("\n=== 3. 使能 ===", flush=True)
    bus.enable(CAN_ID, True)
    time.sleep(0.3)

    # 使能后重读起始位置（未使能时电机不回反馈）
    q0 = None
    for _ in range(30):
        bus.mit(CAN_ID, 0.0, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.02)
        st = state()
        if st:
            q0 = st["q"]
            break
    if q0 is None:
        print("  ✘ 使能后仍收不到反馈，退出", flush=True)
        bus.close()
        return 1
    st = state()
    print("  使能后: q=%+.4f dq=%+.4f tau=%+.4f err=%s"
          % (st["q"], st["dq"], st["tau"], st.get("err")), flush=True)
    print("  实际起点 q0 = %+.4f rad (%.1f deg)" % (q0, q0 * 57.2958), flush=True)

    print("\n=== 4. 斜坡运动：kp=%.1f kd=%.1f  %.3f → %.3f rad（%.1f s）==="
          % (args.kp, args.kd, q0, args.target, args.ramp), flush=True)
    dt = 1.0 / args.rate
    t0 = time.time()
    last_log = -1.0
    while True:
        t = time.time() - t0
        a = min(t / args.ramp, 1.0)
        q_cmd = q0 + (args.target - q0) * a
        bus.mit(CAN_ID, args.kp, args.kd, q_cmd, 0.0, 0.0)
        st = state()
        if st and t - last_log >= 0.5:
            last_log = t
            print("  t=%4.1fs  cmd=%+.3f  q=%+.4f  dq=%+.3f  tau=%+.2f"
                  % (t, q_cmd, st["q"], st["dq"], st["tau"]), flush=True)
        if a >= 1.0:
            break
        time.sleep(dt)

    print("\n=== 5. 保持直到稳定 ===", flush=True)
    settled_since = None
    t_hold = time.time()
    ok = False
    while time.time() - t_hold < 12.0:
        bus.mit(CAN_ID, args.kp, args.kd, args.target, 0.0, 0.0)
        st = state()
        if st:
            err = abs(st["q"] - args.target)
            if err < 0.03 and abs(st["dq"]) < 0.1:
                if settled_since is None:
                    settled_since = time.time()
                elif time.time() - settled_since > 0.5:
                    print("  ✔ 已稳定: q=%+.4f (目标 %.3f)  误差 %.4f rad = %.2f°"
                          % (st["q"], args.target, err, err * 57.2958), flush=True)
                    ok = True
                    break
            else:
                settled_since = None
        time.sleep(dt)
    if not ok:
        st = state()
        print("  ⚠ 12 秒未达到 0.03 rad 判据，当前 q=%+.4f (误差 %.4f rad = %.2f°)"
              % (st["q"], abs(st["q"] - args.target),
                 abs(st["q"] - args.target) * 57.2958), flush=True)
        print("     —— kp=%.1f 时的比例下垂是正常的；要更准就加 kp 或加积分/重力前馈"
              % args.kp, flush=True)

    print("\n=== 6. 失能 ===", flush=True)
    bus.enable(CAN_ID, False)
    time.sleep(0.3)
    st = state()
    if st:
        print("  失能后: q=%+.4f tau=%+.4f err=%s"
              % (st["q"], st["tau"], st.get("err")), flush=True)

    print("\n=== 7. 关闭 ===", flush=True)
    bus.close()
    print("DONE", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
