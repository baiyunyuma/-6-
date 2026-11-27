import sys
import time

sys.path.insert(0, "/home/sky/zfy_arm/Zfy_Arm_real_control/py")

from motor_config import MotorSpec
from socketcan_bus import SocketCanBus

spec = MotorSpec("joint1", 1, 1, "4340", 0, "MIT")
bus = SocketCanBus([spec], channel="can0", bitrate=1000000)

t0 = time.monotonic()
for i in range(160):  # 8 s @ 20 Hz
    bus.mit(1, 0.0, 0.0, 0.0, 0.0, 0.0)
    time.sleep(0.05)
    if i % 20 == 19:
        st = bus.states()[1]
        print("t=%.1fs q=%+.3f dq=%+.3f tau=%+.3f err=%d rx=%.1f" % (
            time.monotonic() - t0, st["q"], st["dq"], st["tau"], st["err"],
            bus.rx_hz))
bus.close()
