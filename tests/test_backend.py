import sys
import time

sys.path.insert(0, "/home/sky/zfy_arm/Zfy_Arm_real_control/py")

from motor_config import MotorSpec
from socketcan_bus import SocketCanBus

spec = MotorSpec("joint1", 1, 1, "4340", 0, "MIT")
bus = SocketCanBus([spec], channel="can0", bitrate=1000000)

for i in range(30):
    bus.mit(1, 0.0, 0.0, 0.0, 0.0, 0.0)
    time.sleep(0.05)
    if i % 10 == 9:
        print("t=%.2fs states=%s" % ((i + 1) * 0.05, bus.states()))

print("rx_hz=%.1f" % bus.rx_hz)
bus.close()
