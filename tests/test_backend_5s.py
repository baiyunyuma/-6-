import sys
import time

sys.path.insert(0, "/home/sky/zfy_arm/Zfy_Arm_real_control/py")

from motor_config import MotorSpec
from socketcan_bus import SocketCanBus

IFACE = "can0"


def stat(name):
    with open("/sys/class/net/%s/statistics/%s" % (IFACE, name)) as fh:
        return int(fh.read().strip())


spec = MotorSpec("joint1", 1, 1, "4340", 0, "MIT")
bus = SocketCanBus([spec], channel=IFACE, bitrate=1000000)

tx0, rx0 = stat("tx_packets"), stat("rx_packets")
t0 = time.monotonic()
n = 1250
for i in range(n):
    bus.mit(1, 0.0, 0.0, 0.0, 0.0, 0.0)
    time.sleep(0.004)
dt = time.monotonic() - t0
time.sleep(0.3)

print("sent=%d in %.3fs (%.1f Hz)" % (n, dt, n / dt))
print("states=%s" % bus.states())
print("rx_hz=%.1f last_error=%s" % (bus.rx_hz, bus.last_error))
print("tx_delta=%d rx_delta=%d dropped=%d errors=%d" % (
    stat("tx_packets") - tx0,
    stat("rx_packets") - rx0,
    stat("tx_dropped"),
    stat("tx_errors"),
))
bus.close()
