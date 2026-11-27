import socket, struct, time, os

IFACE = "can0"
CAN_RAW = 1
FRAME = struct.Struct("=IB3x8s")
DATA = bytes.fromhex("7FFF7FF0000007FF")  # MIT zero torque, q=0, dq=0


def stat(name):
    with open("/sys/class/net/%s/statistics/%s" % (IFACE, name)) as fh:
        return fh.read().strip()


s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, CAN_RAW)
s.bind((IFACE,))
s.settimeout(0.05)

print("start tx=%s rx=%s" % (stat("tx_packets"), stat("rx_packets")))
for i in range(5):
    s.send(FRAME.pack(0x001, 8, DATA))
    print("sent #%d  tx=%s rx=%s" % (i + 1, stat("tx_packets"), stat("rx_packets")))
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.25:
        try:
            buf = s.recv(16)
        except socket.timeout:
            continue
        cid, dlc, payload = FRAME.unpack(buf)
        print("   rx id=0x%03X dlc=%d data=%s" % (cid, dlc, payload[:dlc].hex()))
    time.sleep(0.05)

print("end tx=%s rx=%s" % (stat("tx_packets"), stat("rx_packets")))
