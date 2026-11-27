path = "/home/sky/gs_usb_vendor618/gs_usb.c"
src = open(path).read()

repls = [
    ("#define GS_MAX_RX_URBS 30",
     "/* WSL USB/IP: 30 outstanding RX URBs overwhelm the DM bulk endpoint */\n#define GS_MAX_RX_URBS 4"),
    ("if (atomic_read(&dev->active_tx_urbs) >= GS_MAX_TX_URBS)",
     "if (atomic_read(&dev->active_tx_urbs) >= 2)"),
]

for old, new in repls:
    n = src.count(old)
    assert n >= 1, (old, n)
    print("%-60s -> %d" % (old[:60], n))
    src = src.replace(old, new)

open(path, "w").write(src)
print("patched RX URBs=4, TX inflight=2")
