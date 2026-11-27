import os

src_path = "/mnt/c/Users/sky/Desktop/dm-tools-master/dm-tools-master/gs_usb_drives/gs_usb_sppe_Orin_NX/gs_usb_6.8/gs_usb.c"
out_dir = "/home/sky/gs_usb_vendor618"
os.makedirs(out_dir, exist_ok=True)
src = open(src_path).read()

repls = [
    ("static u64 gs_usb_timestamp_read(const struct cyclecounter *cc)",
     "static u64 gs_usb_timestamp_read(struct cyclecounter *cc)"),
    ("dev->can.data_bittiming", "dev->can.fd.data_bittiming"),
    ("dev->can.do_set_data_bittiming", "dev->can.fd.do_set_data_bittiming"),
    ("struct ethtool_ts_info *info", "struct kernel_ethtool_ts_info *info"),
]

for old, new in repls:
    n = src.count(old)
    print("%-52s -> %d" % (old[:52], n))
    src = src.replace(old, new)

out_path = os.path.join(out_dir, "gs_usb.c")
open(out_path, "w").write(src)
print("wrote", out_path)
