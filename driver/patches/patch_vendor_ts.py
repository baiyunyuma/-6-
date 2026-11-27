path = "/home/sky/gs_usb_vendor618/gs_usb.c"
src = open(path).read()

old = "\tdev->feature = FIELD_GET(GS_CAN_FEATURE_MASK, feature);\n"
new = old + """
\t/* DM USB2CANFD: the firmware advertises HW timestamps but the
\t * timestamp control request fails under WSL USB/IP (-121), which
\t * destabilises the bulk endpoint.  Disable the feature for this
\t * vendor so no timestamp mode/control traffic is generated.
\t */
\tif (le16_to_cpu(interface_to_usbdev(intf)->descriptor.idVendor) == 0x1d50 &&
\t    le16_to_cpu(interface_to_usbdev(intf)->descriptor.idProduct) == 0x606f)
\t\tdev->feature &= ~GS_CAN_FEATURE_HW_TIMESTAMP;
"""
count = src.count(old)
assert count == 1, count
open(path, "w").write(src.replace(old, new))
print("patched HW_TIMESTAMP off for 1d50:606f")
