path = "/home/sky/gs_usb_vendor618/gs_usb.c"
src = open(path).read()

old = """static void gs_usb_xmit_callback(struct urb *urb)
{
\tstruct gs_tx_context *txc = urb->context;
\tstruct gs_can *dev = txc->dev;
\tstruct net_device *netdev = dev->netdev;

\tif (urb->status)
\t\tnetdev_info(netdev, "usb xmit fail %u\\n", txc->echo_id);
}
"""
new = """static void gs_usb_xmit_callback(struct urb *urb)
{
\tstruct gs_tx_context *txc = urb->context;
\tstruct gs_can *dev = txc->dev;
\tstruct net_device *netdev = dev->netdev;

\tif (!urb->status)
\t\treturn;

\tnetdev_info(netdev, "usb xmit fail %u: %pe\\n", txc->echo_id,
\t\t    ERR_PTR(urb->status));
\tnetdev->stats.tx_dropped++;
\tnetdev->stats.tx_errors++;
\tcan_free_echo_skb(netdev, txc->echo_id, NULL);
\tgs_free_tx_context(txc);
\tatomic_dec(&dev->active_tx_urbs);
\tnetif_wake_queue(netdev);
}
"""
count = src.count(old)
assert count == 1, count
open(path, "w").write(src.replace(old, new))
print("patched vendor xmit error cleanup")
