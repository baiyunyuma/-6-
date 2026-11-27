#!/usr/bin/env bash
# Install the bundled gs_usb.ko into /lib/modules for the running kernel.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
KVER="$(uname -r)"
VM="$(modinfo -F vermagic "$ROOT/gs_usb.ko")"
DIR="/lib/modules/$KVER/kernel/drivers/net/can/usb"

echo "kernel:   $KVER"
echo "vermagic: $VM"
case "$VM" in
    "$KVER"*) ;;
    *) echo "WARNING: vermagic does not match the running kernel." >&2 ;;
esac

sudo cp "$ROOT/gs_usb.ko" "$DIR/gs_usb.ko"
sudo depmod -a
echo "installed: $DIR/gs_usb.ko"
echo "reload with: sudo rmmod gs_usb; sudo modprobe gs_usb"
