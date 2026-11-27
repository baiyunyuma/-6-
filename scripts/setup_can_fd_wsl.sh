#!/usr/bin/env bash
# Configure DM USB2CANFD (gs_usb firmware) interfaces for CAN FD 1M/5M.
#
# Usage:
#   sudo ./setup_can_fd_wsl.sh                 # auto-detect all gs_usb can*
#   sudo ./setup_can_fd_wsl.sh can0
#   sudo ./setup_can_fd_wsl.sh can0 can1
#
# Note: the DM motor protocol frames are classic 8-byte CAN frames, but the
# bus/motor is configured for CAN FD with 5 Mbps data phase.  Configuring the
# interface with fd on + dbitrate 5000000 is what lets the motor ACK/reply.
set -euo pipefail

BITRATE=1000000
DBITRATE=5000000
SP=0.875

IFACES=("$@")
if [ ${#IFACES[@]} -eq 0 ]; then
    for node in /sys/class/net/can*; do
        [ -e "$node" ] || continue
        drv="$(basename "$(readlink -f "$node/device/driver" 2>/dev/null)" 2>/dev/null || true)"
        if [ "$drv" = "gs_usb" ]; then
            IFACES+=("$(basename "$node")")
        fi
    done
fi

if [ ${#IFACES[@]} -eq 0 ]; then
    echo "no gs_usb CAN interface found; is the module attached to WSL?" >&2
    exit 1
fi

for iface in "${IFACES[@]}"; do
    echo "configuring $iface: bitrate=$BITRATE dbitrate=$DBITRATE fd on"
    ip link set "$iface" down 2>/dev/null || true
    ip link set "$iface" type can bitrate "$BITRATE" sample-point "$SP" \
        dbitrate "$DBITRATE" fd on
    # first "up" after enumeration can time out; retry once
    ip link set "$iface" up || ip link set "$iface" up
    ip -d link show "$iface" | sed -n '1,10p'
done

echo "done: ${IFACES[*]}"
