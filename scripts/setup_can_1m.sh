#!/usr/bin/env bash
# Configure the DM USB2CANFD gs_usb interface for classic CAN 1 Mbps.
# Usage: sudo ./setup_can_1m.sh [can0]
set -euo pipefail
IFACE="${1:-can0}"

ip link set "$IFACE" down 2>/dev/null || true
ip link set "$IFACE" type can bitrate 1000000 sample-point 0.75 fd off loopback off
# The first "up" after enumeration can time out on this firmware; retry once.
ip link set "$IFACE" up || ip link set "$IFACE" up
ip -d link show "$IFACE" | sed -n '1,10p'
