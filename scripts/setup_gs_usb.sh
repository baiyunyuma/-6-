#!/usr/bin/env bash
# 在 Jetson 上加载 gs_usb.ko 并配置所有 gs_usb CAN 接口。
#
# 用法：
#   ./setup_gs_usb.sh [ko路径] [bitrate] [接口...]
#
#   ./setup_gs_usb.sh                                  # 自动找全部 gs_usb 接口
#   ./setup_gs_usb.sh ~/gs_usb_build/gs_usb.ko 1000000 # 同上
#   ./setup_gs_usb.sh ~/gs_usb_build/gs_usb.ko 1000000 can1 can2
#
# 双路模块（USB2CANFD Dual）会得到两个接口，顺序即 channel 0/1：
#   can_channels: ["can1", "can2"]  -> J1-J3 走第一路，J4-J6 走第二路
#
# 持久化（重启后自动加载）：
#   sudo cp ~/gs_usb_build/gs_usb.ko /lib/modules/$(uname -r)/extra/
#   sudo depmod -a
#   echo gs_usb | sudo tee /etc/modules-load.d/gs_usb.conf
#
# 注意：必须先刷对应固件（双路：dm_usb2canfd_dual_gsusb_1004.enc），
# 出厂固件不会枚举成 gs_usb。
set -euo pipefail

KO="${1:-$HOME/gs_usb_build/gs_usb.ko}"
BITRATE="${2:-1000000}"
if [[ $# -ge 2 ]]; then
    shift 2
else
    shift "$#"
fi
IFACES=("$@")
if [[ ${#IFACES[@]} -eq 1 && "${IFACES[0]}" == "auto" ]]; then
    IFACES=()
fi

if [[ ! -f "$KO" ]]; then
    echo "找不到 $KO；先在 Jetson 上编译 gs_usb.ko" >&2
    exit 1
fi

if ! lsmod | grep -q '^gs_usb'; then
    echo "加载 $KO"
    sudo insmod "$KO"
else
    echo "gs_usb 已加载"
fi

if [[ ${#IFACES[@]} -eq 0 ]]; then
    for node in /sys/class/net/can*; do
        [[ -e "$node" ]] || continue
        drv="$(basename "$(readlink -f "$node/device/driver" 2>/dev/null)" 2>/dev/null || true)"
        if [[ "$drv" == "gs_usb" ]]; then
            IFACES+=("$(basename "$node")")
        fi
    done
fi

if [[ ${#IFACES[@]} -eq 0 ]]; then
    echo "没有找到 driver=gs_usb 的 CAN 接口。检查：" >&2
    echo "  1) lsusb 是否出现 gs_usb 固件的设备" >&2
    echo "  2) dmesg | tail  是否有 gs_usb 绑定的日志" >&2
    ip -d link show type can 2>/dev/null || true
    exit 1
fi

for iface in "${IFACES[@]}"; do
    echo "配置 $iface @ ${BITRATE} bit/s（经典 CAN，fd off）"
    # 说明书要求先 down 再改 type/bitrate
    sudo ip link set "$iface" down 2>/dev/null || true
    sudo ip link set "$iface" type can bitrate "$BITRATE" sample-point 0.75
    sudo ip link set "$iface" up
    ip -d link show "$iface" | head -4
done

echo
echo "检测到 ${#IFACES[@]} 路：${IFACES[*]}"
printf 'motors_socketcan_dual.yaml 建议：\n  can_channels: ['
for i in "${!IFACES[@]}"; do
    [[ $i -gt 0 ]] && printf ', '
    printf '"%s"' "${IFACES[$i]}"
done
printf ']\n'
echo "验证反馈:  candump ${IFACES[0]}   /   candump ${IFACES[1]:-${IFACES[0]}}"
