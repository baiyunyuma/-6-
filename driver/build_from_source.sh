#!/usr/bin/env bash
# Rebuild gs_usb.ko against a prepared WSL kernel source tree.
# The kernel tree must match `uname -r` and have been prepared with:
#   zcat /proc/config.gz > .config && make olddefconfig && make modules_prepare
# See README section "重新编译驱动".
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
KDIR="${KDIR:-/root/WSL2-Linux-Kernel-linux-msft-wsl-6.18.54.1}"

if [ ! -d "$KDIR" ]; then
    echo "kernel source not found: $KDIR" >&2
    echo "set KDIR=/path/to/WSL2-Linux-Kernel-linux-msft-wsl-<version>" >&2
    exit 1
fi

make -C "$KDIR" M="$ROOT" KBUILD_MODPOST_WARN=1 modules
echo "built: $ROOT/gs_usb.ko"
