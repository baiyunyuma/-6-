#!/usr/bin/env bash
# ZFY 电机上位机启动脚本
#
# 必须在启动前设 LD_LIBRARY_PATH：libdm_device.so 需要 GLIBCXX_3.4.32 和
# libusb 1.0.27+，而 Ubuntu 22.04 自带的是 3.4.30 / 1.0.25。
# numpy / PyQt5 会先加载系统 libstdc++，之后再"预加载"新版是无效的
# （SONAME libstdc++.so.6 已被占用），所以只能靠 LD_LIBRARY_PATH。
set -e
cd "$(dirname "$0")"
export LD_LIBRARY_PATH="$(pwd)/dm_backend/lib:${LD_LIBRARY_PATH:-}"
exec python3 hmi.py "$@"
