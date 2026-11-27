# zfy_arm_socketcan_ctrl

ZFY 六轴机械臂 **单路 DM-USB2CANFD + Linux SocketCAN** 的可复现工作版。

本目录整理自 2026-11-16 在 WSL2 上完整跑通的版本：6 轴连接探测全部应答、`/joint_states` 实时反馈、250 Hz 连续运行 0 丢帧。

---

## 1. 已验证的环境和结果

| 项 | 值 |
|---|---|
| 模块 | 达妙 USB2CANFD **单路**，`VID:PID = 1d50:606f`，gs_usb 固件 |
| 总线 | 经典 CAN，**1 Mbps**，sample-point 0.75，60 Ω 终端 |
| 机械臂 | ZFY 六轴，DM4340 ×3 + DM4310 ×3，CAN ID 1..6，MIT 模式 |
| 操作系统 | WSL2 Ubuntu 22.04，内核 `6.18.54.1-microsoft-standard-WSL2` |
| USB 转发 | usbipd-win 5.3 + `vhci_hcd`（USB over IP） |
| ROS | ROS 2 Humble |
| 驱动 | 厂商 gs_usb 6.8 源码移植到 6.18，并做 USB/IP 并发修正 |

实测结果：

```text
连接探测: joint1..joint6 = 应答(mode=1)
使能前状态: 6 轴全部 err=0，反馈年龄 0.10 s
250 Hz 连续 5 s: sent=1250, tx_delta=1250, rx_delta=2500, dropped=0, errors=0
/joint_states: position/velocity/effort 实时更新
```

---

## 2. 目录结构

```text
zfy_arm_socketcan_ctrl/
├── README.md
├── config/
│   └── motors_socketcan_wsl.yaml      # 工作配置：can0 / 1M / 6 轴
├── driver/
│   ├── gs_usb.c                       # 厂商 6.8 移植 + 并发修正（工作版源码）
│   ├── gs_usb.ko                      # 为 6.18.54.1 预编译
│   ├── Makefile
│   ├── install_ko.sh                  # 安装预编译 ko 到 /lib/modules
│   ├── build_from_source.sh           # 用 WSL 内核源码重新编译
│   └── patches/                       # 记录四步修改的脚本（参考）
├── py/
│   ├── socketcan_bus.py               # SocketCAN 后端（经典 8 字节帧）
│   ├── motor_config.py                # 型号 → MIT 量程
│   ├── core.py / telemetry.py / ...   # 单电机 Qt HMI 相关
│   └── run_hmi.sh
├── ros2/
│   ├── zfy_arm_interfaces/            # ROS 2 msg/srv
│   ├── zfy_arm_sim2real/              # motor_driver、launch、config
│   └── hmi/arm_hmi.py + arm_hmi.sh    # 整臂 HMI
├── scripts/
│   ├── setup_can_1m.sh                # 配置 can0 经典 1M
│   ├── build_ros2.sh                  # colcon 编译两个包
│   ├── start_driver.sh                # 启动 motor_driver
│   ├── start_hmi.sh                   # 启动整臂 HMI
│   └── attach_usbipd.ps1              # Windows 侧 attach 辅助
├── tests/
│   ├── test_backend.py                # 20 Hz 反馈测试
│   ├── test_backend_5s.py             # 250 Hz / 5 s 压测
│   ├── slow_mit.py                    # 单帧/慢速 MIT
│   └── disabled_drag_test.py          # 失能拖动反馈测试
└── docs/
    └── ZFY_SocketCAN实现报告.md        # Jetson 参考报告（上游 vs 厂商驱动）
```

> `py/` 里**不包含**厂商 USB SDK（`dm_backend/`）。SocketCAN 后端不需要它；
> `py/hmi.py --backend socketcan` 可以单独使用，`vendor` 后端不可用。

---

## 3. 关键问题与修复（重要）

### 3.1 现象

上游/微软内置 `gs_usb` 或厂商默认驱动，在 WSL 的 USB/IP 下：

```text
can0 能 UP、bitrate 正确、CAN state ERROR-ACTIVE
cansend 返回 0，但 tx_packets 停在 1~2
qdisc: dropped 100+，backlog 11
控制器 CAN 错误计数全 0
```

单帧有时能收到电机反馈，连续几帧后模块的 bulk OUT 端点停摆，必须 USB 重新枚举/拔插才能恢复。

### 3.2 根因

厂商 gs_usb 6.8 驱动默认：

```c
#define GS_MAX_RX_URBS 30          // 同时 30 个 RX URB
GS_MAX_TX_URBS = 10                // 最多 10 个在途 TX
```

在 **WSL USB/IP** 下，这个并发量会压垮达妙模块的批量端点。Jetson 原生 USB 没有这个限制，所以报告里同一驱动可以 6 轴 250 Hz。

### 3.3 修复内容（`driver/gs_usb.c` 已包含）

1. **厂商 6.8 → 6.18 API 移植**
   - `can.data_bittiming*` → `can.fd.data_bittiming*`
   - `struct ethtool_ts_info *` → `struct kernel_ethtool_ts_info *`
   - cyclecounter `read` 回调去掉 `const`
2. **对 `1d50:606f` 关闭 HW_TIMESTAMP**
   - 该固件声明支持，但时间戳控制请求在 USB/IP 下返回 `-121`，会扰动端点
3. **TX 错误路径释放 context**
   - 厂商 6.8 的错误回调只打印日志，不释放 context；补上
     `can_free_echo_skb` + `gs_free_tx_context` + `atomic_dec` + `netif_wake_queue`
4. **降低 USB 并发（关键）**
   - `GS_MAX_RX_URBS: 30 → 4`
   - TX 在途上限：`10 → 2`

修复后 20 Hz / 250 Hz 连续运行均 0 丢帧。

### 3.4 通信层修复（Python / SocketCAN）

1. **发送改为非阻塞**：`send(MSG_DONTWAIT)`。发送队列满时立即丢帧并累计
   `tx_dropped`，驱动每 2 s 最多告警一次，避免 250 Hz 控制环被 `send` 卡住
   （原先 socket 有 0.2 s 超时，6 帧最坏可卡 1.2 s）。
2. **参数操作必须收到电机应答**：`set_mode` / `write_param` / `save` 只有在
   收到 0x55 / 0xAA 应答后才报成功；否则返回失败，且 `set_mode` 不改本地模式。
   避免“界面说切换成功、实际还在 MIT、后续控制帧发到错误偏移”。
   `write_param` 还会回读并比对，回读超时/不一致均报失败。
3. **控制 ID / 反馈 ID 分离**：`can_ids` = 控制 ID（ESC_ID / RID 8），
   `mst_ids` = 反馈 ID（MST_ID / RID 7），两者可不同；参数回复两种 ID 都接受。

---

## 4. 快速开始

以下命令都在 WSL 中执行，`ROOT` 为本目录：

```bash
cd /home/sky/zfy_arm_socketcan_ctrl_test
export ROOT="$PWD"
```

### 4.1 安装驱动

预编译版（内核必须正好是 `6.18.54.1-microsoft-standard-WSL2`）：

```bash
./driver/install_ko.sh
sudo rmmod gs_usb 2>/dev/null || true
sudo modprobe gs_usb
lsmod | grep gs_usb
```

看到 `gs_usb` 加载即可。首次加载可能出现：

```text
gs_usb: loading out-of-tree module taints kernel
gs_usb: no symbol version for module_layout
```

这是正常的（强制/无 CRC 加载），不影响使用。

### 4.2 把模块 attach 到 WSL

Windows PowerShell：

```powershell
usbipd list
# 找到 1d50:606f 的 BUSID，例如 5-2
# 若 STATE = Not shared（需要管理员）：
usbipd bind --busid 5-2
usbipd attach --wsl --busid 5-2
```

或使用：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\attach_usbipd.ps1 -BusId 5-2
```

> BUSID 会随 USB 口/拔插变化。当前这台机器最后一次是 **`5-2`**。

WSL 侧确认：

```bash
lsusb -d 1d50:606f
ip -br link | grep can      # 应出现 can0
```

### 4.3 配置 can0 经典 1M

```bash
sudo ./scripts/setup_can_1m.sh can0
```

预期：

```text
can0: <NOARP,UP,LOWER_UP,ECHO> ... can state ERROR-ACTIVE
bitrate 1000000 sample-point 0.750
```

> 首次 `ip link set up` 偶尔会 `Timer expired`，脚本会自动重试一次。

### 4.4 编译 ROS 2 包

```bash
./scripts/build_ros2.sh
source ros2/install/setup.bash
```

### 4.5 启动驱动和整臂 HMI

终端 1：

```bash
./scripts/start_driver.sh
```

预期：

```text
CAN 通道 0 (can0): operstate=up driver=gs_usb bitrate=1000000
关节 (控制ID/反馈ID): joint1=0x11/0x01 ... joint6=0x16/0x06
连接探测: joint1..joint6 = 应答(mode=1)
```

> **电机 ID**：控制 ID = ESC_ID（RID 8）= `can_ids`，反馈 ID = MST_ID（RID 7）
> = `mst_ids`。当前配置为控制 `0x11..0x16`、反馈 `0x01..0x06`；夹爪 J7 为
> 控制 `0x17` / 反馈 `0x07`。二者必须与电机内部保存的 ESC_ID/MST_ID 一致。

终端 2：

```bash
./scripts/start_hmi.sh
```

HMI 通过 WSLg 显示。界面里点「连接」即可。

---

## 5. 测试

> 直接跑 `tests/` 前，先停掉 `motor_driver`（它会占用 can0），并确保 can0 已配置 1M。

```bash
sudo python3 tests/test_backend_5s.py
```

预期：

```text
sent=1250 in ~5.1s (243 Hz)
states={1: {'q': ..., 'dq': ..., 'tau': ..., 'err': 0}}
rx_hz=~243
tx_delta=1250 rx_delta=2500 dropped=0 errors=0
```

单帧/失能反馈：

```bash
sudo python3 tests/slow_mit.py
```

---

## 6. 失能拖动 / 拖动示教

1. 驱动启动、6 轴反馈正常；
2. HMI 里点 **「开始发送」**（或调用 `/motor_driver/sending`）；
3. 点 **「拖动示教」**；
4. 手动拖动机械臂，`/joint_states` 会实时跟随；
5. 安全：手臂扶稳、急停可触达；使能前电机不出力，使能后可能保持/运动。

---

## 7. 重启 / 持久化

- 驱动已经安装在 `/lib/modules/$(uname -r)/kernel/drivers/net/can/usb/gs_usb.ko`，
  WSL 重启后 `modprobe gs_usb` 会自动加载它；
- 但 USB 设备需要重新 attach；BUSID 可能变化；
- 每次 WSL 重启后：

```bash
# Windows
usbipd attach --wsl --busid <当前 BUSID>
# WSL
sudo ./scripts/setup_can_1m.sh can0
./scripts/start_driver.sh
./scripts/start_hmi.sh
```

- WSL 没有常驻进程时会自动关闭，USB 设备会 detach；开发时建议保持一个终端打开。

---

## 8. 重新编译驱动

如果 WSL 内核升级，预编译 `.ko` 不能再用，需要重新编译。

### 8.1 准备内核源码

```bash
sudo apt install -y build-essential bison flex bc libssl-dev libelf-dev dwarves
cd /root
curl -L -o wsl-kernel.tar.gz \
  "https://codeload.github.com/microsoft/WSL2-Linux-Kernel/tar.gz/refs/tags/linux-msft-wsl-6.18.54.1"
tar xzf wsl-kernel.tar.gz
cd WSL2-Linux-Kernel-linux-msft-wsl-6.18.54.1
zcat /proc/config.gz > .config
./scripts/config --enable DEBUG_INFO_BTF --enable DEBUG_INFO_BTF_MODULES \
                 --enable MODULE_ALLOW_BTF_MISMATCH
make olddefconfig
make modules_prepare -j$(nproc)
```

> `DEBUG_INFO_BTF_MODULES` 必须开启（需要 pahole/dwarves）。
> 否则 `struct module` 大小与运行内核不一致，`insmod` 会报
> `section size must match the kernel's built struct module size`。

### 8.2 编译并安装

```bash
cd /home/sky/zfy_arm_socketcan_ctrl_test
KDIR=/root/WSL2-Linux-Kernel-linux-msft-wsl-6.18.54.1 ./driver/build_from_source.sh
./driver/install_ko.sh
sudo rmmod gs_usb; sudo modprobe gs_usb
```

---

## 9. 常见问题

| 现象 | 处理 |
|---|---|
| `ip link set can0 up` 报 `Timer expired` | 再执行一次 `up`；脚本已自动重试 |
| `tx_packets=0` / 连续发送停摆 | 确认用的是本目录的 `gs_usb.ko`，不是内置/上游驱动 |
| `gs_usb probe failed -62/-110` | 模块控制端点卡死，USB 重新枚举或拔插一次 |
| `usbipd attach` 找不到 BUSID | `usbipd list` 确认 BUSID；换 USB 口后编号会变 |
| Windows 完全看不到模块 | 换**能传数据**的 USB 线、换机身后置口，不要经过 Hub |
| 驱动加载 `Invalid module format` | 内核版本不匹配；按第 8 节重编 |
| `section size must match ...` | 内核源码缺少 `DEBUG_INFO_BTF_MODULES`；按 8.1 重编 |
| HMI 打不开 | 需要 WSLg；确认 `echo $DISPLAY` 和 `xdpyinfo` |
| 电机无反馈 | 确认 can0 已 UP、6 轴供电、60 Ω、CAN_H/L 未接反 |

---

## 10. 参考

- `docs/ZFY_SocketCAN实现报告.md`：Jetson 原生 USB 的完整验证报告，
  其中明确记录了“上游 gs_usb 不行、厂商驱动才行”的结论。
- 厂商固件/驱动资料：`C:\Users\sky\Desktop\dm-tools-master\...`
  - `gs_usb_drives/gs_usb_sppe_Orin_NX/gs_usb_6.8/`（本驱动来源）
  - `USB2CANFD/固件/socketcan/gsusb免驱固件/dm_usb2canfd_gsusb_1003.enc`
