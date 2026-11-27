# ZFY 六轴机械臂 SocketCAN 通信实现报告

- 日期：2026-11-05
- 平台：NVIDIA Jetson，JetPack 6.2 / Ubuntu 22.04.5 / 内核 5.15.148-tegra / aarch64
- 硬件：达妙 USB2CANFD（单路，VID:PID = 1d50:606f）+ ZFY 六轴臂（DM4340 / DM4310）
- 软件：ROS 2 Humble、Python 3.10、厂商 gs_usb 5.15 驱动

---

## 1. 目标与结论

目标：让 Jetson 通过 Linux SocketCAN 直接控制 ZFY 六轴臂的达妙电机，替代只能在 x86 上运行的厂商 USB SDK，并保持 MIT 控制、重力/摩擦前馈、ROS 2 和 MoveIt 接口不变。

最终结果：

- `can1` 正常 UP，1 Mbps 经典 CAN，驱动为 `gs_usb`；
- 6 个电机全部应答，模式均为 MIT，ERR=0；
- `motor_driver` 以实机 250 Hz 运行；
- HMI、MoveIt 轨迹桥均可通过同一底层控制；
- 未使能时只读探测，不输出力矩。

---

## 2. 系统组成

```text
上位机 HMI / MoveIt / 脚本
        │  ROS 2 话题、服务、Action
        ▼
motor_driver（ROS 2 节点，250 或 1000 Hz）
        │  MIT 目标、重力/摩擦前馈、使能/失能
        ▼
SocketCanBus（Python，AF_CAN / SOCK_RAW / CAN_RAW）
        │  8 字节经典 CAN 帧
        ▼
gs_usb 内核驱动
        │  USB
        ▼
达妙 USB2CANFD
        │  CAN_H / CAN_L、120 Ω
        ▼
DM4340 / DM4310 电机
```

关键文件：

| 文件 | 作用 |
|---|---|
| `Zfy_Arm_real_control/py/socketcan_bus.py` | SocketCAN 收发、MIT 编码/解码、参数读写、双路路由 |
| `Zfy_Arm_real_control/zfy_arm_sim2real/zfy_arm_sim2real/motor_driver.py` | ROS 2 驱动、使能/发送、MIT 伺服、重力/摩擦前馈 |
| `zfy_arm_sim2real/config/motors_socketcan.yaml` | 单路实机配置 |
| `zfy_arm_sim2real/config/motors_socketcan_dual.yaml` | 双路实机配置 |
| `zfy_arm_sim2real/traj_bridge.py` | MoveIt FollowJointTrajectory → 实机控制 |
| `tools/setup_gs_usb.sh` | 加载 gs_usb、配置所有 gs_usb CAN 接口 |

---

## 3. 为什么选 SocketCAN

厂商 `libdm_device.so` / 捆绑 libstdc++ / libusb 都是 x86-64；Jetson 是 aarch64，无法加载。达妙资料里的 aarch64 库是旧版 v1.0.0，缺少当前 dmcan 需要的符号，不可用。

SocketCAN 后端是纯 Python 标准库实现，不依赖厂商库，因此适合 Jetson。MIT 帧本身就是 8 字节经典 CAN 帧，SocketCAN 完全能承载。

---

## 4. 实现步骤

### 4.1 环境确认

```bash
uname -a
# Linux nvidia-desktop 5.15.148-tegra ... aarch64
cat /etc/nv_tegra_release
# R36 (release), REVISION: 4.3  -> JetPack 6.2
source /opt/ros/humble/setup.bash
ros2 pkg list | grep -E 'moveit|control_msgs'
```

内核配置：

```text
CONFIG_CAN=m
CONFIG_CAN_RAW=m
CONFIG_CAN_DEV=m
CONFIG_CAN_GS_USB 未启用（需要自己编译）
```

Jetson 板载 `can0` 是 `mttcan`，不是 USB 模块；USB 模块通常出现在 `can1`。

### 4.2 内核驱动：最容易踩的坑

先用上游 Linux v5.15 的 `gs_usb.c` 编译了模块：

```bash
mkdir -p ~/gs_usb_build
curl -fsSL -o gs_usb.c \
  https://raw.githubusercontent.com/torvalds/linux/v5.15/drivers/net/can/usb/gs_usb.c
make -C /lib/modules/$(uname -r)/build M=$PWD modules
```

模块能加载，接口能 UP，但发送不出去：

```text
cansend rc=0
tx_packets=0
TX errors=0
CAN state=ERROR-ACTIVE
```

即使开启 loopback，`candump` 也收不到帧，`tx_packets` 仍为 0。说明不是总线没有 ACK，而是模块/USB 发送路径没有工作。

后来在达妙资料中找到了 **Jetson Orin 专用的 gs_usb 驱动**：

```text
dm-tools-master/gs_usb_drives/gs_usb_sppe_Orin_NX/gs_usb_5.15/
```

其 `gs_usb.ko` 的 vermagic 正好是：

```text
5.15.148-tegra SMP preempt mod_unload modversions aarch64
```

源码包含上游 5.15 没有的 CAN FD data bittiming 支持：

```text
GS_USB_BREQ_DATA_BITTIMING
GS_USB_BREQ_BT_CONST_EXT
CAN_CTRLMODE_FD
```

说明之前的问题是：**驱动版本/厂商补丁不匹配**，不是接线。

正确的做法是使用厂商源码，并在当前内核上重新编译：

```bash
cd ~/gs_usb_vendor/gs_usb_5.15
make -j6
cp gs_usb.ko ~/gs_usb_build/gs_usb.ko
```

### 4.3 加载模块和配置接口

```bash
# 先 down，再卸载旧模块
sudo ip link set can1 down 2>/dev/null || true
sudo rmmod gs_usb 2>/dev/null || true

# 加载厂商模块
sudo insmod ~/gs_usb_build/gs_usb.ko

# 配置经典 CAN 1 Mbps
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 sample-point 0.75
sudo ip link set can1 up

# 确认
ip -d link show can1
```

预期：

```text
can1: <NOARP,UP,LOWER_UP,ECHO>
can state ERROR-ACTIVE
bitrate 1000000 sample-point 0.750
driver=gs_usb
dtseg1/dtseg2/dbrp ...   ← 厂商驱动支持 FD data timing
```

注意：Jetson 上不能把这个模块绑到 `can0`，那是 `mttcan`。驱动会校验 `/sys/class/net/<iface>/device/driver`，不是 `gs_usb` 会直接报错。

也可以用现成脚本：

```bash
cd ~/zfy_arm/Zfy_Arm_real_control
./tools/setup_gs_usb.sh ~/gs_usb_build/gs_usb.ko 1000000
```

脚本会：

1. `insmod gs_usb.ko`；
2. 找到所有 `driver=gs_usb` 的接口；
3. 先 down，再配置 `bitrate 1000000 sample-point 0.75`；
4. up，并打印建议的 `can_channels`。

### 4.4 Python SocketCAN 后端

核心文件：`py/socketcan_bus.py`。

创建 socket：

```python
sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
sock.bind((channel_name,))
sock.settimeout(0.2)
```

帧结构（Linux `struct can_frame` 为 16 字节）：

```python
_CAN_FRAME = struct.Struct("=IB3x8s")
# 4 字节 can_id + 1 字节 dlc + 3 字节 pad + 8 字节 data
```

发送：

```python
buf = _CAN_FRAME.pack(int(can_id) & 0x7FF, n, data.ljust(8, b"\0"))
sock.send(buf)
```

接收线程：

```python
raw_id, dlc, payload = _CAN_FRAME.unpack(sock.recv(16))
```

MIT 编码与厂商 SDK 逐字节一致：

```text
q_uint  = float_to_uint(q,  -q_max,  q_max,  16)
dq_uint = float_to_uint(dq, -dq_max, dq_max, 12)
kp_uint = float_to_uint(kp, 0,       500,    12)
kd_uint = float_to_uint(kd, 0,       5,      12)
tau_uint= float_to_uint(tau,-tau_max, tau_max,12)
```

字节顺序：

```text
data[0] = q_uint >> 8
data[1] = q_uint & 0xFF
data[2] = dq_uint >> 4
data[3] = ((dq_uint & 0xF) << 4) | (kp_uint >> 8)
data[4] = kp_uint & 0xFF
data[5] = kd_uint >> 4
data[6] = ((kd_uint & 0xF) << 4) | (tau_uint >> 8)
data[7] = tau_uint & 0xFF
```

支持的帧：

| 功能 | CAN ID | 内容 |
|---|---|---|
| MIT | `can_id + 0x000` | 8 字节 MIT 目标 |
| POS_VEL | `can_id + 0x100` | `float pos, float vel` |
| VEL | `can_id + 0x200` | `float vel` |
| POS_FORCE | `can_id + 0x300` | `float pos, float tau` |
| 使能 | `can_id + 模式偏移` | `FF FF FF FF FF FF FF FC` |
| 失能 | 同上 | `... FD` |
| 零位 | 同上 | `... FE` |
| 参数读 | `0x7FF` | `[id_lo,id_hi,0x33,rid,0,0,0,0]` |
| 参数写 | `0x7FF` | `[id_lo,id_hi,0x55,rid,data4]` |
| 保存 | `0x7FF` | `[id_lo,id_hi,0xAA,0x01,0,0,0,0]` |
| 状态刷新 | `0x7FF` | `[id_lo,id_hi,0xCC,0x00]` |

一个重要的细节：MIT 反馈帧 byte0 的低半字节是电机 ID，byte1/2 是 q；当 q 接近 `-q_max` 时，反馈帧看起来可能像 0x33/0x55/0xAA 参数回复。早期实现会把它误判为参数帧并丢弃。修复方法是：只有当前确实有参数交换（或交换后的短排空窗口）时才按参数帧处理。

多通道支持：

```python
SocketCanBus(specs, channels=["can1", "can2"])
```

`MotorSpec.channel` 为 0 的电机走 `channels[0]`，为 1 的走 `channels[1]`。每个通道一个 socket、一个接收线程；发送、反馈、参数回复都按 `(channel, can_id)` 路由，避免串路。

### 4.5 ROS 2 驱动

`motor_driver` 负责：

1. 打开 SocketCanBus；
2. 启动时读取每个电机的 CTRL_MODE 做连接探测；
3. 提供使能/发送/失能服务；
4. 以 `control_rate`（250 或 1000 Hz）发送 MIT；
5. 发布 `/joint_states`；
6. 计算重力/摩擦前馈；
7. 处理超时、反馈丢失、自动失能；
8. 可选 CSV 记录和 MoveIt 轨迹桥。

关节映射：

```text
q_urdf   = sign × (q_motor − offset) / scale
dq_urdf  = sign × dq_motor / scale
tau_urdf = sign × tau_motor × scale

q_motor   = sign × scale × q_urdf + offset
dq_motor  = sign × scale × dq_urdf
tau_motor = sign × tau_urdf / scale
```

当前配置：

```yaml
joint_signs:  [1.0, -1.0, 1.0, -1.0, -1.0, -1.0]
joint_offsets: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
joint_scales:  [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
```

注意：`joint_signs` 是通过“失能手拖 + 3D 跟随”实测确定的；当前 `joint_offsets` 还未做 fold 零位标定，开重力前应先补上。

重力前馈来源：

```text
q_measured（编码器反馈）
  → MuJoCo mj_rne(q, qvel=0, qacc=0)
  → 关节重力矩
  → sign / scale
  → MIT 的 tau_ff
```

只有反馈新鲜（<0.5 s）时才用实测角；否则退回指令角。摩擦使用实测 dq 的库仑 + 粘性模型。

### 4.6 上位机与 MoveIt

- HMI：`ros2_ws/tools/arm_hmi.py`，支持 SocketCAN 单路/双路预设，失能手拖、3D 跟随、单轴 MIT、6 轴轨迹、重力/摩擦、拖动示教。
- MoveIt：`traj_bridge.py` 作为 `/zfy_arm_controller/follow_joint_trajectory` 的实机 action server，把轨迹转换成 `/arm_joint_goal` 的 MOVE/APPEND，复用驱动里的 MIT 伺服和重力/摩擦。
- 注意：不能同时运行 `zfy_mujoco_driver` 和实机 `traj_bridge`，两者会抢 action 和 `/joint_states`。

---

## 5. 实测结果

### 5.1 vcan 单元/集成测试

- MIT 编码与厂商实现逐字节一致；
- 反馈解码、参数帧误判回归、RID 10 模式同步；
- 使能/失能、参数读写应答、POS_VEL 偏移；
- 双路路由：J1-J3 只出现在 vcan0，J4-J6 只出现在 vcan1，不串路；
- 共 **12 项全部通过**。

### 5.2 实机

解决驱动问题后：

```text
can1: UP, 1 Mbps, driver=gs_usb
tx_packets 增加（总线有 ACK）
连接探测: joint1..joint6 全部应答(mode=1)
状态: 全部 ERR=0、失能、反馈 0.10 s
```

当前初始位置（示例）：

```text
joint1=-0.001  joint2=0.004  joint3=0.002
joint4=-0.001  joint5=0.001  joint6=0.168
```

### 5.3 MoveIt

`traj_bridge` 与 MoveIt 的 `sync` / `through` 模式已在仿真和实机链路上验证；实机动作测试需要先使能、发送，并保持重力关闭/低速。

---

## 6. 注意事项与常见坑

### 6.1 接口和驱动

- Jetson 的 `can0` 是板载 `mttcan`，**不要**把驱动绑到它；
- 用 `/sys/class/net/<iface>/device/driver` 检查是不是 `gs_usb`；
- 改 bitrate 前必须先 `ip link set <iface> down`；
- 上游 5.15 `gs_usb` 缺少厂商固件需要的 TX/FD 补丁，建议直接用达妙 Orin NX 厂商驱动；
- 厂商 `.ko` 必须与当前内核匹配，最好在当前内核上重新编译；
- 未签名模块会出现 `module verification failed ... tainting kernel`，不影响使用；
- **重启后模块不会自动加载**，需要重新 `insmod` + 配置，建议后续加 systemd/udev。

### 6.2 总线物理层

- `cansend` 返回 0 **不代表**总线有节点 ACK；要看 `tx_packets` 是否增加；
- `tx_packets=0` 通常意味着没有 ACK（没供电、CAN_H/L 反、没有终端电阻、波特率不一致）或驱动发送路径问题；
- 两端 120 Ω、模块终端电阻开关、电机 24V、急停都要检查；
- 建议先用 loopback 区分“模块自身发送”与“外部总线”：
  ```bash
  sudo ip link set can1 down
  sudo ip link set can1 type can bitrate 1000000 loopback on
  sudo ip link set can1 up
  candump can1        # 另一个终端
  cansend can1 123#1122334455667788
  ```
  如果 loopback 都收不到且 `tx_packets=0`，不要怀疑接线，先查驱动/固件。

### 6.3 CAN 帧与 MIT

- MIT 是 8 字节经典 CAN 帧；`q/dq/kp/kd/tau` 的位序必须与厂商一致；
- `q_max`、`dq_max`、`tau_max` 由型号决定（4340 / 4310 不同），填错会导致角度/力矩比例错；
- 参数回复和反馈帧可能长得像，必须用“是否有参数交换”门控；
- 反馈率在 MIT 发送时等于命令率；不发送时用 0xCC 轮询，约 10 Hz。

### 6.4 控制频率与总线负载

- 单路 6 电机 250 Hz：约 3000 帧/s，1 Mbps 负载约 33–39%；
- 单路 6 电机 500 Hz：约 67–78%，开始吃紧；
- 单路 6 电机 1 kHz：约 12000 帧/s，**不可能**；
- 双路 3+3 1 kHz：每路约 6000 帧/s，约 67–78%，可行但余量小；
- Jetson 仿真实测 1 kHz 时 `motor_driver` 单核 CPU 约 93%，实机 CAN RX 只会更高，建议实机先 500 Hz；
- 要更稳的 1 kHz，需要 CAN FD 5 Mbps，并且 SocketCAN 后端要支持 `canfd_frame` + BRS，当前只实现经典帧。

### 6.5 关节映射和重力

- `joint_signs` 错会导致方向和重力前馈反；
- `joint_offsets` 未标定会导致重力矩大小/方向偏；
- `joint_scales` 是外部传动比，当前全部 1.0，不要仅凭“3D 幅度差”就随便填比例；
- 重力前馈用**实测编码器角**计算；反馈超时才退回指令角；
- 开重力前先确认零位/方向，`gravity_scale` 从 0.2 开始，扶住机械臂、急停在手；
- `kp=0, kd=0` 时 MIT 变成纯力矩/零阻抗模式，确实可以手推，但它不是位置保持，也不是正式拖动示教；正式示教用「拖动示教」模式。

### 6.6 安全

- 驱动不会自动使能；必须显式使能 + 开始发送；
- 使能只发 0xFC，不发力；开始发送才输出保持力矩；
- 反馈持续超时会自动零力矩并失能；
- 机械臂第一次运动必须有人扶、急停可触达、周围清空；
- 失能后电机没有保持力矩，手臂可能因重力下落。

---

## 7. 复现命令清单

```bash
# 1. 编译厂商 gs_usb
cd ~/gs_usb_vendor/gs_usb_5.15
make -j6
cp gs_usb.ko ~/gs_usb_build/gs_usb.ko

# 2. 加载并配置
sudo ip link set can1 down 2>/dev/null || true
sudo rmmod gs_usb 2>/dev/null || true
sudo insmod ~/gs_usb_build/gs_usb.ko
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 sample-point 0.75
sudo ip link set can1 up
ip -d link show can1

# 3. 总线自检
cansend can1 123#1122334455667788
cat /sys/class/net/can1/statistics/tx_packets

# 4. 启动实机驱动（不使能）
cd ~/zfy_arm/Zfy_Arm_real_control
source /opt/ros/humble/setup.bash
source install/setup.bash
CFG=$(ros2 pkg prefix zfy_arm_sim2real)/share/zfy_arm_sim2real/config/motors_socketcan.yaml
ros2 launch zfy_arm_sim2real motor_driver.launch.py sim:=false config:=$CFG
```

---

## 8. 当前状态与后续

已确认：

- 单路 SocketCAN 通信打通；
- 6 轴全部应答，MIT 模式；
- HMI、MoveIt 桥接可用；
- 单轴测试通过。

待做：

1. fold 零位标定，写入 `joint_offsets`；
2. 全轴保持、重力/摩擦小比例验证；
3. 6 轴低速轨迹、拖动示教；
4. MoveIt 实机慢速执行；
5. 动力学参数辨识（重力/摩擦已具备数据采集工具）；
6. 开机自动加载 gs_usb + 配置 CAN 的持久化脚本。
