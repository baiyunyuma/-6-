# zfy_arm_sim2real

真机部署项目的第一阶段：**用 ROS 2 控制 ZFY 机械臂的达妙电机（MIT 模式）**。

CAN 层直接复用上位机的 `../py/core.py`（`MotorBus`），所以使能时序、模式切换、
反馈解析与 Qt 上位机完全一致；本包只做 ROS 2 接口，不重复实现 SDK。

> 当前阶段只包含“电机直控”。还没有关节角标定、重力补偿、MoveIt2 轨迹接口，
> 详见文末的限制与下一步。

## 目录

```
zfy_arm_sim2real/
├── config/motors.yaml        # 6 关节注册表 + 保持增益 + 频率
├── launch/motor_driver.launch.py
└── zfy_arm_sim2real/
    ├── motor_driver.py       # 驱动节点
    └── mit_demo.py           # 单关节正弦测试
```

## 构建

```bash
cd ~/zfy_arm/Zfy_Arm_real_control
source /opt/ros/humble/setup.bash
colcon build --base-paths zfy_arm_sim2real \
  --packages-select zfy_arm_sim2real --symlink-install
source install/setup.bash
```

`--base-paths` 不能省略：上一层目录本身带 `CMakeLists.txt`，会被 colcon 当成
一个包，从而不再向下发现本包。构建会在 `Zfy_Arm_real_control` 下生成
`build/ install/ log/`。节点会自动向上查找 `Zfy_Arm_real_control/py`；
如果把包装到别处，用参数 `hmi_py_path` 指过去。

## 仿真链路自测（不碰硬件）

```bash
# 终端 1
ros2 launch zfy_arm_sim2real motor_driver.launch.py sim:=true

# 终端 2
ros2 topic echo /joint_states --once
ros2 run zfy_arm_sim2real mit_demo --confirm --joint 4 --amplitude 0.1 --seconds 12
```

预期：`mit_demo` 先使能（仅 0xFC）、再开始发送 MIT，让 joint4 做两圈正弦；
`/joint_states` 的位置随之变化；结束后自动零力矩失能。

## 实机运行

1. **停掉 Qt 上位机和任何旧的 SDK/测试进程**：USB2CANFD 只能被一个进程打开。
2. 确认机械臂已上电、物理急停可触达，机械臂处于支撑或低风险姿态。
3. 启动驱动（`sim:=false` 才会打开 CAN）：

   ```bash
   ros2 launch zfy_arm_sim2real motor_driver.launch.py sim:=false
   ```

   launch 会自动把 `py/dm_backend/lib` 加进子进程的 `LD_LIBRARY_PATH`，用来加载
   达妙 SDK 自带的新版 `libstdc++`/`libusb`。如果改用 `ros2 run` 手动启动，需要先
   `export LD_LIBRARY_PATH=~/zfy_arm/Zfy_Arm_real_control/py/dm_backend/lib:$LD_LIBRARY_PATH`，
   否则会报 `GLIBCXX_3.4.32 not found`。

4. 看日志：
   - `连接探测: joint1(0x01)=应答, ...` 六台都应答；
   - 若提示“通道波特率回读不符”，检查 `device_type`、`channels`、`canfd`。
5. 另开终端检查反馈（位置应随手动/静止状态变化，而不是全 0）：

   ```bash
   ros2 topic echo /joint_states --once
   ```

6. 使能（只发 0xFC，**不发任何控制帧**，电机不会立刻出力）：

   ```bash
   ros2 service call /motor_driver/enable std_srvs/srv/SetBool "{data: true}"
   ```

7. 开始发送 MIT（对应上位机的“发送数据”，先按当前位置保持）：

   ```bash
   ros2 service call /motor_driver/sending std_srvs/srv/SetBool "{data: true}"
   ```

   暂停发送（仍保持使能，按 `hold_kp/hold_kd` 锁住当前位置）：

   ```bash
   ros2 service call /motor_driver/sending std_srvs/srv/SetBool "{data: false}"
   ```

8. 首次运动建议只测小关节、小幅度：

   ```bash
   ros2 run zfy_arm_sim2real mit_demo --confirm --joint 4 \
       --amplitude 0.05 --period 8 --seconds 8
   ```

   确认方向、幅度和急停流程后，再逐步加大幅度或测试基座关节。

**停止方式**：优先按物理急停/断电。正常 `Ctrl+C` 停止 launch 时，驱动会先发零力矩
并失能；`kill -9` 或进程崩溃不会走清理流程。

## ROS 2 接口

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/joint_states` | `sensor_msgs/JointState` | 100 Hz（可配）；`position/velocity/effort` 为电机原始量 |
| `/mit_command` | `std_msgs/Float64MultiArray` | 30 个数：`[kp×6, kd×6, q×6, dq×6, tau×6]` |
| `/motor_driver/enable` | `std_srvs/SetBool` | `true` 只发 0xFC 使能，不发控制帧；`false` 停流 + 零力矩 + 失能 |
| `/motor_driver/sending` | `std_srvs/SetBool` | `true` 开始 250 Hz MIT 流；`false` 停止发送（保持使能） |
| `/motor_driver/release` | `std_srvs/Trigger` | 停流 + 零力矩 + 失能 |

`/mit_command` 只在**已使能且正在发送**时接受；顺序与 `joint_names` 一致。
未发送控制帧时，驱动用 `0xCC` 状态刷新（`state_refresh_period`）维持
`/joint_states` 更新，不会让电机出力。手动发一帧：

```bash
ros2 topic pub --once /mit_command std_msgs/msg/Float64MultiArray \
  "{data: [5,5,5,5,5,5, 0.3,0.3,0.3,0.3,0.3,0.3, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]}"
```

这会把六个关节都拉到 0 位（仿真里可见，实机上相当于一次回零运动，慎用）。
若只发一帧，`command_timeout` 后会保持这一帧的目标位姿。

## 主要参数（`config/motors.yaml`）

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `sim` | `true` | `true` 不打开 CAN，用一阶模型出反馈 |
| `canfd` | `false` | 当前 USB2CANFD 按经典 CAN 使用 |
| `joint_names` / `can_ids` / `mst_ids` / `models` / `channels` | 1..6 | 必须与实物一致；`can_ids` = 控制 ID（ESC_ID/RID 8），`mst_ids` = 反馈 ID（MST_ID/RID 7），两者可不同（如控制 0x11、反馈 0x01） |
| `hold_kp` / `hold_kd` | 5.0 / 0.3 | 使能与指令超时时的保持增益，初值来自上位机，实机需重调 |
| `control_rate` | 250 Hz | MIT 下发频率 |
| `state_rate` | 100 Hz | `/joint_states` 频率 |
| `command_timeout` | 0.3 s | 超时后保持最后一帧目标位姿 |
| `feedback_timeout` | 0.5 s | 反馈超时只告警，不会自动释放 |
| `state_refresh_period` | 0.5 s | 未发送 MIT 时用 0xCC 查状态的周期 |
| `hmi_py_path` | 空 | 显式指定上位机 `py` 目录 |

## 安全行为

- **不会自动使能，也不会自动开始发送**；未使能或未发送时的 `/mit_command` 会被忽略并告警。
- 使能只发 0xFC；发送状态独立，未开始发送时不会产生任何 MIT 控制帧。
- 使能前要求所有关节都有新鲜反馈，否则拒绝（避免拿 0 位姿上电）。
- 开始发送时先按当前实测位置保持，之后才接受 `/mit_command`。
- 停止发送前补 5 帧“当前位置保持”，避免电机继续执行最后一条运动指令。
- 指令按型号量程钳位：`q ±12.5 rad`、`dq`/`tau` 按 4310 / 4340 额定值。
- 指令中断超过 `command_timeout` 后，用 `hold_kp/hold_kd` 保持最后一帧目标位姿。
- 反馈中断只告警并继续发送保持指令，避免直接掉臂；是否自动断电由上层决定。

## 已知限制 / 下一步

1. **没有重力/摩擦补偿**：当前只有 PD 保持，伸展姿态下会下垂。下一阶段接入
   仿真里的 `qfrc_bias`（重力+科氏）和 `friction_torque` 作为 MIT 的 `tau` 前馈。
2. **没有关节角标定**：`/joint_states` 是电机原始角度，尚未做方向/零位映射，
   不能直接喂给 MoveIt2。需要逐关节确定 `direction`、`offset`。
3. **没有轨迹接口**：下一阶段加 `FollowJointTrajectory` action server（或
   `ros2_control` 的 `joint_trajectory_controller`），由 MoveIt2 规划后流式下发；
   驱动内部做插值与 MIT 保持。
4. **软实时**：Python + rclpy 的 250 Hz 足够台架调试，但不是硬实时。正式部署建议
   用 C++ 节点 / `ros2_control`，并给进程 SCHED_FIFO、隔离 CPU。
5. 从 WSL 换到原生 Ubuntu 时，需要为 USB2CANFD（`34b7:6877`）配置 udev 权限，
   其余 ROS 2 Humble 流程与这里一致。
