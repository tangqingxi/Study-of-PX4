# PX4 HITL 案例学习

## 0. 实验对象与通信关系

| 对象 | 本案例配置 | 作用 |
| --- | --- | --- |
| 飞控 | CUAV V5 nano（FMU-v5） | 在真实 MCU 上运行 PX4 固件 |
| 遥控器 | Futaba T6K | 产生人工控制输入 |
| 接收机 | Futaba R3006SB | 通过 S.BUS 向飞控发送多通道遥控数据 |
| 仿真器 | jMAVSim 或 Gazebo Classic | 计算动力学并向飞控发送模拟传感器数据 |
| 地面站 | QGroundControl | 配置、监视和任务控制 |

![PX4 HITL 通信架构](images/hitl-communication.png)

HITL 中 PX4 运行在真实飞控上，仿真器通过 USB/UART 与飞控交换 MAVLink HIL 消息，并通过 UDP 将链路转发给 QGroundControl。遥控器可以直接经 S.BUS 接入飞控，不必经过 QGroundControl。

## 第一步：建立并验证 Futaba 遥控链路

### 1.1 连接关系

```text
Futaba T6K  ──2.4 GHz T-FHSS Air──>  R3006SB  ──S.BUS──>  CUAV V5 nano
                                                    SB/6       DSM/SBUS/RSSI
```

R3006SB 的 `SB/6` 是复用接口：

| 接收机模式 | `SB/6` 输出 | 能否用于本案例 |
| --- | --- | --- |
| Mode A（默认） | 普通 PWM 第 6 通道 | 否，只包含单通道信号 |
| Mode B | S.BUS 多通道串行信号 | 是 |

接线前关闭接收机和飞控电源，使用匹配的 S.BUS 线连接 `SB/6` 与 V5 nano 的 `DSM/SBUS/RSSI` 接口；确认插头方向及 `Signal / VCC / GND` 对应正确。该接口只用于 RC 接收机，不连接其他负载。

### 1.2 将 R3006SB 切换到 Mode B

1. 关闭 T6K，单独给 R3006SB 上电；约 3 秒后等待红灯常亮。
2. 按住接收机 `Mode` 键超过 5 秒，红灯和绿灯同时闪烁时松开，进入通道模式设置。
3. 短按 `Mode` 键在 Mode A 与 Mode B 之间切换；**红灯连续闪 2 次表示 Mode B**，闪 1 次表示 Mode A。
4. 在 Mode B 状态下按住 `Mode` 键超过 2 秒，红绿灯同时闪烁时松开以保存。
5. 断电重启接收机，再进入设置状态复核：红灯应以 2 次为一组闪烁。

> 接收机正在与发射机通信时不能修改通道模式，因此切换 Mode B 时必须关闭 T6K。

### 1.3 T6K 与 R3006SB 对频

1. 在 T6K 中将 `RX SYSTEM` 设置为 `T-FHSS Air`。若更改过制式，必须重新对频。
2. 将发射机和接收机置于约 0.5 m 范围内，并关闭附近其他正在对频的 T-FHSS Air 设备。
3. T6K 进入 `LINK` 页面，长按 Jog 键约 1 秒进入接收机对频状态。
4. 给 R3006SB 上电；接收机会在上电后的约 3 秒内等待对频。
5. 接收机 LED 从红色闪烁变为**绿色常亮**，表示已接收到有效信号。
6. 接收机断电再上电，拨动摇杆和开关，确认它仍由当前 T6K 控制。

安全上电顺序为“先开 T6K，再给接收机/飞控上电”；关闭时顺序相反。对频和后续测试期间拆除螺旋桨。

### 1.4 在 QGroundControl 验收

使用 USB 连接 V5 nano，进入 **Vehicle Setup → Radio**：

| 检查项 | 通过条件 |
| --- | --- |
| 链路识别 | 页面出现遥控通道，飞控无 `Manual control lost` 报警 |
| 四个主通道 | 横滚、俯仰、油门、偏航分别响应，通道不串扰 |
| 行程与中位 | 摇杆回中稳定，端点可覆盖校准要求的输入范围 |
| 开关通道 | 预定模式开关在对应通道产生清晰、稳定的档位变化 |
| 失联 | 关闭 T6K 后，飞控在合理时间内识别 RC 丢失 |

完成 QGC 遥控校准并保存。若后续同时启用 RC 与 QGC Joystick，需要再用 `COM_RC_IN_MODE` 明确输入来源及优先级；本案例直接使用 S.BUS 时，应保证 RC 输入未被禁用。

## 第二步：准备包含 HITL 输出模块的 FMU-v5 固件

### 2.1 `pwm_out_sim` 的作用

HITL 机架不把控制量发送给真实电机，而由 `pwm_out_sim` 接收 PX4 的执行器控制结果，按 `HIL_ACT*` 配置生成模拟执行器输出，再通过 MAVLink 送往仿真器。该模块并非所有飞控的默认固件都会编译。

### 2.2 检查模块状态

在 QGroundControl 中打开 **Analyze Tools → MAVLink Console**，执行：

```sh
pwm_out_sim status
```

| 返回结果 | 含义 | 下一步 |
| --- | --- | --- |
| `nsh: pwm_out_sim: command not found` | 固件未包含该模块 | 修改板级配置并重新编译 |
| `not running` | 模块已编译，但当前未启动 | 选择 HITL 机架并重启后复查 |
| 输出 `HIL_ACT`、工作队列和 Channel Configuration | 模块已编译且正在运行 | 检查通道功能后继续 |

本次实测输出包含：

```text
INFO  [mixer_module] Param prefix: HIL_ACT
INFO  [mixer_module] Switched to rate_ctrl work queue
Channel 0: func: 101
Channel 1: func: 102
Channel 2: func: 103
Channel 3: func: 104
```

结论：`pwm_out_sim` 已存在并运行，Channel 0～3 分别配置为 Motor 1～4。仿真器尚未连接时出现 `cycle: 0 events` 或 `control latency: 0 events` 是正常现象，不能据此判断模块失效。

### 2.3 将模块加入 FMU-v5 固件

若控制台返回 `command not found`，在 PX4 源码中编辑：

```text
boards/px4/fmu-v5/default.px4board
```

加入或启用：

```text
CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y
```

也可以使用图形化板级配置界面：

```bash
make px4_fmu-v5 boardconfig
```

在 `modules → Simulation` 中启用 `pwm_out_sim`，保存后编译：

```bash
make px4_fmu-v5_default
```

成功产物位于：

```text
build/px4_fmu-v5_default/px4_fmu-v5_default.px4
```

### 2.4 处理 Flash 空间超限

FMU-v5 的 Flash 容量有限，加入 `pwm_out_sim` 后可能出现固件超限。处理原则是**按本次 HITL 试验的功能清单裁剪，并保留可追溯的配置差异**：

1. 保存完整编译错误，确认失败项确实是固件镜像超过 Flash 上限，而非 RAM、链接或依赖错误。
2. 在 `default.px4board` 或 `boardconfig` 中关闭本实验明确不用的设备驱动、载荷功能或通信协议，每次只裁剪一组后重新编译。
3. 不关闭 `commander`、参数系统、uORB、MAVLink、控制器、估计器、控制分配及 HITL 依赖的核心模块。
4. 不因 HITL 使用模拟传感器就随意删除传感器框架；HITL 启动逻辑仍依赖正常的 PX4 系统结构。
5. 用 Git 查看最终差异并记录每个被关闭模块的理由，避免生成“能编译但缺少关键功能”的固件。

可用以下命令确认修改范围：

```bash
git diff -- boards/px4/fmu-v5/default.px4board
```

### 2.5 刷写与最终验收

1. QGroundControl 进入 **Vehicle Setup → Firmware**。
2. 选择 **Advanced settings → Custom firmware file**，刷写生成的 `.px4` 文件。
3. 飞控重启后再次执行 `pwm_out_sim status`。
4. 通过条件：命令可识别；选择 HITL 机架后模块进入运行状态；通道功能与目标机架一致。

至此只证明固件具备 HITL 能力。模拟传感器替代、`HIL_ACT*` 默认值及对应模块的自动启动，需要在下一步选择标准 HITL 机架后才完整生效。

## 第三步：选择标准 HITL 机架

待续。

## 参考资料

- [PX4：Hardware-in-the-Loop Simulation](https://docs.px4.io/main/en/simulation/hitl)
- [PX4：CUAV V5 nano 接线说明](https://docs.px4.io/v1.11/en/assembly/quick_start_cuav_v5_nano)
- [Futaba：R3006SB 使用说明书](https://www.rc.futaba.co.jp/downloads/W8C848N2105290129sm4ea.pdf?mode=view)
