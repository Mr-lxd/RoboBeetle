# RoboBeetle 下水调试前重点代码带教手册

阅读基线：2026-10-19，`main` / `HEAD` / 本地 `origin/main` 均为 `35d11756dfb03031313c8e6a5779631f2aa383be`。这是本次只读 Git 核对结果，不证明机器人已安装此版本。下文 `路径:行号` 均相对 `D:\RoboBeetle`，固定于这个提交；更新代码后须重新核对行号。

本手册只分析 Firmware、Pi 网关与 Console；FOMO 外部仓库不在分析范围。代码摘录保留关键原句，右侧中文注释为教学补充；省略部分以出处说明。现场步骤标为【操作建议】，它们是负责人要执行和记录的检查，不是本次已经完成的实测。

先纠正三个版本前提：当前 CPG **名义参数为 2.5162 s**，校准目标是实际约 2 s；当前视觉日志为 **visual-csv-v8**，保留 v5 的重要字段并在末尾追加 `yaw_sway_deg / assoc_shift_px`（表头：`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:11`）；当前视觉方向符号固定为 **turn_sign=+1、pitch_sign=+1**，应用不再要求每会话选择并确认方向。依据：`RoboBeetleFirmware/Core/Motion/cpg_parameters.c:7-20`、`RoboBeetleConsole/src/vision/VisualPolicyConfig.h:26-65`、`RoboBeetleConsole/src/vision/VisualDispatchSession.h:126-130`。旧交接的人工确认流程属于历史：`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:29-30`、`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:74-76`。

几个先用的大白话：ACK = 对方说“这条命令我收到了并处理了”，不等于机械动作完成；PWM = 送给舵机的定时脉冲，用脉冲宽度告诉它要转到哪里，raw PWM就是直接给脉宽；CPG = 让四个划水足按节拍摆动的“节拍器”；CSV = 可以用 Excel 按行列打开的文本日志。TCP 是 Console 与 Pi 间传字节的网络连接；USART 是 Pi 与 STM32 间传字节的串口。schema是数据包/日志格式的版本；逻辑姿态已知是程序记得关节的语义角度，不能代替实际角度反馈。记录/清除逻辑角的代码：`RoboBeetleFirmware/Core/Servo/servo_service.c:348-349`、`RoboBeetleFirmware/Core/Servo/servo_service.c:395-397`。

证据必须这样读：

| 项目标签 | 能说明什么 | 不能顺带说明什么 |
|---|---|---|
| Host Test | 桌面测试覆盖了指定逻辑 | STM32 上的速度、真实舵机、水下安全 |
| ARM Build | ARM 编译器能生成目标固件 | 已烧录、已运行 |
| Program Verify | 特定镜像已烧录并校验 | 当前机械安装和水下行为正确 |
| Hardware Verified | 记录中明确观察过的实物路径/动作 | 不同镜像、安装或环境同样通过 |
| Bench Measured | 台架上确实量过的值 | 水中推进效率、防水能力 |
| Pending | 尚缺指定证据 | 已经通过 |
| Pending Water Verification | 水下行为仍待负责人实测 | 桌面通过就是水下通过 |

上述分层来自 `docs/engineering-lessons.md:129-139`、`docs/engineering-lessons.md:158-160`。本次未运行 Host Test、ARM Build、Program Verify 或硬件程序；后文历史 PASS 均注明出处和范围。特别是 USART2 心跳路径已记录 Hardware Verified，但该记录中的传感器遥测、Pi 失联安全与运动负载仍 Pending：`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:1778-1801`。

## 0. 一页速读

### 下水前十项必查

以下均为【操作建议】；未满足的项目记为 Pending，不跳过。

1. **防水与回收**：FrontAxis/Depth 防水能力仍为【Unverified】；没有可靠密封/IP 依据时，不安排该未验证部件直接浸水。先落实封装与可执行的人工回收方式。文档边界：`docs/engineering-lessons.md:90-92`。
2. **版本来源**：核对实装 Firmware、Pi、Console 来自同一发行版本，保存固件/程序来源；旧镜像 PASS 不转移。依据：`docs/proportional-control.md:3-6`、`docs/engineering-lessons.md:129-139`。
3. **自动输出关闭**：开始时 `Visual dispatch` 不勾选，Auto follow 显示 DRY RUN；Exec 会恢复上次选择，须重新核对；Dry run 不记忆，Arm 仍须手动操作，不要把 READY 当作已经布防。Exec恢复与Dry run初值：`RoboBeetleConsole/src/ui/MainWindow.cpp:3459-3461`、`RoboBeetleConsole/src/vision/VisualProportionalDispatchStateMachine.h:19-21`；默认开关与单独 Arm：`RoboBeetleConsole/src/vision/VisualDispatchSession.h:126`、`RoboBeetleConsole/src/ui/MainWindow.cpp:1492-1495`、`RoboBeetleConsole/src/ui/MainWindow.cpp:1656-1663`。
4. **停止入口找得到**：认清 Motion Control 中央 `STOP`、Operator tools 顶部 `Disable All`。`Emergency Stop` 是禁用按钮，不能依赖。依据：`RoboBeetleConsole/src/ui/MainWindow.cpp:2377-2395`、`RoboBeetleConsole/src/ui/MainWindow.cpp:2903-2922`。
5. **控制权与使能**：确认连接和控制权有效，所需舵机使能；布防还要求逻辑姿态已知。raw PWM会清掉逻辑姿态证据，准备自动跟随时须重新用已确认的Neutral/角度路径建立姿态。依据：`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:15-30`、`RoboBeetleFirmware/Core/Servo/servo_service.c:343-349`、`RoboBeetleFirmware/Core/Servo/servo_service.c:388-397`。
6. **漏水可信**：Leak Detection 为 Dry 且更新正常；Unknown 不是“没有漏水”。WET 需要你主动停机、出水。依据：`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:903-913`、`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1344-1347`。
7. **深度先置零**：机器人停住、视觉撤防，在已知水面基准取得稳定样本后按 Zero；控制深度年龄不超过 1300 ms。依据：`RoboBeetleConsole/src/robot/DepthControl.h:8-15`、`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1016-1062`。
8. **方向需实物确认**：手动低风险检查转向和升潜实际方向。当前 `+1` 是配置常量，不是你的会话确认；机体坐标与磁/偏航标定仍 Pending。依据：`RoboBeetleConsole/src/vision/VisualPolicyConfig.h:26-35`、`docs/engineering-lessons.md:3`。
9. **步态参数已回读**：CPG 面板 Applied 为 `Matches firmware`；记录当前幅值、名义周期、后端和 Front / Rear 协调。默认幅值 10°不自动等于水下安全值。依据：`RoboBeetleFirmware/Core/Motion/cpg_parameters.c:9`、`RoboBeetleConsole/src/ui/CpgParametersPanel.cpp:187-229`。
10. **日志真的在录**：打开视觉 CSV，确认 `CSV Recording` 和对应 Motion CSV 路径，没有 CSV error；两个日志都有 32 MiB 上限。依据：`RoboBeetleConsole/src/ui/MainWindow.cpp:3006-3028`、`RoboBeetleConsole/src/vision/VisualPolicyConfig.h:62-65`、`RoboBeetleConsole/src/robot/MotionStateCsvLogger.cpp:44-47`。

### 紧急情况三步处理

1. **先停控制输出**：【操作建议】漏水、撞限位、方向失控时，链路和控制权仍可用就立即按 `Disable All`；一般暂停用中央 `STOP`。两者都会触发视觉手动接管/撤防。不要等禁用的 `Emergency Stop`，也不要等 ACK 来判断是否已经停稳。按钮依据：`RoboBeetleConsole/src/ui/MainWindow.cpp:2388-2392`、`RoboBeetleConsole/src/ui/MainWindow.cpp:2908-2922`。
2. **人工回收出水**：【操作建议】若按钮变灰、控制权丢失、未观察到停止，直接执行预定回收方式，不把重连成功当作停机证明。STOP/Disable 按钮受控制权限制：`RoboBeetleConsole/src/ui/MainWindow.cpp:4632-4663`；断链只能依靠第 1 课的停心跳保护及人工兜底。
3. **安全断电、保留证据**：【操作建议】出水后按已确认的实物断电方式处理，保存 CSV 和发生时刻；原因未查清前不重新 Enable/Arm。【未在代码中找到】可替代实物断电的舵机电源切断指令；`disable_all` 做的是驱动 stop 和清状态：`RoboBeetleFirmware/Core/Servo/servo_service.c:214-238`、`RoboBeetleFirmware/Core/Servo/servo_driver_stm32.c:231-245`。

## 1. 安全停机链路：收据、刹车与断开驱动是三件事

### 在机器人身上起什么作用

这条链是通信失效后的最后一道软件保护。看门狗 = 一段时间收不到有效心跳，就自动停止控制输出的保护。心跳 = 控制端周期发送的“我还在”的消息。普通 STOP 像慢慢收桨；失联安全停机像直接停止驱动，**不保证把腿带回零位**。依据：`RoboBeetleFirmware/Core/Safety/safety_supervisor.c:26-45`、`RoboBeetleFirmware/Core/App/app_main.c:114-124`。

```text
普通暂停：Qt STOP → TCP → Pi → USART2 → STM32 接受/ACK
                                      → STOPPING → 750 ms 回零 → STOPPED

连接失效：TCP EOF/错误或控制租约过期 → Pi revoke/abort
          → 丢弃串口待发数据/关闭串口 → 停发心跳
          → STM32 有效心跳年龄 >500 ms → 中止运动 + disable_all
```

链路出处：`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:38-59`；当前实现位置见下面时间表。

### 关键代码在做什么

摘自 `RoboBeetleFirmware/Core/App/app_main.c:114-124`，省略诊断条件编译块，6 行：

```c
static void app_main_apply_safety_stop(void)                 // 所有失联安全停止集中到这个入口
{                                                          // 开始执行保护
    motion_manager_stop_immediate(&motion_manager);          // 立即中止运动，不走750 ms回零
    servo_service_disable_all(&servo_service);               // 停各已使能舵机的PWM，清使能/姿态证据
    protocol_dispatcher_invalidate_action_cache(&protocol_dispatcher); // 丢掉旧动作的重发记忆
}                                                          // 恢复心跳不会在这里自动恢复动作
```

主循环先做 Safety，再允许 Motion：`RoboBeetleFirmware/Core/App/app_main.c:895-908`。保护条件是**严格大于** 500 ms，整数毫秒下首次满足为 501 ms：`RoboBeetleFirmware/Core/Safety/safety_supervisor.h:7`、`RoboBeetleFirmware/Core/Safety/safety_supervisor.c:35-43`。PWM关闭在定时器和compare中断正常服务的前提下等待下一安全比较边沿，策略最多额外一个PWM帧，另需计中断延迟；“立即中止逻辑”不是“机械瞬间静止”：`RoboBeetleFirmware/Core/Servo/servo_pwm_stop_policy.h:6-12`、`RoboBeetleFirmware/Core/Servo/servo_driver_stm32.c:257-338`。

### 三条时间预算，不能混用

| 触发 | 代码行为与名义时间 | 依据及证据边界 |
|---|---|---|
| 普通 STOP | 接受即抢占、ACK OK、进入 STOPPING；从接受起 750 ms 插值回零，按 10 ms 有效 tick 调度，逻辑完成预算 760 ms；重复 STOP 不重启倒计时 | `RoboBeetleFirmware/Core/Communication/protocol_dispatcher.c:448-459`；`RoboBeetleFirmware/Core/Motion/motion_config.h:4-5`；`RoboBeetleFirmware/Core/Motion/motion_manager.c:946-983`、`RoboBeetleFirmware/Core/Motion/motion_manager.c:466-508`、`RoboBeetleFirmware/Core/Motion/motion_manager.c:1026-1039`；Host Test 记录 `ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:40-44` |
| STM32 收不到有效心跳 | 最后一条有效心跳后 >500 ms，调用上面的安全入口；不走 750 ms | `RoboBeetleFirmware/Core/Safety/safety_supervisor.c:35-43`；`RoboBeetleFirmware/Core/App/app_main.c:897-903` |
| 手柄比例输入停更、心跳仍正常 | >600 ms 没有新的合法比例输入，进入普通 STOP，保留舵机使能 | `RoboBeetleFirmware/Core/Motion/motion_manager.c:1012-1016`；`docs/proportional-control.md:49-50` |

Qt/TCP 断线后的完整预算要从**哪个端点已经识别断线**开始算：

- Pi 检测 EOF/错误后：owner 调度名义留约 100 ms + 最后残余有效串口心跳 + 501 ms + 固件主循环延迟，因此约 **601 ms 加残余/调度延迟**。这里100 ms来自默认心跳驱动的动态poll预算，不是固定owner定时器，也不构成调度上界。`abort()` 本身不发送 STOP。依据：`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:46-58`、`RoboBeetlePi/link_core/include/robobeetle/link_core/link_config.hpp:9`、`RoboBeetlePi/runtime/src/link_runtime.cpp:81-88`、`RoboBeetlePi/gateway/linux/src/linux_onboard_application_port.cpp:344-346`、`RoboBeetlePi/session/src/serial_session.cpp:131-178`。
- 没有及时 TCP 通知、靠控制租约判断静默：最后一次有效 control heartbeat 起，名义 **1000 + 100 + 501 = 1601 ms**，还要加残余/调度延迟。控制租约 = Pi 给控制端的一张短时“驾驶证”，要定期续期。依据：`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:53-59`、`RoboBeetlePi/gateway/src/control_gateway_core.cpp:12`、`RoboBeetlePi/gateway/src/control_gateway_core.cpp:554-558`、`RoboBeetlePi/gateway/src/control_gateway_core.cpp:878-885`。
- 从物理拔线开始，**走TCP EOF/错误路径时**还要加TCP检出时间；静默失联若先由lease识别，使用上一条自最后control heartbeat起的预算，不再额外叠加无界TCP等待。【未在代码中找到】所有网络故障固定毫秒触发EOF的保证。575 ms SafetyQuiet是重新开串口的保护等待，不是停稳证明。依据：`docs/task05-stop-transition-red.md:107-128`、`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:57-59`。

以上是代码预算，**Pending 实机时间测量 / Pending Water Verification**，不是硬实时上界。普通 STOP、disable_all、机器人惯性减速也必须分别观察。

### 下水时要盯什么

看 Operator tools 顶部运动状态 `Running / Stopping / Stopped / Faulted`；看五舵机使能状态；自动跟随看 `ARMED / STOPPING / FAULT` 与 STOP timeout。界面状态生成：`RoboBeetleConsole/src/ui/MainWindow.cpp:4610-4663`、`RoboBeetleConsole/src/ui/MainWindow.cpp:3101-3118`。**Console的Stopped还可能由本地STOP定时器推断，不能单凭这个字当作STM32回报或实物反馈**：`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:91-95`。同时看实物足是否停止摆动、是否仍漂移。

TCP keepalive = 用探活消息辅助发现连接已经失效；半开连接 = 一边已经离开、另一边仍以为连接存在。网关只容纳一个TCP客户端，旧半开连接会让新Console被拒绝；没有TCP keepalive的已知限制有明确交接记录：`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:31-33`。具体代码与现场记录方法见第7课。重连不是紧急刹车方法；先安全回收，关闭旧Console，再由负责人处理网关服务。

### 异常对照表

| 看到什么 | 可能原因 | 先做什么 |
|---|---|---|
| STOP 已 OK，足还在收回 | 正常 750 ms 回零，ACK 只确认接受 | 【操作建议】保持撤防并观察；若仍失控按 Disable All、出水。依据见本课时间表 |
| 断线后腿变软、停在原角度 | 停 PWM 而非回零；该现象已有离水交接记录 | 【操作建议】人工回收；不立即重新 Enable。`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:87-88` |
| STOP timeout / OUTCOME_UNKNOWN | 未获得可信命令结果，不知道是否执行 | 【操作建议】Disable All 能发就发，并回收出水。`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:173-176` |
| 新 Console 显示 remote host closed | 旧连接仍占用单客户端槽位【推测】 | 【操作建议】先回收/撤防，再关旧 Console；仍失败按交接处理网关。`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:31-33` |

自测：1.1 STOP 的 ACK OK 能证明腿已回到零位吗？1.2 比例输入 600 ms 超时与心跳 500 ms 超时，哪个保留舵机使能？答案在第 12 节。

## 2. 漏水检测：这是指示灯，当前还不是自动刹车

### 在机器人身上起什么作用

PA11 读取漏水模块数字输出 D0，就像“地面有没有积水”的开关。它只报告 Dry/Wet/Unknown；**固件不因 WET 自动 STOP/disable_all，Console 也只更新显示，没有自动报警声音或停机接线**。这不是推测：采样路径只更新传感器状态，远程接收只设置 LeakState，界面只改文案/颜色。依据：`RoboBeetleFirmware/Core/App/app_main.c:802-805`、`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:903-913`、`RoboBeetleConsole/src/ui/MainWindow.cpp:4340-4362`；交接明确 safety response Pending / Not Implemented：`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:144-159`。

```text
漏水模块 D0 → PA11电平 → Dry/Wet状态 → LeakStatus → Pi → Qt Leak Detection
                                           当前没有 → 自动STOP/蜂鸣/电源切断
```

### 关键代码在做什么

摘自 `RoboBeetleFirmware/Core/Sensors/leak_sensor.c:8-13`，6 行：

```c
leak_sensor_state_t leak_sensor_state_from_gpio_level(bool gpio_high) // 接收“引脚是高还是低”
{                                                                  // 只做状态换算
    return gpio_high                                                // 如果读到高电平
               ? LEAK_SENSOR_STATE_DRY                              // 就报告Dry
               : LEAK_SENSOR_STATE_WET;                             // 否则报告Wet，没有停机函数
}                                                                  // 后续由遥测把结果送出去
```

首次采样前Unknown：`RoboBeetleFirmware/Core/Sensors/leak_sensor.c:3-5`。GPIO = 芯片用来读取高低电平的引脚，NOPULL = 没开启内部上拉/下拉；当前PA11为INPUT、NOPULL：`RoboBeetleFirmware/Core/Inc/main.h:64`、`RoboBeetleFirmware/Core/Src/main.c:446-451`。输出级和上拉假设仍需电气测量；没有软件消抖（滤掉短时跳动）、湿态锁存（湿过后保持告警直到解除）或Safety动作：`docs/engineering-lessons.md:96-125`。漏水遥测有500 ms刷新策略，但依赖有效心跳交换机会，不是水滴到屏幕的500 ms上界：`RoboBeetleFirmware/Core/Sensors/leak_telemetry_policy.c:13-31`、`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:145-168`。

### 下水时要盯什么

Dashboard 的 **Leak Detection**：绿灯 Dry；红灯 `LEAK DETECTED`；灰灯 Unknown。依据：`RoboBeetleConsole/src/robot/LeakStatus.h:27-37`、`RoboBeetleConsole/src/ui/MainWindow.cpp:4340-4362`。远程 1500 ms 无更新回 Unknown：`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1344-1347`。Unknown 表示信息不可信，不能当成 Dry。

历史 PA11→固件→Qt 漏水监视路径是 Hardware Verified，但没有记录数值响应时间；防水能力仍【Unverified】，电气检测通过也不证明舱体、接头或舵机能防水。依据：`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:150-168`、`docs/engineering-lessons.md:92`。人工必须兜底：持续看屏幕、观察实物进水迹象、主动 Disable All/撤防、人工回收、出水后安全断电。这里没有“等漏水自动停”的阶段。

### 异常对照表

| 看到什么 | 可能原因 | 先做什么 |
|---|---|---|
| 红灯 / LEAK DETECTED | PA11 低电平；实际漏水或线路电气异常，软件无法区分 | 【操作建议】立即 Disable All、撤防、出水，随后检查。依据：`RoboBeetleFirmware/Core/Sensors/leak_sensor.c:8-13` |
| Dry/Wet 来回跳 | 【推测】接触/输出级/浮动或间歇潮湿；无消抖与锁存 | 【操作建议】停机、出水，不继续用“多数是Dry”判断安全。文档边界：`docs/engineering-lessons.md:103-109` |
| 入水后 Unknown | 遥测过期、断线、非法状态等；不是漏水被排除 | 【操作建议】停止本阶段、回收。代码：`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:906-909`、`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1344-1347` |

自测：2.1 看到 WET 后可以等机器人自己停吗？2.2 Unknown 与 Dry 有什么区别？

## 3. 深度：尺子、零点和“围栏”要分开看

### 在机器人身上起什么作用

深度传感器是一把尺子；Zero 是把当前已知水面设为尺子的零点；depth gating（深度门控）= 在指定控制路径上拦住不允许的升潜动作；envelope（允许区域）= 把当前深度归为水面、正常、软限位、硬限位等状态。**Depth Sensor 是传感器，舵机卡片的 Depth 是 FrontAxis 执行器，两者不是一件硬件。**依据：`RoboBeetleConsole/src/robot/ServoDescriptor.cpp:49-65`、`RoboBeetleConsole/src/robot/DepthControl.h:18-32`。

```text
MS5837/ROVMAKER解码板 → USART6/PC7 → 固件DepthSnapshot → Pi → Console rawDepth
                                                               ↓ Zero减本地均值
                                                         calibratedDepth
                                                               ↓ 年龄/深度区域
                                             视觉Pitch/Both、手柄相关门控
```

传感器路径历史依据：`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:330-343`；本地置零与门控实现见下面。

### 关键代码在做什么

摘自 `RoboBeetleConsole/src/robot/DepthControl.cpp:35-44`，10 行：

```cpp
if (d >= config.hardMaxM)                                      // 零后深度达到0.35 m
    return {DepthEnvelopeState::HardLimit, DepthEnvelopeMemory{true}}; // 报硬限位

if ((previous.softFloorLatched && d > config.softReleaseM) || d >= config.softMaxM) // 到0.25 m，或还没退回0.22 m
    return {DepthEnvelopeState::SoftFloor, DepthEnvelopeMemory{true}}; // 继续保留软限位

const DepthEnvelopeMemory cleared{false};                      // 可以清掉软限位记忆
if (d <= config.surfaceMarginM)                                // 零后深度不超过0.02 m
    return {DepthEnvelopeState::Surface, cleared};              // 归为水面区域
return {DepthEnvelopeState::Normal, cleared};                   // 其余归为正常区域
```

软限位有滞回 = “过线进入，退远一点才解除”，防止数值在边缘跳动反复切换。无样本/不新鲜先返回 UNAVAILABLE，有新鲜样本但未 Zero 返回 NOT_ZEROED：`RoboBeetleConsole/src/robot/DepthControl.cpp:27-34`。

### 数值与含义

默认配置全部取自 `RoboBeetleConsole/src/robot/DepthControl.h:8-15`，这些是控制策略值，**不是传感器精度、额定耐压或已证明的水下安全距离**。

| 数值 | 含义 |
|---|---|
| 名义采样间隔 540 ms，约 1.85 Hz | 配置注释记录的临时采样速率；不是保证每包都按时到达 |
| 控制新鲜度 ≤1300 ms | `depthValid` 且综合样本年龄在窗口内，才可用于控制 |
| Zero 至少 5 个不同样本，极差 ≤0.015 m | 有效行计数去重，不把同一条数据的重复遥测算作新测量 |
| Zero 样本跨度 ≤3240 ms | `max(1000,(5+1)*540)`；运动中拒绝 Zero |
| SURFACE：≤0.02 m | 视觉离散候选 ASCEND 改 FORWARD；已在 ASCEND 时先 STOP；比例控制阻止上浮方向 pitch |
| SOFT_FLOOR：≥0.25 m 进入；≤0.22 m 解除 | 视觉离散候选 DESCEND 改 FORWARD；已在 DESCEND 时先 STOP；比例控制阻止下潜方向 pitch |
| HARD_LIMIT：≥0.35 m | 视觉 Pitch/Both STOP 并撤防；手柄硬限位路径停动作并关闭手柄控制 |

Zero 的去重/窗口/均值依据：`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:970-985`、`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1016-1062`。视觉门控依据：`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:24-30`、`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:142-169`、`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:190-194`；手柄依据：`RoboBeetleConsole/src/ui/MainWindow.cpp:4817-4825`、`RoboBeetleConsole/src/ui/MainWindow.cpp:4830-4878`、`RoboBeetleConsole/src/ui/MainWindow.cpp:4901-4910`。

**保护范围很关键**：视觉 Yaw 不依赖深度门；普通手动 ASCEND/DESCEND 的 `startMotion()` / `isMotionReady()` 不检查深度，只检查控制权、运动状态和使能等。固件 MotionManager 没有把 DepthSnapshot 作为统一深度保护输入。因此不要把 0.35 m 当作任何模式都能自动停的全局保护。依据：`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.h:41-42`、`RoboBeetleConsole/src/ui/MainWindow.cpp:2365-2368`、`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:352-367`、`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:562-584`。手动深度动作由你人工兜底。

### 置零和过期怎么理解

- Zero 只把 `rawDepthM − 最近稳定样本均值` 算成 calibratedDepth，写在 Console 内存中，不改传感器标定。偏移在 Console 存活期间保留、重连仍保留、不持久保存；重启应用要重新建立零点。依据：`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1011-1012`、`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1058-1060`、`RoboBeetleConsole/src/remote/RemoteRobotController.h:247`。
- 【操作建议】先 STOP、撤防，在**已知水面位置**静置取样再 Zero。不能在水下某处乱 Zero：这样会把整个深度围栏一起下移。代码只知道样本稳定、不知道实物是否真的在水面；运动中拒绝：`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1025-1026`；布防时 UI 禁 Zero：`RoboBeetleConsole/src/ui/MainWindow.cpp:3334-3340`。
- `depth_age_ms = 固件采样时已有年龄 + 本地收包后经过时间`，不是只看“刚收到一包”。计算：`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:999-1012`。
- 固件采样过期界限是 3000 ms，清有效标志和数值；UI 包过期界限为 3500 ms，但控制门 1300 ms 更早失效。屏幕还能显示原始读数时，控制可能已经拒绝使用。依据：`RoboBeetleFirmware/Core/Communication/depth_telemetry.h:22-32`、`RoboBeetleFirmware/Core/Communication/depth_telemetry.c:238-250`、`RoboBeetleFirmware/Core/App/app_main.c:632-646`、`RoboBeetleConsole/src/robot/DepthMonitor.h:33`、`RoboBeetleConsole/src/robot/DepthControl.h:9`。
- 视觉 Pitch/Both 深度失效/未置零/硬限位：布防拒绝；运行中 STOP、撤防。Yaw 不因此自动停。手柄比例模式先禁止 pitch，实际遥测仍有禁用方向 pitch 时 STOP。依据：`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:24-30`、`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:142-148`、`RoboBeetleConsole/src/vision/VisualProportionalDispatchStateMachine.cpp:265-278`、`RoboBeetleConsole/src/ui/MainWindow.cpp:4830-4878`。

绝对深度标定/端点仍 Pending，稳定连接下传感器功能路径曾 Hardware Verified；松动连接器会出现貌似合法但错误的值。依据：`docs/engineering-lessons.md:3`、`docs/engineering-lessons.md:245-251`、`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:358-405`。本地 Zero 不补齐绝对标定。

### 下水时要盯什么

Dashboard 的 **Depth Sensor** 看原始 Depth、Temp、Age；其下方 0–0.50 m 条看**零后深度与区域**，0.50 m 只是绘图上限。进度条饱和不意味着真实深度停在 0.50 m。依据：`RoboBeetleConsole/src/ui/MainWindow.cpp:1793-1850`、`RoboBeetleConsole/src/ui/MainWindow.cpp:3308-3328`、`RoboBeetleConsole/src/ui/MainWindow.cpp:4465-4483`。CSV 看 `depth_raw_m / depth_cal_m / depth_age_ms / envelope_state`；第 8 课解释单位和空格。

### 异常对照表

| 看到什么 | 可能原因 | 先做什么 |
|---|---|---|
| NOT_ZEROED / Zero失败 | 还没置零、样本不足、不稳或正在运动 | 【操作建议】STOP、撤防；回已知基准再取样，不在下潜途中置零。`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1025-1055` |
| Age >1300 ms，条显示 Stale | 控制信息过期；可能连接松动/数据停更，原因未由年龄定位 | 【操作建议】STOP、出水检查；不靠仍显示的旧Depth继续升潜。代码见本课过期说明 |
| SOFT_FLOOR，DESCEND成FORWARD | 正常软门控；不是传感器坏 | 【操作建议】停止本次下潜、观察深度趋势；手动按钮不会继承这条门控。`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:190-194` |
| HARD_LIMIT | 零后深度达到0.35 m；也可能错误零点/错误读数 | 【操作建议】Disable All、撤防、人工回收；不可用重Zero消除告警。阈值依据见本课数值表 |

自测：3.1 原始深度 0.14 m，在水面 Zero 后变为 0.00 m，是否完成了传感器绝对标定？3.2 手动 DESCEND 时，0.35 m 围栏是否一定替你停机？

## 4. 姿态与 IMU：看“仪表在更新”，还要确认“仪表代表哪个方向”

### 在机器人身上起什么作用

IMU = 测量加速度、转动速度和姿态的“小水平仪加转速表”。JY901S 通过 USART3/PB11 把数据送给 STM32；Acc 是加速度，Gyro 是角速度（每秒转多少度），Euler Angle 是 roll/pitch/yaw 三个姿态角。它们首先属于**传感器自己的轴**，不自动等于机器人前后/左右/上下。路径及证据：`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:172-189`、`docs/engineering-lessons.md:3`。

```text
JY901S → USART3 → 校验/解析 → Acc、Gyro、Angle → 低速ImuSnapshot → Console IMU
                              ↓ 每收到有效Gyro
                         MotionState采样/分片 → Pi → motion_state CSV
```

### 关键代码在做什么

摘自 `RoboBeetleFirmware/Core/App/app_main.c:849-856`，8 行：

```c
if (event == JY901S_PARSER_EVENT_ANGLE)              // 收到合法姿态角数据
{                                                  // 不把它当作新Gyro样本
    jy901s_last_angle_ms = jy901s_last_valid_frame_ms; // 只记录姿态角最后更新时间
}                                                  // 结束Angle分支
else if (event == JY901S_PARSER_EVENT_GYRO)           // 收到合法陀螺仪数据
{                                                  // 此时才截取一份运动状态
    motion_state_capture(jy901s_last_valid_frame_ms); // 把Gyro、角度、步态等一起记录
}                                                  // 这是记录，不是推进一次CPG计算
```

`motion_state_capture` 读取当前 IMU/运动/相位，并不推进节拍器：`RoboBeetleFirmware/Core/App/app_main.c:409-480`。批次 = 一批样本分多片发送，像同一订单分几个包裹；FIFO = 按进入顺序取出的缓存。当前 sampler 缓存 64 个、每批最多 32 个，schema2 每个线上分片最多 1 个 35 字节样本：`RoboBeetleFirmware/Core/Communication/motion_state_sampler.h:6-7`、`RoboBeetleFirmware/Core/Communication/motion_state_sampler.c:29-51`。

### 哪些可信，哪些还待验证

| 内容 | 正确理解 | 依据 |
|---|---|---|
| 收到合法 Acc/Gyro/Angle，数值随传感器动作变化 | 历史实物接收与解码路径 Hardware Verified；不等于当前安装轴向已确认 | `ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:289-328` |
| 机体轴向、roll/pitch正负方向 | 最终机体坐标映射 Pending，需要受控的实物方向记录 | `docs/engineering-lessons.md:3` |
| 磁力计、绝对yaw/航向 | 磁/偏航标定 Pending；0x54 Mag合法但不解码，不能靠它宣称航向已校准 | `RoboBeetleFirmware/Core/Sensors/jy901s_parser.c:145-149`；`docs/engineering-lessons.md:172-177` |
| UI Receiving | 收到合法ImuSnapshot；未证明其中每个传感器量仍在产生新样本 | `RoboBeetleConsole/src/robot/ImuMonitor.cpp:25-29`、`RoboBeetleConsole/src/robot/ImuMonitor.cpp:44-54` |
| Motion CSV 的Gyro | 有效Gyro事件触发的采样；要同时看valid、age、drop字段 | `RoboBeetleFirmware/Core/App/app_main.c:842-855`；`RoboBeetleConsole/src/robot/MotionStateCsvLogger.cpp:64-72` |

**旧值风险**：parser 解出各量后置 valid，低速 ImuSnapshot 从保留状态发包；其界面 3500 ms stale 主要检查包有没有来。【推测：由上述保持状态路径推出】如果 IMU 自身停更、固件还在发送旧快照，绿灯可能仍存在。因此要看 RX / valid frame 计数是否继续增加，以及受控动作时数值是否响应。依据：`RoboBeetleFirmware/Core/Sensors/jy901s_parser.c:119-141`、`RoboBeetleFirmware/Core/App/app_main.c:554-582`、`RoboBeetleConsole/src/robot/ImuMonitor.cpp:25-29`、`RoboBeetleConsole/src/robot/ImuMonitor.cpp:44-54`。这是待讨论风险，本次未注入故障验证。

历史 listen-only 不覆盖后来默认关闭的一次性配置选项；不要从曾经配置 20 Hz 推断当前实物正在 20 Hz。选项是否启用须核对实装固件来源：`RoboBeetleFirmware/CMakeLists.txt:43`、`RoboBeetleFirmware/Core/Sensors/jy901s_oneshot_config.h:6-12`。

### 下水时要盯什么

看 Dashboard **IMU** 的 Acc/Gyro/Angle，Operator tools→**Telemetry Details→IMU Diagnostics** 的 RX/valid/checksum/UART/overflow/re-arm 计数；同页 **Motion Telemetry** 行看Gyro Hz、Missing batches/fragments、Firmware samples dropped、Gateway fragments dropped。Motion CSV看 `gyro_x/y/z`、`roll/pitch`、`angle_age_ms`、`gyro_valid/angle_valid`、累计 drop/gap。面板代码：`RoboBeetleConsole/src/ui/MainWindow.cpp:4365-4416`、`RoboBeetleConsole/src/ui/MainWindow.cpp:2648-2667`、`RoboBeetleConsole/src/ui/MainWindow.cpp:2968-2972`；日志列：`RoboBeetleConsole/src/robot/MotionStateCsvLogger.cpp:9-19`。checksum = 检查一条 IMU 数据是否被传坏的校验值；计数不断增大比某个孤立历史数更值得记录。

### 异常对照表

| 看到什么 | 可能原因 | 先做什么 |
|---|---|---|
| 姿态明显变化但数值/RX计数不动 | 数据停更或快照仍持旧值【推测】 | 【操作建议】STOP、撤防、出水记录计数和最后响应时刻。依据见本课旧值风险 |
| checksum/UART/drop持续增长 | 串口质量/缓存丢失；单个计数不能定位根因 | 【操作建议】停机，保存增量和动作阶段，出水分段排查。`docs/engineering-lessons.md:172-178` |
| “抬头”却pitch符号相反 | 安装轴向/符号未映射，不是自动证明传感器坏 | 【操作建议】STOP，暂缓Pitch/Both，记录实物方向与数值。`docs/engineering-lessons.md:3` |

自测：4.1 IMU 绿灯能证明机器人 yaw 正方向已标定吗？4.2 Motion CSV 有10行，是否就代表10次不同姿态角更新？

## 5. 运动、CPG、五舵机和手柄：节拍器与机械方向之间还有标定层

### 在机器人身上起什么作用

运动命令选择前进、转向、升潜；CPG 给四个划水舵机生成节拍；FrontAxis/Depth 改前轴偏置。标定层把“关节多少度”换成各舵机自己的 PWM，处理左右镜像。它像同一乐谱由不同乐器演奏，不能因为左右舵机 PWM 一增一减就判断动作相反。依据：`docs/engineering-lessons.md:74-76`、`RoboBeetleFirmware/Core/Motion/cpg_gait_generator.c:158-200`、`RoboBeetleFirmware/Core/Servo/servo_calibration.c:6-45`。

```text
按钮/手柄 → 运动模式/比例需求 → CPG逻辑角度
                               → Motion前后足限位 → ServoService标定 → 五路PWM
                                                        FrontAxis是偏置，不是第五振荡器
```

### 关键代码在做什么

摘自 `RoboBeetleFirmware/Core/Motion/cpg_parameters.c:7-20`，14 行：

```c
void cpg_parameters_default(cpg_parameters_t *p)           // 上电默认参数的入口
{                                                         // 参数顺序由结构体固定
    *p = (cpg_parameters_t){10.0, 10.0, 2.5162, .75, 0.0, 0.0, 2.0, 0x3c}; // 前幅/后幅/T/β/F/L/w/连接
}                                                         // 不是把T默认设为2.0
cpg_parameters_result_t cpg_parameters_validate(const cpg_parameters_t *p) // 收参数先检查
{                                                         // 开始检查
    if (!p || (p->mask & 0xc0))                            // 空参数或使用未知连接位
        return CPG_PARAMETERS_INVALID_PAYLOAD;            // 拒绝坏格式
    const double v[] = {p->front_amp, p->rear_amp, p->period, p->beta, p->F, p->L, p->w}; // 七个数
    const double lo[] = {0, 0, .5, .1, -180, -180, 0}, hi[] = {28, 30, 10, .9, 180, 180, 5}; // 各自上下界
    for (unsigned i = 0; i < 7; ++i)                       // 逐个核对
        if (!isfinite(v[i]) || v[i] < lo[i] || v[i] > hi[i]) // 无穷/坏数字/越界都不收
            return CPG_PARAMETERS_OUT_OF_RANGE;           // 返回超出允许范围
    return CPG_PARAMETERS_OK;                             // 只说明软件参数合法，不说明水下效果安全
```

最后函数闭括号见原文件第 21 行；摘录为控制篇幅省略。参数结构顺序：`RoboBeetleFirmware/Core/Motion/cpg_parameters.h:8-12`。

### CPG面板每个参数是什么

字段名/上下界与 UI 顺序：`RoboBeetleConsole/src/ui/CpgParametersPanel.cpp:95-114`、`RoboBeetleConsole/src/ui/CpgParametersPanel.cpp:174-185`；默认与固件验收范围见上面摘录。

| Console字段 | 默认 / 软件允许范围 | 大白话含义 |
|---|---|---|
| Front amp (deg) | 10° / 0–28° | 前足目标摆动大小；大不是必然游得更快 |
| Rear amp (deg) | 10° / 0–30° | 后足目标摆动大小 |
| Nominal period (s) | 2.5162 / 0.5–10 s | 数学模型的名义周期参数，不是水中实测一摆的时间 |
| β | 0.75 / 0.1–0.9 | 模型中划水/回程节奏不对称的参数；不能直接当实物占空时间测量 |
| Front–rear phase (deg)，F | 0 / −180–180° | 前后振荡器的相对节拍偏移 |
| Left–right phase (deg)，L | 0 / −180–180° | 左右振荡器的相对节拍偏移 |
| Coupling strength，w | 2 / 0–5 | 把相位关系拉向设定目标的强度，不是要求所有足都同相 |
| Coupling diagram | mask=0x3C，6对连接开关 | FL–FR、RL–RR、FL–RL、FR–RR、FL–RR、FR–RL，可分别开启双向耦合 |
| Measured period (FL) | 缓存为空时显示-- | 根据前左相位绕一圈估计；停止后显示(last)，同参数重启可能暂留旧均值 |
| Applied / Apply / Reset defaults | Matches firmware / Pending / Rejected | Apply才发参数；Reset defaults只改本地编辑值，不能当作已经恢复固件 |

相位 = 周期动作现在走到一圈的哪个位置；耦合 = 不同节拍器之间互相协调；mask = 用一组二进制开关表示哪些连接/舵机参加，cdeg是百分之一度，1000 cdeg=10°。连接/偏移公式依据：`docs/task19-cpg-parameters.md:24-27`。β和名义周期不能独立当成真实周期的原因、需要重新校准的条件：`docs/cpg-gait-core.md:218-231`。

生产默认名义 2.5162 s 的历史 Host Test 校准测得约 **2.000053309 s**，不属于本次运行结果，不证明硬件时钟/水中周期。Console 测周期会丢弃第一个完整间隔、平均最近最多三个间隔；参数版本或后端变化才清均值缓存，状态变化重启相位绕圈追踪却可保留旧均值，所以STOP后同参数再RUNNING时可能先看到上一段测值。它与该 Host Test 的5 s暂态剔除方法不同。依据：`docs/cpg-gait-core.md:218-231`、`RoboBeetleConsole/src/robot/MotionStateMonitor.cpp:79-114`、`RoboBeetleConsole/src/robot/MotionStateMonitor.cpp:136-140`、`RoboBeetleConsole/src/ui/CpgParametersPanel.cpp:225-229`。

Apply 要有控制权、选 CPG、收到 schema2 与参数回读、运动 STOPPED；参数在 RAM 中，MCU 重启恢复默认。依据：`RoboBeetleConsole/src/ui/CpgParametersPanel.cpp:195-224`、`docs/task19-cpg-parameters.md:3-4`。Backend 可选 CPG / SimpleGait / Experimental Flex；Front / Rear 的 Same/Opposite 是后足划动方向协调，不等于改 F/L 相位。Backward 仍禁用：`docs/task19-cpg-parameters.md:5-6`、`docs/task19-cpg-parameters.md:64-68`、`RoboBeetleConsole/src/ui/MainWindow.cpp:2360-2363`。

### 五舵机当前语义与标定

以下以当前代码为准，**不把旧交接中的 FrontRight/FrontLeft 表搬过来**。表中三点是“标定函数定义”，不是下水前要求你实际打到所有端点；尤其前足正45°存在下述风险。

| ID / 卡片 | 作用与绑定 | 标定三点：负端 / 0 / 正端（μs） | raw PWM 命令窗（μs） |
|---|---|---|---|
| 0 FrontRight | 前右，TIM3_CH1 | −45/0/+45° → 1900 / 1450 / 1000 | 1160–1900 |
| 1 FrontLeft | 前左，TIM3_CH2 | −45/0/+45° → 1140 / 1580 / 2020 | 1140–1860 |
| 2 Depth（内部FrontAxis） | 前轴升潜，TIM3_CH3 | −90/0/+90° → 2430 / 1745 / 1060 | 1060–2430 |
| 3 RearRight | 后右，TIM4_CH1 | −45/0/+45° → 1110 / 1570 / 2030 | 1110–2030 |
| 4 RearLeft | 后左，TIM4_CH2 | −45/0/+45° → 1940 / 1450 / 960 | 960–1940 |

固件依据：`RoboBeetleFirmware/Core/Servo/servo_descriptor.c:17-110`；Console 对应及 calibrationPending=false：`RoboBeetleConsole/src/robot/ServoDescriptor.cpp:11-103`。false 表示软件描述已配置，**不是水下校准证据**。

运动公共保护窗前足 −45…+28°、后足 −30…+45°：`RoboBeetleFirmware/Core/Motion/motion_config.h:15-18`。但手动 Set Angle 根据 descriptor 的 ±45°检查后直接换算写 PWM，没有复用窄 raw PWM 窗：`RoboBeetleFirmware/Core/Servo/servo_service.c:382-393`。FR +45°→1000 μs 小于1160；FL +45°→2020 μs 大于1860。【代码推导】手动角度端点可能越过已收窄的命令窗；是否碰壳需要实物验证。**【操作建议】不要用前足 ±45°端点探索作为下水准备，记录到风险清单另行讨论。**

证据按时间和范围读：早期前足有 Bench Measured 与 Symmetry-Derived / User Accepted 两类；后足有 Bench Hardware Verified 记录。2026-09-14 SimpleGait/CPG 桌面机械动作有 Hardware Verified，水中推进/转向/升潜仍 Pending Water Verification。后续代码映射不能靠更早记录自动升级。最新README说明：2026-09-22前右PA6/CH1、前左PA7/CH2台架身份已确认，但修正后±10°逻辑方向的端到端复验仍 Pending；Depth −10°向上/+10°向下为 Bench Mechanical Verified，post-fix端到端仍 Pending。当前安装身份、方向和范围要分别核对。依据：`RoboBeetleFirmware/README.md:35-37`、`docs/engineering-lessons.md:88-92`、`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:1510-1530`、`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:1540-1564`；当前物理绑定说明：`docs/engineering-lessons.md:72`。

### ASCEND/DESCEND 与手柄比例控制

当前 ASCEND 的前轴偏置 **−1000 cdeg = −10°**，DESCEND **+1000 cdeg = +10°**；注释记录来自2026-10-05桌面方向验证。按当前 Depth 标定，−10°对应较大的PWM，+10°对应较小的PWM。这只是执行器机械指令关系，真正水中升潜效果仍 Pending Water Verification。依据：`RoboBeetleFirmware/Core/Motion/motion_config.h:11-13`、`RoboBeetleFirmware/Core/Servo/servo_descriptor.c:55-72`；桌面与水中边界：`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:1550-1560`。

手柄比例控制 = 摇杆推得越多，需求越大，而不是只选择左/右几个档。当前规则：

- 打开手柄控制、每次比例 STOP 后，必须看见四轴回中且 B 松开；等待 StartProportional ACK 时忽略输入。左Y正向为推进，右X转向，右Y升潜；左X只参加回中检查。默认 deadzone=0.15（小晃动不算输入），gamma=1.5（小输入更柔和）。依据：`docs/proportional-control.md:55-62`、`RoboBeetleConsole/src/input/ProportionalGamepadMapper.h:11-17`。
- 最新值策略 = 每层只留最新摇杆位置，覆盖旧位置，不排队补做历史动作；Start 命令有 ACK，后续比例输入没有逐条 ACK，用会话号和递增序号防旧值。依据：`docs/proportional-control.md:22-37`。
- 比例会话有效时，**四个轴全部回到deadzone以内**才触发回中STOP，B按下边沿也STOP；映射后需求恰好为零不等于四轴回中，例如左Y负向、或pitch被深度门屏蔽。零推进时只给转向不能原地转。转向通过减小一侧幅值实现，FrontAxis偏置 `−pitch_limit × pitch`，因此比例pitch正值是ASCEND方向，负值是DESCEND；不要把它与图像ey正负混为一谈。依据：`RoboBeetleConsole/src/input/ProportionalGamepadMapper.h:63-83`、`docs/proportional-control.md:41-61`。
- 深度不可用/未置零时阻止 pitch；SURFACE阻止正pitch，SOFT_FLOOR阻止负pitch；已有实际禁用pitch时STOP。硬限位关闭手柄控制。依据：`docs/proportional-control.md:64-68`、`RoboBeetleConsole/src/ui/MainWindow.cpp:4830-4878`、`RoboBeetleConsole/src/ui/MainWindow.cpp:4901-4910`。

- **拍照/录像快捷键**：X = Snapshot，Y = Start/Stop Recording；每次按下只触发一次，长按不连发，连接/重连时已按住的键要先松开。依据：`RoboBeetleConsole/src/input/GamepadCaptureMapper.h:8-27`。快捷键只在对应屏幕按钮已启用时生效，沿用同一相机可用/忙碌等检查；无需运动控制权。依据：`RoboBeetleConsole/src/ui/MainWindow.cpp:4784-4813`。

### 下水时要盯什么

Operator tools 的 Motion Control 看 Current/Backend/Front / Rear、五舵机卡片看语义身份和使能、CPG parameters 看 Applied 与 Measured period (FL)、Gamepad 看模式和回中提示；Motion CSV 看实际 `control_mode / effective_throttle / effective_turn / effective_pitch / stop_reason / cpg_param_version`。界面：`RoboBeetleConsole/src/ui/CpgParametersPanel.cpp:138-156`、`RoboBeetleConsole/src/ui/CpgParametersPanel.cpp:187-229`、`RoboBeetleConsole/src/ui/MainWindow.cpp:4731-4760`；CSV：`RoboBeetleConsole/src/robot/MotionStateCsvLogger.cpp:73-82`。

### 异常对照表

| 看到什么 | 可能原因 | 先做什么 |
|---|---|---|
| 前足碰壳/用力顶住 | 安装身份错误、方向错误、手动角度绕过窄窗【推测】 | 【操作建议】立即 Disable All、出水；不调大幅值“试一下”。依据见舵机风险段 |
| Applied一直Pending/Rejected | 未回读、参数不合法、后端或运动状态不满足 | 【操作建议】STOP，确认控制权/CPG/版本后再处理。`RoboBeetleConsole/src/ui/CpgParametersPanel.cpp:203-224` |
| Nominal2.5162，Measured约2 | 校准后的预期区别；不是仅凭数字就判故障 | 【操作建议】保持小幅安全动作并记录实测；若伴随异常先STOP。`docs/cpg-gait-core.md:223-231` |
| 手柄连接后推杆不动 | 尚未回中、Start ACK待确认、无控制权或深度阻止pitch | 【操作建议】先STOP/撤防，再回中核对状态，不反复猛推。`docs/proportional-control.md:55-68` |

自测：5.1 前左右PWM变化方向相反，一定代表逻辑动作相反吗？5.2 CPG允许前幅28°，是否说明手动前足+45°也被同一限位保护？

## 6. 视觉自动跟随：看见目标、认定同一个目标、允许动作是三道门

### 在机器人身上起什么作用

Console 接收外部 FOMO 的检测结果，然后选择/关联目标、算画面误差、形成建议，最后由布防状态机决定能不能发送。TargetAssociation = 尽量连续认住同一个目标的选择器；EMA = 新值只占一部分、保留一部分旧值的平滑方法；dwell = 切换动作前要满足的等待时间。像看着同一条鱼掌舵：先认鱼，再判断偏左偏右，最后确认自己有驾驶权。接线：`RoboBeetleConsole/src/ui/MainWindow.cpp:3747-3751`；检测字段解码：`RoboBeetleConsole/src/vision/DetectionStreamDecoder.cpp:79-104`、`RoboBeetleConsole/src/vision/DetectionStreamDecoder.cpp:140-178`。本课不评价检测器训练、精度或阈值。

```text
外部FOMO检测 → Console DetectionClient → TargetAssociation
                                      → 原图u/v → ex/ey → EMA/滞回/dwell
                                      → VisualDispatchStateMachine（Discrete）
                                        或比例执行状态机（Proportional/Continuous）
                                      → Visual dispatch开关 + Arm + 控制/姿态/深度门
                                      → Pi → STM32 → 舵机
```

当前执行器选择：`RoboBeetleConsole/src/vision/VisualDispatchSession.cpp:452-455`。

### 关键代码在做什么

摘自 `RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:149-156`，8 行：

```cpp
} else if (input.state == VisualState::NoTarget ||            // 短暂没有可用目标
           (input.state == VisualState::Tracking && input.suggestion == ProposedCommand::Hold)) { // 或策略要求保持
    // A transient miss is HOLD, not LOST. Never assume an old mode // 缺一会儿先保持，不立即当成长期丢失
    // after arming, manual takeover or link-loss invalidation. // 重新布防/接管后不能沿用失效确认
    if (!modeConfirmed_) beginStop(input.nowMs);             // 没有ACK确认过的模式就STOP；有则保留运动
} else if (input.state == VisualState::Lost || input.suggestion == ProposedCommand::Stop) { // 长期丢失或STOP建议
    beginStop(input.nowMs);                                 // 开始STOP过程，这里仍保留布防
}                                                          // 后面还会继续查深度和待确认超时
```

**NO_TARGET 不等于停住**，这8行是下水时最值得记住的视觉代码。深度/控制权/姿态失效、待确认请求超时具有更高优先级，仍可提前停止：`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:142-176`。

### 从像素到建议

`ex=(u−W/2)/(W/2)`，`ey=(v−H/2)/(H/2)`：画面右方、下方为正，无量纲；是原图坐标而非缩放后窗口坐标。依据：`RoboBeetleConsole/src/vision/TargetState.cpp:7-17`。

| 门/参数 | 当前规则 | 依据 |
|---|---|---|
| 首次获取 | 未锁时选最高置信度；锁后选距离关联门中心最近者，等距以置信度决胜 | `RoboBeetleConsole/src/vision/TargetAssociation.cpp:63-100` |
| 关联门 | 原图到关联门中心距离≤80 px可关联；默认不要求同类别。有同一偏航参考时，中心从锁点水平平移 `yaw_ex_per_deg·W/2·Δyaw`；默认0.05，在640 px宽图中为16 px/° | `RoboBeetleConsole/src/vision/VisualPolicyConfig.h:40-49`；`RoboBeetleConsole/src/vision/TargetAssociation.cpp:63-100` |
| 关联MISS | 第一次关联失败起，普通锁≥500 ms解锁；锁带偏航参考且补偿开启时≥1200 ms解锁；下一新帧才重新获取 | `RoboBeetleConsole/src/vision/TargetAssociation.cpp:14-18`、`RoboBeetleConsole/src/vision/TargetAssociation.cpp:28-38`、`RoboBeetleConsole/src/vision/TargetAssociation.cpp:102-106` |
| EMA | alpha=0.3；有完整偏航参考时先稳定方位再滤波（见下段），无陀螺时沿用图像误差EMA；新有效帧更新一次，首样本直接初始化，50 ms定时器不会重滤同一帧 | `RoboBeetleConsole/src/vision/VisualCommandPolicy.cpp:126-144`；`RoboBeetleConsole/src/vision/VisualPolicyConfig.h:19-32` |
| 滞回 | 误差绝对值>0.25进入转向/升潜，<0.12退出；等于边界不触发该跨越 | `RoboBeetleConsole/src/vision/VisualCommandPolicy.cpp:163-185` |
| 建议切换等待 | min_dwell_ms=1000；安全STOP绕过等待 | `RoboBeetleConsole/src/vision/VisualCommandPolicy.cpp:95-109`、`RoboBeetleConsole/src/vision/VisualCommandPolicy.cpp:199-209` |
| 检测冻结STALE | 本地最近递增frame_id到达后≥500 ms没有推进；不是只看HTTP是否有包 | `RoboBeetleConsole/src/vision/VisualTargetStateMachine.cpp:33-51` |
| 目标持续缺失LOST | 第一次没有可用关联目标起≥1500 ms；目标恢复会清计时，进入STALE/OFF等失效状态也终止这次连续计时 | `RoboBeetleConsole/src/vision/VisualTargetStateMachine.cpp:19-25`、`RoboBeetleConsole/src/vision/VisualTargetStateMachine.cpp:33-38`、`RoboBeetleConsole/src/vision/VisualTargetStateMachine.cpp:77-88`；`RoboBeetleConsole/src/vision/VisualPolicyConfig.h:23-24` |

三个不同的钟：**关联MISS普通500 ms / 带偏航参考1200 ms、数据冻结STALE 500 ms、持续无目标LOST 1500 ms**。检测流完全冻结通常先STALE；不断递增的有效检测帧持续没有可用关联目标，才可能NO_TARGET到LOST，非空检测也可能关联失败。依据：`RoboBeetleConsole/src/vision/VisualDiagnosticSession.cpp:104-112`、`RoboBeetleConsole/src/vision/VisualTargetStateMachine.cpp:33-88`。不要把它们相加当成统一停机时间。关联锁保持时限：`RoboBeetleConsole/src/vision/TargetAssociation.cpp:34-38`、`RoboBeetleConsole/src/vision/VisualPolicyConfig.h:40-49`。

### 陀螺偏航摇摆补偿：去摇摆，保留持续转向

像先扣掉相机随划水左右摇头的影响，再判断鱼偏在哪边。`MotionStateMonitor` 将有效 `gyro_z` 送入 `YawIntegrator`，按MCU采样时间积分，再由 `yawAtCapture` 按每帧Pi捕获时间查询航向；平均窗口采用已测步态周期，无测值时用2.5 s。依据：`RoboBeetleConsole/src/robot/MotionStateMonitor.cpp:116-134`、`RoboBeetleConsole/src/robot/YawIntegrator.cpp:13-46`、`RoboBeetleConsole/src/vision/VisualDiagnosticSession.cpp:67-75`。

`ex_f` 现在先对偏航稳定后的方位做EMA，再相对一个步态周期的均值航向报告；均值还加上该周期净转角的一半，目的是去掉周期摇摆、保留持续转弯，不能把它当成实物纠偏已验证。依据：`RoboBeetleConsole/src/robot/YawIntegrator.h:7-18`、`RoboBeetleConsole/src/robot/YawIntegrator.cpp:49-69`、`RoboBeetleConsole/src/vision/VisualDiagnosticSession.cpp:146-153`、`RoboBeetleConsole/src/vision/VisualCommandPolicy.cpp:76-92`、`RoboBeetleConsole/src/vision/VisualCommandPolicy.cpp:126-152`。

NO_TARGET时保持的是方位估计；随着新捕获帧对应的陀螺航向更新，`ex_f` 仍会跟着变化，Continuous 会据此继续调整转向需求，使机体转过来时转向逐渐减弱。没有陀螺参考时，图像误差滤波与短缺失保持行为沿用原规则。依据：`RoboBeetleConsole/src/vision/VisualDiagnosticSession.cpp:71-72`、`RoboBeetleConsole/src/vision/VisualDiagnosticSession.cpp:146-153`、`RoboBeetleConsole/src/vision/VisualCommandPolicy.cpp:76-109`、`RoboBeetleConsole/src/vision/VisualCommandPolicy.cpp:126-138`、`RoboBeetleConsole/src/vision/VisualProportionalDispatchStateMachine.cpp:362-392`。

【操作建议】依赖补偿前，先确认视觉CSV的 `yaw_sway_deg` 已有数值；均值需约一个步态周期的陀螺历史，不要把空值当成零摇摆或补偿生效。依据：`RoboBeetleConsole/src/robot/YawIntegrator.h:10-13`、`RoboBeetleConsole/src/robot/YawIntegrator.cpp:49-57`、`RoboBeetleConsole/src/vision/VisualDiagnosticSession.cpp:128-129`、`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:199-202`。

### 显示状态与实际动作

| 屏幕/CSV状态 | 它表示什么 | 布防后的动作 |
|---|---|---|
| LOCK | 视频标出的被选中目标；不是“已布防” | 本身不直接发运动 |
| ACQUIRED | 无锁后新取得目标 | 本身不自动Arm；随后满足门才可能动作 |
| ASSOCIATED | 新帧继续关联成功 | 更新可用目标/滤波 |
| MISS | 新帧没找到符合关联门的目标；可以同时还有其它检测 | 不等于全图没有鱼，通常进入NO_TARGET处理 |
| NO_TARGET | 尚在缺失宽限 | Discrete保留ACK确认模式与布防，不下发新模式；没确认模式则STOP |
| LOST | 连续缺失达到≥1500 ms | STOP但保留布防；重新获得目标并满足门后可能恢复运动 |
| STALE | 检测冻结或等待视频超时等 | STOP并撤防；恢复数据后须人工重新Arm |
| INFERENCE_OFF | 推理未有效运行 | STOP并撤防 |

标记/关联依据：`RoboBeetleConsole/src/vision/VideoView.cpp:295-300`、`RoboBeetleConsole/src/vision/TargetAssociation.cpp:93-106`、`RoboBeetleConsole/src/vision/VisualDiagnosticSession.cpp:104-112`。停止/保持依据：`RoboBeetleConsole/src/vision/VisualCommandPolicy.cpp:50-109`、`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:114-117`、`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:142-155`。

**lost_ms的来历与待办**：代码现在设1500 ms且参数标为临时；交接明确要求统计连续 NO_TARGET 时长分布作为依据，尚未完成。它不是水下统计得出的最优宽限。依据：`RoboBeetleConsole/src/vision/VisualPolicyConfig.h:18-24`、`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:89-90`。缺失期仍可能保持推进/转向/升潜；第8课要同时统计“NO_TARGET状态段”和“包括LOST的完整目标缺失期”，不能把1500 ms封顶后的图误认为自然缺失分布。

### 当前方向确认机制：旧版要求已不存在

旧交接记录每次会话选±1再确认方向；**当前应用**直接把 confirmedSign 初始化为 configuredTurnSign=+1，pitch_sign也固定+1。页脚是 `Turn 1 (config)`，测试源码还明确检查旧选择框与 Confirm 按钮不存在。依据：`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:74-76`、`RoboBeetleConsole/src/vision/VisualDispatchSession.h:130`、`RoboBeetleConsole/src/vision/VisualPolicyConfig.h:26-35`、`RoboBeetleConsole/src/ui/MainWindow.cpp:1640-1646`、`RoboBeetleConsole/tests/main_window_visual_dispatch_tests.cpp:249-253`。测试这里只作为静态契约阅读，未重跑。

当前默认右侧目标→TurnRight，下方目标→Descend；这是代码符号关系，**水下实际纠偏方向未由本次确认**。隐藏状态中的 `1 confirmed` 表示配置值，不能当成你亲自确认。依据：`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:182-188`、`RoboBeetleConsole/src/vision/VisualCommandPolicy.cpp:26-29`、`RoboBeetleConsole/src/ui/MainWindow.cpp:3270-3277`。水下方向尚未确认意味着可能越纠越偏；【操作建议】先手动验证实物/画面响应，方向不符就STOP、撤防，暂缓真实视觉输出，另开任务讨论符号。不要寻找当前已不存在的确认按钮。

### 轴模式、布防与不同执行方式

Arm = 布防，即明确允许自动控制；READY只表示条件准备好。Discrete 需要远程连接、控制权、所需舵机已使能且姿态已知、TRACKING、有效配置；Yaw用四个划水舵机mask=0x001B，Pitch/Both再加FrontAxis mask=0x0004。依据：`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:10-33`、`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.h:24-46`。

| 选项 | 当前行为/特别门 |
|---|---|
| Yaw | 用水平误差；不要求深度可用，不因深度HardLimit自动获得全局停止保护 |
| Pitch | 用垂直误差；需FrontAxis及新鲜、已Zero深度 |
| Both，Discrete | 水平转向优先，没有转向建议时才采用垂直建议；不是同时转向和升潜 |
| Both，Continuous | 可以同时映射turn和pitch；仍受深度门保护 |

依据：`RoboBeetleConsole/src/vision/VisualCommandPolicy.cpp:188-195`、`RoboBeetleConsole/src/vision/VisualApproachPolicy.cpp:88-105`、`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:24-30`。Pitch/Both深度失效/未Zero/HardLimit拒绝Arm，运行中STOP撤防；软/表面限位按第3课处理。切换轴会撤防，保护中的运动/未确认STOP会发一次操作员STOP：`RoboBeetleConsole/src/vision/VisualDispatchSession.cpp:137-145`、`RoboBeetleConsole/src/vision/VisualDispatchSession.cpp:225-239`。

当前 Exec 的 Discrete / Proportional / Continuous 会通过QSettings `Visual/ExecutionMode/v1` 跨Console重启记忆；未保存时默认Discrete。Dry run 永不记忆，重新启动时为false；Arm仍须手动操作，恢复Exec不等于恢复布防。依据：`RoboBeetleConsole/src/ui/MainWindow.cpp:1572-1588`、`RoboBeetleConsole/src/ui/MainWindow.cpp:3459-3461`、`RoboBeetleConsole/src/vision/VisualProportionalDispatchStateMachine.h:19-21`、`RoboBeetleConsole/src/ui/MainWindow.cpp:1656-1663`。

三种方式不能套用同一套日志解释：

- **Discrete**：发FORWARD/TURN/ASCEND等模式命令，等待ACK。非STOP要同时距离上次非STOP发送与最近STOP接受各≥1000 ms；只有OK建立确认模式，BUSY不立刻重发，其它拒绝/未知结果进入安全停止。依据：`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:173-205`、`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:230-243`。
- **Proportional**：沿离散建议选方向，再用目标面积控制推进需求；**Continuous**：用ex_f/ey_f连续映射turn/pitch，不受离散方向切换的1000 ms dwell。二者共用比例执行器，真实Arm要求五舵机、schema2/feature≥2、已停止、通道空闲和有效目标面积。依据：`RoboBeetleConsole/src/vision/VisualProportionalDispatchStateMachine.cpp:37-74`、`RoboBeetleConsole/src/vision/VisualProportionalDispatchStateMachine.cpp:362-392`、`RoboBeetleConsole/src/vision/VisualApproachPolicy.cpp:88-105`。
- 面积代理默认slow/resume/stop=8/14/20 cells，面积EMA=0.3，到达/恢复dwell各1000 ms；第一份有效面积已≥stop阈值会立即ARRIVED。`cells`是检测热图格子数，**不是米**；面积与接近程度关系需水下验证，配置还可能从QSettings恢复。依据：`RoboBeetleConsole/src/vision/VisualApproachPolicy.h:12-24`、`RoboBeetleConsole/src/vision/VisualApproachPolicy.cpp:148-175`、`RoboBeetleConsole/src/ui/MainWindow.cpp:3178-3180`、`RoboBeetleConsole/src/ui/MainWindow.cpp:3442-3458`。
- Proportional在NO_TARGET宽限内持续刷新此前完整setpoint；Continuous在有保持的ex_f与面积时会重新映射转向，有陀螺参考时继续减弱转向需求；LOST停止且保留布防，STALE/OFF停止并撤防。目标还在但面积连续缺失>500 ms会另触发AREA_MISSING_LOST。依据：`RoboBeetleConsole/src/vision/VisualProportionalDispatchStateMachine.cpp:269-305`、`RoboBeetleConsole/src/vision/VisualProportionalDispatchStateMachine.cpp:343-392`、`RoboBeetleConsole/src/vision/VisualApproachPolicy.cpp:130-144`。

**DRY_RUN要认准**：不勾 `Visual dispatch` 时是诊断建议；Proportional/Continuous另有 `Dry run` 复选框，可模拟接近过程而不发送真实控制。Discrete没有这个比例模拟复选框的作用；记录CSV本身不自动禁用已布防的输出。【操作建议】初次诊断用功能关闭、机器人STOP且不Arm的路线；要模拟APPROACH/ARRIVED，则先STOP并撤防→选Proportional或Continuous→勾Dry run→打开Visual dispatch→满足目标/面积及对应深度门→点击Arm。这里Arm的是模拟执行器，start/stream/stop不调用真实控制；SIMULATED不能当成真实ACK或水下执行证据。界面/执行门：`RoboBeetleConsole/src/ui/MainWindow.cpp:1492-1520`、`RoboBeetleConsole/src/ui/MainWindow.cpp:1572-1588`、`RoboBeetleConsole/src/vision/VisualDispatchSession.cpp:211-222`、`RoboBeetleConsole/src/vision/VisualDispatchSession.cpp:424-430`、`RoboBeetleConsole/src/vision/VisualProportionalDispatchStateMachine.cpp:37-74`、`RoboBeetleConsole/src/vision/VisualProportionalDispatchStateMachine.cpp:114-121`、`RoboBeetleConsole/src/vision/VisualProportionalDispatchStateMachine.cpp:167-185`。

### 下水时要盯什么

Auto Follow 看 `Visual dispatch / Exec / Dry run / Yaw-Pitch-Both / Arm-Disarm`、状态pill和缺项清单；视频诊断看 state、LOCK/MISS、ex_f/ey_f、PROPOSED、retained/HOLD、dwell；CSV看实际 dispatch/outcome与深度门。界面出处：`RoboBeetleConsole/src/ui/MainWindow.cpp:1492-1619`、`RoboBeetleConsole/src/ui/MainWindow.cpp:3197-3209`、`RoboBeetleConsole/src/vision/VideoView.cpp:104-154`。

### 异常对照表

| 看到什么 | 可能原因 | 先做什么 |
|---|---|---|
| 无目标但仍在游 | NO_TARGET保留确认模式或比例需求；Continuous可随陀螺航向调整转向 | 【操作建议】若接近危险区域立即STOP/撤防，不能等LOST。`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:149-156`、`RoboBeetleConsole/src/vision/VisualProportionalDispatchStateMachine.cpp:362-392` |
| 已LOST停住，随后又开始 | LOST仍保留布防，重获目标后可以恢复 | 【操作建议】想保持停机就明确Disarm。`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:154-179` |
| STALE后检测恢复却不动 | 已安全撤防，符合代码 | 【操作建议】保持停止，查冻结/视频等待原因，再决定是否重新Arm。`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:142-148` |
| 越跟越偏/越潜越偏 | 【推测】固定符号与水中响应不符，或目标切换 | 【操作建议】立即STOP、撤防、出水；保存误差和动作方向。符号依据见本课方向段 |
| n_detections>0仍MISS | 最近候选到关联门中心的距离超过80 px；有同一偏航参考时中心已按陀螺预测平移，其他检测不能替代当前锁 | 【操作建议】先STOP/撤防，再看原图位置、assoc_dist_px及assoc_shift_px。`RoboBeetleConsole/src/vision/TargetAssociation.cpp:63-106` |

自测：6.1 新检测帧持续到来，但已确认FORWARD的目标缺失900 ms，会仅因此停止吗？6.2 Both中水平和垂直误差都很大，会同时转向下潜吗？

## 7. 通信与延迟：仪表必须先认清“测的是哪一段”

### 在机器人身上起什么作用

Pi管理唯一控制权、心跳、命令结果和断链；RBRP = Console与Pi之间的消息协议，携带控制权和命令身份。CRC = 用来发现数据在传输中被改坏的校验数；RTT = 从发请求到收到对应结果的往返用时。ACK RTT短只能说明某条请求结果回来快，不能说明电机机械响应同样快。控制权/撤销代码：`RoboBeetlePi/gateway/src/control_gateway_core.cpp:807-834`。

```text
Qt/RBRP → TCP → Pi控制网关 → Protocol V2 → /dev/serial0
                                 → USART2 PA3(RX)/PA2(TX) → STM32
Qt监视的远程RTT ← CommandOutcome ← Pi ← STM32 ACK / Pi终态处理
```

当前主机绑定为USART2，115200、8数据位、无校验、1停止位、无流控（8N1）；Pi TX→STM32 PA3，Pi RX←STM32 PA2，共地。依据：`RoboBeetleFirmware/Core/Src/main.c:109-116`、`RoboBeetleFirmware/Core/Src/main.c:336-343`、`RoboBeetleFirmware/Core/Src/stm32f4xx_hal_msp.c:254-262`、`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:1764-1766`。USART1/APC220保留初始化，但不是当前备用控制权链路：`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:1803-1808`。

CommandOutcome也可以是Pi撤权时生成的OutcomeUnknown，而非收到STM32 ACK；不能只看到终态事件就算执行成功。依据：`RoboBeetlePi/gateway/src/control_gateway_core.cpp:793-801`、`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:805-815`。

### 关键代码在做什么

两段真实摘录，共12行，分别来自 `RoboBeetleConsole/src/remote/RemoteRobotController.cpp:805-808` 与 `RoboBeetleConsole/src/ui/MainWindow.cpp:3912-3919`：

```cpp
const PendingCommand pending = *it;                         // 保存已匹配请求及原发送时刻
const qint64 rtt = terminalNow_() - pending.terminalSentMs;  // 从请求发送到终态结果到达的时间差
monitor_.lastAckRttMs = rtt >= 0 ? rtt : -1;                 // 不合法时间差标为不可用
pending_.erase(it);                                        // 移除完成的待决请求
txCount_->setText(QString::number(monitor.txPacketCount));  // 显示发送帧累计数
rxCount_->setText(QString::number(monitor.rxPacketCount));  // 显示接收帧累计数
crcCount_->setText(QString::number(monitor.crcErrorCount)); // 只显示控制器交来的CRC数
timeoutCount_->setText(QString::number(monitor.timeoutCount)); // 显示该模式定义的超时数
ackRtt_->setText(monitor.lastAckRttMs < 0                    // 有没有可用往返测量
                    ? QStringLiteral("—")                  // 没有就显示横线
                    : QStringLiteral("%1 ms").arg(monitor.lastAckRttMs)); // 最新一次，不是平均值
ackStatus_->setText(monitor.ackStatus);                     // 结果文字要与数值一起看
```

### 时限与正常范围

| 边界/指标 | 当前代码值或正确观察口径 | 依据 |
|---|---|---|
| Qt→Pi control heartbeat / lease | 250 ms / 1000 ms；这不是STM32心跳周期 | `RoboBeetleConsole/src/remote/RbrpClientSession.cpp:363-381`；`RoboBeetlePi/gateway/src/control_gateway_core.cpp:12` |
| Pi→STM32 Heartbeat / ACK timeout / liveness | 默认100 / 200 / 450 ms；liveness是Pi自己的链路存活判断 | `RoboBeetlePi/link_core/include/robobeetle/link_core/link_config.hpp:9-11` |
| STM32最后有效Heartbeat | 严格>500 ms触发安全停止 | `RoboBeetleFirmware/Core/Safety/safety_supervisor.c:35-43` |
| Remote ACK RTT | Qt发送到匹配CommandOutcome终态的耗时，包括网络/Pi/串口/调度；不是纯USART2 RTT | `RoboBeetleConsole/src/remote/RemoteRobotController.cpp:617-618`、`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:794-807` |
| Remote Timeout | 包含RBRP协议错误以及2000 ms等不到命令终态 | `RoboBeetleConsole/src/remote/RemoteRobotController.cpp:15`、`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:161-165`、`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1326-1339` |
| Remote CRC | 字段默认0，当前远程控制器没有把Pi串口CRC计数桥接进来 | `RoboBeetleConsole/src/controller/IConsoleController.h:39-45`；`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1426-1451` |

**【未在代码/现有验收记录中找到】当前HEAD正式的ACK RTT“正常区间”。** 2026-10-04桌面子集有STOP RTT100/91 ms、8次OK非STOP中位132 ms、另一次STOP173 ms，只能当历史Bench Measured样本，不可拼成当前91–173 ms正常范围。依据：`docs/task05-visual-dispatch-wiring-validation.md:62-68`。USART2心跳smoke记录89次OK、0 timeout、0 decode error，未报告RTT：`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:1768-1785`。历史APC220的160–173 ms属于不同链路：`docs/engineering-lessons.md:42-56`。

【操作建议】固定本次配置，先记录离水稳定基线，再看水边/入水后的变化；关注Timeout及错误**增量**、结果类别、RTT长尾，不给没有实测依据的硬性“正常毫秒范围”。Remote CRC=0不能作为USART2零错帧结论；需要后续Pi端对应诊断证据。

### TCP单客户端与完整断线链

网关在 `client_fd<0` 时才接纳连接，已有客户端则关闭新连接：`RoboBeetlePi/gateway/linux/src/tcp_adapter.cpp:360-377`。当前连接设置主要见listener的SO_REUSEADDR，相关网关/Qt远程代码未找到TCP keepalive开启；已有交接明确该限制：`RoboBeetlePi/gateway/linux/src/tcp_adapter.cpp:244-250`、`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:31-33`。

TCP EOF/读错误→source_lost→revoke→abort_once→关闭串口/停心跳，不是发送普通STOP：`RoboBeetlePi/gateway/linux/src/tcp_adapter.cpp:421-431`、`RoboBeetlePi/gateway/linux/src/tcp_adapter.cpp:334-337`、`RoboBeetlePi/gateway/src/control_gateway_core.cpp:703-704`、`RoboBeetlePi/gateway/src/control_gateway_core.cpp:807-834`、`RoboBeetlePi/gateway/linux/src/linux_onboard_application_port.cpp:344-346`、`RoboBeetlePi/application/src/onboard_application.cpp:88-90`、`RoboBeetlePi/runtime/src/link_runtime.cpp:65-67`、`RoboBeetlePi/session/src/serial_session.cpp:131-178`。完整601/1601 ms名义预算与局限见第1课。

【操作建议】水边遇到 `remote host closed`，先确保机器人停机/出水，再关闭旧Console；仍失败才按交接由负责人重启Pi网关服务。不要在机器人仍可能动作时把反复重连当作处置。此处仅写操作顺序，本次未重启服务。

### 视频卡顿：记录现象，不抢着定根因

AwaitingVideo = 检测捕获时间领先于已显示视频；视频时间追到或超过该检测时刻即可，不要求时间戳完全相同。它在宽限内保留先前状态/建议，等待达到≥500 ms就STALE；独立于TCP控制链是否仍活着。依据：`RoboBeetleConsole/src/vision/DetectionMetadata.cpp:24-29`、`RoboBeetleConsole/src/vision/VisualTargetStateMachine.cpp:55-70`、`RoboBeetleConsole/src/vision/VisualDiagnosticSession.cpp:140-155`。

视频等待/卡顿根因未确认，旧验收仅记现象：`docs/target-temporal-association-dry-run-validation.md:142`。**【操作建议】先STOP/撤防，再记录**：

1. Console时刻、动作阶段、实物是否仍摆动、画面是冻结还是间歇跳帧。
2. Vision Details→Video diagnostics的RX FPS、Display paint-event FPS、Latest frame ID、Wire frames、RX replaced、JPEG decode drops、UI replaced；同时记Protocol RTT/Timeout/控制权。面板字段：`RoboBeetleConsole/src/ui/MainWindow.cpp:2766-2771`、`RoboBeetleConsole/src/ui/MainWindow.cpp:3885-3903`。
3. 保留CSV中的frame_id、capture_ts_ns、arrival_mono_ms、local_mono_ms、awaiting_video、state、dispatch/outcome。只在相同时间域比较，不能直接用Windows时间减Pi时间当网络延迟。表头依据：`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:11`。

### 下水时要盯什么

Dashboard **Protocol** 卡看TX/RX/CRC/Timeout/ACK RTT；Operator tools→**Protocol Details** 看ACK status、TX Hex、RX Hex和同组计数；Vision Details看视频诊断。对应位置：`RoboBeetleConsole/src/ui/MainWindow.cpp:1865-1888`、`RoboBeetleConsole/src/ui/MainWindow.cpp:2679-2696`、`RoboBeetleConsole/src/ui/MainWindow.cpp:2973-2977`。

### 异常对照表

| 看到什么 | 可能原因 | 先做什么 |
|---|---|---|
| Timeout持续增加、OUTCOME_UNKNOWN | 命令结果未确认；【推测】可能控制/串口/网络问题，现有数值不能定位原因 | 【操作建议】STOP/Disable All、撤防并回收；不要放宽timeout掩盖。代码见本课指标表 |
| 画面卡但TX/RX还涨 | 视频与控制分别运行，不能凭控制活着推视频新鲜 | 【操作建议】STOP、撤防，记录Video diagnostics和awaiting_video。代码见本课视频段 |
| CRC显示0但动作/遥测异常 | 远程CRC字段不包含串口错误证据 | 【操作建议】停止、回收，保存Pi/Qt对应日志供后续定位。代码见本课指标表 |
| 被拒连/remote host closed | 旧半开客户端占槽【推测】 | 【操作建议】先出水，再关旧Console、按交接恢复服务。`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:31-33` |

自测：7.1 远程ACK RTT=132 ms，是否说明USART2往返恰好132 ms？7.2 远程CRC=0能证明水边串口没有错帧吗？

## 8. 下水后看日志：先认行类型，再做统计

### 在机器人身上起什么作用

日志是事后回放的“行车记录本”，让你对照当时看见的目标、建议、实际下发和收到的结果。它本身不是安全保护，也不证明动作已经完成。当前视觉文件59列、运动状态文件64列；用**字段名**找列，避免不同版本错位。实际表头：`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:11`、`RoboBeetleConsole/src/robot/MotionStateCsvLogger.cpp:9-19`。

```text
检测/状态评估 → frame、transition ─┐
命令提交/结果 → dispatch、outcome ├→ visual-...csv
比例接近事件 → event ─────────────┘
STM32 Gyro触发采样 → Pi时间映射 → sample ─┐
参数/配置/丢失汇总 → 事件行 ────────────┴→ motion_state_...csv
```

### 关键代码在做什么

摘自 `RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:176-187`，12行：

```cpp
if (depthInfo_) {                                           // 只有装了深度信息来源才读取
    const auto depth = depthInfo_();                        // 取此次写行时的控制深度信息
    fields << (depth.rawM ? number(*depth.rawM) : QString{}) // 原始米值；不存在就空白
           << (depth.calibratedM ? number(*depth.calibratedM) : QString{}) // Zero后的米值
           << (depth.ageMs ? QString::number(*depth.ageMs) : QString{}) // 样本年龄ms
           << depth.envelope;                              // 写深度区域名称
} else {                                                   // 没有深度来源
    for (int i = 0; i < 4; ++i) fields << QString{};         // 四列空白，不伪造测量
}                                                          // 深度部分结束
// v5 addition: selected target's component area, frame rows only. // v5新增的是选中目标的面积
fields << (!transition && !s.awaitingVideo && s.target && s.target->target.areaCells // 必须可用的真实帧目标
               ? QString::number(*s.target->target.areaCells) : QString{}); // 面积不存在就留空，不能当0
```

### 从v5读到当前v8：最重要的视觉列

以下前43列涵盖请求中的v5；当前v8保留并扩展。除注明外，顺序/写入依据统一为 `RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:11`、`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:121-198`。

| 字段 | 单位/大白话 | 分析时要留意 |
|---|---|---|
| row_kind | 行类型 | frame才是检测帧；transition是状态变化；dispatch/outcome是命令事件；event是比例事件 |
| local_mono_ms | 本地单调时间，ms | 这一行的评估/事件时间，不是墙上日期时间 |
| arrival_mono_ms | 检测首次本地到达时间，ms | 与local_mono差可看本地等待；不能当纯网络延迟 |
| frame_id / capture_ts_ns | 帧身份 / Pi捕获时间ns | session内帧去重；Pi时间先除1,000,000才是ms |
| n_detections / sel_class / sel_conf | 检测数量/选中类别/0–1置信度 | 检测多不等于关联成功，置信度不等于正确率 |
| u / v | 选中目标原图像素 | 和src_w/src_h一起理解，别用显示窗口尺寸 |
| ex / ey | 无量纲原始误差，右/下为正 | 公式见第6课 |
| ex_f / ey_f | 平滑误差，无量纲；ex_f有完整偏航参考时为去摇摆后的误差 | 缺目标期ex_f可随陀螺航向更新，ey_f保持；不是每行都代表新目标。`RoboBeetleConsole/src/vision/VisualCommandPolicy.cpp:76-109`、`RoboBeetleConsole/src/vision/VisualCommandPolicy.cpp:126-153` |
| yaw_cmd | 策略建议的归一化偏航量 | 不是舵机角度、真实yaw或ACK模式 |
| state | TRACKING / NO_TARGET / LOST / STALE / INFERENCE_OFF | 和assoc_status、dispatch_result分别看 |
| proposed_command / effective_command | 新建议 / 策略当前有效建议 | effective不等于STM32已ACK确认的运动模式 |
| assoc_status / assoc_dist_px | 关联状态 / 到关联门中心的距离px | ACQUIRED/ASSOCIATED/MISS/UNLOCKED；有同一偏航参考时中心已平移，MISS可能仍有其它检测。`RoboBeetleConsole/src/vision/TargetAssociation.cpp:63-104` |
| hc_u / hc_v / hc_conf | 最高置信度候选位置/置信度 | 与关联选中的u/v比较跳变，缺失分母可能不同 |
| src_w / src_h / dets | 原图尺寸px / 全部检测编码 | dets按classId:confidence:u:v:area，用分号分隔目标 |
| policy_version / policy_hash / schema_version | 策略与参数指纹/表格版本 | 参数不同的段不混算；当前策略为visual-command-proposal-v5，schema为visual-csv-v8。`RoboBeetleConsole/src/vision/VisualPolicyConfig.h:50`、`RoboBeetleConsole/src/vision/VisualPolicyConfig.h:63` |
| session_id / awaiting_video | 诊断会话数字 / 0或1等待视频 | 断会话分段；awaiting_video=1不是又一帧控制更新 |
| policy_request_id / request_id | 策略请求号 / 实际请求号 | 命令发送和结果用request_id对应；不能按相邻行配对 |
| dispatch_command / dispatch_result | 真正尝试的命令 / SENT、OK、BUSY等结果 | SENT仅提交；OUTCOME_UNKNOWN表示结果未知 |
| ack_rtt_ms | 请求到终结结果的耗时ms | 必须结合row_kind和结果；未知结果也可能有数值 |
| axis_mode / pitch_sign | YAW/PITCH/BOTH / 配置符号 | 当前pitch_sign固定+1，不能作人工确认凭证 |
| depth_raw_m / depth_cal_m | 原始米值 / Zero后米值 | 空值不是0；depth失效时raw有可能写0，结合门状态/年龄看 |
| depth_age_ms / envelope_state | 控制样本年龄ms / 深度区域 | 超1300不能用于升潜；未勾Visual dispatch时envelope可能是此前评估缓存 |
| sel_area_cells | 选中检测component格子数 | 只在有效frame目标写，缺失留空；不是像素面积、不是米 |

行类型写入/空白依据：`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:121-134`、`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:224-245`、`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:251-305`。v5面积说明：`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:185-187`。深度raw=0的失效处理：`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1004-1012`；envelope缓存与当前深度分别取值：`RoboBeetleConsole/src/vision/VisualDispatchSession.cpp:148-157`、`RoboBeetleConsole/src/vision/VisualDispatchSession.cpp:305-309`。

当前v8还要认识（保留v6/v7字段，末尾追加两列）：

| 新增字段 | 含义和单位 | 依据 |
|---|---|---|
| gyro_z / roll / pitch / gait_phase / motion_age_ms | 与Pi捕获时刻关联的最近过去运动样本；°/s、°、°、rad、ms | `RoboBeetleConsole/src/ui/MainWindow.cpp:3374-3384` |
| exec_mode / dry_run | PROPORTIONAL或CONTINUOUS / 0、1；Discrete比例字段可留空 | `RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:22-30` |
| area_f / approach_state | 平滑面积cells / 接近状态，例如APPROACH、ARRIVED | 同上；`RoboBeetleConsole/src/vision/VisualApproachPolicy.h:12-24` |
| throttle / turn / pitch_setpoint | 协议原始整数：0…1000、−1000…1000、−1000…1000 | `RoboBeetlePi/protocol/include/robobeetle/protocol/proportional_control.hpp:26-40` |
| actually_submitted / approach_event | 0、1提交标记 / CONFIG、ARRIVED等事件 | `RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:22-30`、`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:207-219` |
| yaw_sway_deg（v8，第58列） | 最新捕获时刻的积分陀螺航向减均值航向，单位°；需完整周期历史才有均值，非frame行留空 | `RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:11`、`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:199-202`、`RoboBeetleConsole/src/vision/VisualDiagnosticSession.cpp:128-129`、`RoboBeetleConsole/src/robot/YawIntegrator.cpp:49-57` |
| assoc_shift_px（v8，第59列） | 陀螺预测的锁点水平位移，单位px；同一偏航参考才计算，只在已评估关联的frame行写入，其余留空 | `RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:11`、`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:144-148`、`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:199-202`、`RoboBeetleConsole/src/vision/TargetAssociation.cpp:63-71` |

`actually_submitted=1`不证明ACK或物理动作；event行的配置细节放在dispatch_result，不要拿CONFIG文字做ACK统计。`OK (covered)`表示同一次STOP已被另一个OK覆盖，没有独立RTT。依据：`RoboBeetleConsole/src/vision/VisualDispatchSession.cpp:370-373`、`RoboBeetleConsole/src/vision/VisualDispatchSession.cpp:404-413`。

### 运动状态CSV：读实际样本和丢失，不把事件当传感器

当前schema为motion-state-csv-v2，行类型字段是 **record_type**，不是视觉的row_kind。分析Gyro/周期先筛 `record_type=sample`；其它有totals、cpg_parameters、proportional_configuration。依据：`RoboBeetleConsole/src/robot/MotionStateCsvLogger.cpp:52-85`、`RoboBeetleConsole/src/robot/MotionStateCsvLogger.cpp:96-107`、`RoboBeetleConsole/src/robot/MotionStateCsvLogger.cpp:120-146`。

| 重要字段 | 含义/单位 | 怎么看 |
|---|---|---|
| session_id / link_epoch / batch_seq / fragment_index / sample_index | 文件会话/链路代次/批号/分片/样本索引 | epoch是一次链路身份；重连分段，批号按16位会环绕 |
| mcu_ms / mcu_unwrapped_ms | 采样MCU ms / 处理环绕后的采样ms | 不是Pi时间；不要把重启直接接到上一段 |
| mcu_tx_ms / pi_rx_ms | MCU发包ms / Pi收到ms | 两个不同机器时钟，直接相减不是串口延迟 |
| sample_pi_ms / clock_error_ms | 映射到Pi的采样ms / 串口wire-time参考ms | clock_error这个名字不表示测量误差上界 |
| gyro_x / gyro_y / gyro_z | 角速度°/s | 结合gyro_valid；没有Acc或Yaw列 |
| roll / pitch / angle_age_ms | 姿态角° / 姿态样本年龄ms | 结合angle_valid；多条Gyro可以使用同一份Angle |
| gait_phase_u16 / gait_phase / phase_age_ms | FL一圈0…65535 / rad / 相位年龄ms | 结合phase_valid；phase不是实测关节角 |
| backend / active_mode / target_mode / coordination / motion_state / in_transition | 实际后端、模式、协调、状态枚举与过渡标志 | 同时看实际和目标模式，切换不是瞬间完成 |
| sampler_drop_total / gateway_drop_total / batch_gap_total / fragment_gap_total | 累计丢样/丢包/缺批/缺片 | 看同epoch末值减初值，不能逐行求和 |
| control_mode / stop_reason / cpg_param_version | 离散或比例 / 停止原因 / 参数版本 | 用来区别普通停止、输入超时和失联等 |
| effective_throttle / effective_turn / effective_pitch | 当前实际需求：0…1、−1…1 | CSV已把遥测百分数除100；不是视觉setpoint的千分整数 |
| cpg_phases_valid / phase_fr、rr、rl、fl | CPG相位有效 / 四腿相位rad，另有_u16 | CPG且phase_valid才写；非CPG留空 |
| cpg_front_amp_deg等参数 | 幅值°、名义周期s、β、相位°、耦合强度/mask | cpg_parameters事件保存参数，sample行主要保存版本 |
| prop_max_scale / prop_turn_gain / prop_pitch_limit_cdeg / prop_slew_per_second / prop_deadzone / prop_gamma | 比例配置原始值/0.01°/原始千分值每秒/无量纲 | 对照配置事件解释，别把原始1000读成1000倍实际需求 |

列与换算依据：`RoboBeetleConsole/src/robot/MotionStateCsvLogger.cpp:9-19`、`RoboBeetleConsole/src/robot/MotionStateCsvLogger.cpp:58-82`、`RoboBeetleConsole/src/robot/MotionStateCsvLogger.cpp:96-107`、`RoboBeetleConsole/src/robot/MotionStateCsvLogger.cpp:138-146`。时间映射在最近10秒窗口内选择offset最小的串口锚点，不是最新锚点；带未知队列/传输偏差，clock_error_ms不是误差上界；配视觉时atCapture没有年龄上限，务必看motion_age_ms。依据：`RoboBeetleConsole/src/robot/MotionStateMonitor.cpp:59-78`、`RoboBeetleConsole/src/robot/MotionStateMonitor.cpp:124-129`。

用Excel读枚举数字可直接对照，不必会改代码。定义依据：`RoboBeetleFirmware/Core/Motion/motion_types.h:7-60`。

| 字段 | 当前数字对应 |
|---|---|
| backend | 0=SimpleGait；1=CPG；2=Experimental Flex |
| active_mode / target_mode | 0=STOP；1=FORWARD；2=BACKWARD（仍不支持）；3=TURN_LEFT；4=TURN_RIGHT；5=ASCEND；6=DESCEND |
| motion_state | 0=STOPPED；1=RUNNING；2=STOPPING；3=FAULTED |
| coordination | 0=SameDirection；1=OppositeDirection |
| control_mode | 0=Discrete；1=Proportional；视觉Continuous也使用比例通道 |
| stop_reason | 0=NONE；1=OPERATOR；3=INPUT_TIMEOUT；5=LINK_LOST；其它值不要自行补含义 |

视觉Continuous使用比例执行器的依据：`RoboBeetleConsole/src/vision/VisualDispatchSession.cpp:415-420`、`RoboBeetleConsole/src/vision/VisualDispatchSession.cpp:452-455`。

### 文件在哪里，怎样确认没有白录

Vision Details→`Record visual CSV`、CSV directory、Browse和CSV status；默认Documents下 `RoboBeetle/visual-logs`。文件名含UTC时间和UUID，以NewOnly新建防覆盖，默认250 ms刷盘、32 MiB封顶。依据：`RoboBeetleConsole/src/ui/MainWindow.cpp:2784-2798`、`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:53-96`、`RoboBeetleConsole/src/vision/VisualPolicyConfig.h:62-65`。

远程模式会同时创建 `motion_state_<visual文件基名去掉visual->.csv`；状态区显示对应路径。任一 recorder 失败可停止另一份日志。依据：`RoboBeetleConsole/src/ui/MainWindow.cpp:3006-3028`、`RoboBeetleConsole/src/ui/MainWindow.cpp:3388-3398`。**视觉session_id是诊断数字，motion session_id是文件时间/UUID字符串，不能直接等值连接两列**；按配对文件名、各自session/epoch及Pi捕获/映射时刻处理。依据：`RoboBeetleConsole/src/vision/VisualDiagnosticSession.cpp:45-57`、`RoboBeetleConsole/src/ui/MainWindow.cpp:3390-3391`。

### 三种Excel就能做的分析

以下均为【分析建议】，不是本次已完成统计。先保留原文件，用副本分析；把frame_id、capture_ts_ns等长整数列按文本导入，避免Excel数值精度改变身份。其来源是64位字段：`RoboBeetleConsole/src/vision/DetectionStreamDecoder.cpp:79-104`。

**方法一：统计缺目标持续多久，给lost_ms待办提供证据。**

1. 只保留frame、transition，按诊断session与local_mono_ms排，保留原行顺序；同一session同一时刻的重复状态取最后状态。把local_mono_ms、state、session_id复制到工作表A/B/C列。
2. D列是本行到下一行覆盖的秒数，E列只累计NO_TARGET，F列给连续段编号，G列只保留NO_TARGET组号。按下表填公式，D/E/G从第2行向下填，F3从第3行向下填。末行/跨session的空D表示没有可靠结束时刻，不能改成0假装结束。这比“连续多少行×50 ms”可靠，因transition不是固定采样。写行规则：`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:301-305`。

   | 单元格 | 公式 |
   |---|---|
   | D2 | `=IF(AND(C2=C3,ISNUMBER(A2),ISNUMBER(A3),A3>=A2),(A3-A2)/1000,"")` |
   | E2 | `=IF(AND(B2="NO_TARGET",ISNUMBER(D2)),D2,0)` |
   | F2 | `=--(B2="NO_TARGET")` |
   | F3 | `=F2+--AND(B3="NO_TARGET",OR(B2<>"NO_TARGET",C3<>C2))` |
   | G2 | `=IF(B2="NO_TARGET",F2,"")` |

3. 透视表筛G非空，按G分组求E总和，就得到严格NO_TARGET段。另做“完整缺目标期”：H2写 `=--OR(B2="NO_TARGET",B2="LOST")` 并向下填；按H从0→1或session变化编号，累计对应D，直到恢复TRACKING才结束。**进入LOST是停止策略生效，不是重新看见目标**；严格NO_TARGET段会被1500 ms截断。状态计时：`RoboBeetleConsole/src/vision/VisualTargetStateMachine.cpp:77-88`。
4. 录制开头已经缺目标，或前面STALE/OFF后首次可靠记录已缺目标，标“起点不完整”（不知道真正起点）；结束仍缺失、遇STALE/OFF或换session标“终点不完整”（只知道至少缺失到边界时刻，未见恢复）。这两类边界不完整统称删失，不能当作完整短缺失。完整段可用 `PERCENTILE.INC(时长列,0.95)`、最大值和计数描述，必须同时报告不完整段数量；不能删掉它们后声称总体长尾已经测清。交接待办：`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:89-90`。

**方法二：看选错目标/画面跳变与滤波效果。**

筛frame、awaiting_video=0、有u/ex的行，在同session计算 `=ABS(本行u-上一有效行u)`，另算hc_u；画local_mono_ms对u/hc_u与ex/ex_f的散点图，结合assoc_status/MISS判断跳变发生在哪条路径。报告各自有效相邻对数，不能把u与hc_u不同缺失分母混用。关联与滤波依据：`RoboBeetleConsole/src/vision/TargetAssociation.cpp:63-106`、`RoboBeetleConsole/src/vision/VisualCommandPolicy.cpp:126-144`。与Gyro联看时要检查motion_age_ms，避免旧样本造成虚假同步。

**方法三：做命令结果、延迟和深度异常透视表。**

在同一配对文件和连续控制会话中，按request_id配dispatch/outcome；不跨多次应用运行只凭相同数字ID拼接，未配到结果保留为未知。透视计数OK/BUSY/REJECTED/OUTCOME_UNKNOWN，另对**有独立OK结果**的ack_rtt_ms算中位数和95分位。排除SIMULATED、covered、未知结果和空值，不能把终结耗时混成真实ACK分布。结果写法：`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:224-245`、`RoboBeetleConsole/src/vision/VisualDispatchSession.cpp:370-373`。再筛depth_age_ms>1300、HARD_LIMIT、STALE，核对同阶段STOP的dispatch/outcome；**CSV没有armed/feature_enabled列，撤防与真实输出开关状态须靠现场UI、视频或人工事件记录确认**。要结合记录的feature/axis，不能要求Yaw/普通手动动作都执行Pitch门。表头依据：`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:11`。Motion日志丢失只算同epoch累计末值−初值，不把每行累计数相加：`RoboBeetleConsole/src/robot/MotionStateCsvLogger.cpp:69-72`。

### 下水时要盯什么

CSV status是否仍Recording，两文件路径是否存在、是否出现CSV error；每个阶段记一个事件时刻和动作描述。不能仅等回岸后才发现32 MiB已封顶。停止录制/写错误只影响记录，不等于自动停机器人：`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:101-106`、`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:308-315`、`RoboBeetleConsole/src/ui/MainWindow.cpp:3388-3398`。

### 异常对照表

| 看到什么 | 可能原因 | 先做什么 |
|---|---|---|
| CSV OFF/Error或文件封顶 | 写入失败、32 MiB上限、配对日志失败 | 【操作建议】STOP、撤防，结束这段试验，保存已有文件后重新建立记录。代码见本课文件段 |
| state=NO_TARGET但effective=FORWARD | 正常保留建议；可能仍在运动 | 【操作建议】危险距离前立即STOP/撤防，按实际dispatch和motion核对。`RoboBeetleConsole/src/vision/VisualCommandPolicy.cpp:100-109` |
| depth_raw_m=0但UNAVAILABLE | 有效性失效后的代码值，不是真水面证据 | 【操作建议】STOP/撤防，不拿0继续深度动作。`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1004-1012` |
| drop累计不断长或motion_age很大 | 缓存/传输丢失或旧运动样本匹配【推测】 | 【操作建议】停机、保留日志；不据此直接声称物理同步/反应延迟。`RoboBeetleConsole/src/robot/MotionStateMonitor.cpp:61-78`、`RoboBeetleConsole/src/robot/MotionStateMonitor.cpp:124-129` |

自测：8.1 视觉CSV有1000行，是否就是1000帧？8.2 ack_rtt_ms非空，是否说明舵机动作完成？

## 9. 下水实测检查单

以下是【操作建议/待执行检查单】。每次只改变一个因素，未通过不进入下一阶段；紧急处置始终按第0节。**当前防水Unverified的直接浸水限制未被本手册解除**：先落实密封/部件防水与人工回收依据，再进入入水阶段。边界来源：`docs/engineering-lessons.md:92`。本次未执行任何一项硬件检查，不新增Hardware Verified。

| 阶段 | 要做、要记录 | 通过条件 | 中止条件与第一步 | 代码/边界依据 |
|---|---|---|---|---|
| 下水前（离水） | 完成十项；核对版本、安装身份、方向、STOP/Disable；录两份CSV，建立本次延迟基线 | 无未解决密封/回收问题；按钮可用、控制权有效、Leak Dry且更新、所需方向已人工观察 | 不明防水、Wet/Unknown、无法安全停止、身份/方向不符：Disable All、撤防，留在离水阶段 | `docs/engineering-lessons.md:92`；`RoboBeetleConsole/src/ui/MainWindow.cpp:2903-2922`；`docs/proportional-control.md:3-6` |
| 入水静置 | 自动输出关闭、无运动；观察Leak、原始深度/温度/Age、IMU响应；在已知水面STOP/撤防后Zero | Leak保持Dry；深度新鲜，5不同稳定样本可Zero，零后状态/读数与位置相符；真实传感器响应有记录 | Leak湿/失效，深度跳变/过期，出现非预期动作：Disable All、撤防、出水 | `RoboBeetleConsole/src/remote/RemoteRobotController.cpp:903-913`、`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1016-1062`；`RoboBeetleConsole/src/robot/DepthControl.h:8-15` |
| 手动低速动作 | 从已经离水确认的小幅设置开始，逐项前进/左转/右转/STOP；记录真实方向、足摆动/是否碰壳、实际周期 | 每项方向与安装预期一致；STOP逻辑状态和实物分别检查；无撞限位；没有旧动作重放 | 不受控、错方向、卡死、停止不能确认：Disable All、撤防、出水 | `RoboBeetleFirmware/Core/Motion/motion_config.h:4-18`；`RoboBeetleFirmware/Core/Servo/servo_service.c:382-393` |
| 深度动作 | 保持人工回收条件，升/潜逐项短时验证；看cal深度、Age、区域，同时盯实物趋势；普通手动按钮由你主动中止 | 机械符号与真实升潜效果已分别记录；深度证据持续新鲜；不以软限位改Forward当“不会再变深” | 深度过期/跳变、真实趋势相反、接近计划中止深度、HardLimit：STOP/Disable All、撤防、回收 | `RoboBeetleFirmware/Core/Motion/motion_config.h:11-13`；`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:352-367`、`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:562-584`；`RoboBeetleConsole/src/robot/DepthControl.h:12-15` |
| 视觉DRY_RUN | 先用STOP、不Arm、Visual dispatch关闭路线看诊断；再按第6课Dry run模拟入口检验比例接近过程，模拟可Arm但不发真实控制 | 当前阶段无视觉真实控制输出；图像/目标方向符合人工观察；CSV正常；SIMULATED只算模拟证据，缺失/冻结与真实执行门分别看懂 | 任意意外运动、误差/方向对不上、视频长期等待、日志失效：STOP、撤防，必要时出水 | `RoboBeetleConsole/src/vision/VisualDispatchSession.cpp:211-222`；`RoboBeetleConsole/src/vision/VisualTargetStateMachine.cpp:33-88`；`RoboBeetleConsole/src/vision/VisualProportionalDispatchStateMachine.cpp:114-121`、`RoboBeetleConsole/src/vision/VisualProportionalDispatchStateMachine.cpp:167-185` |
| 视觉布防 | 前阶段通过后才明确开Visual dispatch并Arm；首次从已验证的轴/执行模式开始；连续盯控制权、状态、深度与真实运动 | Arm门全部满足；真实纠偏缩小对应误差且无危险趋势【待水下观察】；已知NO_TARGET仍可能持续动作，能主动中止；两日志持续记录 | 错方向、异常推进、Wet/Unknown、失控、STALE/OFF、深度失效/硬限位、STOP未知：STOP/Disable All、撤防、回收 | `RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:10-33`、`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:142-205`；`RoboBeetleConsole/src/vision/VisualProportionalDispatchStateMachine.cpp:37-74`、`RoboBeetleConsole/src/vision/VisualProportionalDispatchStateMachine.cpp:265-305` |

不要把“达到0.35 m才自动停”当作测试目标；那是Pitch/Both相关软件硬门，普通手动/Yaw并不享有同样保护。真实中止深度、操作时长与回收距离由负责人结合已验证防水和现场条件制定，**【未在代码中找到】可以代替这些现场判断的水下安全认证**。

验收记录每项至少写：实装三组件版本、阶段、参数/轴/Exec、时间、期望、实际、CSV文件、证据标签、下一步。一次实际动作观察可记Hardware Verified的具体范围；水下方向/推进实测记录清楚后才能关闭相应Pending Water Verification，不能把整张清单一次升级。

## 10. 待讨论风险清单

按现场影响排序。全部只报告，不修复；建议由负责人决定是否另开任务。P0/P1/P2是本手册讨论优先级，**不是本次新增的实机故障证明**。

| 优先级 / 风险 | 影响与证据 | 现场兜底与另开任务建议 |
|---|---|---|
| P0：漏水只监视，没有自动安全动作 | WET只更新状态与颜色，无STOP/Disable/声响；软件不会替无人注视的操作员处理进水。`RoboBeetleFirmware/Core/App/app_main.c:802-805`；`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:903-913`；`RoboBeetleConsole/src/ui/MainWindow.cpp:4340-4362` | 人工持续看Leak，异常立即Disable/出水；讨论漏水Safety策略、锁存、恢复条件和告警，并用独立实测验收 |
| P0：防水仍Unverified，代码不能证明密封 | 舵机/舱体/连接器物理进水可能绕过软件控制；现有文档不允许无可靠依据直接浸水。`docs/engineering-lessons.md:92`；`RoboBeetleFirmware/README.md:35` | 防水/回收证据未落实则停在离水；另开硬件封装与密封验收，不能用程序补证明 |
| P1：深度保护不是全局固件保护 | 普通手动ASCEND/DESCEND不查envelope，Yaw不依赖深度；不能指望所有模式在0.35 m自动停。`RoboBeetleConsole/src/ui/MainWindow.cpp:2365-2368`；`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:352-367`、`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:562-584`；`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.h:41-42` | 手动升潜由负责人主动中止；讨论是否统一保护边界和失效策略，保持人工回收能力 |
| P1：视觉固定方向、操作者确认门已移除 | 配置+1直接作为confirmed；水下方向未验证时可能越纠越偏，旧文档/隐藏confirmed文字会误导。`RoboBeetleConsole/src/vision/VisualPolicyConfig.h:26-35`；`RoboBeetleConsole/src/vision/VisualDispatchSession.h:130`；`RoboBeetleConsole/src/ui/MainWindow.cpp:3270-3277` | 先手动确认画面与机体实际响应；不符立即STOP撤防；讨论方向确认机制与旧文档同步 |
| P1：前足手动角度可绕过窄PWM窗 | descriptor允许+45°，映射FR1000/FL2020超窄窗；Motion的+28°保护不覆盖手动SetAngle。【代码推导】存在撞壳风险，未做实物复现。`RoboBeetleFirmware/Core/Servo/servo_descriptor.c:17-53`；`RoboBeetleFirmware/Core/Servo/servo_service.c:382-393`；`RoboBeetleFirmware/Core/Motion/motion_config.h:15-18` | 不做前足完整端点探索；讨论让角度/PWM/运动限位一致，并对实际安装验证 |
| P1：NO_TARGET宽限保留运动，lost_ms尚无水下统计依据 | 默认1500 ms可能继续已有需求；LOST仍布防、可恢复动作。`RoboBeetleConsole/src/vision/VisualPolicyConfig.h:24`；`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:149-155`；`RoboBeetleConsole/src/vision/VisualProportionalDispatchStateMachine.cpp:292-305`、`RoboBeetleConsole/src/vision/VisualProportionalDispatchStateMachine.cpp:343-392`；`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:89-90` | 缺目标且有危险趋势就人工STOP/撤防；先按第8课统计完整缺失期/删失，再讨论lost_ms，勿现场凭感觉改 |
| P1：断线停止依赖检测、租约、调度与心跳；Disable不是电源切断 | 601/1601是名义预算，机械停稳还需额外时间；Emergency Stop按钮禁用。`docs/task05-stop-transition-red.md:107-128`；`RoboBeetleConsole/src/ui/MainWindow.cpp:2917-2922`；`RoboBeetleFirmware/Core/Servo/servo_driver_stm32.c:231-245` | 确认可执行的回收/实物断电方案；另开端到端断链计时与独立停止手段验证 |
| P2：IMU低速Receiving可能掩盖传感器旧值 | parser保持valid，快照仍可能重复旧状态。【推测】停止传感器但继续心跳时可能绿灯旧值，未做运行复现。`RoboBeetleFirmware/Core/Sensors/jy901s_parser.c:119-141`；`RoboBeetleFirmware/Core/App/app_main.c:554-582`；`RoboBeetleConsole/src/robot/ImuMonitor.cpp:25-54` | 看源端RX/valid增量及受控响应；讨论按域新鲜度和显示有效性 |
| P2：单客户端无keepalive，旧半开连接妨碍恢复 | 新连接被拒，control lease撤权不等于TCP槽立即释放。`RoboBeetlePi/gateway/linux/src/tcp_adapter.cpp:360-377`；`ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md:31-33` | 先确保停止/回收，再关旧Console处理网关；另议连接清理与keepalive |
| P2：远程CRC与日志时间名字容易过度解释 | CRC无Pi串口桥接；clock_error_ms不是误差界；atCapture不设年龄上限；envelope可能为缓存。`RoboBeetleConsole/src/controller/IConsoleController.h:39-45`；`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1426-1451`；`RoboBeetleConsole/src/robot/MotionStateMonitor.cpp:61-78`、`RoboBeetleConsole/src/robot/MotionStateMonitor.cpp:124-129`；`RoboBeetleConsole/src/vision/VisualDispatchSession.cpp:148-157` | 按第7/8课口径记证据，别声称精密同步/零错帧；另议诊断桥接、年龄门和命名 |
| P2：视频卡顿未定因，CSV可停止而运动继续 | 缓存/解码/显示/传输需分别查；录制到上限或写失败停止日志，不是运动Safety。`docs/target-temporal-association-dry-run-validation.md:142`；`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:101-106`、`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:308-315` | 卡顿/日志失效就停这一试验段；保存计数、等待状态和事件，另开有同步证据的诊断任务 |

## 11. 术语表

本节汇总前文已解释的术语；定义用于阅读代码，不额外声称实测结果。

| 术语 | 大白话 |
|---|---|
| Firmware / STM32 | 直接控制舵机和读取传感器的底层程序/芯片 |
| Pi gateway | 树莓派上的转发与控制权管理程序 |
| Console / Qt | 负责人点击按钮、看画面和记录数据的电脑程序/界面库 |
| Host Test / ARM Build / Program Verify | 桌面逻辑测试 / 目标编译 / 特定镜像烧录校验，分别算证据 |
| Hardware Verified / Bench Measured | 明确观察过的实物路径 / 台架实际测过的数值，不自动算水下通过 |
| Pending Water Verification | 对应水中现象还没完成验收 |
| Heartbeat / watchdog | “我还活着”的消息 / 太久收不到就自动保护的检查 |
| ACK / RTT / OutcomeUnknown | 请求结果收据 / 结果往返时间 / 不知道对方到底有没有执行 |
| STOP / STOPPING / STOPPED | 请求普通停止 / 正在平滑回零 / 逻辑停止，仍需观察实物 |
| Disable All | 停所有已使能PWM并清状态；没有证明切断舵机供电 |
| PWM / μs / cdeg | 舵机控制脉冲 / 百万分之一秒 / 百分之一度，1000 cdeg=10° |
| CPG / phase / coupling / β | 节拍器 / 一圈进行到哪 / 节拍器相互协调 / 模型节奏不对称参数 |
| logical angle / calibration / remap | 给关节的语义角 / 角到PWM换算 / 安装身份或方向重新映射 |
| IMU / Gyro / Euler Angle | 姿态测量单元 / 每秒转动多少度 / 三个姿态角 |
| body frame / yaw / pitch / roll | 以机器人为基准的轴 / 左右转头 / 抬低头 / 左右侧倾；轴向仍需标定 |
| fresh / stale / depth_age_ms | 有效且够新 / 过期不可当当前值 / 深度样本综合年龄 |
| Zero / depth gating / envelope | 本地取基准零点 / 拦住禁止的升潜需求 / 当前允许区域分类 |
| EMA / hysteresis / dwell | 平滑新旧值 / 进入退出用不同阈值 / 切换或到达前等待 |
| TargetAssociation / LOCK / MISS | 连续认同一目标 / 标出已选目标 / 当前帧关联失败 |
| NO_TARGET / LOST / STALE | 短暂无可用目标 / 持续缺失 / 数据冻结过期；动作规则不同 |
| Arm / Disarm / DRY_RUN | 允许自动控制 / 撤销允许 / 只诊断或模拟，不向机器人发视觉运动 |
| throttle / turn / pitch setpoint | 推进、转向、升潜需求；不是实际速度/转角/深度 |
| cells / area proxy | 检测热图格子数 / 用面积估计接近程度，不是米 |
| TCP / USART / CRC | 网络字节连接 / 串口字节连接 / 发现传坏数据的校验 |
| control lease / authority / epoch | 要续期的控制资格 / 当前驾驶权 / 一次链路身份 |
| EOF / keepalive / half-open | 对端连接结束 / 检查连接还在的机制 / 一边以为连接仍存在的残留状态 |
| batch / fragment / FIFO / latest-value | 样本批次 / 批次分片 / 按顺序取出的缓存 / 只保留最新需求 |
| CSV / row_kind / record_type | 行列文本日志 / 视觉行类型 / 运动日志行类型 |
| mono time / timestamp / censoring | 只算经过时间的时钟 / 事件时间标记 / 没观察到缺失期完整起止，需标为不完整 |

单位/字段定义依据集中见 `RoboBeetleConsole/src/robot/MotionStateCsvLogger.cpp:9-19`、`RoboBeetleConsole/src/robot/MotionStateCsvLogger.cpp:64-82`、`RoboBeetleConsole/src/vision/VisualPolicyConfig.h:11-65`、`RoboBeetleConsole/src/robot/DepthControl.h:18-32`；动作定义和证据出处见对应课程。

## 12. 自测题答案

| 题号 | 答案与回看位置 |
|---|---|
| 1.1 | 不能。ACK确认接受；750 ms逻辑回零、有效tick和实物停止分别观察。第1课时间表；`RoboBeetleFirmware/Core/Motion/motion_config.h:4-5` |
| 1.2 | 比例输入超时走普通STOP，保留使能；心跳超时立即中止并disable_all停PWM。`docs/proportional-control.md:49-50`；`RoboBeetleFirmware/Core/App/app_main.c:120-121` |
| 2.1 | 不可以。当前WET只显示，要人工停止、撤防、回收。`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:903-913` |
| 2.2 | Dry是有效状态报告干燥；Unknown是未采样/过期/断线/非法等信息不可用，不排除漏水。`RoboBeetleConsole/src/robot/LeakStatus.h:27-37`；`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1344-1347` |
| 3.1 | 没有。Zero只减Console本地均值，不修改压力传感器绝对标定。`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1011-1012`、`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1058-1060` |
| 3.2 | 不一定。普通手动startMotion/isMotionReady没有envelope检查。`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:352-367`、`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:562-584` |
| 4.1 | 不能。Receiving是收到快照；机体轴/磁/yaw标定仍Pending。`RoboBeetleConsole/src/robot/ImuMonitor.cpp:25-29`；`docs/engineering-lessons.md:3` |
| 4.2 | 不是。先筛record_type=sample，参数/totals行不是采样；即使10条sample，也可能共享同一份Angle，需看angle_age/valid。`RoboBeetleFirmware/Core/App/app_main.c:849-855`；`RoboBeetleConsole/src/robot/MotionStateCsvLogger.cpp:73`、`RoboBeetleConsole/src/robot/MotionStateCsvLogger.cpp:100`、`RoboBeetleConsole/src/robot/MotionStateCsvLogger.cpp:123` |
| 5.1 | 不一定。左右镜像和校准可以使PWM相反而逻辑动作一致。`RoboBeetleConsole/src/robot/ServoDescriptor.cpp:11-46` |
| 5.2 | 没有。CPG/Motion公共窗与手动SetAngle不同，后者前足±45°检查可越过窄PWM窗。`RoboBeetleFirmware/Core/Servo/servo_service.c:382-393` |
| 6.1 | Discrete不会仅因900 ms NO_TARGET停；有确认模式会保持，1500 ms变LOST才STOP。控制权/深度/姿态等异常可更早停。`RoboBeetleConsole/src/vision/VisualDispatchStateMachine.cpp:142-176` |
| 6.2 | Discrete及Proportional离散选向水平优先；Continuous可同时映射turn/pitch，仍有深度门。`RoboBeetleConsole/src/vision/VisualCommandPolicy.cpp:188-195`；`RoboBeetleConsole/src/vision/VisualApproachPolicy.cpp:88-105` |
| 7.1 | 不是，是Qt发命令到Pi返回终态的整条路径用时。`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:805-807` |
| 7.2 | 不能，远程CRC没有Pi串口计数桥接。`RoboBeetleConsole/src/controller/IConsoleController.h:39-45`；`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:1426-1451` |
| 8.1 | 不是，要筛frame；transition/dispatch/outcome/event都不代表新检测帧。`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:224-245`、`RoboBeetleConsole/src/vision/VisualCsvLogger.cpp:301-305` |
| 8.2 | 不能。非空可能是未知终态耗时；即使OK也不是机械完成证明。`RoboBeetleConsole/src/remote/RemoteRobotController.cpp:805-807`；`RoboBeetleFirmware/Core/Communication/protocol_dispatcher.c:448-459` |
