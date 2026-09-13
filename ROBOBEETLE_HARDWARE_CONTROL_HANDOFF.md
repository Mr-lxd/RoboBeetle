# RoboBeetle 硬件控制交接审计

本次收口对应的近期 Servo、LeakStatus、JY901S 与 Depth 实机运行均使用：
`Qt Console → Windows COM13 → DAP UART/USB serial bridge → STM32 USART1`
（9600 8-N-1）。APC220 仅保留为早期/legacy transport 记录，未参与近期
验证，也不是当前启用的硬件链路。

## 2026-09-10 Leak detection sensor bring-up（PR #9，end-to-end Hardware Verified）

本阶段在 PR #8 五舵机分支之后采用 stacked branch，实现第一条最小数字漏水检测路径及其 monitoring-only Protocol V2/Qt 显示，不改变 Servo 行为或 Safety 行为。漏水模块由 3.3 V 供电，与 STM32 共地；数字输出 `D0` 接 STM32 `PA11`，模拟输出 `A0` 暂不使用。PA11 → Firmware → Protocol V2 → Qt 的完整路径已经完成实机验收。

### 当前实现

```text
leak module D0
  → PA11 GPIO input
  → leak_sensor_stm32 raw-level reader
  → leak_sensor pure-C mapper
  → app_main 内部状态（主循环 polling）
  → Protocol V2 LeakStatus `0x20`（Heartbeat ACK 完成后）
  → Qt Leak indicator
```

- `PA11` 在现有 `MX_GPIO_Init()` 中配置为 `GPIO_MODE_INPUT` + `GPIO_NOPULL`，没有新增 EXTI/NVIC。
- `PA11` 已对照当前 `.ioc`、`main.c`、HAL MSP、USART1、TIM3/TIM4、SWD 与既有 GPIO 资源确认在软件配置层面 free；本阶段将其作为漏水 D0 输入。
- 当前集中定义的初始极性是 `PA11 HIGH → LEAK_SENSOR_STATE_DRY`、`PA11 LOW → LEAK_SENSOR_STATE_WET`；首次采样前状态为 `UNKNOWN`。
- `GPIO_NOPULL` 是 bring-up assumption，不是已验证的电气结论；资料尚未可靠确认 D0 输出级是推挽还是开漏。若实机显示浮动/不稳定，另行依据测量结果决定 pull 配置。
- 轮询只更新内部状态；LeakStatus 是无 ACK 的 monitoring-only telemetry，不触发 Servo disable、Emergency Stop、报警或其它 Safety 动作。
- Firmware 只在 accepted Heartbeat 的正常 ACK 已完整发送后，在首次有效采样、状态变化或 500 ms refresh opportunity 发布一帧；telemetry 使用独立序列空间，不进入 command pending/ACK matching。
- Qt 将 `0=UNKNOWN`、`1=DRY`、`2=WET` 显示为 `Leak: Unknown`、`Leak: Dry`、`LEAK DETECTED`；断开、当前 host-link liveness loss、非法 payload 或 1500 ms（3 × 500 ms opportunity，provisional）无更新时回到 Unknown。

### 验证分层

| 项目 | 当前状态 |
|---|---|
| Leak HIGH→Dry / LOW→Wet 纯 C 逻辑 | **Host Test: PASS** |
| Firmware ARM Build | **ARM Build: PASS**（STM32CubeIDE GNU Tools for STM32 14.3.1，CMake Debug configure/build；生成 `RoboBeetleFirmware.elf`） |
| Program Verify | **Program Verify: PASS**（重建当前 ELF 后完成烧录与校验） |
| PA11 Dry/Wet 检测路径 | **Hardware Verified** |
| Protocol V2 LeakStatus `0x20` 实链路 | **Hardware Verified** |
| Qt Leak indicator | **Hardware Verified** |
| 端到端 Leak monitoring | **Hardware Verified** |
| Leak Safety Supervisor response | **Pending / Not Implemented** |
| Protocol V2 LeakStatus `0x20` codec/controller path | **Host Test: PASS; Hardware Verified** |
| Qt leak visualization / stale-disconnect behavior | **Host Test: PASS; Hardware Verified** |

### Leak 实机验收记录

1. 重新构建当前 Firmware ELF，完成 Program Verify；先前持续显示 `Leak: Unknown` 的问题由错误/过期 build artifact provenance 引起。
2. 使用已验证的当前镜像运行 PA11 leak detection，确认干/湿状态能沿 Firmware `leak_sensor` 路径进入 LeakStatus telemetry。
3. 通过真实 Protocol V2 链路观察 Qt indicator 状态变化，确认 LeakStatus 不产生 ACK、不触发 Servo 或 Safety 动作。
4. 本次验收未记录数值电压或响应时间；这些数值不应从本次 PASS 结论中推导。

传感器 bring-up 顺序固定为：`Leak detection (including LeakStatus telemetry) → JY901S IMU → depth/sensor board`。在 PR #9 LeakStatus closeout 的历史记录中，JY901S 与 Depth 尚未开始；当前 JY901S phase 见下节，Depth 仍未开始。

## 2026-09-10 JY901S listen-only bring-up（Hardware Verified / UART quality follow-up open）

本轮只实现 JY901S 的 RX/parser-only 路径，不发送任何传感器配置命令：

```text
JY901S TX
  → STM32 PB11 / USART3_RX
  → one-byte interrupt RX
  → 独立 256-byte ring buffer（255-byte effective capacity）
  → 11-byte pure-C frame parser
  → Acc / Gyro / Angle 内部状态
```

- JY901S TX 接 PB11；JY901S RX 接 PB10；共地。
- STM32 USART3 本地使用 9600 baud、8-N-1、TX/RX、无硬件流控；这不是向 JY901S 写入波特率。
- PB10 仅按硬件设计配置；应用层不发送 baud、输出频率、output mask、保存、重启、校准或其它 JY901S command。
- 按当前持久化/default 配置监听，预期约 10 Hz，通常包含 `0x51` Acc、`0x52` Gyro、`0x53` Angle，也可能持续收到合法 `0x54` Mag。
- `0x54` Mag 是 known-but-not-decoded：checksum 正确时保持 parser 同步、增加 `mag_frame_count`，不改变 Acc/Gyro/Angle state，也不增加 `unsupported_frame_count`。
- 其它 checksum-valid 未知类型才增加 `unsupported_frame_count`；checksum error 单独统计。

### 实机诊断顺序

如果完全没有合法帧，不得立即加入自动配置；按以下顺序记录：

1. `rx_byte_count` 是否增加；
2. ring buffer push/pop 是否增加，是否 overflow/drop；
3. parser 是否看到 `0x55` header；
4. `valid_frame_count` 是否增加；
5. `checksum_error_count` 是否增加；
6. `mag_frame_count` 与 `unsupported_frame_count` 的分类；
7. Acc/Gyro/Angle valid flags、数值与 last-valid tick。

USART3 的 RX callback/error callback 只记录事件并标记 needs-rearm；foreground
的 `app_main_process` 通过一次轻量 poll 重新尝试
`HAL_UART_Receive_IT(..., 1U)`。`HAL_BUSY` 单独计入 deferred counter，只有
`HAL_ERROR` 等真正失败才进入 hard re-arm failure counter。ISR 不解析、不阻塞、
不循环重试。foreground 状态转换带 generation re-check，避免 HAL 开启下一次
接收期间到达的 callback/error 事件被成功路径清掉。当前 USART1 host-link
（DAP UART/COM13）的已有 RX、ring 和 transmit 行为保持不变；APC220 为
legacy transport 记录。

### PR #10 listen-only evidence matrix

以下 ARM Build 与 Program Verify 只适用于 PR #10 bring-up ELF；它们不自动
覆盖 PR #11 image。matching PR #11 Firmware + Qt run 提供本节的 JY901S
physical RX 与 end-to-end Hardware Verified 证据。

| 项目 | 状态 |
|---|---|
| Host Test | **PASS**：parser、USART3 transport mock、ring-buffer 与全部当前 Firmware regressions |
| ARM Build | **PASS**：PR #10 STM32 target build，0 errors、0 warnings；RAM 2680 B / 128 KB，FLASH 23260 B / 512 KB |
| Program Verify | **PASS**：PR #10 DAP/OpenOCD programming flow 完成并报告 `Verified OK` |
| Hardware Verified | **PASS**：matching PR #11 Firmware + Qt run 完成 JY901S 实机端到端观测 |
| USART3 physical RX | **Hardware Verified**：PB11 / USART3 RX、ring path 与持续接收 |
| JY901S valid real frames | **Hardware Verified**：valid frame counter 持续增加、overflow 为 0 |
| Acc/Gyro/Angle real data | **Hardware Verified**：Acc 合理、静止 Gyro 接近零、Angle 正确响应 |
| Re-arm diagnostics | **Hardware Verified**：matching PR #11 post-fix runs 的 hard re-arm failures 均为 0；HAL_BUSY deferred 独立统计 |
| USART3 UART/checksum physical quality | **Pending / non-blocking**：UART/checksum 计数可见，物理来源尚未由本证据确定 |
| 0x54 Mag recognition | **Hardware Verified**：Mag frames 被识别为 known-but-ignored，unsupported 为 0 |
| Pending | 最终 body-frame mapping、magnetic/yaw calibration，以及 USART3 physical-link quality follow-up |

2026-09-10，用户在 hardware-verification checkout 完成了当前
`RoboBeetleFirmware.elf` 的 STM32 target build：0 errors、0 warnings，RAM
2680 B / 128 KB，FLASH 23260 B / 512 KB；ELF 为
`D:\RoboBeetle\RoboBeetleFirmware\build\Debug\RoboBeetleFirmware.elf`，
记录的 LastWriteTime 为 2026-09-10 17:07:11。随后使用 DAP/OpenOCD（SWD
100 kHz、SYSRESETREQ、halt、program、verify、reset-run）完成编程与校验，
记录为 `Programming Finished`、`Verify Started`、`Verified OK`。这些证据关闭
ARM Build 与 Program Verify；matching PR #11 Firmware + Qt 实机运行另外提供
了本节所记录的 JY901S physical RX 与 Acc/Gyro/Angle Hardware Verified 结果。

matching PR #11 Firmware + Qt 的 post-fix short hardware regression：

- Run A：RX bytes 43295，headers 4006，valid frames 3831，checksum failures
  175，overflow 0，hard re-arm failures 0，UART errors 180，Mag frames 958，
  unsupported 0。
- Run B：RX bytes 73444，headers 6815，valid frames 6448，checksum failures
  366，overflow 0，hard re-arm failures 0，UART errors 376，Mag frames 1618，
  unsupported 0。

两次运行中 RX bytes 与 valid frames 持续增加，overflow 与 hard re-arm failures
均为 0。UART aggregate/subtype 与 checksum errors 保持可观测；本记录不把
USART3 physical-link quality 解释为无错误，也不推断错误物理来源。

本轮不涉及 Protocol V2 IMU telemetry、Qt IMU display、Depth sensor、Safety、自动 JY901S configuration 或 body-frame calibration。

## 2026-09-10 PR #11 JY901S low-rate telemetry + Qt monitor（Hardware Verified / re-arm follow-up closed）

本阶段从 PR #10 的 listen-only bring-up HEAD 叠加，只消费已经存在的
JY901S Acc/Gyro/Angle 内部状态，不改变 USART3 RX/parser 路径，也不向
JY901S 发送任何配置、保存、重启、校准或其它 command。新增链路为：

```text
JY901S parser state + diagnostics
  → fixed Protocol V2 ImuSnapshot `0x21`
  → existing STM32 USART1 host link / DAP UART/COM13
  → Qt RobotController / ImuMonitor
  → IMU — JY901S read-only panel
```

### 固定协议与调度

- `ImuSnapshot` 使用独立 telemetry sequence，不进入 command/ACK matching；不发送 raw JY901S frame、ASCII 或平台相关 struct memcpy。
- payload 固定 56 bytes：schema `0x01`、Acc/Gyro/Angle validity flags、little-endian fixed-point values，以及 USART3/parser diagnostics counters。Acc 为 mg，Gyro 为 0.1 dps，Angle 为 0.01 degree；无效 domain 编码为零。
- Firmware 只在 accepted Heartbeat 的正常 ACK 已完成发送后评估 IMU/Depth policy；每次 opportunity 最多发送一个 non-ACK telemetry frame。只要 LeakStatus `0x20` due 就立即优先；Leak 不 due 时，在仍 due 的 ImuSnapshot `0x21` 与 DepthSnapshot `0x22` 之间公平轮转。失败发送不 mark published，保持该 policy 可重试。
- IMU policy interval 为 1 s；成功发送后才 mark published。在 nominal accepted Heartbeat cadence 下，ImuSnapshot 的实际有效刷新率 up to approximately 1 Hz；ACK opportunity 延迟或 LeakStatus pending refresh 会使实际速率更低。56-byte payload 的最大 wire frame 为 68 bytes；9600 8-N-1 下按现有 Protocol V2 host-link/Heartbeat/Leak 预算计算，不宣称实机吞吐已验证；不增加独立 IMU TX timer。
- 当前 DAP UART/COM13 host link 不增加独立 IMU TX timer；IMU 不会创建、释放、重试或重排 ACK pending request。USART1 host-link 与 LeakStatus 行为保持不变，Apc220HalfDuplex 仅保留为 legacy-named conservative policy。

### Qt 监视器边界

`IMU — JY901S` 面板只显示 status、Acc、Gyro、Euler Angle 与 bring-up
diagnostics，不包含 3D、历史曲线、校准、控制动作或 raw passthrough。
状态为 `Unknown`（未收到/断链/host-link liveness loss）、`Receiving`（合法帧）、
`Stale`（3500 ms 无合法更新）或 `Error`（非法 payload）。Stale、Error、
断链与 liveness loss 都清空 live snapshot；单独无效的 Acc/Gyro/Angle domain
显示 `--`，不能把旧值继续显示为当前值。IMU frame 不影响 ACK 或 Leak state。

### PR #11 验证分层

| 项目 | 状态 |
|---|---|
| Host Test | **PASS**：全部当前 Firmware regressions（含 Leak sensor / Leak telemetry policy）、JY901S telemetry codec/scheduler、Console protocol/controller/descriptor/IMU/MainWindow tests |
| ARM Build | **PASS**：matching PR #11 Firmware build 用于本次实机运行 |
| Program Verify | **Pending**：本次 closeout 未提供独立 programming/verify 记录 |
| Hardware Verified | **PASS**：JY901S → USART3/PB11 → ring/parser → Acc/Gyro/Angle → ImuSnapshot → STM32 USART1 → DAP UART/COM13 → Qt |
| Re-arm diagnostics follow-up | **PASS / resolved**：post-fix Run A、Run B 的 hard re-arm failures 均为 0 |
| USART3 UART/checksum physical quality | **Pending / non-blocking**：aggregate/subtype UART 与 checksum errors 保持可观测 |
| Pending | 最终 body-frame mapping 与 magnetic/yaw calibration |
| PR #10 listen-only ARM/Program evidence | **PASS**：仅适用于 PR #10 记录的 bring-up ELF，不自动覆盖 PR #11 |

本次 matching PR #11 Firmware + Qt 实机结果：IMU status = `Receiving`；Acc
实时且物理合理；静止时 Gyro 接近零；Angle 实时响应。初始 pre-fix 诊断快照为
RX bytes `131663`、headers `11967`、valid frames `11957`、checksum errors
`10`、overflow `0`、UART errors `20`、Mag frames `2989`、unsupported `0`，
并显示 `rx_rearm_failure_count = 166240`。根因是诊断语义，而不是证明这
166240 次都是真正 hard failure：旧实现把所有非 `HAL_OK` 返回都计入同一个
counter，且没有保留 HAL status。本仓库 STM32F4 HAL 的
`HAL_UART_Receive_IT()` 在 `RxState` 不是 `HAL_UART_STATE_READY` 时返回
`HAL_BUSY`；但正常 one-byte `UART_Receive_IT()` 路径会在调用
`HAL_UART_RxCpltCallback` 前先把 `RxState` 置为 `READY`。因此正常完成
callback 本身不能证明发生了“HAL 尚未完成导致的 BUSY”，旧 aggregate 也
无法事后拆分；在与其它 active receive 或 error/foreground 状态切换重叠时，
仍可能观察到 busy。PR #11 让 callback 只标记 pending，由 foreground 每次
poll 最多尝试一次 re-arm；`HAL_BUSY` 单独计为 deferred，`HAL_ERROR` 及其它
非成功状态才计入 hard failure；generation re-check 防止较新的 callback/error
event 被旧的成功路径清掉。

Post-fix short hardware regression 已关闭该 follow-up：Run A 的 hard re-arm
failures 为 0；Run B 的 hard re-arm failures 也为 0，且 RX bytes/valid frames
持续增加、overflow 为 0。UART aggregate、各 subtype 与 checksum counters
继续保持可观测；不把 USART3 physical-link quality 解释为无错误，也不推断
其物理来源。Qt 单次 `RX rejected: Invalid length` 仍是 observation；现有
Console sticky/concatenated frame tests 已覆盖连续帧边界，CRC errors 与
timeouts 均为 `0`，目前不进行协议重设计。

最终 robot body-frame mapping 与 magnetic/yaw calibration 仍为 **[Pending]**。
不得因任何单次异常在本阶段加入自动 JY901S configuration/init。

## 2026-09-11 Depth Sensor / ROVMAKER decoder bring-up（stable connection Hardware Verified; connector/calibration pending）

本阶段实现 ROVMAKER 水深传感器解码板的 listen-only 接收与 monitoring-only
遥测，不发送任何 decoder-board configuration、保存、重启、校准或其它命令：

```text
ROVMAKER decoder board
  → STM32 PC7 / USART6_RX（115200 8-N-1）
  → one-byte interrupt RX
  → 独立 512-byte ring buffer（511-byte effective capacity）
  → bounded ASCII line parser
  → DepthSnapshot `0x22`
  → existing USART1 host link / DAP UART/COM13
  → Qt `Depth Sensor — ROVMAKER` read-only monitor
```

STM32 同时按生成式配置保留 PC6 / USART6_TX，但应用层不发送解码板命令。
官方 [ROVMAKER 解算板手册](https://docs.rovmaker.cn/产品手册/水深传感器产品手册/深度传感器解算板V1.0.html)
记录了 115200 8-N-1、canonical line
`Depth:XX.XXm Temp:XX.XXC\r\n`，以及精确示例
`Depth:1.21m Temp=25.27C`。因此 parser 只兼容完整的 `Temp:` 与 `Temp=`
两种格式；不接受猜测的 compact `T=...D=...`、任意 separator、substring、
bare LF 或 trailing data。该手册还要求板和传感器在水面通电，以环境空气压
建立深度零点；这是厂商操作指导，不是本机器人实机验证结果。

物理安装边界按以下拓扑记录：湿侧 pressure face/probe → pressure hull 的
sealed penetration/threaded installation → pressure hull 内部 cable → 干侧
ROVMAKER decoder board → STM32 PC7/USART6_RX。稳定连接下的 USART6 接收、
DepthSnapshot、DAP/COM13 与 Qt 端到端功能路径已 **[Hardware Verified]**；
这里不推断具体 O-ring、螺纹或密封结构。传感器到解码板的连接器/线束在被
触碰或扰动时曾导致异常值或 Qt Stale，重新压紧/就位后恢复，因此连接器
retention、strain relief、布线检查、适用的 sealing 与装配后 continuity/
stability test 仍为 **[Pending mechanical/electrical integration follow-up]**，
不将其归因于 Firmware，也不声明其已达到 production-ready。zeroing、最终
installed reference point、fresh/seawater density、body installation offset
与 pool accuracy 同样保持 **[Pending]**。本地 `ms5837.py` 仅是 Raspberry Pi
直连 MS5837 的 I2C/PROM/ADC/补偿/density 参考，不证明解码板 UART 格式、
cadence 或电气接口，Firmware 不引入第二条 I2C 路径。

稳定连接实测记录：Qt 状态为 `Receiving`，depth 连续更新，temperature 约
24 °C 且数值合理，sample age 持续刷新，RX bytes 与 valid lines 持续增加，
parse errors 约为 0/极低，overflow 与 hard re-arm 均为 0。该记录证明功能
路径，不替代连接器可靠性、安装密封或绝对深度标定。

### DepthSnapshot contract

`DepthSnapshot` 是 unacknowledged、独立 telemetry sequence 的 Protocol V2
消息，message ID `0x22`，payload 固定 38 bytes，schema `1`，little-endian。
byte 0 为 schema；byte 1 的 bit 0/1 分别为 depth/temperature valid，其他位
必须为零；bytes 2–5 为 `depth_mm:int32`，6–7 为
`temperature_centi_c:int16`，8–9 为 `sample_age_ms:uint16`（无 sample 或
saturation 为 `0xffff`），10–37 为七个 `uint32` diagnostics：RX bytes、
valid lines、parse errors、overlong lines、RX ring overflows、hard re-arm
failures、UART errors。无效 numeric field 必须编码为零；Qt 以本地 packet
arrival 与既定 telemetry lifecycle 判断 liveness，不单独依赖 sample age。

Depth telemetry 不满足 ACK、不改变 LeakStatus、不进入 Servo command queue，
也不改变 Safety 行为。Firmware 使用 provisional 3000 ms sensor freshness，
超时后清除 depth/temperature valid 并将数值编码为 0，但保留 diagnostics；
该策略独立于一秒 publication 与 Qt host-packet stale timeout。调度上 due
LeakStatus 立即优先，Leak 不 due 时只在 IMU/Depth 之间公平轮转。现有
Protocol V2/USART1/JY901S/Leak 路径保持原边界。

### 验证分层

| 项目 | 状态 |
|---|---|
| Host Test | **PASS**：全部当前 Firmware regressions、Depth parser/transport/codec、Console CTest 与 Depth monitor/controller/MainWindow tests |
| Console CTest | **PASS** |
| HAL / `.ioc` / C portability checks | **PASS**：生成式 USART6 配置与直接标准头审计通过 |
| ARM Build | **PASS**：matching PR #12 STM32CubeIDE/CMake Debug target build，0 errors / 0 warnings |
| Program Verify | **PASS**：known-good DAP/OpenOCD flow 报告 `Programming Finished`、`Verify Started`、`Verified OK` |
| Hardware Verified | **PASS**：stable connection 下 ROVMAKER decoder → PC7/USART6 → DepthSnapshot → USART1/DAP/COM13 → Qt |
| Sensor-to-decoder connector/harness robustness | **Pending**：触碰/扰动会造成异常值或 Stale，需机械/电气集成 follow-up |
| Absolute depth calibration / installed reference | **Pending** |
| External GitHub Review | **Resolved for PR #12 closeout** |

本阶段与旧的 `FrontAxis`/Depth 舵机 PWM calibration window 是两条不同的
范围：旧记录中的 Depth 是舵机语义/机械标定；本节的 Depth Sensor 是新的
ROVMAKER 串口传感器输入。两者不共享硬件验证结论。

## 2026-09-12 Servo calibration and software-limit update（PR #13）

### 当前状态

本轮冻结四个划水舵机统一的 logical joint angle convention：`0 degrees = mechanical neutral`；`+45 degrees = paddle 往后拨，产生前进推进方向`；`-45 degrees = 相反方向`。未来 gait/CPG 只输出 logical angle；左右镜像与 PWM 增减方向由 Servo calibration 层处理。该 contract 不涉及 CPG、gait 或 motion command 实现。

| Servo | -45 deg / -4500 cdeg | 0 deg / Neutral | +45 deg / +4500 cdeg | raw PWM command limits | evidence |
|---|---:|---:|---:|---:|---|
| `FrontRight` | `1000 us` — **[Symmetry-Derived / User Accepted]** | `1450 us` — **[Bench Measured]** | `1900 us` — **[Bench Measured]** | `1000–1900 us` | final paddle descriptor |
| `FrontLeft` | `2020 us` — **[Symmetry-Derived / User Accepted]** | `1580 us` — **[Bench Measured]** | `1140 us` — **[Bench Measured]** | `1140–2020 us` | final paddle descriptor; PWM decreases with positive logical angle |
| `RearRight` | `1110 us` — **[Bench Hardware Verified]** | `1570 us` — **[Bench Hardware Verified]** | `2030 us` — **[Bench Hardware Verified]** | `1110–2030 us` | final paddle descriptor |
| `RearLeft` | `1940 us` — **[Bench Hardware Verified]** | `1450 us` — **[Bench Hardware Verified]** | `960 us` — **[Bench Hardware Verified]** | `960–1940 us` | final paddle descriptor; PWM decreases with positive logical angle |

`FrontAxis`/用户界面 `Depth` 保持 PR #13 已批准的独立 actuator calibration，不是单独的 ROVMAKER depth sensor：`1060 us = -90 degrees face down`, `1745 us = 0 degrees vertical paddling`, `2430 us = +90 degrees face up`; measured 180 degree sweep. Software PWM command limits are `1060–2430 us`; software angle limits are `-90 to +90 degrees`; Neutral is `1745 us`; Console `calibrationPending` is `false`。这组事实不代表 hydrodynamic optimization、installed trim、autonomous depth-control calibration、magnetic/yaw calibration 或 final body-frame calibration。

Firmware 与 Console 两张独立 descriptor table 必须完全一致。对于 `FrontLeft`/`RearLeft`，calibration endpoint 顺序可以是下降的 `pulse(-45) → pulse(0) → pulse(+45)`，但 raw PWM numeric validation 必须使用升序 bounds；不得假设 `min_pulse < neutral < max_pulse`。

既有硬件事实保持不变（以下来自既有记录，不是本 feature 新 Firmware image 的 ARM Build、Program Verify 或 Hardware Verified 证据）：

- `FrontRight`：**[Historical Hardware Verified]**（既有记录/old image only；不验证本 feature image）；
- `FrontLeft`：**[Historical Hardware Verified]**（既有记录/old image only；不验证本 feature image）；
- `RearLeft`：**[Historical Hardware Verified]**（既有记录/old image only；不验证本 feature image）；
- `RearRight` STM32/A12 PWM output path：**[Historical Hardware Verified]**（既有记录/old image only；不验证本 feature image）；原 RearRight servo actuator/线束为 hardware fault，计划更换，不属于 Firmware bug。
- Depth 在 `1480/1500/1520 μs` 的既有台架运动方向观察（PWM 减小 → front A 上翻，PWM 增大 → front A 下翻）不覆盖本轮新标定或新 Firmware image 验证。

上述 evidence labels 逐点适用：FrontRight 的 `1000 us` 与 FrontLeft 的 `2020 us` 是 **[Symmetry-Derived / User Accepted]**，不是 Hardware Verified；RearRight/RearLeft 三点是用户完成实机检查后的 **[Bench Hardware Verified]**。本 feature 的新 Firmware image 没有独立的 ARM Build、Program Verify 或整机 Hardware Verified 证据；因此三项均保持 **[Pending]**。不得把既有 PR 或旧 image 的 PASS 复制到本 feature 状态；Host Test、ARM Build、Program Verify、Hardware Verified、Bench Hardware Calibrated 和 Symmetry-Derived / User Accepted 仍是不同证据类别。

未来数据流仅记录为 architecture boundary：`Motion Command → Gait / CPG Generator → Logical Joint Target → ServoService set_angle → Servo Calibration → PWM`。本 PR 不实现 gait、CPG、Motion command 或 Qt gait controls。

### 下一轮五舵机执行器验证计划

后续目标验证必须使用本 feature 对应的新 Firmware image：先完成 ARM Build，再 Program Verify，最后在台架 exercise 四个 paddle 的 `-45/0/+45 degrees`、各自 exact PWM endpoints、raw PWM boundaries 和 Neutral，以及 `FrontAxis/Depth` 的 `-90/0/+90 degrees`、`1060/1745/2430 us` 和 Neutral `1745 us`。在这些步骤有独立记录前，不能把本 feature image 的软件限位、角度动作或 Neutral 标为整机 Hardware Verified。该验证不覆盖 ROVMAKER 深度传感器、hydrodynamics、installed trim、autonomous depth control、magnetic/yaw 或 final body-frame calibration。

### 验证分层与烧录提醒

`Host Test`、`ARM Build`、`Program Verify`、`Hardware Verified`、`Bench Hardware Calibrated`、`Pending` 是不同证据层级，不能用笼统的 “tested” 互相替代。当前 feature evidence matrix：

| 项目 | 状态 |
|---|---|
| FrontRight / FrontLeft endpoint evidence | **[Bench Measured]** plus symmetry-derived endpoints **[Symmetry-Derived / User Accepted]** |
| RearRight / RearLeft three-point bench evidence | **[Bench Hardware Verified]** |
| FrontAxis/Depth user-provided actuator measurements | **[Bench Hardware Calibrated]** |
| This feature's new Firmware image — ARM Build | **[Pending]** |
| This feature's new Firmware image — Program Verify | **[Pending]** |
| This feature's new Firmware image — Hardware Verified | **[Pending]** |

已验证的 DAP/OpenOCD 稳定流程为 `SWD clock 100 kHz → SYSRESETREQ → halt → program → verify`；若烧录后 UART 异常，先完整断电再上电，不加入软件 workaround。

## 2026-09-13 Motion / SimpleGait 第一版基础（Bench-Provisional；本 feature）

本 feature 在 PR #13 五舵机 semantic descriptor 基础上加入第一版 Motion / Gait
控制基础，完整 contract 见 [`docs/motion-simple-gait.md`](docs/motion-simple-gait.md)。
当前只声明 Host Test / software evidence；本 feature image 的 ARM Build、Program
Verify 和 physical Motion exercise 尚未在本轮执行，仍为 **Pending**。

### STOP contract

- Protocol V2 `SetMotionMode` 使用 `0x15`，payload 严格为 `schema=1, mode, action`。
- 普通 `STOP` ACK 只表示 stop request accepted；Firmware 立即进入
  `MOTION_STOPPING`，不是已经回到 neutral。
- 在集中配置的 `MOTION_TRANSITION_DURATION_MS=750U`、10 ms cooperative tick
  内，amplitude/bias 与五个 logical joint targets 平滑收敛到零；最后一次
  neutral write 完成后才释放 Motion ownership 并进入 `MOTION_STOPPED`。
- STOPPING 期间 manual Enable/SetPWM/SetAngle/Neutral 必须返回 `BUSY=7`，不能
  与停止轨迹争夺 actuator ownership。
- Disable/Disable All、heartbeat/host-liveness loss 和现有 SafetySupervisor
  fail-safe 路径立即 abort/disable，不等待 750 ms。`app_main` 先处理 Safety，
  Protocol Dispatcher 对显式 Servo Disable 先 abort Motion。
- 本仓库当前 PA11 leak path 明确是 monitoring-only，尚无 leak-to-Safety trip；
  如果后续加入 trip，必须复用同一 immediate takeover path，不能走 graceful ramp。
- 被中断的 STOP 不会在 reconnect/heartbeat recovery 后自动 resume；必须重新
  Enable 并发送新的 Motion START。STOP 在 STOPPED 时安全幂等。

### 实现边界与验证

```text
0x15 Motion command
  → MotionManager
  → SimpleGaitGenerator
  → logical joint targets (cdeg)
  → ServoService Motion ownership
  → Servo calibration / PWM
```

Bench-provisional profile 当前为 0.5 Hz、1000 cdeg paddle amplitude、π 前后足
phase、turn reduced-side 50%、ASCEND/DESCEND 的 ±1000 cdeg FrontAxis candidate
bias；rear operational clamp 为 −3000…+4500 cdeg，全部集中在
`RoboBeetleFirmware/Core/Motion/motion_config.h`。这不是 pool/hydrodynamic
calibration，也不等同于 ARM 或实机证据。

Firmware host runner：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
  -File .\RoboBeetleFirmware\tests\run_host_tests.ps1
```

本 feature 的 runner 通过 19 个 executable tests 与 1 个 app_main compile
contract，覆盖 STOP state chain、acceptance ACK、约 750 ms duration、target
monotonic convergence、manual BUSY、Disable All/heartbeat immediate takeover、
reconnect no-auto-resume、STOP idempotency、duplicate cache 和 exact `0x15` wire
payload。Qt CTest 另覆盖 Controller/UI lifecycle 及 APC220 bounded-queue safety
priority。具体当前执行结果记录在 feature branch 的 PR 描述和
`docs/motion-simple-gait.md`；Hardware evidence 仍 Pending。

## [Historical Reference] 2026-09-08 五舵机语义 descriptor bring-up（PR #8 snapshot；current contract updated 2026-09-12）

### 历史状态（2026-09-08 snapshot）

PR #8 将 Firmware 与 Qt 各自维护的 descriptor table 冻结为同一组五舵机语义 ID，supported mask 固定为 `0x001F`。descriptor drift 由两端独立测试分别拦截；Firmware 的 `servo_descriptor` 保持 pure C / HAL-independent，抽象 timer/channel 由 `servo_driver_stm32` 映射为 HAL handle 和 channel 常量。

| ID / mask | 语义 | 硬件与 STM32 输出 | 能力 / bring-up 范围 |
|---:|---|---|---|
| `0` / `0x0001` | `FrontRight` | SAVOX SW-0250MG+，TIM3_CH1 / PA6 | PWM 1050–1950 μs；Set Angle −45…+45°；电气 1000/1500/2000 μs |
| `1` / `0x0002` | `FrontLeft` | SAVOX SW-0250MG+，TIM3_CH2 / PA7 | PWM 1050–1950 μs；Set Angle −45…+45°；电气 1000/1500/2000 μs |
| `2` / `0x0004` | `FrontAxis` / 升潜前足轴 | HDKJ S3150D，TIM3_CH3 / PB0 | 电气 500/1500/2500 μs；命令仅 500–2500 μs；Set Angle disabled；`Calibration Pending` |
| `3` / `0x0008` | `RearRight` | GDW IPX896HV，TIM4_CH1 / PD12 | PWM 1020–2020 μs；Set Angle −45…+45°；电气 520/1520/2520 μs |
| `4` / `0x0010` | `RearLeft` | GDW IPX896HV，TIM4_CH2 / PD13 | PWM 1020–2020 μs；Set Angle −45…+45°；电气 520/1520/2520 μs |

TIM3/TIM4 当前均约 333 Hz、1 μs tick（PSC=15、ARR=3002）。FrontAxis 卖家参数记录为 500–2500 μs、中心候选 1500 μs、工作电压 4.8–7.4 V、可控行程 0–270°、死区 4 μs；这些是电气/绝对能力元数据，当前用户命令窗口为 provisional 500–2500 μs endpoint exploration，并非最终机械安全端点。1500 μs 只是 provisional startup/center candidate，不是 calibrated Neutral，也不是 Hardware Verified；扩展窗口的完整端点验收仍 Pending。卖家参数页写“是否防水：否”，商品照片/壳体却标示“Water proof Robot Servo”，因此 Waterproof capability = **[Unverified]**，在获得可靠 IP/密封证据前不得声明或安排直接浸水。

多 bit Enable 采用 all-or-nothing：调用前已 enabled 的 requested channel 完全跳过，不产生 write/start/stop；任一新 channel start 失败时只 stop 本次 newly started channel，并保持调用前 logical/physical state。Console 中 pending Disable 是 motion-command barrier：受影响舵机的 PWM、Neutral、Set Angle 在 Controller 层即被拒绝，不写帧、不进入当前 Protocol V2 host-link queue；Disable Error/timeout 不会释放 Disable 之后的 stale motion。

### PR #8 pre-PR10 Firmware path（Historical Reference）

```text
USART1 IRQ → HAL callback → uart_transport_stm32 → ring_buffer
  → app_main_process → Protocol V2 decode → protocol_dispatcher
  → servo_service → servo_driver_stm32
  → TIM3_CH1/PA6 FrontRight | TIM3_CH2/PA7 FrontLeft
  → TIM3_CH3/PB0 FrontAxis | TIM4_CH1/PD12 RearRight
  → TIM4_CH2/PD13 RearLeft
```

本 2026-09-08 snapshot 的 descriptor/service/driver/dispatcher 软件验证是实现门槛；ARM Build：**[Pending]**；Program Verify：**[Pending]**；Hardware Verified：**[Pending]**。历史 PR #1、PR #3/6 的 Servo1/PA6 硬件证据不自动覆盖新布局，也不验证 2026-09-12 current feature image。

### Layout compatibility break（必须显式隔离）

历史 v0.4 的 Servo1/PA6 bring-up 对象是 `RearLeft`；PR #8 后 PA6/ID0 正式为 `FrontRight`，`RearLeft` 改为 PD13/TIM4_CH2。五舵机重新布线后，禁止旧 v0.4 Console/Firmware 与 PR #8 layout 交叉使用。Qt 的 `ServoId::Servo1` 若存在，仅是 deprecated source alias 指向 `FrontRight`；新 UI、日志、实现和文档必须使用 semantic name，且不保留 `Servo2` alias。

## [Historical Reference] 2026-09-08 APC220 half-duplex scheduler hardware acceptance（PR #7；当前未启用）

### 历史状态（当前未启用）

APC220 Half-Duplex Scheduler：**[Hardware Verified - Bench]**

以下仅保留早期 APC220 桌面台架的历史记录；它不属于近期 COM13/DAP/USART1 的硬件验证，也不代表 APC220 是当前启用的链路：

以下仅保留早期 APC220 桌面台架记录；不属于近期 COM13/DAP/USART1 验证，也不代表 APC220 是当前启用链路：

- APC220 440 MHz 双端链路，以及 Qt → APC220 → STM32 → ACK → APC220 → Qt 完整闭环；
- 60 s idle Heartbeat、Servo1 Enable + ACK、Neutral、0° → +10° → 0° → −10° → 0°、±45°、±90°；
- 连续快速 Set Angle / scheduler queue；
- Disable / Disable All safety priority；
- 主动断开机器人端 APC220、Heartbeat loss/retry、Firmware watchdog safe-disable；
- APC220 恢复后不自动 Enable，必须 fresh Enable + matching ACK 才恢复控制。

正常桌面链路 CRC errors = 0，ACK RTT 约 160–170 ms（全部记录约 160–173 ms）。故障注入期间出现的 Retry / Timeout 属预期安全测试行为，不计入正常链路 timeout 统计；现场未观察到 stale command replay 或自动 re-arm。

### APC220 当前实现边界

- stop-and-wait：同一时刻最多一个 ACK-requiring request in flight；
- Heartbeat coalescing；
- dispatch-anchored Heartbeat safety admission；
- Heartbeat loss fail-closed；
- stale actuator queue purge；
- Disable / Disable All priority；
- liveness recovery 不自动恢复 Servo enabled state；
- fresh Enable + matching ACK 才能重新 armed。

以下参数仍为 **[Provisional]**，不应解读为最终 RF 参数：

- APC220 Heartbeat target = 250 ms；
- ACK timeout = 250 ms；
- Console host-side/local safety admission budget = 490 ms。

本轮仅完成当前桌面环境的 Hardware Verification；实验室水池边、距离、天线姿态和户外 RF characterization 尚未完成。490 ms 是 Console host-side/local safety admission budget，是本地调度准入策略，不是 Windows + RF hard-real-time guarantee。

## [Historical Reference] 2026-09-07 Firmware modularization hardware acceptance（PR #2–#6 old images）

本节仅是 PR #2 至 PR #6 old images 的历史证据，不是当前 Firmware 结构和验收状态的权威摘要。其 PR #3 Servo Service / Calibration 与 PR #6 App/Main 的 Hardware Verified 结论只适用于各自 old image；它们不验证当前 `feature/servo-calibration-depth-limits` branch image，也不改变本 feature 的 ARM Build、Program Verify、Hardware Verified = **[Pending]** 状态。下方更早的审计/基线章节继续保留作为 Historical Reference。

### PR #2–#6 old-image historical module status

- UART Transport / Ring Buffer — **[Historical Hardware Verified]**（PR #2 old image only；不验证当前 feature/branch image）
- Servo Service / Calibration / STM32 Driver — **[Historical Hardware Verified]**（PR #3 old image only；不验证当前 feature/branch image）
- Safety Supervisor — **[Hardware Verified]**（PR #4）
- Protocol Dispatcher — **[Hardware Verified]**（PR #5）
- App/Main orchestration — **[Historical Hardware Verified]**（PR #6 old image only；不验证当前 feature/branch image）

PR #6 old image 的 STM32CubeIDE Build、ST-LINK Download 和 Full physical regression 均 PASS。验收覆盖 cold boot/reset 后 Servo 不自动 Enable、Heartbeat、Enable/ACK、Neutral、Set Angle 0°/±10°/±45°/±90°、Set PWM 1520 us、Disable/Disable All、重新 Enable、Disconnect、严格超过 500 ms 的 safe disable、Reconnect 不自动 Enable、手动 Enable + ACK 恢复及第二次 Disconnect/Reconnect；该 PASS 不验证当前 feature/branch image。

### 2026-09-07 pre-PR10 Firmware path（Historical Reference）

```text
USART1 IRQ
  → HAL callback
  → uart_transport_stm32
  → ring_buffer
  → app_main_process
  → Protocol V2 decode
  → protocol_dispatcher
  → servo_service
  → servo_driver_stm32
  → TIM3_CH1 / PA6
```

### 当前 Safety 路径

```text
Heartbeat
  → protocol_dispatcher
  → safety_supervisor
  → strict >500 ms timeout
  → app_main safe-action wiring
  → servo_service_disable_all()
  → duplicate cache invalidation
```

### 当前 `main.c` 责任

- HAL/CubeMX startup
- peripheral initialization
- `app_main_init()`
- `app_main_process()`
- thin UART callback delegate
- Error/assert handlers

当前 `main.c` 不再拥有 Protocol wire glue、ACK glue、RX drain、Safety/Servo orchestration 或应用状态；这些职责由 `Core/App/app_main.c` 协调并委托给既有模块。

## 2026-09-06 Servo1 Set Angle Qt UI + hardware acceptance（已合并 main）

> 本节记录从 `origin/main` 的 `v0.1.0-servo1-bringup` 基线创建、经 PR #1 合并到 `main` 的小型 Console 功能。仅修改 Console UI/helper/tests 与文档；未修改 Firmware、`.ioc`、Protocol V2 帧格式、Servo calibration 或 CMake 结构。以下均为 old-image/historical wiring evidence，不验证当前 `feature/servo-calibration-depth-limits` image。

### 状态标签

- **Neutral — [Historical Hardware Verified]**：既有 old-image 开发记录确认点击 Neutral 后 Servo1 回到机械零位附近（约 1520 μs）。
- **Set Angle protocol/controller — [Historical Hardware Verified]**：`0x13` 使用 `count=1, servo_id=0, angle_cdeg:int16 LE`；Console 只发送 cdeg，Firmware 继续负责 cdeg→PWM。
- **Set Angle Qt UI — [Historical Hardware Verified]**：Servo1 使用 −90.0…+90.0°、0.1° 步进、默认 0.0° 的 `QDoubleSpinBox`；仅在已连接、Servo1 supported、Enable ACK 且无 pending Disable 时可操作。Disable、Disable All 或断开会立即关闭角度控件；Servo2 仍为 `Unsupported / Planned`。
- **Set Angle real servo motion — [Historical Hardware Verified]**：受控 old-image 实机验收在 0°、±10°、±45°、±90° 全部通过。

PWM 输入框目前表示用户的调试输入值；Neutral ACK 后不会把它同步成“当前实际位置”，也不承诺始终等于硬件已确认位置。Commanded State / Telemetry UI 留待后续独立设计。

### Servo1 hardware acceptance（[Hardware Verified]）

- Hardware: GDW IPX896HV，`TIM3_CH1 / PA6`，约 333 Hz。
- 角度验收：0°、+10°、0°、−10°、0°、±45°、±90° 全部 PASS。
- 当前 bring-up 对应关系：−90° ≈ 520 μs、0° = 1520 μs、+90° ≈ 2520 μs；仍作为近似标定记录。
- 安全/UI 验收：Disable 后 Set Angle 禁用；Enable 未 ACK 前仍不可用；Enable + ACK 后恢复；Disable All/Disconnect 后禁用；Reconnect 不自动 Enable；手动 Enable + ACK 后恢复。

### 本分支软件验证

- Console fresh MinGW/Qt configure/build 通过。
- `protocol_tests` 通过。
- `robot_controller_tests` 通过，覆盖 −90/−45/0/+45/+90°→cdeg、完整 Set Angle payload、越界/未 Enable/Servo2 不发送及既有重试/ACK 行为。

## 2026-09-05 第二轮 Protocol V2 / Servo1 clean baseline（历史基线快照 / Historical Reference）

> 本节记录 feature/set-angle-ui 之前的 2026-09-05 baseline；其中“本轮”仅指该 baseline 修复。当前 Set Angle UI 状态以上方 2026-09-06 节为准。该 baseline 未连接串口、未烧录 MCU、未驱动舵机，也未开展 Servo2、Pi/TCP、Camera/FOMO、ROS2、CPG 或 STM32 模块化工作。

### 本轮结果

- RoboBeetleConsole 使用 Qt 6.11.2、MinGW-w64 GCC 13.1.0、CMake 3.30.5、Ninja 1.12.1 在全新目录完成 configure/build。
- Console `protocol_tests` 与 `robot_controller_tests` 均通过。覆盖 COBS/CRC、520/1520/2520 μs、无效 PWM、Servo1 Enable、Servo2 拒绝、Neutral 编码/ACK、−9000/0/+9000 cdeg、越界角度、ACK sequence/type 匹配和同帧 retry。
- RoboBeetleFirmware 使用 STM32CubeIDE 2.2.0 自带 GNU Tools for STM32 14.3.1、CMake 4.3.1、Ninja 1.13.2 完成全新 Debug configure/build，链接 `RoboBeetleFirmware.elf`，编译输出无 warning。
- Firmware 独立纯 C golden-vector 测试以 `-Wall -Wextra -Werror` 编译并通过；未向 CMake 工程加入测试框架。

### 冻结后的 Servo1 协议语义

| 项目 | 当前实现 |
|---|---|
| 支持范围 | 仅 Servo1：ID 0、mask `0x0001`、TIM3_CH1/PA6 |
| PWM | 520–2520 μs（含边界），neutral 1520 μs |
| Set Angle `0x13` | `count:u8, servo_id:u8, angle_cdeg:int16 LE`；Console 只发 cdeg，Firmware 使用 `int32_t` 中间量映射 −9000→520、0→1520、+9000→2520 μs；越界返回 `OutOfRange`，不 clamp |
| Neutral `0x14` | `servo_mask:uint16 LE`；要求有效 Heartbeat、合法 mask、Servo 已 Enable；写 1520 μs但不关闭 PWM、不清除 enable |
| unsupported | mask 为 0 返回 `InvalidPayload`；包含 `0x0002` 或任何未知 bit 返回 `UnsupportedServo`，且不执行 Servo1 部分动作；PWM/Angle 的 ID 非0同样失败 |
| ACK result | `OK=0, InvalidPayload=1, HostNotAlive=2, UnsupportedServo=3, ServoNotEnabled=4, OutOfRange=5, HardwareFailure=6` |

### Retry / duplicate 行为

Console 的重试继续复用同一 sequence 和完整 wire frame。Firmware 保留最近一条成功的非 Heartbeat 请求 `{sequence,type,result}`；相同 sequence+type 的直接重试只重发缓存 ACK，不重复 Enable/PWM/Angle/Neutral/Disable 动作。Heartbeat 每次合法到达仍刷新本地在线时间，且不会挤掉动作缓存，因为 100 ms Heartbeat 周期短于 200 ms ACK timeout。500 ms watchdog 超时会停止 PWM、清除 enabled mask、清除 host alive，并同时使 duplicate cache 失效，恢复后仍必须显式 Heartbeat + Enable。

这是 Phase 1 的一项缓存，不是多请求 replay window；不同的成功非 Heartbeat 请求会替换旧缓存。由于只保留最近项，经过其他成功请求后的 16-bit sequence 回绕不会匹配久远请求。

### UI 与硬件验证边界

- Console 主控制器已实现 Set Angle 编码，但本轮没有增加交互角度控件；按钮显示 `Set Angle — UI Not Enabled` 并保持禁用。
- Servo2 面板标记 `Unsupported / Planned`，连接后也不能 Enable。
- 已有 Set PWM/Servo1 实机里程碑继续保留。Neutral 和 Set Angle 本轮只有软件构建/测试证据，必须在安全台架另行验证后才能标记 **[Hardware Verified]**。

### 本轮文件边界

修改了 Console 的协议/控制器/UI源码与两份测试、Firmware 的 `main.c` 与协议结果枚举、新增一个独立 Firmware golden-vector 测试，并同步四份指定 Markdown。活动 `.ioc`、全部 CMake 文件、CubeMX 生成的硬件初始化、USART 9600 8-N-1、TIM3 配置、基础帧、Magic/Version、CRC/COBS 和 `resource/` 均未修改。

## 2026-09-05 第一轮只读审计快照（已由上节修复结果取代）

> 本节保留修复前证据链，便于追溯第二轮为何修改。凡涉及 build blocker、Neutral 缺失、unsupported mask 假成功、duplicate cache 缺失和测试失败的描述，均为第二轮修改前状态，不再代表当前源码。

> 审计对象：`D:\RoboBeetle\RoboBeetleConsole`、`D:\RoboBeetle\RoboBeetleFirmware`、`RoboBeetleFirmware\RoboBeetleFirmware.ioc`。  
> 方法：逐文件静态审计、Console 现有测试二进制执行、Firmware 使用现有编译参数做 `-fsyntax-only`。未访问串口、烧录 MCU 或驱动舵机。  
> 范围：本轮只更新 Markdown；未修改 `.c/.h/.cpp/.hpp/.ioc`、CMake、Qt UI、协议字段或舵机行为。

### 状态标记

- **[Implemented]** 当前源码或活动 `.ioc` 可直接证明。
- **[Hardware Verified]** 当前开发记录明确记载已在实机验证；并非仅由源码推断。
- **[Provisional]** Bring-up 临时范围、初步标定或未完成接口。
- **[Planned]** 后续设计目标，当前尚未实现。
- **[Historical Reference]** 下方 2026-08-30 旧论文/旧资源审计及 `resource/` 内容；除非被当前源码或开发记录再次确认，否则不代表现机。

### 事实来源变更

2026-08-30 的审计发生在找回当前源码之前，因此其“无代码/协议未知”结论现在已经过时。当前事实依据按优先级为：

1. `RoboBeetleConsole` 与 `RoboBeetleFirmware` 当前源码；
2. 活动 CubeMX 文件 `RoboBeetleFirmware\RoboBeetleFirmware.ioc`；
3. 当前开发记录中的实机里程碑；
4. `resource/` 中的论文、PPT、旧 STM32/Simulink/CPG 代码，仅作历史参考。

用户提及的根目录 `D:\RoboBeetle\RoboBeetle.ioc` 在本次审计时不存在；不得用旧 F407ZE 配置替代当前活动 `.ioc`。

### 当前总体架构

当前实机 host-link 的权威路径是：
`Qt Console → Windows COM13 → DAP UART/USB serial bridge → STM32 USART1 → Protocol V2`。
APC220 只作为早期/legacy 记录保留；近期 Servo、LeakStatus 与 JY901S 运行
没有使用 APC220。

```text
[Implemented / Hardware Verified current direct path]
Windows Laptop
  └─ Qt 6.11.2 / C++20 RoboBeetleConsole
      ├─ MainWindow
      ├─ RobotController
      ├─ Protocol V2 codec / retry / monitor
      └─ SerialTransport
            ↕ Windows COM13 → DAP UART/USB serial bridge @ 9600 8-N-1
        STM32F407VET6 USART1 PA9/PA10
          ├─ IRQ + 128-byte ring buffer
          ├─ Protocol V2 dispatcher / ACK / heartbeat watchdog
          ├─ Leak D0 PA11 → LeakStatus `0x20` monitoring telemetry
          └─ five-servo descriptor/service/driver path
                → TIM3/TIM4 PWM outputs

JY901S TX → PB11 / USART3_RX → one-byte interrupt RX
  → independent 256-byte ring → 11-byte parser
  → Acc/Gyro/Angle state → ImuSnapshot telemetry → USART1 host-link

[Planned future path]
Windows Laptop Qt Console
        ↕ TCP
Raspberry Pi Onboard service
        ↕ UART
STM32 Protocol V2 + Safety + Servo Calibration
```

Console 已把 UI、命令/ACK 状态机、协议编解码和字节传输分开；`ITransport` 足以支撑未来增加 `TcpTransport` 的第一步。仍需调整的边界是串口专用的 `TransportConfiguration::baudRate`、MainWindow 的 COM/baud UI、位于 `RobotController` 的串口发现注入，以及两跳链路中的 ACK/重连/序列号所有权。

### 当前 MCU / UART / PWM 事实

| 项目 | 当前事实 | 证据 |
|---|---|---|
| MCU | STM32F407VET6，LQFP100 | 活动 `.ioc`、`STM32F407xx` 构建定义、启动/链接文件 |
| SYS / SWD | HSI 16 MHz、PLL off；PA13 SWDIO、PA14 SWCLK | `.ioc` + `SystemClock_Config()` |
| USART1 | PA9 TX、PA10 RX，9600、8-N-1、无流控 | `.ioc` + HAL 初始化/MSP |
| 当前 host-link | Qt Console → Windows COM13 → DAP UART/USB serial bridge → USART1 | 最新 Servo、LeakStatus、JY901S 实机记录 |
| JY901S RX | PB11 / USART3，one-byte interrupt RX，独立 256-byte ring | `.ioc` + HAL MSP/IRQ + matching PR #11 实机记录 |
| USART1 NVIC | 已启用，0/0 优先级 | `.ioc` + `HAL_NVIC_SetPriority` |
| TIM3_CH1 | PA6 / AF2，PWM mode 1，active high | `.ioc` + HAL MSP |
| PWM 计数 | PSC=15，ARR=3002，CCR1 初始=1520 | `.ioc` + `MX_TIM3_Init()` |
| PWM 频率 | 16 MHz/(15+1)/(3002+1)≈333.0 Hz，1 μs/tick | 由当前时钟/定时器配置计算 |
| 上电输出 | TIM 配置完成但不启动 PWM；首次有效 Enable 才写 1520 并 Start | 当前源码 |

### UART 与安全链

```text
USART1_IRQHandler
  → HAL_UART_IRQHandler
  → HAL_UART_RxCpltCallback
  → uart_rx_push + re-arm HAL_UART_Receive_IT(1 byte)
  → main loop uart_rx_pop
  → protocol_feed_byte
  → COBS/CRC decode
  → protocol_handle_frame
```

- 环形数组 128 字节，保留一个空槽区分满/空，实际可用 127 字节。
- 满时静默丢弃新字节，无 overflow 计数；RX 重挂返回值也未检查。
- ISR 只推入字节并重挂接收；解析、ACK、PWM 均在 main loop。
- ACK 使用 `HAL_UART_Transmit(..., 100 ms)`，是 main-loop 阻塞发送，不在 ISR 中。
- 有效 Heartbeat 更新本地 `last_heartbeat_rx_ms` 并设置 `host_alive`。
- 超过 500 ms 未收到有效 Heartbeat 时停止 Servo1 PWM、清空 enable mask；恢复后必须重新 Heartbeat + Enable。
- 没有独立硬件看门狗、Emergency Stop、漏水安全响应、电池/过流输入或持久故障记录；Leak D0 按轮询方式更新内部状态，并通过 PR #9 的 `LeakStatus (0x20)` 做 monitoring-only 遥测。该遥测不触发 Servo/Safety 动作；PA11 → LeakStatus → Qt 端到端路径已 Hardware Verified。

### Console ↔ Firmware Protocol V2 结论

双方的 Magic `52 42`、Version `02`、小端 Sequence/PayloadLength、64 字节最大 Payload、CRC-16/CCITT-FALSE、COBS 和 `00` delimiter 一致。详细逐字段表见 `RoboBeetleConsole/docs/protocol.md`。

| Message | ID | 当前一致性 |
|---|---:|---|
| Heartbeat | `01` | 一致；Console 100 ms 发送，Firmware 更新 host alive 并 ACK |
| ACK | `02` | 活跃方向一致；4 字节 payload，按 request sequence/type 匹配 |
| Error | `03` | Console 能解析；Firmware 仅定义枚举，不产生/处理 |
| Servo Enable | `10` | Servo1 一致；Firmware 会对未实现 Servo2/未知 mask 也返回成功 |
| Servo Disable | `11` | Servo1 一致；Servo2/未知 mask 存在同样语义偏差 |
| Set Servo PWM | `12` | Servo1 `count=1,id=0,uint16 LE` 和 520–2520 范围一致；Servo2 不支持 |
| Set Servo Angle | `13` | 两端定义；Console 主动禁用，Firmware 分支因三个脉宽宏未定义而无法构建 |
| Neutral | `14` | **不一致**；Console 发送，Firmware 无 case，返回 result 1 |

Console timeout 为 200 ms，原发送后最多重试 3 次，并复用相同 sequence/frame。Firmware 当前没有 duplicate cache 或 ACK replay；重复 Servo Enable 会再次把 CCR1 写回 1520，属于需要优先关闭的执行器副作用风险。

### Servo1 与标定状态

- **[Implemented]** Servo1 ID 0 / mask `0x0001` 映射 TIM3_CH1 / PA6。
- **[Implemented]** Console 与 Firmware 的当前 PWM gate 均为 520–2520 μs，Enable 安全起点 1520 μs。
- **[Provisional]** 约 −90°=520 μs、0°=1520 μs、+90°=2520 μs；不是最终精密标定。
- **[Hardware Verified]** 当前开发记录说明 GDW IPX896HV 已真实运动，舵盘已在约 1520 μs 机械对中。
- 商家资料记录：PWM、333 Hz、约 500/1500/2500 μs、180°±5°、4.8–8.4 V；它是规格参考，不替代逐台标定与机械限位。

### 当前实机里程碑

下列 **[Hardware Verified]** 结论依据当前开发记录，而不是静态代码单独证明：

| 里程碑 | 代码交叉证据 |
|---|---|
| Qt 6 Console 可启动 | 已有 Qt 6.11.2 MinGW 构建产物 |
| SerialTransport 工作，COM13 → DAP UART/USB serial bridge ↔ USART1 跑通 | 当前 SerialTransport/USART1 配置与最新开发记录一致；APC220 不是本次链路 |
| STM32 → PC ASCII bring-up 曾验证 | 仅开发记录；ASCII 路径已不是当前 Protocol V2 主路径 |
| PC ↔ STM32 UART 双向通信 | RX 中断链 + Firmware ACK TX 源码 |
| USART interrupt + ring buffer | 当前源码直接实现；实机成功来自开发记录 |
| Protocol V2 COBS / CRC | 双端 codec 一致；现有 Console protocol tests 通过 |
| Heartbeat 与 STM32 ACK → Qt | 双端状态机源码一致 |
| Qt TX/RX packet 正常、记录中 CRC errors 保持 0 | Console monitor 源码 + 开发记录 |
| Servo Enable / Disable | 双端实现 Servo1；实机成功来自开发记录 |
| 500 ms heartbeat watchdog | Firmware 源码直接实现；实机结果来自开发记录 |
| Qt Set Servo PWM → TIM3 CCR | 双端 payload/dispatch 源码 + 开发记录 |
| TIM3 PWM 驱动 Servo1 / GDW IPX896HV 真实运动 | 开发记录 |
| 1520 μs 机械对中；520/1520/2520 初步标定 | 开发记录，标记为 Provisional |

### 2026-09-05 构建/测试状态

- Console 现有 `protocol_tests.exe`：通过。
- Console 现有 `robot_controller_tests.exe`：失败 3 项；测试仍把 1449/1551 μs 当作越界，而当前源码范围已是 520–2520 μs。
- Firmware 现有 ELF 生成于 2026-09-04，早于 2026-09-05 的当前 `main.c`，属于过期构建证据。
- Firmware `main.c` 使用现有 GCC 14.3.1 参数进行 `-fsyntax-only`：失败，`SERVO1_MIN_PULSE_US`、`SERVO1_NEUTRAL_PULSE_US`、`SERVO1_MAX_PULSE_US` 未定义。

因此，当前源码不能声称“全量构建和测试通过”。本轮按约束只记录问题，未修改实现或测试。

### `main.c` 可维护性与后续拆分（Historical Reference）

当前 `main.c` 同时承担 HAL/CubeMX 初始化、UART RX 回调、ring buffer、Protocol feed/dispatch、ACK TX、heartbeat、servo state、PWM start/stop/CCR、临时 calibration、500 ms safety watchdog、debug counters、GPIO 和主循环调度，职责已经过多。

推荐依赖方向：

```text
main.c
  → App/app_main
      ├─ Communication/uart_transport_stm32   [HAL-aware]
      ├─ Communication/ring_buffer            [pure C]
      ├─ Communication/protocol_dispatcher     [pure C/service interfaces]
      ├─ Protocol/rb_protocol_v2               [pure C]
      ├─ Servo/servo_service                   [pure C policy]
      ├─ Servo/servo_calibration               [pure C data/mapping]
      ├─ Servo/servo_driver_stm32              [HAL-aware TIM]
      └─ Safety/safety_supervisor              [pure C policy]
```

`main.c` 最终只保留 `HAL_Init`、`SystemClock_Config`、`MX_xxx_Init`、`App_Init`、`App_Process` 及立即委托的 CubeMX 回调。Phase 1 不需要拆分已经独立的 codec、不应急于引入 RTOS，也不需要在第二个硬件通道出现前建立大型通用设备框架。

### Set Angle 下一步建议

当前缺口是多项同时存在：Console UI 被禁用、Console method 不发送、没有共享 calibration/capability 模型、Firmware 标定宏缺失导致构建失败，且没有角度路径的双端/台架测试。

对于未来 Laptop Qt ↔ TCP ↔ Raspberry Pi ↔ UART ↔ STM32，以及 ROS 2/自动控制/CPG，建议采用：

> 上层统一发送物理角度 `angle_cdeg`，每台舵机的 angle→pulse calibration、方向、软限位和最终饱和由 STM32 执行。

理由是所有命令来源共享同一物理单位，Pi/网络断开不影响本地安全约束，标定靠近实际执行器，并可避免不同上层重复实现映射。Console 的原始 PWM 应保留为受保护的 bring-up/维修通道，而不是自动控制主接口。启用 Set Angle 前至少应：修复 clean build、定义每舵机标定结构与版本、增加纯 C 边界/单调性测试、冻结 result code/duplicate 语义、完成卸载台架验证，再解除 UI 禁用。

### 当前技术债与建议顺序

1. **P0** 经 Review 后恢复 Firmware clean build，补齐/统一 Servo1 角度标定常量；同步修正 Console 范围测试。
2. **P0** 决定 Neutral 是实现、映射为 1520 μs，还是从 Phase 1 能力中移除；保持双端一致。
3. **P1** 加入 Firmware duplicate request/ACK replay，确保重试不重复执行器副作用。
4. **P1** 对 Servo2/未知 mask 返回明确“不支持”，冻结 ACK result/Error contract。
5. **P1** 拆出 ring buffer/UART transport，增加 overflow 与 RX re-arm 错误观测；评估非阻塞 TX。
6. **P1** 分离 Servo HAL driver、ServoService/calibration 与 SafetySupervisor，并建立 host-side 纯 C 测试。
7. **P2** 完成 Set Angle 台架验证后才开放 Console UI。
8. **P2** 再引入 Pi/TCP，明确端到端 ACK、sequence、reconnect、capability 和安全所有权。
9. **P3** 在稳定遥测契约之后加入 Camera/FOMO、IMU、深度、漏水、电池、曲线与 3D；随后再接 ROS 2/CPG/自动控制。

---

## [Historical Reference] 2026-08-30 论文/旧资源审计

以下原始审计保留用于历史追溯。其“目录无源码”“协议未知”“STM32F407ZET6”等描述只对应当时审计到的旧论文资产，不得覆盖上方当前实现事实。

> 审计对象：`C:\Users\laixindong\Desktop\RoboBeetle\RoboBeetle`  
> 审计日期：2026-08-30  
> 审计方式：只读目录盘点、PDF 文本提取、关键页面视觉核验、静态关键词检索；未运行任何机器人程序，未访问串口、GPIO、PWM、CAN 或网络控制端口。  
> 重要限制：目标目录只有 2 份 PDF 和 1 个 MP4，**没有任何 MCU/Qt/Simulink 源文件、工程文件、配置、原理图、PCB、BOM、数据文件或 Git 元数据**。因此本文能确认的是“论文/文章描述了什么”，不能确认“当前固件实际做了什么”。

## Executive Summary

1. **真正产生运动的已知链路（文档确认）**：PC 上的 Qt 上位机通过两个串口和两对无线模块与机器人通信；机器人内的 STM32F407ZET6 解析上位机命令，定时计算四路步态角度并更新 PWM；四个防水舵机分别驱动左右中足和左右后足；10 个柔性被动关节在水阻和机械 stopper 作用下形成冲程/回程水动力不对称。来源：`高新义毕业论文.pdf` PDF pp.39–43（正文 pp.25–29，§2.5，图2-9至2-12）；`AIS论文/Advanced Intelligent Systems - 2025 - Gao - Design and Analysis for a Diving‐Beetle‐Inspired Swimming Robot with.pdf` PDF pp.3–5（§2.2–2.3，图2–3，表1）。
2. **真正的代码位置无法回答**：目录内无 `.c/.h/.cpp/.py/.m/.slx/.ioc/.pro/CMakeLists.txt` 等源码或工程文件。论文称 Simulink 可生成可在 STM32 上运行的 C 代码，但生成物和调用代码均未入库。任何函数名、命令 ID、字节序、校验、PWM 脉宽、舵机中位或限位都不能从当前目录确认。
3. **上位机/Pi 到底怎样控制底层**：已知链路为“Qt 上位机串口 → 无线串口模块 → STM32 内部串口”，GUI 截图显示接收 COM3、发送 COM8、两路均 9600 baud；期刊表1进一步写明机器人接收为 433 MHz、发送为 440 MHz。数据格式、8N1、包头、长度、命令码、ACK、CRC、超时和重连全部未知。COM3/COM8 只能视为截图当时的示例端口，不能当作现机配置。
4. **FOMO 最自然的接入点**：不是 PWM，也不是未经恢复的串口字节流，而是旧系统“期望偏航角/步态参数”进入闭环控制器之前的上位控制边界。推荐链路为 `TargetObservation → TargetSelector → image error → desired_yaw → IMU yaw controller → left/right amplitude differential → gait generator → servo PWM`。当前样机只有水面平面运动能力，`centroid_x` 可用于偏航，`centroid_y` 不能直接映射到上浮/下潜。
5. **首次真实运动前必须人工确认**：现机 MCU/固件是否与文档一致、无线模块和 USB 串口对应关系、协议抓包或源码、四舵机通道映射、每个舵机的中位/方向/脉宽/机械限位、摄像头安装朝向、yaw 正方向、4S 电池与降压实际输出、防水与断电方式，以及通信丢失后的安全行为。未确认这些信息前，不应从 FOMO 直接发送运动命令。

## 1. Repository / thesis asset inventory

### 1.1 目录事实

- 顶层只有一个 `AIS论文/` 子目录和一份硕士论文 PDF。
- 递归文件数：3；总大小：42,790,416 bytes。
- 目录不是 Git 仓库；`git status --short --branch` 和 `git log --oneline --decorate -20` 均返回 “not a git repository”。
- 未发现 README、Word、源码、ROS/ROS2、Arduino、STM32 工程、Matlab/Simulink 工程、Qt 工程、原理图、PCB、BOM、配置、日志、实验数据或固件二进制。

| 资产 | 类型/规模 | 内容与用途 | SHA-256 | 审计价值 |
|---|---:|---|---|---|
| `高新义毕业论文.pdf` | PDF，110页，6,474,124 bytes | 2025年硕士论文《一种仿生水下机器人设计与运动控制》；包含机电设计、软件框架、CPG、实验与抗偏航控制 | `7DBD3D58AA5BADFD913DD11B34460903C8E68F2DE23E0004D526AFEF8839D265` | P0，最完整系统描述 |
| `AIS论文/Advanced Intelligent Systems - 2025 - Gao - Design and Analysis for a Diving‐Beetle‐Inspired Swimming Robot with.pdf` | PDF，13页，7,419,267 bytes | 期刊文章；表1给出具体 MCU、IMU、舵机、电池和无线频率，§2.3给出 SDSG/PD 控制语义 | `02152DE3F7694E31892AE9A8EC85E37C4F6C305F7BD3C115FC69F486C07F9612` | P0，硬件型号最明确 |
| `AIS论文/AIS_2025.mp4` | MP4，28,897,025 bytes，约63秒，约315 kbps | 论文配套演示视频；目录未提供视频说明、时间轴或可引用文字 | `44B1100B7DD0999B6F88D4E42E76300AA9AB6B0DF779FB792B6AD9445FF72AF5` | P2，仅能作为运动现象佐证，不能建立控制协议 |

### 1.2 证据等级

| 标记 | 含义 |
|---|---|
| **CODE-CONFIRMED** | 被可审计源码直接确认；本目录没有任何此级证据 |
| **DOC-CONFIRMED** | 论文正文、表格或图中明确给出 |
| **SCREENSHOT-ONLY** | 只出现在 GUI/硬件照片截图中，可能是某次实验设置 |
| **INFERRED** | 从多项文档事实推导，但未获代码/接线验证 |
| **UNKNOWN** | 当前目录不能回答，禁止按常识补全 |

## 2. Hardware architecture

### 2.1 控制链

```text
PC / Qt host application
  ├─ TX serial port (GUI screenshot: COM8 @ 9600; current device UNKNOWN)
  └─ RX serial port (GUI screenshot: COM3 @ 9600; current device UNKNOWN)
          │
          ▼
two pairs of RF serial modules
  ├─ robot receive: 433 MHz (journal Table 1)
  └─ robot transmit: 440 MHz (journal Table 1)
          │ internal UART (electrical level/pins UNKNOWN)
          ▼
custom PCB + STM32F407ZET6
  ├─ serial ISR: command reception / telemetry transmission
  ├─ 100 Hz timer: gait calculation and servo angle/PWM update
  └─ 50 Hz telemetry transmission
          │ four PWM channels (frequency/pulse width UNKNOWN)
          ▼
4 waterproof servo motors
  ├─ 2 × Hitec HS-5086WP: middle appendages
  └─ 2 × GDW-BLS896: hind appendages
          │ direct servo-horn/screw connection
          ▼
4 active joints + 10 flexible passive joints + mechanical stoppers
          ▼
asymmetric power/recovery strokes
          ▼
surface forward motion / planar turning / yaw correction
```

来源：`高新义毕业论文.pdf` PDF p.40（正文 p.26，图2-9）明确 STM32F407ZET6、5V/3.3V 电源模块、姿态传感器、433M 无线模块、Qt 上位机和舵机组；PDF p.41（正文 p.27，图2-10）明确 9600 baud、100Hz 计算/舵机更新和 50Hz 上传；期刊 PDF p.5（表1）给出具体舵机、IMU 和双射频频率。

### 2.2 反馈链

```text
JY901S IMU
  → STM32 serial reception
  → actual yaw angle
  → yaw error = prescribed yaw - actual yaw
  → PD/PID-labelled controller (documentation conflict)
  → left/right appendage amplitude adjustment
  → gait generator (SDSG or CPG; documentation conflict)
  → PWM → servos → robot yaw motion

STM32 state / servo position / yaw
  → 440 MHz telemetry → host RX serial
  → Qt plots, status display and data logging

overhead global camera
  → recorded video
  → offline motion-capture analysis of speed/trajectory
  → NOT documented as a real-time feedback input
```

来源：期刊 PDF p.5（图3e）；硕士论文 PDF pp.91–92（正文 pp.77–78，图5-1）。论文明确全局相机采用离线视觉定位，不能把它描述成旧系统实时视觉伺服。

### 2.3 Documentation says / Code actually does / Assessment

| 项目 | Documentation says | Code actually does | Assessment |
|---|---|---|---|
| MCU 主控 | STM32F407ZET6 | 无代码 | 型号为 DOC-CONFIRMED，固件版本 UNKNOWN |
| 上位机 | Qt GUI，双串口、在线调参、显示和保存 | 无 Qt 工程 | 功能为 DOC-CONFIRMED，序列化实现 UNKNOWN |
| MCU 运动代码 | 定时器 100Hz 计算 CPG 并更新舵机；Simulink 生成 C 代码 | 无生成 `.c/.h` 或 STM32 工程 | 无法验证调用链、调度、数值类型或是否为最终运行版本 |
| 无线通信 | 433M 接收、440M 发送；内部串口 | 无驱动/协议代码 | 物理链路较可信，应用协议完全未知 |
| 抗偏航控制 | 论文部分位置写 PID，结论和期刊写 PD | 无控制器代码 | 控制器阶次和参数不能凭截图定案 |
| 视觉定位 | 顶视相机离线分析 | 无视觉代码/数据 | 旧系统没有可复用的实时视觉入口证据 |

## 3. Hardware BOM

| 模块 | 型号 | 作用 | 与谁连接 | 接口 | 电压/信号 | 证据位置 | 可信度 |
|---|---|---|---|---|---|---|---|
| MCU | STM32F407ZET6 | 中枢计算、串口、定时器、PWM、步态与反馈控制 | IMU、无线模块、4舵机、电源 | UART/Timer/PWM；具体引脚未知 | 3.3V rail shown | 期刊 PDF p.5表1；论文 PDF p.40图2-9 | DOC-CONFIRMED |
| 定制控制板 | custom PCB | 集成 MCU 与接口 | 同上 | UNKNOWN | 3.3V/5V rails shown | 期刊 PDF pp.3–4图2e；论文 PDF p.40图2-9 | DOC-CONFIRMED，版号未知 |
| IMU | JY901S | 测量 yaw，文章实验也显示 roll | STM32 | 论文称串口；具体 UART/pins 未知 | UNKNOWN | 期刊 PDF p.5表1；论文 PDF pp.41、98–100 | DOC-CONFIRMED |
| 中足舵机 ×2 | Hitec HS-5086WP | 驱动左右中足主动关节 | STM32 PWM、电源、中足 | PWM | 供电电压/逻辑阈值 UNKNOWN | 期刊 PDF p.5表1 | DOC-CONFIRMED |
| 后足舵机 ×2 | GDW-BLS896 | 驱动左右后足主动关节 | STM32 PWM、电源、后足 | PWM | 供电电压/逻辑阈值 UNKNOWN | 期刊 PDF p.5表1 | DOC-CONFIRMED |
| 无线接收链路 | exact model UNKNOWN | 接收上位机命令 | host TX ↔ STM32 UART | RF serial | 433 MHz | 期刊 PDF pp.3、5；论文 PDF p.40 | DOC-CONFIRMED frequency only |
| 无线发送链路 | exact model UNKNOWN | 回传状态/IMU/舵机信息 | STM32 UART ↔ host RX | RF serial | 440 MHz | 期刊 PDF pp.3、5；论文 PDF p.92 | DOC-CONFIRMED frequency only |
| 机器人电池 | exact model UNKNOWN | 整机供电 | DC-DC/PCB/舵机 | power | 4S Li-polymer，14.8V；论文写1400mAh | 论文 PDF p.39；期刊 PDF p.5表1 | DOC-CONFIRMED，容量存在上下文差异 |
| 降压模块 | UNKNOWN — repository does not establish the exact model | 产生逻辑/外设电源 | 4S battery → PCB/sensors/radio/servos | DC-DC | 5V、3.3V rails shown | 论文 PDF p.40图2-9 | DOC-CONFIRMED rails，额定电流未知 |
| 防水开关 | UNKNOWN | 整机上电/断电 | battery/system | mechanical/electrical UNKNOWN | UNKNOWN | 期刊 PDF p.4图2b；论文 PDF p.38文字提到 | DOC-CONFIRMED presence only |
| 上位机电脑 | UNKNOWN | Qt UI、指令、显示和保存 | 两个串口无线模块、全局相机 | serial/USB likely but unproven | screenshot ports COM3/COM8 | 论文 PDF pp.42–43图2-11/2-12 | SCREENSHOT-ONLY ports |
| 全局相机 | UNKNOWN | 顶视录制，离线速度/轨迹分析 | host/offline software | UNKNOWN | UNKNOWN | 论文 PDF pp.91–92图5-1；期刊 PDF pp.9–10 | DOC-CONFIRMED role |
| 机载相机 | 未发现 | — | — | — | — | 全仓库否定性搜索 | UNKNOWN / not documented |
| 主动关节 | 4个 | 每足一个舵机驱动 | servos → appendages | mechanical | angle in degrees | 论文 PDF pp.28、32–36 | DOC-CONFIRMED |
| 被动关节 | 10个 | 水阻下弯曲/展开，形成非对称阻力 | 中足2/条、后足3/条 | soft rubber + stopper | stiffness-dependent | 论文 PDF pp.32–37；期刊 PDF pp.4、7–8 | DOC-CONFIRMED |
| 壳体/密封舱 | resin shells, O-ring, bolts | 防水、容纳 PCB/电池/无线/传感器 | 上下壳 | mechanical | — | 论文 PDF pp.32、37–39 | DOC-CONFIRMED |
| 浮力单元 | exact material/model UNKNOWN | 使机器人稳定在水面、提供被动 roll 恢复 | 上壳 | hydrostatic | buoyancy ≈11N | 论文 PDF pp.37–38、99–100；期刊 PDF pp.4–5 | DOC-CONFIRMED |
| ATI 六维水下力传感器 | exact ATI model UNKNOWN | **实验台**推力/力矩采集，不是机载反馈 | acquisition box → host | UDP from acquisition box | 24V supply for acquisition box | 论文 PDF pp.54–60 | DOC-CONFIRMED test rig only |
| Raspberry Pi / Jetson | 未发现 | — | — | — | — | 全仓库盘点 | UNKNOWN / not part of documented old system |
| 深度/压力传感器、编码器、ESC、推进器 | 未发现 | — | — | — | — | 全仓库盘点 | UNKNOWN / not documented |

注意：论文 PDF p.60 的 **1500mAh 4S 电池**属于单足推力实验台舵机控制模块；整机 §2.5.1 写的是 **1400mAh 4S**。二者不能混为同一电池。

## 4. Software architecture

### 4.1 Host / Qt

文档中的 Qt 界面包含：外设设置、接收原始数据、应答区、yaw 曲线、四舵机数据曲线、命令下发、参数配置和数据保存。图2-11可见的按钮/字段包括：

- 接收串口 COM3、发送串口 COM8、两路 9600 baud（仅截图设置）；
- “使能舵机”“使能传感器”；
- “舵机安装位置”“舵机待机位置”；
- “左转”“右转”“直游”“直游2”；
- 偏航角、Kp/Ki/Kd、频率、前/后幅值和“开启PID”；
- 手工发送区、保存间隔和定时保存。

来源：`高新义毕业论文.pdf` PDF p.42（正文 p.28，图2-11）。这些标签能证明人机功能存在于论文截图，不能证明具体命令字符串或当前端口。

### 4.2 MCU 调度模型（文档描述）

| 执行上下文 | 工作 | 频率/优先级 | 证据 |
|---|---|---|---|
| startup once | 初始化串口、定时器、IMU和运动算法，进行初步配置/硬件检测 | 一次 | 论文 PDF p.41图2-10 |
| main loop | 获取 IMU 原始数据；轮询接收/发送状态 | continuous | 同上 |
| serial interrupt | 解析上位机命令；解析 IMU 数据 | event-driven | 同上 |
| timer interrupt | 迭代计算 CPG 参数；更新舵机输出角度/PWM | 100Hz | 同上 |
| telemetry | 发送数据至上位机 | 50Hz | 论文 PDF pp.41–43 |
| priority order | 舵机组定时器 > IMU 接收 > 串口发送 > 串口接收 | documented | 论文 PDF p.41 |

没有 RTOS 证据。论文把 STM32描述为“单线程处理器”，但中断优先级、临界区、缓冲区和数据一致性实现均无法验证。

### 4.3 候选步态实现

目录存在两套互相不完全一致的文档设计：

1. **SDSG 正弦驱动（期刊、论文第3章）**

   `theta_i,1 = A_i,1 * sin(2π f t + phi_i-1) + bias_i,1`，其中 `i=1..4`。状态向量为 `S=[A_i,1, f, phi_i,1, bias_i,1]`；`A/phi/bias/theta`按角度描述，`f`为Hz，`t`为秒。来源：期刊 PDF pp.4–5，式(1)；论文 PDF pp.47–48，式(3-4)。

2. **四单元 CPG（硕士论文第4章）**

   四个 CPG 单元分别控制四个舵机；输入为 gait、摆动周期 `T_i`、幅值 `R_i` 和足间相位关系；输出为 CPG1–CPG4 的角度。Simulink 固定步长 10ms，目标 100Hz；`ert.tlc` 自动生成 `.c/.h`，文档称加入 STM32 工程后调用自动生成函数。来源：论文 PDF pp.72–84（正文 pp.58–70，§4.2，特别是PDF pp.82–84）。

**无法确定哪套是现机最终固件。** 期刊稿较早且明确称实际样机采用 SDSG；硕士论文较晚并称 CPG 自动代码生成用于单片机，但源码缺失。最合理的审计结论是“可能发生了控制器迭代”，不是擅自把两者解释为同一实现。

## 5. End-to-end control dataflow

### 5.1 可由文档支持的真实系统级数据流

```text
operator selects gait / enters A, f, desired yaw, controller gains
  ↓
Qt host UI (exact class/function UNKNOWN)
  ↓
command serialization (format UNKNOWN)
  ↓
TX serial screenshot: COM8 @ 9600
  ↓
433 MHz robot-receive wireless link
  ↓
STM32 UART interrupt / command parser (function UNKNOWN)
  ↓
gait parameters: gait, amplitude, frequency, phase, bias
  ↓
SDSG or CPG gait generator (actual firmware version UNKNOWN)
  ↓
four target servo angles, degrees
  ↓
timer/PWM update at documented 100Hz (pulse representation UNKNOWN)
  ↓
two middle + two hind waterproof servos
  ↓
four active joints
  ↓
ten passive joints bend/extend under hydrodynamic load
  ↓
forward thrust or yaw moment
```

### 5.2 每一步的可审计字段

| 步骤 | 文件/位置 | 类/函数 | 输入 | 输出 | 单位/类型/范围 | 调用者→被调用者 |
|---|---|---|---|---|---|---|
| user command | 论文 PDF p.42图2-11 | UNKNOWN | button/fields | conceptual gait/params | UI numeric; serialization UNKNOWN | operator→Qt UI |
| serialize/send | 论文 PDF pp.42–43 | UNKNOWN | gait/params | bytes/strings UNKNOWN | header/ID/CRC UNKNOWN | Qt UI→serial |
| RF transport | 期刊 PDF p.5表1 | N/A hardware | serial stream | serial stream | 433M RX / 440M TX | host modules↔robot modules |
| receive/parse | 论文 PDF p.41图2-10 | UNKNOWN | serial bytes | command state | type/range UNKNOWN | UART ISR→motion state |
| gait generation | 期刊 pp.4–5 or 论文 pp.72–84 | generated function name UNKNOWN | `A,f,phi,bias` or `gait,T,R,phase` | 4 angles | degree, float/fixed type UNKNOWN; amplitude documented 0–45° | command state→gait generator |
| servo update | 论文 PDF p.41 | UNKNOWN | target angles | PWM parameters | 100Hz update; PWM carrier/pulse width UNKNOWN | timer ISR→PWM peripheral |
| actuator | 期刊 PDF p.5表1 | N/A hardware | PWM | shaft angle/torque | servo electrical/mechanical limits UNKNOWN | PWM→servo→active joint |
| body motion | 论文 pp.93–100 | N/A physics | gait/asymmetric thrust | forward/yaw | tested f=0.9/1.1/1.3Hz, A up to45° | appendages→water→body |

**结论：当前目录不足以给出函数级、行号级调用图。** 任何写成 `controller.c::foo()`、`parse_command()` 或 `set_pwm()` 的名字都会是编造。

## 6. Communication protocol

### 6.1 已确认/未确认

| 字段 | 值 | 状态 | 来源/说明 |
|---|---|---|---|
| host↔robot topology | two unidirectional RF serial paths forming full duplex | DOC-CONFIRMED | 论文 PDF p.92；期刊 PDF p.3 |
| robot receive RF | 433 MHz | DOC-CONFIRMED | 期刊 PDF p.5表1 |
| robot transmit RF | 440 MHz | DOC-CONFIRMED | 期刊 PDF p.5表1 |
| host RX port | COM3 | SCREENSHOT-ONLY | 论文 PDF p.42图2-11 |
| host TX port | COM8 | SCREENSHOT-ONLY | 同上 |
| baud rate | 9600, both GUI ports and MCU flowchart | DOC-CONFIRMED configuration used in document | 论文 PDF pp.41–42 |
| internal interface | serial/UART | DOC-CONFIRMED at conceptual level | 论文 PDF pp.40–42 |
| data bits/parity/stop bits | UNKNOWN | not found | 不能默认 8N1 |
| packet header/length/command ID/payload layout | UNKNOWN | not found | 无代码、协议文档或抓包 |
| checksum/CRC/endianness | UNKNOWN | not found | 全文否定性搜索无结果 |
| ACK/status response | conceptual “应答区/指令是否执行” only | DOCUMENTED ONLY | 论文 PDF pp.42、92；格式未知 |
| timeout/heartbeat/reconnect | UNKNOWN / not found | absent | 论文与期刊均未发现 |
| error handling/buffer overflow | UNKNOWN / not found | absent | 无源码 |
| telemetry rate | fixed 50Hz | DOC-CONFIRMED | 论文 PDF pp.41–43 |
| design communication requirement | ≥20Hz | DOC-CONFIRMED requirement | 论文 PDF p.28表2-1 |

### 6.2 文档可识别的命令语义（不是协议字节表）

| Command semantic | ID | Payload | 单位/范围 | 作用 | 发送端 | 接收端 | 证据状态 |
|---|---|---|---|---|---|---|---|
| enable servo | UNKNOWN | UNKNOWN | UNKNOWN | 允许舵机工作 | Qt host | STM32 | UI label only |
| enable sensor | UNKNOWN | UNKNOWN | UNKNOWN | 启用传感器 | Qt host | STM32 | UI label only |
| servo install position | UNKNOWN | UNKNOWN | position unit UNKNOWN | 装配位置动作 | Qt host | STM32 | UI label only |
| servo standby position | UNKNOWN | UNKNOWN | neutral/standby value UNKNOWN | 待机位置动作 | Qt host | STM32 | UI label only |
| forward / forward2 | UNKNOWN | gait + parameters likely, exact payload UNKNOWN | `f` Hz, `A` degree | 直游步态 | Qt host | STM32 | UI + thesis narrative |
| left turn / right turn | UNKNOWN | gait + parameters likely | `f` Hz, `A` degree | 平面转向 | Qt host | STM32 | UI + experiments |
| stop | UNKNOWN | UNKNOWN | UNKNOWN | 停止运动 | Qt host | STM32 | flowchart/text only |
| controller configuration | UNKNOWN | desired yaw, gains, f, front/hind amplitudes shown | degree/Hz; binary type UNKNOWN | yaw loop/step tuning | Qt host | STM32 | UI screenshot only |
| raw manual send | UNKNOWN | arbitrary text/bytes | UNKNOWN | test/debug sending | Qt host | STM32 | UI screenshot only |

没有任何证据能确认这些命令是字符串（例如 `F100`）还是二进制帧。

## 7. Actuator semantics

### 7.1 数量与通道

- 4 个主动关节：左右中足、左右后足各一个舵机。
- 10 个被动关节：每条中足2个、每条后足3个。
- 论文/期刊用 `i=1..4` 表示四个驱动单元，但**未提供 i 与物理方位的无歧义通道表**；图3e可见 SDSG1–4 的空间位置，仍需以固件/接线确认。

### 7.2 已知控制量

| 量 | 文档含义 | 单位 | 已知范围/示例 | 未知项 |
|---|---|---|---|---|
| `theta_i,1` | 第 i 条足的主动关节目标角 | degree | 由 gait generator 输出 | 对应 PWM、零位、正方向 |
| `A_i,1` / `R_i` | 摆动幅值 | degree | 论文称 0–45°；实验常用15/25/35/45° | 每个舵机安全上限、机械限位 |
| `f` | 四足共同摆动频率 | Hz | 机器人文档最大1.3Hz；实验0.9/1.1/1.3Hz | 低频极限、故障值处理 |
| `phi` | 相位/相位差 | degree in parameter tables | 中足与后足采用相反幅值或等效180°关系 | 舵机安装方向导致的符号映射 |
| `bias` / `X_i` | 偏置角 | degree | CPG主参数表为0 | 实机标定偏置 |
| PWM | 舵机电信号 | UNKNOWN | 更新被描述为100Hz；GUI曲线纵轴“Value”约500–2500但无单位 | PWM载波、脉宽µs、占空比、polarity |

不能把 GUI 曲线的 500–2500 直接称为微秒；文档没有声明单位。

### 7.3 动作如何形成

| 动作 | 代码/控制层最终语义 | 证据与限制 |
|---|---|---|
| 前进 | 左右足对称摆动；中足与后足同频但冲程/回程相反。后足冲程时中足回程，半周期后交换。 | 论文 PDF pp.81–82、92–93；期刊 PDF p.5图3a |
| 左转 | 右侧足幅值大于左侧，产生向左力矩；极端步态可使左侧幅值为0。 | 论文 PDF pp.94–100；尤其 p.100说明右侧幅值较大推动向左转 |
| 右转 | 左侧足幅值大于右侧，产生向右力矩；极端步态可使右侧幅值为0。 | 论文 PDF pp.98–99说明增强左侧幅值用于向右纠偏 |
| 抗偏航 | 期望 yaw 与 IMU yaw 做差，控制器调节左右幅值差。 | 期刊 PDF p.5图3e；论文 PDF pp.98–100 |
| 后退 | **未发现机器人后退步态。** 论文提到自然龙虱能后退，不能外推到样机。 | UNKNOWN / unsupported by repository |
| 上浮/下潜 | **样机无主动升潜机构。** 当前为水面机器人；论文把三维运动和新增升潜机构列为未来工作。 | 论文 PDF p.103（正文 p.89，§6展望） |
| pitch | 未发现主动控制 | UNKNOWN / unsupported |
| roll | 重心下置、浮力上置产生被动恢复力矩，不是主动 roll controller。 | 论文 PDF pp.99–100 |

### 7.4 安全相关执行器参数

以下均为 UNKNOWN：舵机 PWM 频率、最小/最大脉宽、中心脉宽、安装零位、方向反转、扭矩/电流限幅、软/硬限位、死区、饱和、校准表、急停时脉宽、通信丢失后的 neutral/disable 行为。论文只显示“待机位置”“停止指令”和“使能舵机”概念，不足以证明安全实现。

## 8. Sensors / feedback

| 传感/观测 | 实时性 | 进入控制环 | 数据 | 备注 |
|---|---|---|---|---|
| JY901S IMU | 实时 | 是，yaw 稳定环 | yaw；实验图也显示 roll | 采样率、滤波、零偏校准、磁航向处理未知 |
| 舵机位置/驱动状态 | 文档称实时回传 | 主要用于显示/状态 | position/drive parameters | 是真实舵机反馈还是命令回显无法由文档确认 |
| overhead global camera | 离线 | 否 | speed/trajectory | 旧系统没有实时视觉闭环 |
| underwater camera on thrust rig | 离线录像 | 否 | appendage pose | 仅单足实验台 |
| ATI 6-axis sensor | 100Hz | 否，实验数据 | force/torque | 仅实验台，经采集盒 UDP 到 PC |
| depth/pressure/leak/battery voltage/current | 未发现 | 否/未知 | — | 首次实机前需补齐或确认 |

## 9. Controllers

### 9.1 Open-loop gait: SDSG

```text
reference: gait + A_i + f + phi_i + bias_i
  → four sine-based signal generators
  → theta_1..theta_4
  → PWM/servos
  → appendage motion
```

这是期刊明确描述的样机运动控制器。无状态反馈时为开环。驱动参数决定前进/转弯；频率与幅值越高，实验中的速度/转弯率通常越高。最大平均直游速度约0.2m/s（0.77BL/s），最大瞬时约0.3m/s，最大转弯率42°/s，均在 `f=1.3Hz, A=45°` 条件附近取得。来源：期刊 PDF pp.10–11；论文 PDF pp.95–101。

### 9.2 Open-loop gait: CPG

```text
reference: gait + T_i + R_i + phase relation
  → four coupled CPG units, fixed 10ms step
  → four smooth angle outputs
  → servo/PWM
```

论文表4-2给出的参数：`k_vi=[1,1,1,1]`、`beta_i=[0.5,0.5,0.5,0.5]`、`phi_ij=0`、`X_i=[0,0,0,0]`、`c_ij=20`、`b_i=20`、`a_i=20`。中足与后足通过相反幅值表示相反运动阶段。来源：论文 PDF pp.80–84（正文 pp.66–70）。这些是模型参数，不等于实机安全标定。

### 9.3 Closed-loop yaw controller

```text
reference: prescribed yaw angle [degree]
measured: JY901S actual yaw [degree]
error: prescribed - actual (diagram sign only; wrap convention UNKNOWN)
controller: called PD in journal/conclusion, PID in thesis §5.5/UI
output: adjustment to left/right A_i only
actuator: four gait-driven servos
```

- 期刊规定 PD 只在 `t=N/(2f)` 时生效以避免 SDSG 输出跳变；例如 `f=1Hz` 时每0.5s调一次幅值。来源：期刊 PDF p.5。
- 硕士论文另称闭环参数更新约100Hz，并在 §5.5称 PID；图2-11截图中 Kp/Ki/Kd 均显示0.5，但这只能视为截图值，不能当成实验最终参数。来源：论文 PDF pp.42、78、98。
- 积分限幅、微分滤波、角度 wrap（±180°/0–360°）、输出饱和和符号定义均未给出。

## 10. Coordinate frames / units / signs

### 10.1 旧系统已定义

- 单足模型：以右后足为例，`O4 x4 y4` 为惯性坐标系，`O4,i x4,i y4,i` 为连杆坐标系，服从右手法则；`phi_4,i` 是相邻连杆夹角，`theta_4,i` 是连杆相对 `O4 x4` 的角。来源：期刊 PDF p.6图4；论文 PDF pp.47–48。
- 整机模型：惯性坐标系 `OI xI yI`；机体坐标原点固定在刚体质心；广义坐标包含10个被动关节角和 `[x0,y0,theta0]`。来源：论文 PDF pp.85–86（正文 pp.71–72，图4-11）。
- 角度在控制参数表中以 degree 表示；动力学公式中的三角函数/刚度使用 rad 的数学量，但代码转换位置未知。
- 实验运动是水平面内的 `x/y/yaw`；当前不是六自由度水下航行器。

### 10.2 未定义且必须实测

- yaw 正方向、JY901S输出范围与 wrap；
- robot body `forward/right/up` 与论文 `x0/y0` 的可执行软件枚举；
- 四舵机角度正方向、安装镜像和通道顺序；
- 相机图像 `x-right/y-down` 到 robot yaw 的符号；
- USB 相机是否前视、俯视、镜像、倒装，光轴相对机体的外参；
- pixels → bearing 的内参/视场角模型。

论文图4-11能证明使用右手系，但不能单凭图推导 FOMO 图像误差的控制符号。

## 11. FOMO integration boundary

### 11.1 推荐数据契约（设计，不实现）

```text
TargetObservation
{
  class_id: int,
  confidence: float,          # [0, 1]
  centroid_x_px: float,       # original-image coordinates, +right
  centroid_y_px: float,       # original-image coordinates, +down
  image_width_px: int,
  image_height_px: int,
  timestamp_monotonic_s: float
}
```

FOMO/letterbox 反变换必须先把 centroid 恢复到原始相机图像坐标，再进入控制。建议外层只产生无量纲图像误差：

```text
ex = (centroid_x_px - 0.5 * image_width_px)  / (0.5 * image_width_px)
ey = (centroid_y_px - 0.5 * image_height_px) / (0.5 * image_height_px)
```

这里的 `ex>0` 只表示“目标在图像右侧”，**不预先等同于右转命令**。需通过断开舵机或安全台架验证 camera mirror/orientation 和 yaw sign 后，再配置 `camera_x_to_yaw_sign ∈ {-1,+1}`。

### 11.2 最自然的 integration boundary

```text
USB camera
  → FOMO inference
  → TargetObservation
  → target selector / confidence & age gate
  → normalized image error ex, ey
  → visual-servo outer loop
       output: desired_yaw or yaw_rate setpoint
  → [INTEGRATION BOUNDARY]
  → existing/ported yaw controller using JY901S actual yaw
  → bounded left/right amplitude differential
  → SDSG/CPG gait generator
  → hardware protocol adapter
  → STM32 → PWM → servos
```

选择此边界的理由：旧系统已经把“期望偏航角→IMU yaw误差→左右幅值差→舵机”作为闭环结构；FOMO 应提供目标方向参考，而不绕过 IMU 稳定环直接控制舵机。协议未恢复前，`hardware protocol adapter` 必须保持抽象/禁用。

### 11.3 当前自由度限制

- `centroid_x`：可作为偏航目标的观测量，经过死区、限幅、丢失目标策略和符号确认后进入 yaw reference。
- `centroid_y`：当前样机无升潜或 pitch 执行机构，只能用于目标选择、日志或“保持不可控误差”提示；不得直接生成上下潜命令。
- `confidence`：应参与目标有效性与超时门控；旧系统没有相应设计。
- target loss：必须先定义 safe stop/hold/neutral；旧仓库没有可复用策略。

## 12. Startup and shutdown

### 12.1 文档所示启动流程

```text
check seals / install battery / power on robot
  → STM32 startup
  → initialize UART (documented 9600), timer, IMU, motion algorithm
  → continuous IMU acquisition and state polling

start PC Qt application
  → select/open host RX and TX serial ports
  → wait for lower-controller initialization/status
  → enable servo (and sensor in UI)
  → optionally command install/standby position
  → set gait/controller parameters
  → send gait command
  → receive/plot/save 50Hz telemetry
```

来源：论文 PDF pp.41–43，图2-10至2-12。图2-12正文称“上位机整体操作流程”，但图注写“下位机逻辑框架图”，属于文档内部标注冲突。

### 12.2 关机/停止

文档流程含“运动停止指令→结束”，GUI也有舵机待机/使能概念，但未给出：

- 停止命令的字节内容和确认机制；
- 停止时是保持当前位置、回中、停止 PWM 还是切断舵机电源；
- 断电顺序、先关 host 还是 robot；
- 通信中断时自动停止；
- IMU 校准和等待稳定所需时间；
- Windows/Qt 版本、可执行文件、运行库、驱动或管理员权限。

因此，以上启动流程只能作为恢复软件时的线索，不能作为现机 SOP。

## 13. Safety / failsafe

| 分类 | 发现 |
|---|---|
| 已实现（代码可验证） | **无。目录没有源码，不能确认任何安全功能已实现。** |
| 部分实现 | 无法从代码确认。文档展示使能舵机、待机位置、停止指令和状态应答，但行为/失败模式未知。 |
| 文档提到但代码未发现 | startup hardware check；servo/sensor enable；standby/install position；stop command；上壳浮力/下置重心带来的被动 roll 恢复；O-ring/螺栓密封。 |
| 完全未发现 | emergency stop、watchdog、command timeout、heartbeat、communication-loss neutral、maximum command enforcement、PWM/angle saturation、overcurrent/overtemperature、battery undervoltage、leak/water detection、CRC/checksum、reconnect、fault log。 |

特别风险：论文把串口接收优先级置于舵机更新、IMU接收和串口发送之后；如果没有 DMA/ring buffer/overflow handling，可能丢命令，但无代码无法判断。第一次联机前必须以源码或逻辑分析仪/抓包确认。

## 14. Stale code and likely real runtime

本目录无任何代码，不能区分 debug/test/backup/final，也不能依据 import、commit 或配置判断“最终运行版”。可建立的文档时间线是：

1. 期刊文章（2024-09投稿、2025年修订）明确描述实际样机使用四个 SDSG 正弦发生器和 PD yaw 控制；
2. 2025年6月硕士论文加入四单元 CPG、Simulink `ert.tlc` 自动代码生成，并称可部署到 STM32；
3. 硕士论文最终实验章节仍混用 PID/PD 表述，且未附固件版本或生成代码哈希。

**评估**：CPG 可能是后期版本，但没有代码、固件、实验记录或版本历史证明它就是当前机器人上的最终运行实现。真正运行代码的位置应视为 UNKNOWN — repository does not contain it。

## 15. Known inconsistencies

| 主题 | Documentation says A | Documentation says B | Assessment |
|---|---|---|---|
| gait generator | 期刊：实际系统使用四个 SDSG 正弦发生器 | 硕士论文第4章：四单元 CPG，Simulink 自动生成嵌入式 C | 可能是版本演进；无源码无法确定现机版本 |
| yaw controller | 期刊与论文总结：PD | 论文§5.5和GUI：PID；GUI还有非零Ki截图值 | 控制器阶次/积分项 UNKNOWN |
| closed-loop update | 期刊：仅 `t=N/(2f)` 更新；f=1Hz时每0.5s | 论文：闭环参数更新约100Hz | 两个不同实现或描述冲突；不可合并 |
| telemetry/control timing | design ≥20Hz；telemetry 50Hz；gait/PWM update 100Hz | 期刊 PD更新取决于半周期 | 必须在恢复源码时分层命名，不应统称“控制频率” |
| wireless | 论文图2-9概括为433M无线模块 | 期刊表1：433M接收、440M发送 | 期刊更具体；仍需核对实物模块标签 |
| mass/gravity/buoyancy | 论文表2-3：985g、9.65N、10.7419N | 期刊表1：约1.1kg、10.78N、约11N | 原型/测量版本可能变化，现机需称重 |
| battery capacity | 整机§2.5.1：1400mAh 4S | 单足实验台§3.4.1：1500mAh 4S；期刊仅14.8V | 不同系统，不应混用 |
| host flow figure | 正文称上位机操作流程 | 图2-12标题称下位机逻辑框架 | 明显标注不一致 |
| turn sign description | 右侧幅值大→左转；左侧幅值大→右转（实验文字） | 无软件通道编号/舵机符号映射 | 物理规律文档明确，软件 sign仍须实测 |

## 16. Critical files and exact line references

仓库只有3个文件，无法提供10–20个不同文件。以下列出全部资产，并把必须阅读的页段细分；二进制 PDF 无源码行号，故用“PDF页 + 正文页 + 章节/图表”精确定位。

### P0 — 必须读

1. `高新义毕业论文.pdf`
   - PDF pp.27–44（正文 pp.13–30，§2）：结构、密封、4S电池、STM32/IMU/无线/舵机、9600 baud、100/50Hz调度、Qt界面。
   - PDF pp.47–48（正文 pp.33–34，§3.2.2）：SDSG公式与 `A/f/phi/bias` 定义。
   - PDF pp.59–60（正文 pp.45–46，图3-8）：单足实验台、无线串口、UDP力传感器链；注意不要误当整机链路。
   - PDF pp.72–84（正文 pp.58–70，§4.2）：CPG模型、参数、10ms步长、100Hz和Simulink代码生成。
   - PDF pp.85–86（正文 pp.71–72，图4-11）：惯性/机体/连杆坐标系。
   - PDF pp.91–100（正文 pp.77–86，§5）：真实游动平台、步态、转向符号、IMU反馈和抗偏航实验。
   - PDF pp.102–103（正文 pp.88–89，§6）：总结与“升潜机构/实时视觉定位仍属未来工作”。

2. `AIS论文/Advanced Intelligent Systems - 2025 - Gao - Design and Analysis for a Diving‐Beetle‐Inspired Swimming Robot with.pdf`
   - PDF pp.3–4（§2.2，图2）：实物、密封舱、无线/PCB/IMU/电池、主动/被动附肢结构。
   - PDF pp.4–5（§2.3，式1、表1、图3）：具体 MCU/IMU/舵机/电池/433–440MHz、SDSG和PD控制边界。
   - PDF pp.9–11（§4.3，图10–13）：实际游泳、速度、转弯、yaw稳定实验。

### P1 — 建议读

3. `AIS论文/AIS_2025.mp4`
   - 约63秒；用于确认实物运动现象、足部相位和实验场景。
   - 无文本时间轴、协议或可定位代码，只能作演示佐证，不能替代固件/接线审计。

### P2 — 参考

- 论文第3章其余动力学和推力实验页，对控制接入不是首要，但可用于理解幅值/频率/关节刚度对推力的影响。

## 17. Unknowns / questions requiring hardware inspection

| 问题 | 仓库是否能回答 | 还需要什么 |
|---|---|---|
| 现机 MCU 是否确为 STM32F407ZET6 | 文档能，现机不能 | PCB丝印/芯片照片、原理图、固件工程 |
| 当前固件是 SDSG 还是 CPG | 否 | STM32工程、bin/hex哈希、构建记录、实机版本回读 |
| Qt 上位机源代码/可执行文件在哪里 | 否 | `.pro/.cpp/.h`或发布包、依赖清单 |
| Simulink模型和自动生成 C 在哪里 | 否 | `.slx/.m/.mat`与生成`.c/.h` |
| 指令协议完整格式 | 否 | 源码、协议文档或离线串口抓包 |
| COM3/COM8 是否仍有效 | 否 | Windows设备管理器/USB-UART标签；不要按截图盲开 |
| 两个无线模块的准确型号/功率/串口电平 | 否 | 模块标签、数据手册、接线图 |
| 433/440链路方向和天线是否现机一致 | 文档部分回答 | 实物核对和不带执行器的环回测试 |
| UART data bits/parity/stop bits | 否 | 上下位机配置或逻辑分析 |
| 命令超时/失联行为 | 否 | 固件审计和舵机断开条件下测试 |
| 四舵机 MCU 通道→左/右中/后足映射 | 否 | 原理图、PCB网名、线缆标签 |
| 舵机中位、安装位、待机位 | 否 | 标定记录和物理治具测量 |
| 舵机正方向/镜像关系 | 否 | 固件常量和断开连杆的低风险台架验证 |
| PWM频率、脉宽范围、单位 | 否 | timer配置、示波器/逻辑分析仪；GUI 500–2500单位未知 |
| 机械安全角、软限位、堵转电流 | 否 | CAD/装配标定、舵机数据手册、实测 |
| 4S电池当前容量、健康、BMS/保护 | 否 | 电池标签、电压/内阻、保护板检查 |
| 5V/3.3V rail 实际额定电流与舵机供电拓扑 | 否 | 原理图、DC-DC标签和万用表空载/负载检查 |
| 防水舱/O-ring当前状态 | 否 | 干态检查、独立泄漏测试 |
| 是否有漏水、欠压、过流检测 | 未发现 | 原理图/固件/实物传感器检查 |
| JY901S安装朝向、yaw正方向、角度范围 | 否 | 安装照片、配置、手动旋转数据记录 |
| IMU校准、磁干扰和滤波参数 | 否 | 配置/固件、现场校准记录 |
| 摄像头是否机载、朝向/镜像/视场角 | 旧仓库只见顶视全局相机 | 当前USB相机安装照片、内外参 |
| FOMO centroid坐标是否已去letterbox | 旧仓库不能 | 当前 `fomo-visual-servo` 接口契约/测试 |
| target loss 的 safe stop策略 | 否 | 安全需求、固件/host设计与验证 |
| 机器人是否被改装过 | 否 | 现机照片、维护/改线记录、逐线 continuity check |
| 当前机器人质量/浮力 | 文档数值冲突 | 现机称重与淡水/目标水体配平实验 |

## 18. Recommended next steps

以下是控制实现前的资料恢复顺序，不包含本轮实现：

1. 找回或镜像保存 STM32工程、Qt工程、Simulink模型、生成 C、原理图/PCB/BOM、固件 bin/hex，并记录各自 SHA-256。
2. 给现机拍摄 PCB正反面、MCU/IMU/无线/DC-DC/舵机/电池标签和完整接线照片；建立连接表与舵机物理通道表。
3. 在**舵机电源断开或舵机信号线隔离**条件下恢复协议：先静态读源码，再做串口环回/抓包；禁止以猜测命令试探机器人。
4. 明确并书面化安全状态机：boot-disabled、armed、running、target-lost、command-timeout、fault、safe-stop；定义每个状态的PWM/电源行为。
5. 在无连杆或机械卸载条件下标定四路舵机的中心、方向、脉宽和硬限位；再安装附肢并缩小范围验证。
6. 确认 JY901S yaw 约定、wrap、刷新率和安装外参；用手动旋转建立“IMU yaw / robot turn / image x”的符号真值表。
7. 在 `fomo-visual-servo` 中只先固定 `TargetObservation`、时间戳、坐标系和目标丢失语义；硬件 adapter 保持禁用，直到协议和安全门完成。
8. 第一轮带水运动前，准备物理断电、系留、限幅、低频低幅、单人发令/单人断电和完整日志方案。

## 19. Audit commands and test status

本轮执行的命令类别均为只读或生成审计临时文件/最终文档：

- `Get-ChildItem ... -Recurse -File`：资产盘点；
- `git -C <RoboBeetle> status --short --branch`、`git ... log --oneline --decorate -20`：确认非Git仓库；
- `Get-FileHash -Algorithm SHA256`：资产指纹；
- bundled Python `pdfplumber`：按页提取两份 PDF 文本到系统临时目录；
- bundled Poppler `pdftoppm`：渲染关键 PDF 页面并视觉核验；
- `rg`/正则按页搜索：串口、协议、CRC、PWM、安全、坐标、控制器和平台关键词；
- Windows Shell metadata：读取 MP4 时长/码率元数据。

未运行 pytest 或任何项目程序：目标目录没有可运行源码或测试；启动潜在硬件程序被任务边界明确禁止。最终 Markdown 仅做结构、关键标题、来源定位和禁止项覆盖检查。

---

**最终判定**：现有目录足以恢复机器人“高层机电结构与控制意图”，但不足以安全恢复“可发送的硬件控制协议”。首次运动之前最关键的缺口不是视觉模型，而是固件/Qt源码、协议、舵机标定、通道映射和失联安全状态。FOMO 应接在期望偏航/步态参数边界，并在这些缺口关闭前保持硬件发送禁用。
