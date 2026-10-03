# Task 02 — Visual Command Proposal (DRY_RUN) 设计说明

**状态：Claude 有条件批准；本次修订落实 B1/S1/S2 后允许实施 PR #42 的①–③。** Codex 是实现工程师，Claude 负责方案与 Review，用户负责实测与最终决策。先推送本设计修订，再 RED→GREEN 实现；CSV 在 #42 合并后以独立后续 PR 开始（预期 #43，实际号码由 GitHub 分配）。

**Goal：** 在现有最高置信度质心/误差显示之上，增加本地检测帧进展看门狗、纯逻辑建议命令、清楚的 DRY_RUN 显示和可选 CSV。建议不会发送给机器人。

**Architecture：** 保留既有检测选择与布尔门控；增加仅用于检测时间戳领先视频的 AwaitingVideo 原因。本地单调时间只由 Qt 适配层提供。状态机和策略以显式记忆值返回下一份值，不持有 QObject、QTimer、时钟或控制器；本 PR 只实现显示，后续 CSV 消费同一诊断快照。

**Tech Stack：** 现有 Qt 6/C++20、CMake/CTest、Qt TCP/HTTP、QElapsedTimer、QTimer；后续 CSV 为 GUI 线程 QFile 缓冲写入和250 ms flush，无 QThread、队列或锁。实际实现先 RED 再 GREEN，分小步提交。

## 0. 基线、证据和本次提交边界

- Task 01 PR [#41](https://github.com/Mr-lxd/RoboBeetle/pull/41) 已由用户授权转 ready 并采用 merge commit 合并。
- 合并后的 main/本分支起点：`3fa89cd5de246ade7da7a7a29c2bbd6df2108134`；PR #41 最后 head：`6c2cc45399561e2b78c9240514e8183ef3abb195`。
- 用户桌面 T1–T8、640×480 数值例子、EXE 哈希一致及 T9 的限制已写入 [Task 01 验证记录](../visual-error-dry-run-validation.md#用户桌面实测用户确认2026-10-03)。T9 进程退出/视频断开不能证明 read 卡住时的墙钟超时已存在。
- Task 01 EXE 代码 commit 为 `657d1262f7865be06b3b69caaa55c65f1ce03aba`，SHA-256 为 `955BE5F63467653CE16F9892AA0D4833BA5DFA3B1D7C017A39985AAB94CD7821`。这是旧版实测来源；**Task 02 没有实现、没有构建、没有新 EXE 哈希**。
- 本分支 `codex/visual-command-dry-run` 对应 draft PR #42；设计修订推送后只实施① config/看门狗、②策略、③显示/loopback。完成交付便携包供用户②③④实测，然后停止等待 Review。CSV 开关、文件写入、用户实测①均不包含在 #42。
- 只允许修改 `RoboBeetleConsole/` 和 `docs/`。不修改 Pi、Firmware、fomo-visual-servo；不引用/调用 IConsoleController 的运动、舵机接口；不修改最高置信度策略；不增加步态幅度或其他通信协议。

## 1. 从现有代码确认的接缝

| 现有文件/接口 | 已确认行为与接入方式 |
|---|---|
| `src/vision/DetectionClient.cpp::handleReadyRead()` | 成功解码、严格递增 frame_id 后发 `metadataReady`；网络重复/倒序 ID 当前报协议错误并断开。这一契约保留，不为新看门狗放宽协议 |
| `src/vision/DetectionMetadata.h/.cpp` | 新增 AwaitingVideo，仅在非零检测时间戳领先当前非零视频时间戳时返回；布尔接口对此仍false。其余1500 ms门限和Task 01显示抑制保持 |
| `src/vision/TargetState.h/.cpp::selectTargetState()` | 最高置信度有效候选，同分首个，使用原图 W/H；完全复用，不引入新类别过滤/追踪 |
| `src/ui/MainWindow.cpp` | metadataReady 是到达事件；视频帧、HTTP 状态及 refreshDetectionOverlay 是重复门控/渲染事件，后者不能刷新本地看门狗 |
| `src/vision/VideoView.h/.cpp` | 既有目标、误差线和数值面板保留，通过新诊断快照补充状态/建议；STALE 时清空旧原始 ex/ey 和目标线 |

只读核对的固件证据：`RoboBeetleFirmware/Core/Motion/motion_config.h` 定义 `MOTION_TRANSITION_DURATION_MS=750U`，Console `src/protocol/Packet.h` 定义 `Busy=7`。1000 ms min_dwell 只是面向后续离散运动的暂定建议节奏，不是 ACK/Busy 处理实现，也不允许本阶段发送命令。

方案取舍：把逻辑放在 VideoView 内会把绘图和时间状态耦合；全部放进 MainWindow 会继续扩大现有大文件。采用小型纯逻辑模块加 Qt 适配器，MainWindow 仅传现有事件与门控上下文。

```text
existing metadataReady + existing video/HTTP validity gate
           ↓                              ↓
        arrival event              gate/view context
           └──────── VisualDiagnosticSession (local monotonic nowMs) ───────┐
                         ↓                                               │
              advanceVisualTargetState (pure)                            │
                         ↓                                               │
              evaluateVisualCommand (pure, explicit memory)              │
                         ↓                                               │
               VisualDiagnosticSnapshot ──→ VideoView + diagnostics label │
                         └── 后续 PR：frame/transition → GUI CSV          │
                 no edge from these modules to robot controller/transport ┘
```

## 2. 拟新增/修改文件

以下相对路径均以 `RoboBeetleConsole/` 为根，除显式 `docs/` 项外：

| 操作 | 文件 | 职责 |
|---|---|---|
| 新增 | `src/vision/VisualPolicyConfig.h` | 全部 [Provisional] 参数、参数版本、验证规则 |
| 新增 | `src/vision/VisualTargetStateMachine.h/.cpp` | 纯状态、递增 ID 本地到达时间、NO_TARGET 计时、截止时间 |
| 新增 | `src/vision/VisualCommandPolicy.h/.cpp` | 独立建议枚举、纯 EMA/滞回/dwell、显式策略记忆 |
| 新增 | `src/vision/VisualDiagnosticSession.h/.cpp` | Qt 本地时钟/外部唤醒、现有输入适配、诊断快照；不持有机器人控制器 |
| 后续 PR 新增 | `src/vision/VisualCsvLogger.h/.cpp` | GUI线程QFile缓冲写入、定时flush、文件限额与错误状态 |
| 新增 | `tests/visual_target_state_machine_tests.cpp` | 注入时间的状态/冻结/重复/重连测试 |
| 新增 | `tests/visual_command_policy_tests.cpp` | 纯策略表驱动测试 |
| 后续 PR 新增 | `tests/visual_csv_logger_tests.cpp` | frame/transition、CSV转义、限额、错误/关闭 |
| 修改 | `src/ui/MainWindow.h/.cpp` | 持有session，传事件/非视频状态标签；后续PR才加CSV；不触及手动控制handlers |
| 修改 | `src/vision/DetectionMetadata.h/.cpp` | B1 AwaitingVideo 原因，布尔结果不放宽 |
| 修改 | `src/vision/VideoView.h/.cpp` | 接收同一快照、扩展面板；停止/失效时数值清空，原有缩放几何保留 |
| 修改 | `CMakeLists.txt` | 登记文件和测试，纯模块不增加机器人依赖 |
| 修改 | `tests/main_window_tests.cpp`、`tests/video_view_tests.cpp` | 扩展原有视觉测试及独立 `main_window_visual_error_tests` wrapper 覆盖的新状态 |
| 修改/新增 | `docs/desktop-visual-error-dry-run.md`、`docs/visual-command-dry-run-validation.md` | 后续构建来源、参数/实测步骤和证据；本次不提前填通过结论 |

`DetectionClient`、decoder、`TargetState` 选择逻辑、控制器/transport/协议/运动层不在实现修改列表中。

## 3. 参数集中表

全部默认值和策略常量集中于 `VisualPolicyConfig.h`，每项 **[Provisional]**。测试可传参数，不复制魔数。CSV常量在后续PR引入，本PR不增加CSV控件或控制参数调参UI。

| 参数 | 初值 | 单位/约束 |
|---|---|---|
| alpha | 0.3 | 无量纲，0 < alpha ≤ 1 |
| e_on | 0.25 | 无量纲，严格大于才进入转向 |
| e_off | 0.12 | 无量纲，严格小于才回 FORWARD；0 ≤ e_off < e_on ≤ 1 |
| min_dwell_ms | 1000 | 本地单调 ms；非负，普通离散建议切换最小间隔 |
| stale_ms (`T_stale`) | 500 | 本地单调 ms；正数，`age >= stale_ms` 即 STALE |
| lost_ms (`T_lost`) | 1500 | 本地单调 ms；正数，连续 NO_TARGET `duration >= lost_ms` 即 LOST |
| K_yaw | 1.0 | 归一化增益，有限且非负 |
| turn_sign | +1 | 仅 ±1；+1 为 ex_f > 0 → TURN_RIGHT；实机符号未验证 |
| ui_tick_ms | 50 | 外部 Qt 适配层最大常规检查间隔；正数 |
| csv_max_file_bytes | 32×1024×1024 | bytes；写入前检查，包括 header；到限额停止记录，不自动无限轮转 |
| csv_flush_ms（后续 PR） | 250 | GUI线程定时flush；文件IO可能产生短暂GUI延迟，采用Review指定的简化方案 |
| policy_version | `visual-command-proposal-v1` | schema/行为版本字符串；参数哈希另按实际 config 计算 |

参数验证失败时不启用建议运算：诊断为 STALE、建议 STOP，CSV 显示配置错误；不静默裁剪无效参数。验证函数是纯逻辑；哈希对固定顺序/固定精度序列化后的全部实际参数做 SHA-256，运行时写入每行，不哈希结构体内存。

u/v/W/H 是原图像素，左上原点、右/下正；ex/ey/ex_f/yaw_cmd 都无量纲。`yaw_cmd=clamp(K_yaw*ex_f,-1,1)` 保留图像方向正号；**turn_sign 只映射离散左右建议**，不偷偷修改 ex 或 yaw_cmd 的图像符号。pitch/depth/距离控制本阶段不实现。

EMA 按新目标帧计算，时间常数随推理 FPS 改变：`tau=-1/(FPS*ln(1-alpha))`，alpha=0.3、25 FPS 时约112 ms（约110 ms）。将来真正发送连续yaw时，turn_sign必须同时作用于发送映射层；当前yaw只按图像符号显示。

## 4. 拟定接口签名（声明，不是实现）

纯逻辑声明放 `rb::vision`；整数时间使用 `std::int64_t` 的本地单调 ms，frame ID/采集 ns 使用 `std::uint64_t`。下面没有引用机器人 MotionMode/命令类。

```cpp
struct VisualPolicyConfig; // 字段和默认值完整列于第 3 节
bool validVisualPolicyConfig(const VisualPolicyConfig &config) noexcept;

enum class VisualState { Tracking, NoTarget, Lost, Stale, InferenceOff };
enum class ProposedCommand { Forward, TurnLeft, TurnRight, Stop, Hold };

struct VisualTargetMemory {
    std::optional<std::int64_t> lastEvaluatedMs;
    std::optional<std::uint64_t> highestArrivedFrameId;
    std::optional<std::int64_t> lastAdvancedArrivalMs;
    std::optional<std::int64_t> noTargetSinceMs;
    VisualState state{VisualState::Stale};
};
struct VisualTargetInput {
    std::int64_t nowMs;
    std::optional<std::uint64_t> arrivedFrameId; // 仅 metadataReady 事件填值
    DetectionDisplayState gate;                // 既有 gate + HTTP 已知状态核对
    std::optional<TargetState> selected;        // 既有最高置信度函数的结果
};
struct VisualTargetResult {
    VisualTargetMemory next;
    VisualState state;
    bool frameAdvanced;
    std::optional<TargetState> usableTarget; // 仅 TRACKING 有值
    std::optional<std::int64_t> nextDeadlineMs;
    bool awaitingVideo; // 仅 gate=AwaitingVideo 且本地尚未超时时；不评估策略
};
VisualTargetResult advanceVisualTargetState(
    const VisualTargetMemory &previous, const VisualTargetInput &input,
    const VisualPolicyConfig &config);

struct VisualCommandMemory {
    std::optional<std::int64_t> lastEvaluatedMs;
    std::optional<double> filteredEx;
    std::optional<std::uint64_t> lastFilteredFrameId;
    ProposedCommand effective{ProposedCommand::Stop}; // 永不为 Hold
    std::optional<std::int64_t> lastSwitchMs;
};
struct VisualCommandInput {
    std::int64_t nowMs;
    VisualState state;
    std::optional<std::uint64_t> usableFrameId;
    std::optional<double> ex; // 仅 TRACKING 中的 usableTarget 提供
};
struct VisualCommandResult {
    VisualCommandMemory next;
    std::optional<double> ex_f;
    double yaw_cmd;
    ProposedCommand proposed;
    ProposedCommand effective;
    bool dwellBlocked;
};
VisualCommandResult evaluateVisualCommand(
    const VisualCommandMemory &previous, const VisualCommandInput &input,
    const VisualPolicyConfig &config);
```

`VisualDiagnosticSnapshot` 是值类型，字段为 sessionId、localMonoMs、可选 frameId/captureTimestampNs、nDetections、可选 TargetState、VisualState、VisualCommandResult、policyVersion、policyHash。未知元数据用 optional，不制造 frame_id=0 或 ex=0。sessionId 是 Console 内部会话序号，不改网络 schema。

Qt 适配器拟公开下列接口（QObject parent 默认 nullptr）：

```cpp
using NowMs = std::function<qint64()>;
VisualDiagnosticSession(VisualPolicyConfig config, NowMs nowMs,
                        QObject *parent = nullptr);
void beginSession(quint64 sessionId); // 清空两份纯逻辑记忆
void onDetectionArrival(const DetectionFrame &frame, const VisualViewContext &context);
void refresh(const VisualViewContext &context); // video/HTTP/tick；不传到达 ID
VisualDiagnosticSnapshot snapshot() const;
// signal: diagnosticChanged(VisualDiagnosticSnapshot snapshot)

// VisualViewContext 字段：DetectionDisplayState gate、optional<TargetState> selected。
// MainWindow 用已有帧/HTTP/视频尺寸与时间戳构造，不把 QObject 传进纯函数。

void VideoView::setVisualDiagnostic(const VisualDiagnosticSnapshot &snapshot);
```

CSV接口在第7节定义，归后续PR。策略模块不用QString/QFile。上述构造函数分别属于同名类；实际实现的轻量session可由MainWindow提供外部QTimer唤醒，纯逻辑无内部timer。

## 5. 看门狗与状态转移

处理顺序：验证 config/单调时间 → 处理严格递增到达 ID → 既有 gate 优先级 → 本地看门狗 → NO_TARGET 时长。初始状态 STALE/STOP，无 frame ID 时不等待 500 ms 才宣称失效。

| 条件（优先级从上到下） | 下一状态 | 建议 |
|---|---|---|
| 无效 config、倒退的 nowMs | STALE | STOP |
| 已有 gate 为 InferenceOff | INFERENCE_OFF | STOP |
| 已有 gate 为 Stale，包括 HTTP 未知/过期/Stop ACK reconcile | STALE | STOP |
| 没有本会话新 ID 到达时间，或 `now-lastArrival >= T_stale` | STALE | STOP |
| gate=AwaitingVideo，本地未超时 | VisualState不变，noTargetSince保留；不更新EMA/effective | 保持上一诊断建议，等待视频 |
| gate=Target，selected 有效 | TRACKING；清空 noTargetSince | 滤波/滞回/dwell 决定 |
| gate=Target 但没有有效 selected | STALE | STOP |
| gate=NoTarget，首次持续有效空检测 | NO_TARGET；设置 noTargetSince=now | HOLD |
| gate=NoTarget，连续空检测持续时间 < T_lost | NO_TARGET；保留 noTargetSince | HOLD |
| gate=NoTarget，连续空检测持续时间 ≥ T_lost | LOST | STOP |

严格递增到达时间只在 `arrivedFrameId > highestArrivedFrameId` 更新；本会话首个已解码 ID 作为第一个新帧（允许 ID=0）。同 ID 重复 refresh/到达、HTTP 刷新、视频到达和重新绘图都不续期。纯函数对重复/倒序 ID 返回 frameAdvanced=false，不改最高 ID；实际 DetectionClient 的网络协议错误断开行为保留。

gate=NoTarget 来自尺寸/时间戳有效的空帧，不由“没有 metadata”伪造。只有持续有新空帧、间隔 <500 ms，NO_TARGET 才能一直计时到 1500 ms；断流先在 500 ms 成为 STALE/STOP。STALE/INFERENCE_OFF 打断 NO_TARGET 连续时段并清空 noTargetSince；LOST 在继续新鲜空帧时保持，恢复有效目标后回 TRACKING。

metadataReady 到达先记录本地ID进展。视频与检测独立TCP连接；检测时间戳领先当前视频时，gate=AwaitingVideo、布尔门控false，原始目标叠加仍抑制，但VisualState/effective/EMA保持。视频追上后对该ID评估一次。若500 ms未追上，或更早触发其他Stale/OFF条件，则失效/STOP。初始无有效样本时保持原初始STALE/STOP，不能凭空宣称TRACKING。严格递增ID即使Pi采集时间相同也续期，本步不推断ID递增但内容重复的质量问题。

等待超时另以最早尚未追上视频的本地到达时间为deadline；不断有更大ID但视频一直没追上，不能无限刷新等待时长。视频追上并成功接受目标/空结果后结束这一等待窗口。AwaitingVideo只用于时间领先，不用于零时间戳、尺寸错、HTTP失效等。

新视频连接、Pi Host/端口切换、检测连接重建或断开触发 beginSession/reset；清空 ID 与 EMA/dwell 记忆，不允许用旧连接的 ID 或到达时间给新会话续期。同连接内 Stop ACK 立即输出 STALE/STOP；权威非运行状态输出 INFERENCE_OFF/STOP。恢复 running 仍须通过既有 gate 和本地年龄门控，没有新 ID 时不能绕过 500 ms。

时钟由 session 的 QElapsedTimer（进程期间不反复 restart）或测试传入的 nowMs 函数提供；Pi 时间戳不与本地时钟相减。**纯函数内部没有 QTimer**。Qt session 使用外部 PreciseTimer，以 50 ms 为常规上限并按 nextDeadlineMs 提前安排唤醒；无数据、无视频更新也会重算并清除旧值。单元测试在精确 500 ms 判 STALE；真实 GUI 受事件循环调度影响，不能声称操作系统提供硬实时保证。

## 6. 建议策略和保持语义

纯函数需要 previous memory 才能表达 EMA 与 dwell；它不读隐藏全局变量、不读取时钟、不发命令。`ex_f = alpha*ex + (1-alpha)*previous.ex_f`；首个样本直接以 ex 初始化。同一 usableFrameId 仅滤波一次，timer/video/HTTP 重绘不会重复 EMA；迟到视频让新 ID 首次可用时才做一次滤波。

| 当前有效建议与条件 | candidate（普通切换需经过 dwell） |
|---|---|
| 首次 TRACKING 或有效建议为 FORWARD/STOP，`abs(ex_f) > e_on` | 按 turn_sign 和 ex_f 的符号选 TURN_LEFT/RIGHT |
| 首次 TRACKING 或 FORWARD/STOP，其余范围 | FORWARD |
| 当前为某个 TURN，`abs(ex_f) < e_off` | FORWARD |
| 当前为某个 TURN，反向且 `abs(ex_f) > e_on` | 反向 TURN |
| 当前为某个 TURN，其余范围（含等于门限） | 保留原 TURN，形成滞回 |

普通 candidate 与 effective 不同且 `now-lastSwitch < min_dwell` 时保留 effective，设置 dwellBlocked=true；等于 1000 ms 可切换。首次没有 lastSwitch 时直接接受首条 TRACKING 建议。只有 effective 真正变化才更新 lastSwitch，不能由重复帧或重复输出重置。

状态映射优先于普通滞回：

- STALE/INFERENCE_OFF/LOST：立即 proposed=effective=STOP、yaw_cmd=0、ex_f 无值；清空滤波，但保留 lastFilteredFrameId 作为本会话已消费样本的下限。STOP **绕过 min_dwell**；若由非 STOP 切到 STOP，记录此次 lastSwitch，后续普通恢复也遵守 dwell。重复 STOP 不更新时间。
- NO_TARGET：proposed=HOLD，effective 为上一条有效建议（初始默认 STOP），不刷新 lastSwitch，不更新 EMA。已有 ex_f/yaw 可保留用于显示，必须标注 retained；原始 u/v/ex/ey/confidence 清空为 `--`。HOLD 不是发送同一条命令，也不是 dwell 切换到一个真实运动模式。
- TRACKING：计算候选并应用 dwell；yaw_cmd 是显示用连续归一化值，可能在 dwell 保持旧离散建议时改变。界面同时显示 dwellBlocked，避免把两者误解成同一种控制量。

策略清空滤波后，只接受比 lastFilteredFrameId 大的 usableFrameId 重新初始化 EMA；已消费的旧 ID 重新被 gate 接受，也不重新滤波。TRACKING 若暂时只有该旧帧且 filteredEx 无值，维持 STOP/yaw=0/ex_f=--，补充 `waiting for new target sample`。未曾消费的首个合法 ID 可初始化。beginSession 才清空 ID 下限；测试验证 Stop/reconcile/恢复不会在重复刷新中产生不同 EMA。两份记忆中的 lastEvaluatedMs 用于拒绝倒退时间，正常每次评价更新；无效时间输出失效/STOP，不推进计时。

## 7. 显示与后续 CSV（S1/S2）

本 PR 面板标题为 `VISION DRY_RUN — no motion output (manual controls live)`；显示state、原始ex/ey/confidence、ex_f、yaw_cmd、`PROPOSED (not sent): <cmd>`、turn_sign与“实机符号未验证”。HOLD标记holding proposal及retained值，dwellBlocked清楚标注。AwaitingVideo提示“waiting for video”，不清空策略输出、不重新滤波；旧原始检测叠加仍按Task 01布尔门控抑制。

小窗口标题换行，不提高视频最小尺寸，不调整运动/舵机布局。视频断开仍清空画面，现有视觉诊断区文字显示状态/建议。无效原始值显示--而非0。

以下只属于 #42 合并后的 CSV PR（预期 #43）：默认关闭，GUI复选框和目录选择；默认 `%USERPROFILE%\Documents\RoboBeetle\visual-logs`（Qt DocumentsLocation/RoboBeetle/visual-logs）。不做.git扫描，不使用QThread、队列、锁；GUI线程QFile缓冲写入，250 ms QTimer flush。用户选择目录，默认不在仓库创建文件；IO失败停止并清楚显示原因。

拟接口：`bool start(const QString &directory)`、`bool appendFrame(const VisualDiagnosticSnapshot&)`、`bool appendTransition(const VisualDiagnosticSnapshot&)`、`void stop()`。内部QFile NewOnly创建唯一命名文件，32 MiB上限含header，每行写前检查；locale无关浮点、UTF-8、CSV转义。缓冲写和flush在GUI线程，可能有短暂IO延迟，采用本次Review指定的简化方案。

header：

```text
row_kind,local_mono_ms,frame_id,capture_ts_ns,n_detections,sel_class,sel_conf,u,v,ex,ey,ex_f,yaw_cmd,state,proposed_command,policy_version,policy_hash,session_id
```

每个新frame_id写frame行；state或effective发生变化时另写transition行，frame_id/capture_ts_ns/n_detections/sel_class/sel_conf/u/v/ex/ey留空。ex_f/yaw/state/proposed保留当时策略值；需要effective列时由后续CSV设计补充以明确HOLD。timer产生STALE/LOST或dwell切换也记transition，无变化不记。新frame导致状态变化则frame与transition各一行，row_kind区分。

CSV测试与用户居中60 s录CSV统计归后续PR；当前PR不创建logger、CSV控件或文件。
## 8. RED → GREEN 测试表与后续实施顺序

实现前按下表加入失败断言，先确认失败针对新增行为；随后实现最小模块，单步 GREEN 后提交。已存在的门控、选择、零机器人写入断言继续保留，禁止通过修改旧保护/失败断言让整套变绿。

| 拟测试 | 确定输入/期望 |
|---|---|
| 状态冻结 | t=0 新 ID=10/有效目标、Pi 时间戳固定、HTTP fresh；t=499 为 TRACKING，t=500 为 STALE/STOP，原 ex/ey 清空 |
| B1 竞态 | 已TRACKING/转向时检测N先到，15 ms后视频追上：全程无新增STOP、不改变dwell时间，EMA对N只算一次；视频500 ms不追上则STALE/STOP；持续更大领先ID也不得无限续期 |
| 重复不续期 | t=400 重复 ID=10/timer/HTTP 更新，lastArrival 仍为 0；t=500 STALE；纯函数重复和实际客户端报错分别验证 |
| 新 ID/会话 | ID=11 可续期；重连 session 重置后低 ID 可作新首帧，旧 session 值不泄漏；无帧启动 STALE |
| NO_TARGET → LOST | t=0 首个空帧，此后每 100 ms 新空 ID；t=1499 NO_TARGET/HOLD，t=1500 LOST/STOP；若空帧之后断流，t=500 先 STALE/STOP |
| 原 gate 优先级 | HTTP stale、尺寸错误、未知状态、未来/超龄采集时间都 STALE；权威 off 则 INFERENCE_OFF，即使没有 metadata；Stop ACK 待核对立即 STALE/STOP |
| EMA | 首帧 ex=0→0，下一帧 ex=1→0.3，再一帧→0.51；同 ID 重绘保持0.3；停止/reset 不重复滤波旧样本 |
| 滞回 | alpha=1、dwell=0：0.24→FORWARD，0.26→TURN_RIGHT，0.20/0.12→仍右转，0.119→FORWARD；左右阈值与严格等号均覆盖 |
| dwell/反向 | t=0 右转；t=999 反向大误差仍右转且blocked，t=1000 可左转；TURN→FORWARD同样遵守；HOLD不刷新switch时间 |
| 优先 STOP/恢复 | 转向后 t=1 STALE、OFF、LOST 均立即 STOP，不等1000 ms；NO_TARGET为HOLD、effective保留；恢复普通建议遵守最近STOP切换时间 |
| 符号/饱和 | turn_sign=-1 且 ex_f>e_on 得 TURN_LEFT；yaw保留图像正号；K_yaw=2/ex_f=0.8 时 yaw=1；非有限ex/config拒绝 |
| CSV（后续PR） | frame/transition、新ID一行、重复无行；限额/NewOnly/错误停记、UTF-8/转义、250 ms GUI flush |
| Qt loopback/绘图 | 现有 TCP/HTTP 保持fresh但停止新ID，在timer唤醒后旧值清空、STALE/STOP；新空帧LOST；Stop ACK/OFF；各新状态 FakeTransport 始终零机器人写入 |

纯测试的 alpha/dwell/限额覆盖参数仍通过同一 config 传入，不能更改产品默认值。只有测试能够制造 read 卡住等价的“无新 ID、时间推进、HTTP仍fresh”，不再把拔摄像头的退出行为当作该用例通过。

拆分后的顺序：PR #42 做①config+看门狗、②策略、③Qt显示+loopback，随后完整CTest/grep/便携包/用户②③④并停下等Review。#42合并后才开始CSV后续PR及用户①。每步RED→GREEN，遇真实失败先定位，不修改通信来掩盖问题。

后续构建用 Qt 6.11.2/MinGW Release，并设置测试字体 `QT_QPA_FONTDIR=C:\Windows\Fonts` 和 `QT_QPA_PLATFORM=offscreen`。完整 CTest 与合并后的 main 独立基线对照，已知名称为 `robot_controller_tests`、`main_window_tests`、`main_window_layout_tests`；记录时序敏感通过/失败，不得新增失败或弱化保护。

## 9. 实现阶段交付与用户清单（本次尚未执行）

- 实现阶段维持 draft PR，先经 Claude Review 再由用户决定后续 ready/合并。此设计 draft PR 不代表实现已经完成。
- 后续文档记录最终代码 commit 与新 EXE SHA-256；用后续 docs-only 提交记录代码 commit，避免自指哈希。设计阶段只有 commit，不伪造 EXE 哈希。
- 新便携包拟为 `D:\RoboBeetleConsole-portable-visual-command-dry-run-20261003`；构建时若目录已存在，使用新后缀而不覆盖。所有既有包保留。
- 沿用 PR #41 的完整 pattern，对 src 新增行用 GNU grep 检查无运动/舵机接口引用；配合 loopback 零机器人写入，覆盖全部新状态。当前 docs-only 变更没有 src 新增行，不作为未来实现扫描的替代。

| 用户桌面实测 | 应观察/记录 |
|---|---|
| ① 居中60 s并录CSV（后续CSV PR） | 记录原始ex/滤波ex_f抖动、状态与建议；核对无丢行/无logger错误后再统计，不假称已经稳定 |
| ② 左右移动 | 建议按门限/符号切换，滞回带内不抖动，普通切换间隔≥1000 ms；只观察 PROPOSED，不执行舵机 |
| ③ Stop Inference | 合法ACK待核对后立即STOP；权威停止后INFERENCE_OFF/STOP，原误差清空 |
| ④ 遮挡/移走 | 新空帧仍到达时NO_TARGET/HOLD，约1.5 s后LOST/STOP；若无新ID则约0.5 s STALE/STOP，明确区分 |

## 10. Claude Review 要点与停止条件

1. 接口/文件分工是否足够小，是否完全保留既有目标选择/渲染门控与网络重复-ID契约。
2. 优先STOP绕过dwell、HOLD/effective分离、停止后恢复dwell、滞回等号/反向语义是否接受。
3. turn_sign只影响离散方向、yaw保持图像误差符号，以及首样本EMA直接初始化是否接受。
4. 会话reset、采样ID去重、视频追上时的一次EMA、NO_TARGET被STALE打断的连续计时是否一致。
5. S1已接受：GUI QFile缓冲写/250 ms flush、32 MiB、NewOnly、错误停记；无队列、锁、QThread和.git检测，归后续CSV PR。
6. S1已接受：row_kind=frame|transition，状态/effective变化写transition，帧字段留空；S2将CSV推迟至#42合并后。

**本次Review已允许：先提交推送本修订，再实现PR #42①–③、交付测试与便携包，之后停下等Review。CSV后续PR必须等待#42合并，本阶段不实现；永不发起真实视觉运动。**
