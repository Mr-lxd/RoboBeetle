# Task 04 — Target Temporal Association (DRY_RUN) 设计与实施顺序

> 设计阶段：仅提交本文件，开 draft PR 后停止，等待 Claude Review；获准后才按下列 RED → GREEN 顺序实施。实现采用当前会话逐步执行，不创建其他用户任务。

**Goal:** 在既有 DRY_RUN 链路中以原图像素距离关联锁定目标，减少最高置信度交替引起的跳变，同时记录同帧最高置信度基线供比较。

**Architecture:** 无时钟/无定时器的纯函数关联器；`VisualDiagnosticSession` 持有关联记忆，复用既有门控、看门狗及建议策略。MainWindow 传完整检测帧，VideoView 使用 session 选中的目标，CSV 记录实际评估后的关联与全部检测。

**Tech Stack:** 当前 Qt 6/C++20、QElapsedTimer 注入时钟、现有 GUI 线程 QFile/250 ms flush、CMake/CTest。

基线：最新 `origin/main` = `8752700d0fb2fb423527692e8aec4272727b1631`（PR #43 merge commit，已核对 merged/closed）。新分支：`codex/target-temporal-association-dry-run`。本设计中的新增类型/接口均为提案，不表示已实现。

## 1. 现状、证据和范围

现有 `selectTargetState(frame)`（TargetState.cpp 中的自由函数）选择合法检测中置信度最高者，同置信度保留流中的第一项。`MainWindow::visualDisplayContext()` 在检测/视频/HTTP 门控后传 `context.selected`；`VisualDiagnosticSession` 已持有目标状态机、EMA/建议记忆和本地时钟。`VideoView::setDetectionOverlay()` 又自行调用最高置信度选择，误差线来自这份目标。因此只改 session 会使显示与建议目标不一致，必须一起改传递路径。

PR #43 用户实测 B 段 7.84–8.05 s：u≈450/547 间切换四次，confidence=0.85–0.89，本次未改变建议命令。门限依据来自用户任务书：A 段 |Δu| P95=7 px、最大19.5 px，B 段误跳约97 px。以上为用户提供的桌面证据，没有独立重新分析原始 CSV 或确认物理身份。

仅改 `RoboBeetleConsole/` 与 `docs/`。不修改 Pi、Firmware、fomo、摄像头/推理/视频检测协议、运动协议或手动控制 handlers；不调用任何运动/舵机接口。`alpha/e_on/e_off/min_dwell_ms`、EMA 公式及其初始化/重置条件保持原样。不得把重新获取目标额外解释为一次 EMA 重置。

按任务书采用最近邻加固定门限。仅平滑最高置信度坐标仍会在两个目标之间产生虚拟位置，无法满足锁定要求；全量多目标追踪/速度预测会增加未请求的状态与调参，本次不采用。此关联是位置连续性假设，不保证同一物理物体身份，目标交叉或48 px内干扰仍可能换目标。

## 2. 文件清单与职责（Review 后实施）

| 文件（相对仓库根目录） | 新增/修改与职责 |
|---|---|
| RoboBeetleConsole/src/vision/TargetAssociation.h / .cpp | 新增：纯关联器、超时释放函数、关联状态名称 |
| RoboBeetleConsole/tests/target_association_tests.cpp | 新增：表驱动关联/时序/合法性测试 |
| RoboBeetleConsole/src/vision/TargetState.h / .cpp | 修改：提取单个合法检测转 TargetState 的公共 helper；原最高置信度规则、公式和 tie 行为保持不变 |
| RoboBeetleConsole/src/vision/VisualPolicyConfig.h | 修改：三项关联参数、v2 policy_version、参数校验；CSV schema 常量独立于策略参数 |
| RoboBeetleConsole/src/vision/VisualDiagnosticSession.h / .cpp | 修改：完整帧 context、关联记忆、快照/参数 hash 和调用顺序 |
| RoboBeetleConsole/src/ui/MainWindow.cpp | 修改：传完整帧；显示全部可渲染检测并传关联选中目标，MISS 不清掉合法的未选检测 |
| RoboBeetleConsole/src/vision/VideoView.h / .cpp | 修改：不再自行最高置信度选目标；关联标记、误差线、lock 状态文字 |
| RoboBeetleConsole/src/vision/VisualCsvLogger.cpp | 修改：30列 schema、序列化全部检测、pending/transition新字段规则；原记录/IO流程不改 |
| RoboBeetleConsole/CMakeLists.txt | 修改：登记关联器和新测试目标 |
| RoboBeetleConsole/tests/target_state_tests.cpp | 修改：helper 与原最高置信度规则一致 |
| RoboBeetleConsole/tests/visual_policy_config_tests.cpp / visual_diagnostic_session_tests.cpp | 修改：参数/hash、单帧一次、超时释放和既有状态/EMA语义 |
| RoboBeetleConsole/tests/video_view_tests.cpp | 修改：显式选中目标、全部点、MISS/等待/清空显示 |
| RoboBeetleConsole/tests/visual_csv_logger_tests.cpp | 修改：schema、关联/hc/dets、pending时序和EMA重算 |
| RoboBeetleConsole/tests/main_window_tests.cpp | 修改：真实 TCP/HTTP loopback 场景；由现有 main_window_visual_error_tests.cpp wrapper 独立运行 |
| docs/target-temporal-association-dry-run-validation.md | 实现交付时新增：真实RED/GREEN、范围证据、源码commit、EXE SHA-256、用户清单 |

`DetectionMetadata.h/.cpp`、`VisualTargetStateMachine.h/.cpp`、`VisualCommandPolicy.h/.cpp` 的算法及外部接口预计无需改动；session 将关联失败映射为其已有 NoTarget 输入。若实施发现必须改变这些语义，先说明真实原因并回到 Review，不顺带重构。

## 3. 接口提案与数据契约

```cpp
enum class AssociationStatus { Unlocked, Acquired, Associated, Miss };

std::optional<TargetState> targetStateAt(
    const DetectionFrame &frame, qsizetype detectionIndex);

TargetAssociationResult associateTarget(
    const TargetAssociationMemory &memory,
    const DetectionFrame &frame, qint64 nowMs,
    const VisualPolicyConfig &config);

TargetAssociationMemory expireTargetAssociation(
    const TargetAssociationMemory &memory, qint64 nowMs,
    const VisualPolicyConfig &config);

TargetAssociationMemory releaseTargetAssociationLock(
    const TargetAssociationMemory &memory);
```

纯函数只使用参数，不读系统时钟、不持有 QObject/QTimer、不改变传入 memory/frame。`expireTargetAssociation` 只处理已开始的 MISS 超时，不重选、不计算距离、不消耗帧；session 的既有 wakeup 调用它，避免超时必须等下一帧才发现。显式 release 不重置“已处理ID”，新会话才整体清空 memory。

| 类型/字段 | 含义 |
|---|---|
| TargetAssociationMemory.lockedTarget | optional TargetState：最后一次成功关联的位置和类别；MISS期间不更新 |
| memory.lastProcessedFrameId | 本会话最后关联过的ID；解除锁定后仍保留，防止旧帧重新获取 |
| memory.missSinceMs | 连续MISS起点；首次MISS设为当前本地评估时间，重复帧不刷新；成功关联/获取清空 |
| memory.lastEvaluatedMs | 最近关联/超时推进的注入时间，用于拒绝回退时间 |
| memory.cachedResult | 最近已评估ID的 selected/index/status/distance；不包含 next，避免递归类型；重复调用返回缓存，不重算 |
| TargetAssociationResult.next | 下一份关联记忆 |
| result.selected / selectedDetectionIndex | optional TargetState及原frame.detections中的索引；仅ACQUIRED/ASSOCIATED有值 |
| result.status | ACQUIRED、ASSOCIATED、MISS；UNLOCKED用于初始化、显式安全释放或无有效输入 |
| result.distancePx | optional double；ASSOCIATED为与上一锁定点距离；MISS为最近合法且类别合格候选的距离（可超过gate）；无候选/ACQUIRED/UNLOCKED为空 |
| result.valid | 时间/config有效标志；非法参数、负时间或时间回退：false、无可用目标，session走STALE/STOP |

新 `VisualViewContext` 保留原 `gate`，将 `selected` 替换为 `std::optional<DetectionFrame> frame`；session仍保留 `onDetectionArrival(frame, context)`、`refresh(context)`、`beginSession(id)`、`snapshot()`。到达事件中的frame负责记录新ID；context.frame用于后续视频追上，必须对应同一已到达最高ID。重复/倒序到达不得覆盖缓存帧或刷新到达时间。

`VisualDiagnosticSnapshot` 保留原字段，增加 `AssociationStatus associationStatus`、optional `associationDistancePx`、optional `associationMissMs`、optional `selectedDetectionIndex`、optional `highestConfidenceTarget`、optional `DetectionFrame detectionFrame`。`target` 改为关联后的目标；最高置信度基线仍调用原 `selectTargetState(frame)`，绝不作为MISS的策略输入。

显示接口改为 `VideoView::setDetectionOverlay(const DetectionFrame &, const std::optional<TargetState> &selected)`；选中索引来自同ID快照。调用方显式传入selected，VideoView不再重新选最高置信度。无诊断的绘图测试也显式传所需目标，不增加隐式回退。

### 合法检测、距离与 ties

- 复用原合法性：W/H>0，confidence有限且在[0,1]，u/v有限且在[0,W−1]/[0,H−1]；类别不额外加新置信度门限。
- 未锁定：合法项最高置信度，tie保留原流序第一项；空/全非法帧为MISS且不创建锁。
- 已锁定：`d=hypot(u-last_u, v-last_v)`，先筛合法项及可选classId一致，再选最近者；严格相同距离时取置信度高者，距离和置信度都相同保留流序第一项。门限含边界 `d <= gate_px`，不加模糊epsilon或把两目标坐标混合。
- `require_same_class=true`时比较classId；false时标签变化不打断关联。成功更新锁定类别供下一帧使用。
- gate_px固定为原图像素，不随窗口缩放或分辨率自动调整。源尺寸变化由现有视频/检测尺寸gate处理；若两路一起换尺寸、gate重新有效，session先解除旧像素坐标锁再处理下一新ID，保留ID去重。用锁内sourceSize识别，不新增视觉状态。
- 时间有效性与现有策略一致：非负单调本地毫秒。同ID/倒序不更新锁点、MISS起点或EMA。超时释放后清空旧selected缓存为UNLOCKED，但保留lastProcessedFrameId；因此旧ID不能再当ACQUIRED样本。

## 4. 参数、版本及单位

新增值全部标注 **[Provisional]**，集中于 `VisualPolicyConfig`，追加到已有有序、locale无关、g17精度的SHA-256输入，不hash结构体padding。

| 参数 | 初值 | 单位/校验/用途 |
|---|---:|---|
| gate_px | 48.0 | 原图px，有限且>0；640宽时等于0.15×W/2，A抖动小于门限、97 px误跳大于门限 |
| max_miss_ms | 500 | 本地ms，>0；连续关联失败后释放锁，与视频新鲜度不是同一计时器 |
| require_same_class | false | bool；允许同一物体标签变化 |
| policy_version | visual-command-proposal-v2 | 对关联后样本应用原策略，版本/hash必须与v1区分 |
| CSV schema_version | visual-csv-v2 | 独立格式常量；旧21列无schema_version，不与新30列混拼 |

原值不变：alpha=0.3，e_on=0.25，e_off=0.12，min_dwell_ms=1000，stale_ms=500，lost_ms=1500，K_yaw=1.0，turn_sign=+1，ui_tick_ms=50。CSV的32 MiB/250 ms不混入policy_hash。测试逐个改变三项新值验证hash随实际参数变化；参数越界显式拒绝并STALE/STOP。

`u/v/hc_u/hc_v/distance`均为原图像素，左上原点，右/下正。`ex=(u-W/2)/(W/2)`、`ey=(v-H/2)/(H/2)`无量纲，左/上负、右/下正。`ex_f`、`yaw_cmd`仍无量纲，仅显示建议；turn_sign实机符号未验证，将来发送连续yaw时也必须作用于发送映射层。EMA按成功接受的关联目标帧计算，时间常数随有效FPS变化，25 FPS约110 ms。

## 5. 关联状态表与超时边界

| 当前/事件 | 关联结果/锁 | 视觉与建议 |
|---|---|---|
| UNLOCKED + 新可用帧含合法目标 | ACQUIRED，最高置信度锁定 | TRACKING，原EMA/滞回/dwell |
| 有锁 + 最近合格候选d≤48 | ASSOCIATED，更新位置，清MISS起点 | TRACKING，原EMA每新ID一次 |
| 有锁 + 无门内候选 | MISS，无selected，位置保持，首次记录missSince | NO_TARGET/HOLD，保留ex_f/effective |
| 空/全非法帧，原先无锁 | MISS，不创建锁 | NO_TARGET/HOLD；持续空帧可进入LOST |
| 连续MISS elapsed≥500 ms | 解除锁，当前已评估帧不再重选；状态为UNLOCKED | 不把超时释放当目标样本，维持NO_TARGET/HOLD |
| 解除锁后的下一新可用帧有合法目标 | ACQUIRED | TRACKING；沿用原EMA记忆，除非原状态机此前已清空 |
| 本地无新ID≥500 ms或原gate判Stale | UNLOCKED，保留去重ID | STALE/STOP，既有误差清空 |
| INFERENCE_OFF | UNLOCKED，保留去重ID | INFERENCE_OFF/STOP |
| NO_TARGET连续≥1500 ms | UNLOCKED，保留去重ID | LOST/STOP |
| beginSession(newId) | 完整清空，接受新会话低ID/0 | 既有新会话初始化行为 |
| AwaitingVideo且尚未超时 | 不关联、不释放、不计算EMA；保持原关联结果 | state/effective/EMA不变；等待行尚不写 |
| AwaitingVideo≥500 ms仍未追上 | UNLOCKED | STALE/STOP；CSV先结束pending后写transition |

超时采用 `>=`。示例：首次MISS在100 ms，新MISS在300 ms；600 ms timer只释放锁，不重评估300 ms的旧ID；下一新可用帧640 ms重新ACQUIRED。若首次检测到超时的是600 ms的新帧，则先释放旧锁，该帧是释放后的下一新帧，可ACQUIRED。若超时与STALE/OFF/LOST同时成立，安全状态优先，禁止同一评估又获取目标。没有新ID时500 ms到达的是STALE；持续新空帧时才可能NO_TARGET→LOST，不能混同两种场景。

AwaitingVideo期间MISS计时不被新awaiting帧重置，但不执行关联/释放；首次追上时以当前注入时间先判安全gate及MISS超时，再最多关联一次。等待超过stale_ms直接安全释放。计时使用本地评估时间而非Pi capture_ts_ns。

## 6. Session接入顺序与单ID一次（Review S1）

1. 到达事件只对递增ID记录本地到达时间和完整帧；重复/倒序payload不能覆盖缓存。
2. 原始gate为InferenceOff/Stale/AwaitingVideo时不关联、不推进MISS释放，直接作为原状态机输入。AwaitingVideo保持关联与EMA，安全状态在第4步释放。
3. 原始gate为Target/NoTarget时先推进MISS超时，再仅对未消费的新ID关联一次；重复ID取缓存。关联selected存在映射Target，否则映射NoTarget。完整帧必须匹配highestArrivedFrameId；不匹配时映射Stale。
4. 原advanceVisualTargetState只调用一次并提交；最终STALE/INFERENCE_OFF/LOST在同一次评估内释放锁，清空本次selected/关联诊断，原策略接收最终安全状态。超过500 ms才追上的帧可暂时关联，但最终STALE释放，不产生目标/EMA/建议输出。
5. 仅对最终非AwaitingVideo结果调用原evaluateVisualCommand；EMA及其初始化/重置、去重、滞回、dwell保持不变。MISS为HOLD，不把hc基线当策略输入。
6. snapshot的target、索引、hc和dets同ID；释放后旧ID不能重新获取。session既有QTimer只负责wakeup，纯函数内部无计时器。

取消两阶段预检。若实施发现此单次顺序会产生不正确输出的具体场景，写复现测试并交回Review，不直接恢复预检。旧frame的去重与原状态机到达记忆仍独立保留。

## 7. 显示一致性

- 保留标题“VISION DRY_RUN — no motion output (manual controls live)”及PROPOSED (not sent)，不添加实际发送开关。
- 增加 `lock: ACQUIRED`、`lock: ASSOCIATED d=__px`、`lock: MISS __ms`；释放显示 `lock: UNLOCKED`。等待视频另显示waiting for video，不能把上一ID的关联结果标成新ID已评估。
- 所有现有合法检测点仍使用橙色标记；选中索引额外青色圆环与LOCK文字，误差线只连到session选中点。重复位置的两检测用索引区分，不用浮点坐标猜身份。
- gate可渲染时，即使关联MISS，也显示其他检测点；没有可用选中目标时不画旧位置的误差线/LOCK标记。NO_TARGET/HOLD仍显示保留ex_f/effective。STALE/OFF/LOST或视频未追上时沿用门控隐藏过期检测。
- 去掉关联目标文字中的“(highest)”；hc仅作为CSV比较基线，不能使图像、面板和CSV分别选不同目标。所有坐标变换仍复用现有letterbox尺寸映射。

## 8. CSV schema与记录时机

原21列顺序保留，尾部新增9列，总计30列；每行都写schema_version，首行仍为CSV header，不加破坏既有解析器的注释/额外元数据行：

```text
row_kind,local_mono_ms,arrival_mono_ms,frame_id,capture_ts_ns,n_detections,sel_class,sel_conf,u,v,ex,ey,ex_f,yaw_cmd,state,proposed_command,effective_command,policy_version,policy_hash,session_id,awaiting_video,schema_version,assoc_status,assoc_dist_px,hc_u,hc_v,hc_conf,src_w,src_h,dets
```

| 新/变化字段 | frame行规则 |
|---|---|
| schema_version | visual-csv-v2，每行（含transition/awaiting）都填 |
| sel_class/sel_conf/u/v/ex/ey | 关联选中的目标；MISS为空，不能填hc目标或旧锁坐标；其余单位/符号不变 |
| assoc_status | ACQUIRED/ASSOCIATED/MISS；未进行关联的帧为空，不能写成MISS |
| assoc_dist_px | 第3节定义的距离，g17数字；没有可计算距离留空，不能当0 |
| hc_u/hc_v/hc_conf | 同一已评估帧中原selectTargetState会选中的合法目标；MISS但有门外目标时仍可填，空/全非法时留空 |
| src_w / src_h | 该ID原始DetectionFrame.sourceSize，单位px；含被放弃awaiting帧，transition空 |
| dets | 本帧全部流内检测，保持原顺序：class:conf:u:v;...；空检测为空字符串；不只保留关联入门限的候选 |

`dets`的class明确使用整数classId（避免className中的冒号/分号与内层格式冲突）；confidence/u/v使用locale无关g17，内部以冒号/分号分隔，之后整字段仍执行标准CSV双引号转义。保留收到的全部观测；关联排除的非法fixture可序列化为nan/inf等显式值，不默默删除/替换0。生产输入上限沿用既有256检测/64KiB协议限制。离线重放恢复classId/conf/u/v及流序；类名不是该紧凑列的内容。CSV在dets之前新增src_w/src_h，记录该ID的原图宽高（px）；重放直接使用这两列，不从像素最大值猜分辨率。

保留PR #43全部pending规则：awaiting的新ID先只保存一个pending；同ID第一次可用时写实际关联目标和本次新EMA；被新ID取代/STALE/OFF/会话切换/关闭录制时先写awaiting_video=1行，再处理新情况。放弃行的sel/误差/ex_f/yaw/assoc/hc全空，dets保留该ID原始检测以便统计未使用样本；不把上一帧关联状态塞入放弃行。

transition触发条件仍只有state或effective变化；不因assoc状态或dets变化额外写transition。transition的原帧字段仍空，schema_version/policy照常填写，assoc_status记录当前锁诊断（安全释放为UNLOCKED）；assoc_dist_px、hc_*、src_w/src_h、dets空，禁止当作关联样本。

local_mono_ms仍表示写行使用的本地评估时间，arrival_mono_ms仍为该ID首次到达时间；不改已有列时序含义或每会话每ID至多一行。默认关闭、Documents目录、NewOnly、32 MiB含header、250 ms GUI线程flush、错误停止及显示原因全部保留，不增加线程/队列/锁。更长行会更快达到原大小上限，不删除保护或截断dets假装完整。

统计/EMA复核使用frame、awaiting_video=0、同session/policy/schema；关联有效样本取ACQUIRED/ASSOCIATED且ex非空。MISS行没有ex、EMA保持，不能当0代入；STOP/会话重置后分段，中途录制首行ex_f作种子。空hc/u和awaiting行必须分别报告，不把两个非相邻有效点当相邻帧静默计入。

## 9. RED → GREEN 测试表

以下均为待实现验证项，设计阶段不填写“通过”。先在旧生产实现/最小可编译stub上获得行为断言RED，保存真实日志，再实现GREEN；不得把纯编译找不到符号作为唯一回归证据。

| 测试/归属 | 输入和精确期望 |
|---|---|
| A1 B段复现 / target_association_tests | 首帧450(.89)锁定，后续450/547的.85/.89交替领先，v相同；始终selected.u=450、ASSOCIATED且d=0；hc_u交替450/547 |
| A2 移动100帧 | 640宽，初始u=20，每新帧+5 px，共100次关联（末u=520）；均ASSOCIATED、d=5，避免fixture越界被合法性过滤 |
| A3 原目标消失 | 已锁450，仅547；首MISS无selected/d=97，NO_TARGET/HOLD且ex_f/effective保持；499 ms仍锁，500 ms释放，下一新ID ACQUIRED547；不存在同旧ID重选 |
| A4 门限 | 锁点(100,100)，(148,100)→ASSOCIATED；独立memory下(148.01,100)→MISS；另用(128.8,138.4)验证二维hypot=48，不能只看Δu |
| A5 ties/classes/合法性 | 等距离高conf优先、再同conf原流序；false允许classId变，true不同class拒绝；空、NaN、inf、越界/非法confidence不遮蔽合法项，获取仍保留原最高置信度规则 |
| A6 reset/非法参数 | gate NaN/0、max_miss_ms≤0、负/回退时钟拒绝；STALE/OFF/LOST解除锁；sourceSize变化重置；新会话低ID/0可获取 |
| S1 单帧去重 / session | 重复/倒序ID/重复refresh不改变锁点、missSince、距离或EMA；超时释放后旧ID不重新ACQUIRED；MISS期间新帧仍刷新原watchdog |
| S2 检测领先15 ms | AwaitingVideo时关联与EMA均不变；视频追上首次关联一次，无中途STOP/dwell锁住；重paint不再关联或EMA |
| S3 领先500 ms | 同ID视频不追上，最终状态机判STALE/STOP并在同次评估释放；晚到的视频不能复活已超时ID；新ID才恢复 |
| S4 原状态边界 | 连续新空帧NO_TARGET→1500 ms LOST/STOP；Pi时间戳冻结、HTTP新鲜、无新ID→500 ms STALE/STOP；MISS500 ms与STALE优先级分别测试 |
| C1 参数/hash | v2、每个新参数变化均改变SHA-256，默认hash稳定；原alpha/e_on/e_off/dwell不变，原VisualCommandPolicy测试原样运行 |
| C2 schema/hc/dets / logger | 30列/每行schema版本；同帧u锁450但hc_u547；MISS raw目标空且hc/dets有值；全部检测按classId流序可重放；CSV转义/locale/大小上限/IO错误仍覆盖 |
| C3 awaiting日志 | 15 ms只有一行实际ASSOCIATED/ACQUIRED、ex/新ex_f齐全且评估−到达=15；替代/500 ms先awaiting行后新帧/transition，dets为旧ID，assoc/hc为空 |
| C4 EMA复核 | 从真实session输出录CSV，排除awaiting/MISS空ex，按alpha=.3重算，连续接受样本与ex_f误差<1e−12；HOLD后获取不添加额外EMA重置 |
| U1 VideoView | 所有点显示，关联目标与hc不同仍LOCK/误差线指向关联目标；MISS保留其他点但无旧锁点；letterbox坐标正确、安全/等待时清空 |
| I1 真实loopback零写入 | 实际视频/检测独立TCP及HTTP：交替目标、MISS/HOLD、解锁再获取、LOST/STALE/OFF/新session/AwaitingVideo及CSV start/record/stop/error；每段保留FakeTransport.writes().isEmpty()断言 |
| I2 旧B1/策略fixture | 旧测试存在单帧160→480等>48 px跳变；把专门验证B1/EMA的连续目标fixture改为门内移动并更新预期算术，或显式新session；不放宽门限默认、不删除旧时序/零写入/单次EMA断言；PR描述逐项列出原意图、修改内容和原因（Review M2） |

完整CTest对比本次最新main的独立baseline，不新增失败目标。PR #43历史是24/26，仅main_window_tests原标签裁切、main_window_layout_tests原Motion/Gait可见性失败；robot_controller_tests历史时序失败在#43未复现。这是既有记录，不冒充本次已复跑结果；实施时保存当前baseline/final日志并列实际差异，不修/放宽无关布局或控制测试。

## 10. 实施顺序、交付与用户实验

- [ ] Claude Review本设计；本次在此之前停止，只推送设计文档和draft PR。
- [ ] 关联器/config：写A1–A6和hash RED，提取targetStateAt但保留原选择契约，最小实现GREEN，保存日志。
- [ ] Session：写S1–S4 RED，接入完整帧与关联记忆/单次状态机，GREEN；原策略测试原样复跑。
- [ ] 显示/CSV：先U1/C2–C4 RED，再实现selected标记/30列/pending字段，GREEN。
- [ ] loopback：I1/I2 RED→GREEN；baseline/final完整CTest，保存真实日志，比较新增失败。
- [ ] 沿用PR #41–#43的src新增行运动/舵机接口grep pattern；结合loopback零写入，不单凭grep证明运行时隔离。
- [ ] Release新便携目录 `D:\RoboBeetleConsole-portable-target-temporal-association-dry-run-20261003\`，若已存在则使用新后缀，不覆盖旧包。验证依赖启动，不把offscreen当硬件实测。
- [ ] 源码提交C1后构建，记录完整C1和EXE SHA-256；后续docs-only C2记交付head，明确构建输入未变，draft PR保持不合并，停下等Review。设计阶段没有新EXE，不提前填写hash或测试结论。

用户桌面实验：运行新包拍摄同一电脑屏幕，重复B段慢扫，放2–3个检测目标，同时观察LOCK标记和建议，录CSV。记录尺寸、持续时间/帧数、session/policy/schema，分别统计 `hc_u` 与关联后 `u` 的相邻新frame跳变 `abs(Δu)>30 px` 次数及可比较对数/比例（严格>30）。只对同session/policy、awaiting_video=0、两相邻frame行对应列都有值的配对统计，不跨MISS/awaiting缺样段拼接；另提供两列共同有值的配对对比、MISS次数/时长、重获取次数、awaiting占比。记录第一次ACQUIRED的对象，不能声称锁定450与输入顺序无关。

预期：置信度交替领先时hc_u可能跳，u沿锁定目标连续移动；目标消失时MISS/NO_TARGET/HOLD，约500 ms解锁后下一新帧可获取别的目标；无检测持续1.5 s为LOST/STOP，无新ID0.5 s为STALE/STOP。若两目标靠近/交叉或原目标每有效帧移动超过48 px，应记录局限并交回Review，不在现场擅自改alpha/滞回/dwell或加入运动输出。

验收仍是桌面DRY_RUN，没有机器人视觉命令发送和水下闭环结论。用户实测不得预填通过；合并/后续任务以用户新指令为准。
