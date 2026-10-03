# Task 04 / PR #44 — Target Temporal Association (DRY_RUN) 验证记录

2026-10-03；基线 `8752700d0fb2fb423527692e8aec4272727b1631`（PR #43已合并），分支 `codex/target-temporal-association-dry-run`。Claude设计批准后按S1/M1/M2实施，交付后保持draft，不合并，等待Review。以下自动化和模拟图像不代表摄像头、机器人或水下实测。

## ① 发现、修改和原因

PR #43实测中最高置信度在u≈450/547之间交替领先。本次只在Console诊断链中增加纯最近邻关联器：第一次按原最高置信度规则获取；随后以原图二维像素距离选最近检测，48 px含边界，距离tie取高置信度，再tie保留流序。MISS无目标，保持锁点，视觉NO_TARGET/HOLD保留ex_f/effective；连续MISS500 ms释放锁，下一新可用ID重新获取。

`VisualDiagnosticSession`持有记忆，MainWindow传完整DetectionFrame，VideoView使用关联后的selected和原流索引；所有检测仍以橙色显示，锁点青色圆环/LOCK，误差线只连锁点。面板增加ACQUIRED/ASSOCIATED d=px/MISS ms/UNLOCKED，继续显示PROPOSED (not sent)和manual controls live。

S1单次顺序：原始Stale/InferenceOff/AwaitingVideo不关联，直接进入原状态机；Target/NoTarget先推进MISS超时，只对新ID关联，映射Target/NoTarget；原状态机只调用一次并提交，最终真正STALE/OFF/LOST在同次评估释放，不输出本次暂时选中的目标。AwaitingVideo保留的初始STALE不等于新发生的超时，仍等待首次评估；已关联ID的去重记忆不会因释放而清空。首次到达时因尚无视频尺寸而gate=Stale的未关联帧，若500 ms内变为可用，允许其第一次关联，保留旧视频门控启动行为。

没有恢复预检，也没有改动`VisualTargetStateMachine.cpp`或`VisualCommandPolicy.cpp`。参数alpha=.3、e_on=.25、e_off=.12、min_dwell_ms=1000、stale_ms=500、lost_ms=1500、K_yaw=1、turn_sign=+1、ui_tick_ms=50保持不变，EMA公式/初始化/安全重置条件不变。MISS后重新ACQUIRED不额外重置EMA。

新增[Provisional]：gate_px=48、max_miss_ms=500、require_same_class=false；policy_version升级`visual-command-proposal-v2`，三项参数都进入原有有序/locale无关SHA-256。默认类别标签变化仍可关联；true时比较classId。gate为固定原图px，不随窗口或分辨率自动缩放；原图尺寸变化先释放旧坐标锁。

关联仅假设位置连续性，不保证同一物理身份。初始获取仍由最高置信度决定，不能保证第一次总选450；目标交叉、门限内干扰或每有效帧位移超过48 px仍可能MISS或换目标，本阶段不加入速度预测/多目标追踪。

## ② CSV v2：30列和单位

原21列顺序保留，追加schema/关联/hc/尺寸/全量检测；Review M1的src_w/src_h紧邻dets之前：

```text
row_kind,local_mono_ms,arrival_mono_ms,frame_id,capture_ts_ns,n_detections,sel_class,sel_conf,u,v,ex,ey,ex_f,yaw_cmd,state,proposed_command,effective_command,policy_version,policy_hash,session_id,awaiting_video,schema_version,assoc_status,assoc_dist_px,hc_u,hc_v,hc_conf,src_w,src_h,dets
```

| 字段 | 含义 |
|---|---|
| schema_version | 每行visual-csv-v2，旧21列文件没有此列，不混拼 |
| sel_class/sel_conf/u/v/ex/ey | 关联后实际选中目标；MISS/无可用目标为空，不借用hc或旧锁点 |
| assoc_status | 已评估frame的ACQUIRED/ASSOCIATED/MISS；未经关联的frame空；transition为当前锁诊断，释放时UNLOCKED |
| assoc_dist_px | 到上次成功锁点的二维hypot，原图px；MISS可为门外最近候选距离，无候选/首次获取为空 |
| hc_u/hc_v/hc_conf | 同帧原最高置信度规则会选中的合法检测；可在MISS时有值；不作为策略输入 |
| src_w/src_h | 此ID的DetectionFrame原图宽高，px；被放弃awaiting帧也保留，transition空 |
| dets | 全部接收检测，原顺序classId:conf:u:v;...；class为整数classId，不是className；可离线重放 |

原图左上为原点，u右正、v下正。`ex=(u-W/2)/(W/2)`、`ey=(v-H/2)/(H/2)`无量纲，左/上负，右/下正。ex_f/yaw_cmd无量纲，仅显示建议，不是角度、角速度或PWM。turn_sign实机符号仍未验证，将来发送连续yaw须作用于发送映射层。EMA按成功接受目标的帧计算，时间常数随有效FPS变化（25 FPS约110 ms）。

数字locale无关g17，UTF-8和标准CSV双引号转义保留；dets内整数类别避免类名冒号/分号冲突，整体仍经过CSV转义。空值不能当0。未通过合法性检查的fixture观测仍可在dets保留为nan/inf等显式值，关联不会使用它们；生产流继续由原decoder验证，未修改Pi协议或检测数量上限。

继承PR #43记录时机：每会话每新递增ID至多一行；Awaiting先pending，首次可用时写实际关联/新EMA结果；被替代、STALE/OFF、会话切换或关闭时先写awaiting=1放弃行，再处理新帧/transition。放弃行sel/原始误差/滤波/yaw/assoc/hc为空，src_w/src_h/dets是该待写ID的原始数据。transition仍仅由state/effective变化触发，新帧字段空，不因assoc或proposed单独改变而加transition。

local_mono_ms是写行使用的本地评估时间，arrival_mono_ms是该ID首次到达时间；二者差是本地评估/放弃等待，不是网络延迟或flush时间。capture_ts_ns为Pi纳秒，不能与本地ms相减。GUI线程QFile、250 ms flush、默认关闭/可选目录、NewOnly、32 MiB含表头上限和出错停止保留，没有线程/队列/锁；不宣称磁盘写入硬实时或非阻塞。

统计仅用同session/policy/schema的frame、awaiting=0。EMA重算跳过MISS空ex，HOLD不会刷新EMA；安全STOP或会话重置分段，中途录制的首行ex_f作种子。不得跨MISS/awaiting缺样段把两个点当作相邻新frame。

## ③ RED → GREEN 和基线对照

真实日志在本worktree `build/task04/`（ignored，不提交构建产物）。源码/文档路径相对于本worktree。所有RED均有可编译行为断言失败，不仅是缺失符号。

| 日志/场景 | 结果 |
|---|---|
| baseline-config.log / baseline-build.log / baseline-tests.log | git archive隔离最新main；Release成功，完整CTest24/26 |
| association-red-build.log / association-red.log | 最高置信度可编译scaffold不满足锁定、移动100帧、MISS、门限和去重等断言 |
| association-green-build.log / association-green.log | 关联器/原TargetState/参数/原建议策略4/4通过；scaffold已替换，未提交 |
| session-red.log / session-green.log | 旧最高置信度session未满足锁定/hash/HOLD/去重；接入单次状态机后通过 |
| startup-awaiting-red.log / green.log | 初始STALE被Awaiting保留时不能消费pendingID；回归先RED，15 ms追上后ACQUIRED通过 |
| startup-gate-red.log / green.log | 误把原gate拒绝的未关联ID标为已消费的mutation被启动无视频尺寸用例检出；恢复后通过 |
| csv-red.log / csv-green.log | 旧21列logger27项失败；30列logger0失败，实际session的15 ms追上、hc跳变、MISS、替代/超时、全量dets/尺寸及EMA重算通过 |
| view-red.log / view-green.log | 旧自行最高置信度绘图5项失败；显式关联选中点、青色圆环、MISS保留其他检测通过 |
| loopback-red.log / loopback-green.log | 临时最高置信度无关联mutation被真实TCP/HTTP交替/MISS/解锁等断言检出；原生产实现恢复并强制重新编译，loopback通过 |
| final-build.log / final-tests.log | 当前Release完整构建成功；完整CTest25/27，只同样两项历史布局失败 |

门限测试保留d=48包含、d=48.01排除。原设计二维十进制fixture(128.8−100,138.4−100)计算值为48.000000000000007，实际在门外；二维验证改为整数偏移(28,36)门内、(36,36)门外，轴向48精确边界仍保留，未给生产门限添加epsilon。

首次启动、检测领先15 ms、等待超过500 ms后视频事件先于timer、重复ID/refresh、释放后旧ID不重放、会话ID归零、OFF/STALE/LOST释放、MISS500 ms和LOST1500 ms分别覆盖。实际loopback继承Stop Inference/HTTP3500 ms/原视频1500 ms/local500 ms边界及所有FakeTransport零写入断言，并新增450/547交替、门外检测仍可见/HOLD、500 ms后新ID获取另一目标、等待视频超时不复活。

临时mutation恢复时Windows Copy-Item保留旧mtime，曾使Ninja沿用较新的mutation对象；核对源/对象时间后刷新源码mtime重新编译，最终各目标和完整CTest均运行恢复后的生产实现。这是构建证据处理，不改变代码或测试要求。

| 完整CTest目标 | baseline / final | 已知原因 |
|---|---|---|
| main_window_tests | 失败 / 失败 | IMU、Depth、Protocol、Leak原标签纵向裁切四项断言 |
| main_window_layout_tests | 失败 / 失败 | 原Motion/Gait comfortable-window fully-visible，offscreen可用800×800、窗口1100×720 |
| robot_controller_tests | 通过 / 通过 | Task01历史APC ACK时序失败本次未复现，不声称已修复 |

使用QT_QPA_PLATFORM=offscreen、QT_QPA_FONTDIR=C:\Windows\Fonts。未删除/放宽布局或控制断言；旧Servo1别名弃用编译warning在baseline与feature均存在。内部只读代码/规格复核未发现阻塞项，不替代Claude Review或用户实测。

## ④ M2：逐项列出旧测试变化（同内容写入PR描述）

| 原测试/数据 | 原意图 | 修改内容及原因 |
|---|---|---|
| visual_diagnostic_session_tests::raceAndTimeout frame11 | 检测领先15 ms、只更新一次EMA、无STOP/dwell中断 | u=480改135，初始u=120、v=240不变；360 px跳变会触发新的48 px MISS，改15 px连续位移；ex_f预期−0.2875改−0.6109375；重复refresh和15/500 ms边界断言保留 |
| visual_diagnostic_session_tests::configProvenance | 默认版本与确定性参数hash | 预期v1改v2，保留same-config/session hash不变及alpha变化hash不同断言，新增三关联参数变化hash断言 |
| main_window_tests::testSlice5DetectionTextOverlayLifecycle frame11及其CSV断言 | 独立TCP先到检测、15 ms追上、重复绘制只做一次EMA | (480,240)改(135,175)，原frame10=(120,160)不变，二维位移≈21.21 px；新ex=−0.578125、ex_f=−0.6109375；到达/评估时间、一次frame行、原目标ex/ey和TURN_LEFT/无STOP断言保留 |
| 同一loopback，Stop后恢复running | HTTP运行状态恢复可用性 | 原“恢复同一旧ID目标”改为“running状态已收到，但释放过的已关联ID不重新获取”；后续原ID12空帧/ID13新目标仍验证NO_TARGET及恢复；Stop POST/ACK/GET和零写入断言保留 |
| 同一loopback，unknown→known状态恢复 | 未知worker判STALE、合法running恢复 | 原“旧ID直接恢复目标”改为“running已收到但已释放ID不能重放”；原STALE与INFERENCE_OFF区分、3500/500 ms边界、之后原新ID14恢复断言保留 |
| 同一loopback CSV解析 | frame/transition完整性与各状态记录 | 列数21改30，新增schema断言；原前21列索引、frame11至多一行、transition帧字段空、五视觉状态/零写入断言保留 |
| visual_csv_logger_tests的header及sample.policyVersion | logger格式/版本、转义/时序/限额 | 预期header追加9列，样本policyVersion v1改v2；sample的u/v/conf/ex、session/frame/timestamp/策略数值不变；根据新header字节数检查原大小上限，不放大32 MiB |
| logger中列数守卫/断言 | 原完整记录、pending、转义、EMA样本链 | 21改30：defaultsAndFirstFrame、transitionsAndHold、awaitingVideoAndMissingTarget、pendingReplacementAndInvalidation、csvReproducesEma、closingPendingFrame、arrivalBeforeRecording、escapingAndLocale。旧列索引和数值不变；原csvReproducesEma仍是纯策略/记录器测试，其−.6/−.2/.1/.8样本不改，另新增真实关联session EMA测试 |
| video_view_tests原7处setDetectionOverlay及preview context | 单点误差/letterbox、所有点、清空与模拟预览 | 新显示接口显式传原selectTargetState结果；textOverlayLifecycleIsIndependentFromVideoFrame（三处含wrongSize）、amberCentroidMarkersAndLabelsRenderForEveryDetection（一处）、visualErrorGeometryUsesImageRectangle（两处）、saveDiagnosticPreviews（一处）；原坐标/置信度/缩放/像素断言不变；preview context传完整overlay而非已选TargetState |

三类原断言均保留：**只做一次EMA、零机器人写入、时序边界**。旧测试预期变化只反映关联门限或安全释放后不重放，不放宽默认gate或改变原建议策略测试。新增关联/CSV/像素/loopback用例不是对旧断言的替换。

## ⑤ 范围验证和源码/EXE

只改Console/docs；Pi、Firmware、fomo以及原运动/舵机接口均未改动。src新增行沿用PR #41–#43 pattern，grep exit=1、无匹配，输出`build/task04/motion-interface-grep.log`为空：

```powershell
$pattern='IConsoleController|RobotController|RemoteRobotController|MotionManager|SimpleGait|CPG|ServoService|ServoId|ServoPwm|ServoAngle|MotionMode|startMotion|stopMotion|setGaitBackend|setFrontRearCoordination|enableServo|disableServo|disableAll|neutralServo|acquireControl|releaseControl|controller_->|#include.*"(controller|robot|remote|transport)/'
git diff --unified=0 origin/main...HEAD -- RoboBeetleConsole/src |
  & D:\Git\usr\bin\grep.exe -E '^\+[^+]' |
  & D:\Git\usr\bin\grep.exe -nE $pattern
$LASTEXITCODE # 1
```

源码/构建commit：**`ec5841f30ace8eadc551b5893a849087c553aa67`**。此后的交付提交仅改docs，Console构建输入保持不变；完整交付head在包内BUILD_INFO.txt记录，避免commit自引用。

便携包EXE：**`D:\RoboBeetleConsole-portable-target-temporal-association-dry-run-20261003\RoboBeetleConsole.exe`**。

EXE SHA-256（构建与包内一致）：**`C6A525E920FC1DB7624F60967FF161AAC50C07C4678816A13C73A1405F6E1C78`**。

windeployqt Release部署运行库及qjpeg/qwindows/qoffscreen，旧包保留不覆盖。仅系统目录PATH的offscreen启动持续3 s，确认包内qoffscreen加载；只结束本次新PID20420，未停止用户进程。此检查仅证明依赖启动，不证明真实摄像头/CSV或控制方向。

## ⑥ 如何运行与观察（等待用户实测）

无需自行编译：运行新包EXE，按原方式连接Pi摄像头并Start Inference。摄像头拍电脑屏幕，放2–3个检测目标，重复B段慢速扫动。观察初始ACQUIRED锁住哪个点，橙色全部检测与青色LOCK、ASSOCIATED距离；置信度交替时锁点应连续，门外目标不能立即替代。遮挡原目标：MISS/NO_TARGET/HOLD，约500 ms后下一新有效帧可获取别的目标；持续新空帧约1.5 s LOST/STOP，断新ID约0.5 s STALE/STOP。建议全部未发送，原手动控件仍可操作。

开启Vision Details的CSV，选择仓库外目录，结束时关闭录制并确认CSV OFF保存。核对schema=visual-csv-v2、30列表头、source尺寸、单session/policy及frame_id递增。分别统计hc_u、u的相邻新frame `abs(Δu)>30 px`次数及有效配对数/比例，只用awaiting=0且两相邻frame对应列都有值的同session/policy段，不跨MISS/awaiting缺样拼接；另给两列共同有效配对的比较、MISS时长/次数、重新ACQUIRED次数、awaiting行数占比与评估−到达等待分布。空值不当0，注明首次锁住对象。结果等用户提供，不预填通过。

源码构建（在本worktree，PowerShell）：

```powershell
$env:PATH='D:\Qt\6.11.2\mingw_64\bin;D:\Qt\Tools\mingw1310_64\bin;'+$env:PATH
& D:\Qt\Tools\CMake_64\bin\cmake.exe -S RoboBeetleConsole -B build/task04/feature -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DCMAKE_PREFIX_PATH=D:/Qt/6.11.2/mingw_64 -DCMAKE_MAKE_PROGRAM=D:/Qt/Tools/Ninja/ninja.exe -DCMAKE_CXX_COMPILER=D:/Qt/Tools/mingw1310_64/bin/g++.exe
& D:\Qt\Tools\CMake_64\bin\cmake.exe --build build/task04/feature --parallel 6
$env:QT_QPA_PLATFORM='offscreen'; $env:QT_QPA_FONTDIR='C:\Windows\Fonts'
& D:\Qt\Tools\CMake_64\bin\ctest.exe --test-dir build/task04/feature --output-on-failure --parallel 4
```

本次交付完成后停下，PR #44保持draft、不合并，等待Claude Review；不开始下一任务。
