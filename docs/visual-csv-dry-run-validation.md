# Task 03 / PR #43 — Visual CSV (DRY_RUN) 验证记录

2026-10-03；基于 PR #42 合并后的 main `2c82494e4b305818a8b30874d36214d26c9a54b8`，分支 `codex/visual-csv-dry-run`。按已批准 S1 实现；交付 draft 后停下等 Claude Review。本页的自动化结果不代表用户摄像头/舵机或水下实测。

## ① 发现、修改及原因

#42 已可显示视觉建议，但无法记录居中抖动，也无法筛选连续选中目标的片段观察滞回。本次只在既有 `VisualDiagnosticSession::diagnosticChanged` 后记录快照，增加 Vision Details 的 CSV 开关、目录编辑/Browse 和文件/错误状态。

`VisualCsvLogger` 在 GUI 线程使用 QFile 缓冲写入，每250 ms QTimer flush。默认关闭，用户开启后才创建文件。没有后台线程、工作队列、锁或.git扫描；不宣称磁盘IO非阻塞/硬实时。目录默认 Windows Documents/RoboBeetle/visual-logs（通常 `%USERPROFILE%\Documents\RoboBeetle\visual-logs`，由 Qt DocumentsLocation 解析，也支持系统重定向）。用户可选其他目录，建议使用仓库外目录；录制时禁止改目录。

文件名使用 UTC 毫秒时间及 UUID，`QIODevice::NewOnly` 保证不覆盖。单文件上限 **32 MiB = 33,554,432 bytes**，包括表头，写入前检查；达到上限、open/write/flush失败会停止，取消勾选，显示并保留原因，不自动重启/轮转。关闭开关或正常退出会 flush/close；强杀进程不保证最后250 ms缓冲写出。日志错误或磁盘故障可能使末尾数据不完整，分析时检查错误提示和CSV尾行。

新增 logger、单元测试；修改 CMake、MainWindow 和现有 config 头中的独立 `VisualCsvConfig`（max_file_bytes、flush_ms）。原运动策略参数/hash、最高置信度选择、看门狗、视频/检测/HTTP协议及原手动 handlers 均沿用 #42。

## ② CSV 契约、符号及单位

UTF-8，标准双引号CSV转义；数字使用 locale 无关的小数点、double g17，未提供的值留空，不能当作0。完整 **20列**（此前计划“19列”为计数笔误，实际列名未减少）：

```text
row_kind,local_mono_ms,frame_id,capture_ts_ns,n_detections,sel_class,sel_conf,u,v,ex,ey,ex_f,yaw_cmd,state,proposed_command,effective_command,policy_version,policy_hash,session_id,awaiting_video
```

| 字段 | 约定 |
|---|---|
| row_kind=frame | 每会话内首次看到的递增frame_id写一行；重复/倒序不再写；新session允许低ID/0重新开始 |
| row_kind=transition | state或effective变化写一行，frame_id至ey共9个帧字段留空；不因proposed单独变化写；第一份快照建立比较基线 |
| local_mono_ms | Console既有QElapsedTimer的本地经过毫秒，不是UTC，也不是Pi采集时刻；一个Console实例内跨session仍使用同一本地时钟 |
| capture_ts_ns | 原Pi采集时间戳，纳秒；不能与local_mono_ms直接相减 |
| session_id | Console检测会话ID；分段时与frame_id一起使用，不代表物理目标ID |
| sel_class / sel_conf | 既有最高置信度选中的className与confidence；没有新增追踪/选择策略 |
| u/v | 原图像素坐标，左上原点，右/下正；不是窗口缩放后的坐标 |
| ex / ey | `ex=(u-W/2)/(W/2)`、`ey=(v-H/2)/(H/2)`；无量纲，左/上负，右/下正 |
| ex_f / yaw_cmd | #42既有EMA误差及`clamp(K_yaw*ex_f,-1,1)`，均无量纲；不表示角度、角速度或PWM |
| proposed_command / effective_command | FORWARD/TURN_LEFT/TURN_RIGHT/STOP/HOLD；HOLD时effective明确记录保留的上一条建议，全部仅用于显示 |
| policy_version / policy_hash | 每行记录既有建议策略的版本及参数hash；CSV参数不混入运动策略hash |
| awaiting_video | 1表示检测领先视频，0表示没有这种等待；不是第六种VisualState |

NO_TARGET/HOLD保留ex_f/effective，原始目标字段为空；LOST/STALE/INFERENCE_OFF为STOP，原始及滤波误差清空。timer-only状态转移仍记录transition，可看见无新检测时的STALE/STOP。具体状态条件仍见 [#42验证记录](visual-command-dry-run-validation.md)。

B1采样约定：检测领先视频的新ID按首次到达快照写frame行，原始sel_class/conf/u/v/ex/ey为空，旧state/ex_f/effective保留，awaiting_video=1。视频追上后同ID不补第二行frame；真实state/effective改变仍记transition。因此不要把awaiting行的旧滤波值当作该帧已计算的新EMA，不要用这些行做原始抖动统计。

默认turn_sign=+1：正ex_f建议TURN_RIGHT，负建议TURN_LEFT；**实机符号未验证**。yaw仍遵循图像符号；将来发送连续yaw时，turn_sign必须同时作用于发送映射层。EMA按帧计算，时间常数随推理FPS变化，25 FPS约110 ms。

## ③ RED → GREEN 与验证

日志根目录：本worktree `build/qt-visual-csv-dry-run/`（ignored，不提交构建产物）。

| 证据 | 结果 |
|---|---|
| task03-ui-red.log | 编译成功，CSV默认开关/目录/Browse/OFF四项新增断言失败 |
| task03-logger-red.log | 可编译stub的11项行为断言失败，然后才实现记录器 |
| task03-logger-green.log、task03-ui-green.log | logger及扩展真实TCP/HTTP loopback通过 |
| task03-iofault-red.log / green.log | 测试专用Windows文件范围预约导致真实QFile write/flush失败；忽略错误的临时mutation产生2项RED，恢复生产检查后GREEN；生产logger没有锁 |
| task03-final-build.log | Release完整构建成功 |
| task03-baseline-tests.log | git archive独立构建最新main：23/25 |
| task03-final-tests.log | 最终24/26；仅与基线相同的两项布局失败；新增logger通过，扩展loopback通过 |

单元测试覆盖20列顺序、首帧/重复/倒序、新session低ID/0、state/effective转换与空帧字段、HOLD保留、AwaitingVideo、UTF-8逗号/引号/CRLF转义、德国locale小数点、唯一文件保留、32MiB配置和小额度边界、open错误、250ms真实timer、stop/destructor flush及真实Windows write/flush故障。loopback录CSV时覆盖TRACKING、NO_TARGET/HOLD、LOST/STOP、STALE/STOP、INFERENCE_OFF/STOP，含冻结Pi时间戳而HTTP新鲜、B1单ID一行、实际Stop Inference；start/record/stop/error全过程保留FakeTransport零机器人写入断言。

| 三项已知基线 | 当前main / 最终 | 说明 |
|---|---|---|
| robot_controller_tests | 通过 / 通过 | Task01历史APC时序敏感失败testApc220SustainedLoadPreservesHeartbeatSafetyMargin；170ms模拟RTT、450ms等待内Enable ACK前置条件未建立，后续队列请求被拒。本次未复现，确切历史调度根因未确认 |
| main_window_tests | 失败 / 失败 | 原有IMU、Depth、Protocol、Leak标签纵向裁切四项断言 |
| main_window_layout_tests | 失败 / 失败 | 原有Motion/Gait comfortable-window fully-visible；offscreen可用800×800，窗口1100×720 |

使用`QT_QPA_FONTDIR=C:\Windows\Fonts`，未修改/放宽旧布局或控制断言。内部规格/代码质量复核仅辅助实现，不替代Claude Review。

## ④ 范围及grep

仅Console/docs变更，不涉及Pi、Firmware、fomo-visual-servo、目标选择或连续步态协议。对src新增行使用与#41/#42相同pattern，grep无匹配（exit=1）：

```powershell
$pattern='IConsoleController|RobotController|RemoteRobotController|MotionManager|SimpleGait|CPG|ServoService|ServoId|ServoPwm|ServoAngle|MotionMode|startMotion|stopMotion|setGaitBackend|setFrontRearCoordination|enableServo|disableServo|disableAll|neutralServo|acquireControl|releaseControl|controller_->|#include.*"(controller|robot|remote|transport)/'
git diff --unified=0 main...HEAD -- RoboBeetleConsole/src |
  & D:\Git\usr\bin\grep.exe -E '^\+[^+]' |
  & D:\Git\usr\bin\grep.exe -nE $pattern
$LASTEXITCODE # 1，无匹配
```

文本grep与真实loopback零写入结合；已有手动控件仍可实际驱动机器人，CSV/视觉没有实机发送开关。

## ⑤ 构建、包及用户操作

便携包：`D:\RoboBeetleConsole-portable-visual-csv-dry-run-20261003\RoboBeetleConsole.exe`，未覆盖两个保留包。EXE SHA-256：`8AD18554450064D65A5B850FEFB9324E780B0E7C464E6B77BF1ED984C1EE7772`，构建与包内一致。源码commit在交付后续docs-only提交中登记，包内BUILD_INFO.txt同时记录。

windeployqt Release部署JPEG/windows/offscreen及运行库；仅系统目录PATH的offscreen启动检查持续3s，日志确认插件加载；仅结束本次检查新建PID23864，用户正在运行的旧包PID55132未停止。该检查证明依赖能启动，不证明真实摄像头、CSV60s数据或运动硬件实测。文档/测试中的模拟视频同样不是实机证据。

无需编译，运行新包并按既有操作连接Pi/Start Inference。进入下方 **Vision Details**，向下滚动到Visual proposal后的CSV区域；选择仓库外目录，勾选 **Record visual CSV (no motion output)**，观察 **CSV Recording** 和完整文件路径。关闭开关后应显示 **CSV OFF - saved**；文件错误/32MiB上限应显示 **CSV Error - stopped** 并取消勾选。

源码构建（PowerShell，先进入本worktree）：

```powershell
$env:PATH='D:\Qt\6.11.2\mingw_64\bin;D:\Qt\Tools\mingw1310_64\bin;'+$env:PATH
& D:\Qt\Tools\CMake_64\bin\cmake.exe -S RoboBeetleConsole -B build/qt-visual-csv-dry-run/feature-build -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DCMAKE_PREFIX_PATH=D:/Qt/6.11.2/mingw_64 -DCMAKE_MAKE_PROGRAM=D:/Qt/Tools/Ninja/ninja.exe -DCMAKE_CXX_COMPILER=D:/Qt/Tools/mingw1310_64/bin/g++.exe
& D:\Qt\Tools\CMake_64\bin\cmake.exe --build build/qt-visual-csv-dry-run/feature-build --parallel 6
$env:QT_QPA_PLATFORM='offscreen'; $env:QT_QPA_FONTDIR='C:\Windows\Fonts'
& D:\Qt\Tools\CMake_64\bin\ctest.exe --test-dir build/qt-visual-csv-dry-run/feature-build --output-on-failure --parallel 4
```

## ⑥ 用户桌面实测清单（待用户验证，不预填通过）

| 实验 | 操作及应观察 |
|---|---|
| ① 中心60s | 开始CSV，目标保持中心约60s，关开关。取frame行、TRACKING、awaiting_video=0、ex有值、同session连续片段，统计ex均值/标准差/峰峰值，结合画面剔除误检/跳目标 |
| ②/②b 左中右、中心扰动 | CSV看ex→ex_f→建议，门限e_on=.25/e_off=.12、dwell≥1000ms；按sel_class/u/v连续性并结合视频人工确认同一目标片段，剔除awaiting/空目标及session改变。相同className不等于同一物体，本次没有追踪器，不能自动保证身份连续 |
| ③ Stop Inference | 继续录制，操作Stop；transition应记录STALE/STOP→INFERENCE_OFF/STOP，原始/滤波清空。建议未发送 |
| ④ 遮挡/移走 | 有新空帧：NO_TARGET/HOLD（effective保留上一建议、ex_f保留）→约1.5s LOST/STOP；若新ID停止则约0.5s STALE/STOP，不能混为LOST |

## 历史日志归档位置

清理前已逐个复制并核对源/目标大小及SHA-256：Task01七个日志在`D:\RoboBeetle-results\logs\task01\`，Task02两个日志在`...\task02\`，Slice7五个测试日志在`...\slice7\`的原目录层级中。完整14项清单：`D:\RoboBeetle-results\logs\cleanup-20261003-archive-manifest.json`。旧worktree中的历史路径已不再可用，以归档为准。
