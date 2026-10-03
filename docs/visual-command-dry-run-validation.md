# Task 02 — Visual Command Proposal (DRY_RUN) 验证记录

2026-10-03；[PR #42](https://github.com/Mr-lxd/RoboBeetle/pull/42)，分支 `codex/visual-command-dry-run`。仅交付设计①–③；下文保留实现交付时的测试证据。用户已确认②③④并授权转 ready、以 merge commit 合并。CSV 按 S1 简化方案留待本地清理经用户逐项确认并完成后另开 PR #43，当前无 CSV 开关、文件写入或录制功能。

## ① 发现及本次修改

Task 01 使用视频/Pi 时间戳门控；HTTP 仍新鲜且无新检测 ID 时，旧误差可能继续显示。检测与视频独立 TCP，检测领先视频也会被旧门控归为 Stale。

保留摄像头、FOMO、最高置信度选择、网络协议和手动运动代码。新增纯状态机、纯建议策略及 Qt 适配显示。本地 QElapsedTimer 到达时钟不与 Pi 采集时间相减；严格递增 frame_id 才续期。纯函数注入 nowMs，无内部 QTimer。Qt session 以50 ms常规间隔及更近截止时间唤醒，不依赖视频事件。

B1：仅时间戳领先返回 AwaitingVideo，`detectionOverlayRenderable()` 仍 false，原始叠加抑制不变；等待时保留 VisualState、effective、EMA/dwell。视频15 ms后追上只滤波一次，全程不新增STOP。最早等待500 ms仍未追上则STALE/STOP，后续领先ID不能无限延期；接受晚到视频前同样检查deadline，覆盖“catchup事件先于过期timer”的路径。

## ② 状态及建议语义

| 状态 | 条件 | 显示建议 |
|---|---|---|
| TRACKING | 当前合法目标且本地帧进展新鲜 | EMA/滞回/dwell 后的 FORWARD 或 TURN_LEFT/RIGHT；恢复期间可暂为 STOP |
| NO_TARGET | 新鲜空检测，连续不足1500 ms | HOLD；effective、滤波值保留并注明 retained，原始u/v/ex/ey/conf清空 |
| LOST | 持续有新鲜空帧，连续达到1500 ms | STOP，滤波清空、yaw=0 |
| STALE | 无帧、HTTP/原门控失效、无递增ID达到500 ms或等待视频超时 | 立即STOP，绕过dwell，原始及滤波值清空 |
| INFERENCE_OFF | 新鲜权威状态确认推理不运行 | 立即STOP并清空；Stop ACK尚待核对时先STALE/STOP |

AwaitingVideo 是显示门控原因，不是第六种 VisualState。视频断开清空画面，Vision Details 中仍可看到状态/建议。检测断开/错误/重建及视频新会话重置ID、EMA/dwell记忆。

普通建议切换至少间隔1000 ms；STOP优先绕过dwell。NO_TARGET的HOLD仅是保留建议的显示语义，不发送任何命令。标题注明 `no motion output` 和 `manual controls live`，手动按钮仍可实际控制机器人。

## ③ 参数、符号和单位

所有当前参数集中 `src/vision/VisualPolicyConfig.h`，标注 [Provisional]：alpha=0.3，e_on=0.25，e_off=0.12，min_dwell_ms=1000，stale_ms=500，lost_ms=1500，K_yaw=1.0，turn_sign=+1，ui_tick_ms=50。

- 原图u/v/W/H单位为像素，左上原点、向右/下为正。`ex=(u-W/2)/(W/2)`、`ey=(v-H/2)/(H/2)`，都是无量纲：左/上负、右/下正。ex_f与yaw_cmd也无量纲，yaw范围[-1,1]，不代表角度、角速度或PWM。
- 默认turn_sign=+1：正ex_f对应TURN_RIGHT，负对应TURN_LEFT；界面注明“实机符号未验证”。yaw只按图像符号计算 `clamp(K_yaw*ex_f,-1,1)`。将来发送连续yaw时，turn_sign必须同时作用于发送映射层。
- EMA按每个首次可用的新目标ID计算，首样本直接初始化；重复渲染不滤波。时间常数随推理FPS变化，alpha=.3、25 FPS时 `-1/(25*ln(.7))≈112 ms`（约110 ms）。
- 进入转向须 `abs(ex_f)>e_on`，回FORWARD须 `<e_off`，等号保持滞回。中心是建议FORWARD，不表示机器人已经前进。
- policyVersion=`visual-command-proposal-v1`；policyHash是版本及全部当前实际参数的固定顺序、浮点g17、UTF-8文本SHA-256，存在快照中。当前未实现CSV参数。

## ④ RED → GREEN 和测试证据

日志位于本工作目录 `build/qt-visual-dry-run/`，不提交构建产物。先确认新增断言失败，再实现对应模块；未删除或放宽原控制/布局断言。

| 功能 | RED日志 | 最终验证 |
|---|---|---|
| B1兼容门控 | task02-b1-red.log | detection_stream_decoder_tests通过 |
| 参数/看门狗 | task02-config-red.log、task02-watchdog-red.log | visual_policy_config_tests、visual_target_state_machine_tests通过 |
| EMA/滞回/dwell/符号/状态映射 | task02-policy-red.log（90个失败断言） | visual_command_policy_tests通过；132项断言 |
| Qt适配与独立唤醒 | task02-session-red.log | visual_diagnostic_session_tests通过 |
| 显示/loopback | task02-ui-red.log | video_view_tests、main_window_visual_error_tests通过 |
| Review补充：晚到deadline及参数来源 | task02-review-red.log（2个测试目标失败） | task02-review-green.log：2/2通过 |

注入时钟覆盖冻结Pi时间戳/HTTP新鲜、499/500 ms边界、同ID不续期、空帧1500 ms LOST、B1 15 ms单次EMA/500 ms超时/晚到视频、会话reset。真实loopback TCP/HTTP覆盖领先检测、fresh HTTP但停止ID进展、NO_TARGET→LOST、Stop ACK→权威停止及断开重连；各状态保留并扩展 FakeTransport 零机器人写入断言。实际Qt定时唤醒测试不依赖新视频或HTTP事件。

最终Release完整构建成功，`task02-final-build.log`；完整CTest **23/25**，`task02-final-tests.log`。独立归档、构建合并后main `3fa89cd5de246ade7da7a7a29c2bbd6df2108134`，同环境完整基线 **19/21**，`task02-baseline-tests.log`；四个新增测试目标全部通过，没有新增失败目标。

| 三项已知基线名称 | 本轮基线/最终 | 已有失败原因 |
|---|---|---|
| robot_controller_tests | 通过/通过 | Task 01历史时序敏感失败：testApc220SustainedLoadPreservesHeartbeatSafetyMargin，170 ms模拟RTT、450 ms等待内Enable ACK未建立enabled前置条件，随后七次队列请求被拒；确切调度根因未确认。本轮没有复现，不算当前失败 |
| main_window_tests | 失败/失败 | IMU、Depth、Protocol、Leak原有标签纵向裁切断言，main独立复现 |
| main_window_layout_tests | 失败/失败 | Motion/Gait comfortable-window fully-visible原有断言；offscreen可用800×800，窗口1100×720，main独立复现 |

使用系统字体 `QT_QPA_FONTDIR=C:\Windows\Fonts`，不能用方框字体产生的假通过。240×180及640×480左右/中心/空目标预览已人工检查（task02-final-previews/），标题可换行且建议/状态完整可见；预览使用模拟检测，不是实机证据。

## ⑤ grep 范围检查

沿用 PR #41 的完整pattern，PowerShell运行（grep exit=1表示无匹配，不是运行错误）：

```powershell
$pattern = 'IConsoleController|RobotController|RemoteRobotController|MotionManager|SimpleGait|CPG|ServoService|ServoId|ServoPwm|ServoAngle|MotionMode|startMotion|stopMotion|setGaitBackend|setFrontRearCoordination|enableServo|disableServo|disableAll|neutralServo|acquireControl|releaseControl|controller_->|#include.*"(controller|robot|remote|transport)/'
git diff --unified=0 main...HEAD -- RoboBeetleConsole/src |
  & D:\Git\usr\bin\grep.exe -E '^\+[^+]' |
  & D:\Git\usr\bin\grep.exe -nE $pattern
$LASTEXITCODE # 1，无输出
```

范围检查只针对src新增行，已有手动控制不删除；测试FakeTransport引用有意用于零写入断言。文本检查与实际loopback零写入互相补充。修改路径仅Console和docs，未修改Pi/Firmware/FOMO、目标选择或协议。

## ⑥ 构建、运行与用户实测

- EXE对应源码 commit：`321f4b2d784052f7acb92fcffab4fcf945de31c1`。后续提交仅更新文档，不改变构建输入，最终PR head另见Git/包内BUILD_INFO.txt。
- EXE SHA-256：`9190E73D48A8736E06CEF5F26EC10AF3657759FF7768C355FADFB2AE44D8D266`；Release构建与包内EXE一致。
- 新包：`D:\RoboBeetleConsole-portable-visual-command-dry-run-20261003\RoboBeetleConsole.exe`，旧包保留。windeployqt部署JPEG/offscreen/windows插件；系统目录PATH+自带依赖启动持续3 s，日志确认offscreen插件加载，仅终止检查创建的PID。这只证明依赖启动，不证明摄像头、FOMO、舵机或水下控制。

无需重新编译即可启动新包，沿用已验证Pi连接及Start/Stop Inference操作。视觉路径始终DRY_RUN、没有切换到实机发送的开关。源码构建参考（PowerShell，先切到本worktree）：

```powershell
$env:PATH='D:\Qt\6.11.2\mingw_64\bin;D:\Qt\Tools\mingw1310_64\bin;'+$env:PATH
& D:\Qt\Tools\CMake_64\bin\cmake.exe -S RoboBeetleConsole -B build/qt-visual-dry-run -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DCMAKE_PREFIX_PATH=D:/Qt/6.11.2/mingw_64 -DCMAKE_MAKE_PROGRAM=D:/Qt/Tools/Ninja/ninja.exe -DCMAKE_CXX_COMPILER=D:/Qt/Tools/mingw1310_64/bin/g++.exe
& D:\Qt\Tools\CMake_64\bin\cmake.exe --build build/qt-visual-dry-run --parallel 6
$env:QT_QPA_PLATFORM='offscreen'; $env:QT_QPA_FONTDIR='C:\Windows\Fonts'
& D:\Qt\Tools\CMake_64\bin\ctest.exe --test-dir build/qt-visual-dry-run --output-on-failure --parallel 4
```

| 原交付用户清单（结果见下方实测记录） | 应观察 |
|---|---|
| ② 左右缓慢移动屏幕目标 | ex/ex_f/yaw连续变化；超过门限切TURN，中心回FORWARD；滞回带不抖动，普通建议切换间隔≥1 s。只有PROPOSED变化，视觉不产生运动输出 |
| ③ Stop Inference | ACK待核对先STALE/STOP；权威状态停止后INFERENCE_OFF/STOP，原始误差/目标线清空、ex_f=--、yaw=0 |
| ④ 遮挡/移走 | 若新空帧持续到达：NO_TARGET/HOLD，约1.5 s LOST/STOP；如果新ID也停止：约0.5 s STALE/STOP；不要混同两种场景 |

用户①“中心60 s并录CSV统计抖动”属于合并后的CSV PR，本版不能录CSV。Task 01 用户T1–T8记录仍保留。

## 用户桌面实测（2026-10-03）

以下结果由用户确认，属于摄像头拍摄电脑屏幕的桌面显示/建议验证，不是视觉运动发送或水下闭环验收。

- EXE SHA-256 一致：`9190E73D48A8736E06CEF5F26EC10AF3657759FF7768C355FADFB2AE44D8D266`；对应源码 commit 仍为 `321f4b2d784052f7acb92fcffab4fcf945de31c1`。
- **② 通过**：`u=466.7 → ex=0.458 → TURN_RIGHT`；`u=288.0 → ex=-0.100 → FORWARD`；`u=120.0 → ex=-0.625 → TURN_LEFT`。
- **③ 通过**：`Stop Inference → STALE/STOP → INFERENCE_OFF/STOP`。
- **④ 通过**：`NO_TARGET/HOLD`，保留 `TURN_LEFT`，ex_f 保留值 `-0.836`；约1.5 s后变为 `LOST/STOP`。
- **②b 未验证**：多目标误检导致最高置信度选中的目标逐帧跳变，无法单独观察滞回。滞回由现有单元测试覆盖；桌面实机验证移到 PR #43，用CSV筛选“被选目标连续不变”的片段完成。本轮不改目标选择策略。

用户授权 PR #42 转 ready 并以 merge commit 合并。后续先只读盘点并停止等待逐项清理确认；清理完成后才开始 CSV PR #43。
