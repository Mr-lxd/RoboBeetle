# 桌面视觉调试：目标质心与归一化误差

> 本文保留 Task 01 记录。Task 02 的本地看门狗和建议命令已在 draft #42 实现；新包、状态含义及用户②③④步骤见 [Task 02 验证记录](visual-command-dry-run-validation.md)。CSV 尚待 #42 合并后的独立 PR。

本步完成 `现有 FOMO 检测 → TargetState → ex/ey → Qt 画面`，用于先验证坐标。视觉路径固定为 **DRY_RUN：只计算、只显示，没有运动输出**。现有手动运动按钮仍然保持原来的行为；这个标注不是手动控制的全局禁用开关。

## ① 发现了什么

现有树莓派 LIVE 服务已经分别提供视频和检测元数据。检测包含类别、置信度及 `original_x/original_y` 原图质心，没有 bounding box。Qt 的 `DetectionClient` 已接收这些数据；`MainWindow::refreshDetectionOverlay()` 已检查推理状态、状态新鲜度、原图尺寸和视频/检测时间戳。

本步直接复用这条数据链：

```text
USB Camera → CameraOwner → FrameHub → InferenceWorker / FOMO
                                            ↓
                                   existing detection stream
                                            ↓ TCP 47012
DetectionClient → existing MainWindow freshness gate → selectTargetState
                                                           ↓
                                                    VideoView ex/ey

Existing video stream → TCP 47010 → existing VideoView image
```

摄像头、推理、TCP 协议、机器人控制器、CPG、正弦步态、STM32 和安全限幅都未修改。

## ② 修改了什么

- 新增纯计算模块 `RoboBeetleConsole/src/vision/TargetState.h/.cpp`，保留检测帧来源、目标类别、置信度、原图尺寸、质心和 `ex/ey`。
- `VideoView` 增加青色图像中心十字、中心到选中质心的青色误差线，以及 `VISION DRY_RUN` 数值面板。
- 保留全部原有橙色检测质心和类别/置信度标签。
- 增加坐标、窗口缩放、数据清空和真实 loopback 元数据链的自动测试。

多目标时暂时选**置信度最高的有效候选**；置信度相同则保留检测数组中的第一个。检测阈值继续由现有 FOMO 端决定。这里没有跨帧身份跟踪，多目标时选中类别可能变化。第一次实验请尽量让屏幕里只有一个可识别目标。

## ③ 为什么这样计算

使用检测原图的 `W × H`，不是网络输入的 192×192，也不是 Qt 控件尺寸：

```text
uc = W / 2.0
vc = H / 2.0
ex = (u - uc) / uc
ey = (v - vc) / vc
```

坐标原点在原图左上角，u 向右增大、v 向下增大；u、v、W、H 的单位是**像素**。`ex/ey` 是除以半幅尺寸后的**无量纲归一化误差**，不是像素、角度、角速度或 PWM。

因此：画面左侧 `ex < 0`，右侧 `ex > 0`；上方 `ey < 0`，下方 `ey > 0`。这只定义**图像坐标误差**，没有定义实机转向/俯仰的正负方向。

Qt 缩放图像时，辅助线按同一个图像矩形映射坐标，忽略外侧留白。零尺寸、非有限数值和越界质心不会产生有效目标状态。

## ④ 本步代码边界

`TargetState` 和 `VideoView` 不调用运动控制器，不生成舵机 PWM，也不发送机器人命令。本步没有 `yaw_cmd`、Kp、deadband、滤波、CSV 或允许真实视觉运动的开关；这些留给后续的小步骤。

`MainWindow::refreshDetectionOverlay()` 仍通过既有 `detectionOverlayRenderable()` 门控，只有被接受的检测帧才进入 `VideoView` 并计算 `ex/ey`。为了显示清空原因，新增 `detectionOverlayState()`；原布尔接口委托给它，尺寸、时间戳和年龄门限保持原值。

| 诊断状态 | 含义 | 数值显示 |
|---|---|---|
| `TARGET` | 推理运行、状态新鲜、帧有效，有可选择的目标 | 显示 u/v、confidence、ex/ey 和误差线 |
| `NO_TARGET` | 推理运行，尺寸与时间戳有效的新鲜检测帧，其检测数组为空 | 清空目标，ex/ey 为 `--` |
| `STALE` | HTTP 状态过期或未知、控制 ACK 后状态待核对、检测断开/缺失、尺寸不匹配、时间戳无效/超龄，或无法选择有效候选 | 清空目标，ex/ey 为 `--` |
| `INFERENCE_OFF` | 新鲜且可解析的状态确认推理不运行，或已有 stopping/retrying 操作 | 清空目标，ex/ey 为 `--` |

优先级为：未知/过期/待核对状态 → `STALE`；明确不运行 → `INFERENCE_OFF`；运行状态下再判断检测帧。停止时不会把断开元数据流误标成“新鲜空检测”。这些是**显示有效性状态**，尚不是 `TRACKING / CENTERED / NO_TARGET` 运动状态机。

点击现有 **Stop Inference** 会发送现有 HTTP POST。收到合法 ACK、进入 `inferenceReconcilePending()` 后立即清空误差并显示 `STALE`；后续新鲜状态确认 stopping/disabled 后显示 `INFERENCE_OFF`。请求尚未被确认时，不能据按钮点击声称推理已经停止。停止失败仍按现有服务状态显示；没有增加运动 STOP 命令。

沿用的检测年龄门限为相对当前视频帧 1500 ms；HTTP 状态新鲜度为 3500 ms。这不是本地墙钟计时的 target lost timeout；视频停止推进且状态仍新鲜时，不能保证旧检测会按墙钟超时。重新收到新鲜 running 状态后，仍可能显示符合既有门限的保留检测帧。后续运动控制必须另做失目标计时和安全 STOP。

## ⑤ 如何运行

Windows 上直接启动新便携包：

```powershell
& 'D:\RoboBeetleConsole-portable-visual-error-dry-run-status-20261003\RoboBeetleConsole.exe'
```

1. 树莓派继续使用你原来能工作的摄像头/FOMO 启动方式，本步没有部署或改动树莓派软件。
2. 在 Console 的 `Pi Host` 填入现有树莓派地址，点击 `Apply`，沿用现有视频端口。
3. 点击 `Connect Video`。如推理尚未运行，点击 `Start Inference`，等待显示 Running。
4. 让 USB 摄像头拍电脑屏幕，屏幕显示一个 FOMO 能识别的目标。
5. 检查摄像头画面右上角的 `VISION DRY_RUN`、类别、`conf`、`u/v`、`ex/ey`。

这次坐标实验只需要视觉服务；不需要点击机器人连接、Enable PWM 或运动按钮。原来的 `D:\RoboBeetleConsole-portable-motion-gait-flex-ui53a0e7` 仍保留。

开发者可在独立工作树重建（使用本机已安装的工具链）：

```powershell
Set-Location 'C:\Users\laixindong\.codex\worktrees\visual-error-dry-run\RoboBeetle'
$env:PATH = 'D:\Qt\6.11.2\mingw_64\bin;D:\Qt\Tools\mingw1310_64\bin;' + $env:PATH
& 'D:\Qt\Tools\CMake_64\bin\cmake.exe' -S RoboBeetleConsole -B build/qt-visual-dry-run -G Ninja '-DCMAKE_BUILD_TYPE=Release' '-DCMAKE_CXX_COMPILER=D:/Qt/Tools/mingw1310_64/bin/g++.exe' '-DCMAKE_MAKE_PROGRAM=D:/Qt/Tools/Ninja/ninja.exe' '-DCMAKE_PREFIX_PATH=D:/Qt/6.11.2/mingw_64' '-DBUILD_TESTING=ON'
& 'D:\Qt\Tools\CMake_64\bin\cmake.exe' --build build/qt-visual-dry-run --parallel 6
$env:QT_QPA_PLATFORM = 'offscreen'
$env:QT_QPA_FONTDIR = 'C:\Windows\Fonts'
& 'D:\Qt\Tools\CMake_64\bin\ctest.exe' --test-dir build/qt-visual-dry-run --output-on-failure -R '^(target_state_tests|video_view_tests|main_window_visual_error_tests)$'
```

`offscreen` 和字体目录只用于自动测试；正常双击便携 EXE 无需设置它们。测试失败时先保留真实输出，不修改通信参数或安全限制来让测试通过。

## ⑥ 应观察什么

| 屏幕目标位置（以摄像头画面为准） | 应观察到的 ex | 辅助线/显示 |
|---|---|---|
| 左侧 | 负数 | 青色线连向左侧橙色质心 |
| 中心附近 | 接近 0 | 误差线变短 |
| 右侧 | 正数 | 青色线连向右侧橙色质心 |
| 上方 / 下方 | ey 为负 / 正 | 纵向误差随位置变化 |
| 新鲜帧的检测数组为空 | `ex=-- ey=--` | `NO_TARGET`，目标误差线消失 |
| 状态/检测过期、断开或停止 ACK 后待核对 | `ex=-- ey=--` | `STALE`，目标误差线消失 |
| 已确认停止/禁用推理 | `ex=-- ey=--` | `INFERENCE_OFF`，目标误差线消失 |

640×480 中的计算例子：`(160,240) → ex=-0.5`、`(320,240) → ex=0`、`(480,240) → ex=+0.5`。真实 FOMO 的质心会有网格量化和检测抖动，不要求中心位置每帧恰好为零。

验收重点是：**屏幕目标左右移动 → 画面选中的质心左右移动 → ex 从负到零再到正**。如果符号或位置不符，先记录画面中的类别、u/v、图像尺寸和错误提示，不进入真实舵机测试。

## 三项视觉测试与基线对照

| 视觉测试 | 内容 |
|---|---|
| `target_state_tests` | 左/中/右、上/下、不同原图分辨率；最高置信度和同分首选；空数组、零尺寸、非有限值及越界候选 |
| `video_view_tests` | 数值面板、中心十字和误差线；缩放/留白映射；尺寸不匹配、清空/断开后目标和图形消失 |
| `main_window_visual_error_tests` | 真实 loopback TCP/HTTP 元数据到画面；既有新鲜度；空检测、停止 POST/ACK/状态核对、断开/重连；始终零机器人控制写入 |

Release 构建和上述三项测试通过；兼容性检查 `detection_stream_decoder_tests` 也通过。加载系统字体后，最终完整 CTest 为 **18/21**，以下三项失败均曾在未修改的 `243755648dc0ac050bae8f39170adf7fea08eaf1` 基线独立复现。前一轮为 19/21，仅两项布局失败，体现 APC220 测试的时序敏感性：

| 基线测试名称 | 真实失败内容与解释 |
|---|---|
| `robot_controller_tests` | `testApc220SustainedLoadPreservesHeartbeatSafetyMargin`：170 ms 模拟 RTT 下，450 ms 等待内 Enable ACK 没建立 enabled 前置条件，随后七次用户队列请求被拒绝。前一轮通过、最终轮复现失败，属于时序敏感失败；具体计时/调度根因未确认，不能据此断言实机通信故障 |
| `main_window_tests` | IMU、Depth、Protocol、Leak 标签高度不足，触发原有纵向裁切断言；正常字体下的基线布局问题 |
| `main_window_layout_tests` | 原有 comfortable window 下 Motion/Gait fully-visible 断言失败；offscreen 可用屏幕为 800×800，启动窗口为 1100×720，基线同样复现 |

本步没有修改通信/运动/保护实现，也没有放宽这些断言。测试记录与 grep 命令见 [验证记录](visual-error-dry-run-validation.md)。未指定字体的测试会出现方框文字，因此不作为正常字体布局验收。模拟检测预览和启动检查不代表真实摄像头/FOMO 或机器人硬件验收；摄像头拍屏幕的左/中/右实验仍待实际观察。

## 构建来源与发布记录

- 分支：`codex/visual-error-dry-run`；源码基线：`243755648dc0ac050bae8f39170adf7fea08eaf1`。
- **EXE 对应代码 commit：`657d1262f7865be06b3b69caaa55c65f1ce03aba`**。
- **EXE SHA-256：`955BE5F63467653CE16F9892AA0D4833BA5DFA3B1D7C017A39985AAB94CD7821`**。
- EXE：`D:\RoboBeetleConsole-portable-visual-error-dry-run-status-20261003\RoboBeetleConsole.exe`；与本机最终 Release 构建 EXE 的哈希一致。
- 上述代码提交之后的提交仅记录文档，不改变构建输入。文档不尝试记录包含自身内容的提交哈希；最终文档提交/PR head 从 Git 或便携包 `BUILD_INFO.txt` 核对。
- 新便携 EXE 使用系统目录 PATH 加自带 Qt 插件持续启动 3 秒；只停止检查启动的 PID，JPEG 插件已部署。此项属于依赖启动检查。
- 原运动便携包和前一版 `D:\RoboBeetleConsole-portable-visual-error-dry-run-20261003` 均保留；前一版不含本次新增的失效原因区分，实验请使用上述 `status` 包。
- 本步按用户要求提交、推送并以 draft PR 交付；不合并 main、不刷写固件、不执行真实运动。
