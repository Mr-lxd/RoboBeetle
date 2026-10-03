# PR #43 — Visual CSV DRY_RUN Implementation Plan

> 按已批准 S1 执行；使用 TDD 和内部规格/质量复核，交付 draft 后停下等 Claude Review。

**Goal:** 将 #42 的诊断快照记录为 CSV，便于居中60 s统计和在稳定选中目标片段中观察滞回；不发送运动命令。

**Architecture:** 一个 GUI 线程 VisualCsvLogger，使用 QFile 缓冲写入、250 ms QTimer flush。MainWindow只传既有诊断快照并提供关闭默认的开关/目录选择/状态文字。没有QThread、队列、锁或.git扫描，不改变检测选择、看门狗、策略或通信。

**Tech Stack:** Qt 6.11.2/C++20、QFile/QDir/QStandardPaths、CMake/CTest/MinGW Release。

基线：main `2c82494e4b305818a8b30874d36214d26c9a54b8`；分支 `codex/visual-csv-dry-run`。只改 Console/、docs/。清理和日志归档已先完成，旧硬件验收包、#42最新包和FOMO内容保留。

## ① Logger与测试

- [ ] 新增 `src/vision/VisualCsvLogger.h/.cpp`、`tests/visual_csv_logger_tests.cpp`；CSV默认常量放现有 `VisualPolicyConfig.h` 中独立 VisualCsvConfig，不改变运动策略参数/hash。
- [ ] 接口：`start(directory)`、`record(snapshot)`、`stop()`、`flush()`；`isRecording/filePath/lastError`供GUI及测试读取；状态变更signal。
- [ ] 先 RED：默认关闭、首帧/重复ID/新会话低ID、timer-only状态/effective转移、HOLD、frame/transition列、UTF-8/转义/locale、NewOnly、限额、open/flush错误及关闭flush。
- [ ] GREEN：默认目录 DocumentsLocation/RoboBeetle/visual-logs；唯一时间/UUID文件名、QIODevice::NewOnly；32 MiB含header，写前检查；错误停止并保留原因，不轮转、不覆盖。
- [ ] 每个新frame_id仅一行frame；按session重置ID下限。重复/倒序ID不重新记录。
- [ ] state或effective变化写transition，frame_id/capture_ts_ns/n_detections/sel_class/sel_conf/u/v/ex/ey空；起始快照作为初始转移基线。timer产生STALE/LOST/dwell变化也记。
- [ ] header：`row_kind,local_mono_ms,frame_id,capture_ts_ns,n_detections,sel_class,sel_conf,u,v,ex,ey,ex_f,yaw_cmd,state,proposed_command,effective_command,policy_version,policy_hash,session_id,awaiting_video`。
- [ ] effective_command用于明确HOLD保留的建议；awaiting_video用于剔除领先视频而无法使用原始目标的样本。新ID到达时记录当时快照；等待视频的frame行原始目标字段空，视频追上后的重复ID不另写frame行，避免重复采样；后续真实state/effective变化仍记transition。
- [ ] 不自动声称同类别就是同一个物体。稳定目标片段需结合画面、class/u/v连续性检查，无追踪器/目标选择变化。

## ② GUI与loopback

- [ ] 先加失败断言：Vision Details中的开关默认关闭、默认目录、选择/更改目录、记录开始/关闭/error提示；目录不写入仓库。
- [ ] 在既有diagnosticChanged连接中调用logger.record；状态文字同步active/path/error，错误自动取消勾选，不递归重启。
- [ ] MainWindow销毁前关闭flush；录制中禁用目录更改。
- [ ] 扩展真实TCP/HTTP loopback：启用CSV后B1、TRACKING、NO_TARGET/HOLD、LOST、STALE、INFERENCE_OFF转换写对应行；零机器人写入断言保持。测试使用QTemporaryDir，不在真实默认目录创建文件。
- [ ] 错误目录触发显式停止并显示原因；不改手动handlers或视频绘图。

## ③ 验证、打包和停止

- [ ] 独立归档合并后main并完整CTest，设置offscreen和系统字体；对照已知controller时序/两项布局失败，不新增失败目标。
- [ ] 内部先规格再质量复核，修复实质问题；复跑受影响测试和完整CTest。
- [ ] grep沿用#41运动/舵机接口pattern，对src新增行无匹配；logger还核对无线程/锁/队列/.git扫描。
- [ ] Release新便携包 `D:\RoboBeetleConsole-portable-visual-csv-dry-run-20261003`，不覆盖旧包；依赖启动检查。
- [ ] 代码提交C1，后续docs-only C2记录C1和EXE SHA-256，验证构建输入相同。正常push、draft PR、attach，停下等Review，不合并。
- [ ] 用户：①居中60 s录CSV；②移动目标并筛选稳定目标片段观察滞回；③Stop Inference的transition；④NO_TARGET→LOST或断流STALE。实测结论不得提前填通过。

GUI文件写入/flush可能有短暂磁盘延迟；这是Review明确批准的简化方案，不宣称非阻塞IO或硬实时。
