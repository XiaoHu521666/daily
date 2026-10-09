# 大车防撞录包

报警消息与录制状态分开：每条本组有效报警都将录制截止时间更新为当前时间加 5 秒，首次开录补写前 2 秒缓存。单次录制从首次触发起最多 120 秒，后续消息不刷新这个上限。没有新的报警消息，5 秒到期就关闭，不依赖解除信号。

例如第 0 秒首次报警，截止时间为第 5 秒；第 3 秒再次收到报警，截止时间更新为第 8 秒；第 6 秒再次收到则更新为第 11 秒。一直收到报警时，当前包最多录到第 120 秒，后续消息触发下一包。预录的历史 2 秒不占触发后的 120 秒计时。

达到上限后关闭旧包，后续有效报警消息直接开始新包，不等待静默或解除。新包补写前 2 秒缓存，每条报警继续刷新当前时间加 5 秒的截止时间，120 秒上限重新计时。因此持续收到报警时会连续产生多个独立录制包，相邻包的预录数据允许重叠；没有后续报警则不创建新包。正常消息、无效距离或来源离线不刷新截止时间。

## 保存内容

launch 保留左右两个实例：left 由区域 1/3 触发和续期，right 由区域 2/4 触发和续期。每包均保存全部四路 PointCloud2、完整 PlcMsg、四个区域的防撞 SafetyDetectMsg，共六个话题。两侧同时触发会各自产生一个包。

报警要求 frame_id 为 `gantry_collision_detect_frame`、区域为本组区域、距离有限且非负并严格小于 16 米。区域 1–4 的正确来源消息均可保存；另一组消息仅保存，不刷新本组截止时间。侧面集卡来源、空 frame_id、区域 7/8 被忽略。车速不作为触发条件。

区域与原配置保持一致：1=L21 左前，3=L18 左后，2=L20 右前，4=L19 右后。每包四路全录不依赖这项方向分组决定保存哪两路数据，但现场左右映射仍需实际核对。不保存相机图像，因此涉及相机融合的报警不能只靠本包完整复现。

## 参数与职责

- `pre_record_seconds: 2.0`：首次开录前缓存时间，允许为 0。
- `post_record_seconds: 5.0`：每条有效报警刷新后的续录时间，必须为有限正数。
- `max_record_seconds: 120.0`：首次触发后的最大录制时间，必须为有限正数。
- `pre_record_max_mb: 256`：每实例缓存序列化容量上限 MiB；两个实例合计最多约 512 MiB 缓存负载，另有 DDS 和写盘副本开销。
- `max_bagfile_size_mb: 1024`：DB3 分卷阈值，达到后换卷，不是停止条件。
- `data_timeout_seconds: 3.0`：PLC、点云健康监测，不控制录制停止。
- `min_free_space_gb: 2.0`：可用空间不足时异常退出，保留已有数据，不自动删除。

`recording_window.hpp` 只负责消息续期和每包最大截止时间；`pre_record_buffer.hpp` 只负责历史缓存；节点负责消息过滤、订阅、写盘及健康检查。节点不再使用 AlarmState 的解除状态控制录制，旧 `clear_hold_seconds` 参数已移除。保留旧状态类与独立测试作为历史逻辑，未接入当前节点。

计时使用 steady_clock，50 ms 定时器与回调入口检查结束条件，历史补写前也检查最大时长。所有回调在单线程 executor 顺序执行。原消息字段不变，bag 时间优先使用 DDS 接收时间，缺失时退回系统时间；写入范围为首次报警前预录时间至动态截止时间，且受最大时长限制。

## 部署与输出

默认 recorder.yaml 对应 654；650–655 配置均为左右实例填写全部四路雷达。先在服务器核对实际 topic 名称、PointCloud2 类型及 PLC/报警接口版本，再构建和重启：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select rtg_gantry_collision_bag_recorder
source install/setup.bash
ros2 launch rtg_gantry_collision_bag_recorder gantry_collision_bag_recorder.launch.py
```

其他设备通过 launch 的 config 参数指定对应 YAML。只 source 不会更新运行中的进程。旧两路 YAML 会被参数校验拒绝。输出默认相对于启动目录，现场建议设绝对目录。

新事件目录按“时间戳_设备前缀_左右”命名，例如 `2026_09_09_16_04_23_124436144_rtg_655_gantry_collision_right`。时间戳包含固定九位纳秒，按名称升序排列即可混合左右事件按时间排序；日期父目录保持不变，旧目录不重命名。

事件目录保存 DB3、metadata.yaml 和 event.txt，重名追加序号，不覆盖旧事件。event.txt 保存请求起止时间、录制参数、六话题计数和状态：

- `recording`：录制中，崩溃时可能残留。
- `capture_window_complete`：最后报警消息后的 5 秒到期。
- `max_record_duration`：首次触发后 120 秒到期。
- `shutdown_before_clear`、`error_before_clear`：沿用旧名称，表示人工退出或异常中断，不表示算法是否解除。

Writer 关闭后才输出结束日志。最终落盘情况须核对 DB3/metadata，以及关闭后文件大小和消息数是否不再增长。

## 验证与限制

本机没有 ROS2/colcon，纯 C++ 测试不代表实际录包验收：

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude test/test_recording_window.cpp -o /tmp/test_gantry_recording_window && /tmp/test_gantry_recording_window
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude test/test_pre_record_buffer.cpp -o /tmp/test_gantry_pre_record_buffer && /tmp/test_gantry_pre_record_buffer
```

服务器使用独立 ROS_DOMAIN_ID，持续发布四路带帧号/时间戳的点云和 PLC：

1. 预热至少 3 秒，单条报警后不发解除，应保存约前 2 秒至后 5 秒并关闭。
2. 第 0、3、6 秒发报警，应在第 11 秒关闭同一个包；正常消息与无效距离不续期。
3. 每秒发报警超过 240 秒，应每约 120 秒关闭旧包，由后续报警直接开新包；逐包核对前 2 秒缓存、触发消息仅一次、四路雷达及新包的独立120秒计时。
4. 停止报警消息至少 5 秒，再发报警，应产生新包并重新计时，不要求解除消息。
5. 两侧同时报警，每包六话题、四路点云都有真实数据；核对触发点云、PLC 与报警原始时间戳及吞吐。
6. 断线、空 frame_id、侧方来源、其他区域不能续期；只有点云/PLC 持续发布也不能续期。

scripts/source_integration_test.py 用于来源人工核验，不自动断言 DB3，也不代替上述续期和 120 秒测试。

磁盘开盘、写盘和关盘是同步操作。I/O 完全阻塞会延迟检查与关闭，120 秒不是操作系统强杀保证。预录容量不足、DDS 丢包、算法延迟超过 2 秒或开盘补写过慢仍可能漏掉触发输入；必须使用真实点云验收。进程重启后需要重新积累预录缓存。
