# rtg_auto_bag_compress

这是一个只录制以下两个 ROS 2 Topic 的 24 小时事件录包节点：

- `/rtg/plc_topic_35`：`sensor_device_interfaces/msg/PlcMsg`
- `/rtg/task_exec_status`：`rtg_algorithm_interfaces/msg/ExecStatusMsg`

## 录制规则

1. 两路消息独立建立初始状态，不把某个 Topic 启动时的第一帧误判为变化。
2. 任一路业务字段变化时创建一个 bag，先写入触发 Topic 的变化前一条完整消息，再写入触发后的当前消息和另一条 Topic 的最新状态；另一条 Topic 不存在时不会阻塞录包。
3. 事件期间两个 Topic 实际收到的所有消息都会持续写入。
4. 任一路再次变化都会重新计算静默时间；已经收到的业务字段都稳定达到 `quiet_period_seconds` 后关闭并保存该 bag。
5. 连续变化达到 `max_segment_seconds` 时切成下一段，防止 24 小时运行产生超大单文件。
6. 按服务器本地日期创建 `YYYY_MM_DD` 子目录；连续事件跨过零点时自动封存前一天的段，并在新日期目录续录。

每个事件包的 metadata 固定登记两个目标 Topic。某个 Topic 在该事件段没有收到消息时，`ros2 bag info` 仍会显示它，消息数为 `Count: 0`。

`header.stamp` 和 PLC 的 `source_timestamp` 只表示采集时间，不参与变化判断，否则每帧都会被误判为变化。浮点字段按 YAML 中的 epsilon 量化后比较，用来过滤毫米级以下的抖动；实际写入 bag 的仍是未经修改的完整原消息。

每个事件保存为一个标准 rosbag2 目录，路径格式为
`<output_base_directory>/YYYY_MM_DD/rtg_change_event_YYYY_MM_DD_HH_MM_SS_mmm`。
SQLite 文件使用 rosbag2 原生 zstd 文件压缩。程序退出时会正常关闭当前事件段并生成完整的 `metadata.yaml`。

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
cd ~/ws_ros2_rtg
colcon build --packages-select rtg_auto_bag_compress --symlink-install
source install/setup.bash
ros2 launch rtg_auto_bag_compress change_event_bag_recorder.launch.py
```

部署前应修改 `config/recorder.yaml` 中的 `output_base_directory` 为服务器上的绝对目录，并确认 zstd 插件存在：

```bash
ros2 pkg prefix rosbag2_compression_zstd
```

查看事件段：

```bash
ros2 bag info /绝对路径/YYYY_MM_DD/rtg_change_event_YYYY_MM_DD_HH_MM_SS_mmm
```

## 按时间段精确解压

`extract_time_range.py` 按 bag 数据库中的服务器接收时间筛选消息，时间精度为纳秒。
时间范围采用 `[start, end)`：包含开始时刻，不包含结束时刻。工具先读取所有
`metadata.yaml` 找到与查询时间相交的事件包，再通过 ROS 2 Humble 的
`ros2 bag convert` 转换为未压缩 SQLite bag、按 `messages.timestamp` 裁剪消息，
最后重新生成 `metadata.yaml`。转换在系统临时目录中的事件包副本上进行，结束后
临时副本会自动删除，源压缩归档不会新增 `.db3` 或被修改。
候选事件包如果在精确时间范围内没有任何消息，不会在导出目录留下空 bag。

先只预览将命中哪些事件包：

```bash
ros2 run rtg_auto_bag_compress extract_time_range.py \
  --source /home/dnt/ws_ros2_rtg/ros_bag/change_events \
  --start "2026-08-14 23:59:30.000000000" \
  --end "2026-08-15 00:00:10.000000000" \
  --list-only
```

确认后精确解压到一个全新的导出目录：

```bash
ros2 run rtg_auto_bag_compress extract_time_range.py \
  --source /home/dnt/ws_ros2_rtg/ros_bag/change_events \
  --start "2026-08-14 23:59:30.000000000" \
  --end "2026-08-15 00:00:10.000000000" \
  --output /home/dnt/ws_ros2_rtg/ros_bag/exports/2026_08_14_23_59_30_to_2026_08_15_00_00_10
```

输入时间默认使用 `Asia/Shanghai`。这里的时间是录包节点收到消息时调用 `now()`
写入 bag 的服务器时间，不是消息内部的 `header.stamp` 或 PLC `source_timestamp`。

24 小时服务必须使用正常的 SIGINT 或 SIGTERM 停止；不要使用 `kill -9`，否则正在写入的最后一个 bag 可能来不及生成完整元数据。
