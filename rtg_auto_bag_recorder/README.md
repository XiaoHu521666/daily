# RTG 无人值守自动录包节点

节点监听 `/rtg/task_exec_status`、`/algl/TruckDetectMsg` 和 `/rtg/plc_topic_35`。
抓箱任务中确认区域内存在 `OTK` 集卡后，节点会提前创建并暂停 rosbag2 Recorder；
集卡方向为 `left`、`right` 或 `none` 均不影响该条件。
检测到 `spreader_lock` 上升沿且 `trolley_pos <= 1.5 m` 时恢复写入。闭锁先到时会暂存触发，
小车随后进入 1.5 m 范围也会自动开始。检测到 `spreader_unlock` 上升沿、
起升高度达到阈值或录制超时后，节点暂停并关闭当前 bag，然后预热下一次录制。

`config/` 只保留 `recorder_650.yaml` 至 `recorder_655.yaml` 六份场景配置；
启动时必须通过 `config:=` 指定当前场景。每份配置录制对应的四路雷达和一路 PLC。
655 回放的四终端命令及落盘核验见 [650-655录包完整测试流程.md](650-655录包完整测试流程.md)。
按测试流程通过 C++ 时间显示程序启动时，终端中 ROS 方括号的 Unix 时间戳会显示为北京时间；
预热、开始和结束录包的消息也会显示北京时间和纳秒。
bag 目录名仍使用本地日期时间。
节点启动时会等待并检查四路雷达 Topic 是否存在发布者；超过
`topic_check_timeout_seconds` 仍有雷达缺失时，将打印缺失的 Topic 并退出。

## 编译

```bash
cd /home/dnt/ws_ros2_rtg
source /opt/ros/humble/setup.bash
colcon build --packages-select rtg_auto_bag_recorder
source install/setup.bash
```

## 启动

```bash
ros2 launch rtg_auto_bag_recorder auto_bag_recorder.launch.py config:=/home/dnt/ws_ros2_rtg/install/rtg_auto_bag_recorder/share/rtg_auto_bag_recorder/config/recorder_655.yaml
```

覆盖输出目录和停止高度：

```bash
ros2 run rtg_auto_bag_recorder auto_bag_recorder_node --ros-args \
  --params-file install/rtg_auto_bag_recorder/share/rtg_auto_bag_recorder/config/recorder_655.yaml \
  -p output_base_directory:=/mnt/win_share/Data_Storage/研发/HIT/rosbag \
  -p stop_hoist_height:=10.0
```

网络共享目录必须预先通过 CIFS 挂载成 Linux 路径，并确保运行节点的用户具有写权限。

## 触发逻辑测试

先发布抓箱任务：

```bash
ros2 topic pub /rtg/task_exec_status rtg_algorithm_interfaces/msg/ExecStatusMsg \
  "{task_id: 'test001', truck_type: 'OTK', action_type: 2}" --once
```

再发布集卡检测结果：

```bash
ros2 topic pub /algl/TruckDetectMsg rtg_algorithm_interfaces/msg/TruckDetectMsg \
  "{region_has_truck: true, truck_direction: {result: 'none'}, truck_type: {result: 'OTK'}}" --once
```

PLC 通常由真实设备发布。离线测试时，应先发一帧未闭锁状态，再发闭锁状态，从而形成
`false -> true` 上升沿：

```bash
ros2 topic pub /rtg/plc_topic_35 sensor_device_interfaces/msg/PlcMsg \
  "{spreader_lock: false, spreader_unlock: true, trolley_pos: 1.5, hoist_height: 2.0}" --once

ros2 topic pub /rtg/plc_topic_35 sensor_device_interfaces/msg/PlcMsg \
  "{spreader_lock: true, spreader_unlock: false, trolley_pos: 1.5, hoist_height: 2.0}" --once
```

模拟吊具上升到停止高度：

```bash
ros2 topic pub /rtg/plc_topic_35 sensor_device_interfaces/msg/PlcMsg \
  "{spreader_lock: true, spreader_unlock: false, trolley_pos: 1.5, hoist_height: 10.0}" --once
```

或者模拟放箱开锁：

```bash
ros2 topic pub /rtg/plc_topic_35 sensor_device_interfaces/msg/PlcMsg \
  "{spreader_lock: false, spreader_unlock: true, spreader_landed: true, hoist_height: 2.0}" --once
```

如果现场没有稳定发布 `/rtg/task_exec_status`，可把 `require_pick_task` 改成 `false`，此时仅根据
集卡检测条件和 PLC 的闭锁上升沿启动录制。若也不需要集卡检测门控，可把
`require_truck_detect` 改为 `false`。
