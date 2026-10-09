# ROS2 大 DB3 按大车静止贝位分割

本目录是 ROS2 `ament_cmake` 包，可通过 `ros2 run` 启动。程序直接扫描原始 DB3 中的：

- `/rtg/plc_topic_35`：四个 `gantry_velocity_*` 必须持续全部为 `0`，任意非零速度立即切断静止段；
- `/algl/GantryPosMsg`：自动读取每个静止区间占多数的 `block` 和 `bay_no`，无需预先指定贝位；
- 小车位置、速度不作为筛选条件，因此静止段内小车移动和静止数据都会保留；
- 输出包保留时间窗内的全部 Topic，并生成可供 `ros2 bag info/play` 使用的 `metadata.yaml`。

## 编译

把本目录放到 ROS2 工作区的 `src/rtg_split_db3` 后执行：

```bash
cd ~/cpp_ws_ros2_rtg
source /opt/ros/humble/setup.bash
colcon build --packages-select rtg_split_db3 --symlink-install
source install/setup.bash
```

程序使用当前已 source 工作区中的 `sensor_device_interfaces/PlcMsg` 和
`rtg_algorithm_interfaces/GantryPosMsg`。它们必须与录制该 DB3 时的消息版本一致。

## 运行

先只分析，不写文件：

```bash
ros2 run rtg_split_db3 rtg_split_db3 \
  "/mnt/data_storage/研发/HIT/新版堆扫/651_2026_07_30_40ft_dual20大小车交替动贝位.尺寸10.40.08.20.07.20" \
  --dry-run
```

确认区间后正式分割：

```bash
ros2 run rtg_split_db3 rtg_split_db3 \
  "/mnt/data_storage/研发/HIT/新版堆扫/651_2026_07_30_40ft_dual20大小车交替动贝位.尺寸10.40.08.20.07.20"
```

程序会自动识别全部有效静止贝位，输出数量不固定。这个测试数据集应识别出
10、8、7 三个贝位。若同一贝位中间发生大车小额调整，会拆成多个包，例如：

```text
651_7_2026_07_30_1/
651_7_2026_07_30_2/
```

默认输出到原 bag 目录的 `split_by_gantry_stationary/`。程序先在本地临时目录
生成并关闭 SQLite 文件，再传入最终输出目录，避免直接在 CIFS 网络盘创建
DB3 时触发文件锁。每个小包目录中包含 `*_0.db3` 和 `metadata.yaml`；输出
目录已存在时程序会停止，不会覆盖。

如网络盘不允许写入，可显式指定输出位置：

```bash
ros2 run rtg_split_db3 rtg_split_db3 SOURCE_BAG \
  --output-dir /home/wxh/code/db3_split_output
```

默认判据为：大车四路速度必须持续全部等于 `0`，静止区间至少 10 秒，
并将每段首尾各裁掉 2 秒。任何非零速度都会立即结束当前静止段，不会跨过
小额调整移动区间进行合并。可通过 `--min-stationary-seconds` 和
`--edge-trim-seconds` 调整时间条件。
