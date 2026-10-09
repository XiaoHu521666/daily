# rtg_recover_db3

本包把 `rtg_rosbag2_pointcloud` 生成的 PCD/JSON 提取目录重建为标准 ROS2
SQLite3 bag。默认输出包含真实业务 Topic 和消息类型，`metadata.yaml` 由
rosbag2 根据 DB3 内容自动生成。

## 业务 DB3 重建

只需传提取目录：

```bash
ros2 run rtg_recover_db3 rtg_recover_db3 /绝对路径/源_extracted
```

程序会：

- 从目录名自动识别 651–655 机号并加载对应 YAML；
- 在源目录同级自动创建 `源名称_recovered_bag`；
- 同名目录已存在时自动改用 `_2`、`_3`，不覆盖旧结果；
- 先在服务器本地临时目录生成 DB3，关闭 SQLite 后复制到目标目录；
- 复制成功后自动删除本地临时目录。

652 提取目录可重建：

- `L1–L4/*.pcd` → 四路 `sensor_msgs/msg/PointCloud2`；
- `plc/*.json` → `/rtg/plc_topic_35`；
- `GantryPosMsg/*.json` → `/algl/GantryPosMsg`；
- `1–4/*.pcd` 与 `plc_frame_35/*.json` → `/rtg/lidar_top_sync` `DataSyncMsg`；
- `/rtg/task_exec_status` 和 `/algl/ResetScanRes` 作为零消息 Topic 保留在 metadata。

重建的 `PointCloud2` 沿用原包的 26 字节 `point_step`，只声明提取文件中实际
存在的 `x/y/z/intensity` 字段，其余空间不声明为已丢失的字段。这避免 PCL
的 32 字节内存对齐导致 DB3 额外膨胀。

仍支持手动指定输出目录和配置：

```bash
ros2 run rtg_recover_db3 rtg_recover_db3 \
  /绝对路径/源_extracted \
  /绝对路径/输出_bag \
  652.yaml
```

## 信息边界

提取目录没有保存的原 CDR、DataSync `group/anchor_stamp_ns`、DB 接收时间和被
同名毫秒文件覆盖的消息无法原样恢复。程序只使用提取目录中实际存在的数据；
缺失的固定 ROS 字段会是消息类型默认值。

如果只需要字节级保存提取文件，可使用：

```bash
ros2 run rtg_recover_db3 rtg_archive_extracted /源_extracted /保真归档_bag
```

该归档只能用 `rtg_restore_extracted` 恢复，不是业务 Topic bag。

## 编译与检查

```bash
source /opt/ros/humble/setup.bash
cd <工作区>
colcon build --packages-select rtg_recover_db3 --symlink-install
source install/setup.bash
ros2 bag info /绝对路径/输出_bag
```
