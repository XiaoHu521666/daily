# 大车防撞录包交接记录

更新日期：2026-09-07。下一对话先阅读本文件，再核对 live 文件；不要从最初四路方案重新开始。

## 1. 工作位置与修改范围

- 工作区：`/home/wxh/code/ws_ros2_rtg/rtg_auto_bag_recorder_654_20260723`
- 新建独立包：`rtg_gantry_collision_bag_recorder/`
- 参考录包包：`rtg_stack_scan_bag_recorder/rtg_auto_bag_recorder/`
- 防撞源码：`ws_ros2_rtg/src/algl_gantry_collision_detect_package/`
- 控制侧：`ws_ros2_rtg/src/rtg_main_package/rtg_main_package/ros2/controller_api_node.py`、`service/rtg_service.py`
- 已有录包包、防撞算法未修改。用户明确要求新建文件夹写独立程序，不在参考包里改业务。
- 用户偏好：中文沟通；代码功能分块、逻辑清晰连续、鲁棒但不过度防御，不添加冗余代码。当前工作区存在上层仓库及其他用户文件，避免无关整理、删除或覆盖。

## 2. 最终业务规则

1. 程序全天监听，仅保存报警事件；从报警开始，到报警解除。
2. 不录报警前几秒，不加解除后的延时。
3. 最新已改成按方向保存两路雷达，不再每个包保存四路。
4. 左右各一个节点进程、各一个 Writer，独立触发和结束。两侧同时报警生成两个事件包，每包仍只有两路点云。
5. 每包四个话题：本组两路完整 `sensor_msgs/msg/PointCloud2`、完整 `sensor_device_interfaces/msg/PlcMsg`、本组 `rtg_algorithm_interfaces/msg/SafetyDetectMsg`。报警消息为实现时补充的数据，用于保存触发/解除证据；没有录相机图像。
6. 当前实现直接按障碍物报警触发，**没有非零车速门槛**。用户最初描述“大车移动遇障”，后续强调报警开始到解除；实现已在此前回复中明确说明静止收到报警也会录。不要误称已经实现运动门控。
7. 进入减速报警区仍可能移动，进入停止报警区可能停住。开始录制后停车不会提前结束。
8. 到目标贝位正常减速属于运动规划，不以车速下降触发录包。

| 实例/目录标签 | 监测区域 | 点云 | 正常结束条件 |
|---|---|---|---|
| left | 1、3 | L21、L18 | 区域1和3均明确正常 |
| right | 2、4 | L20、L19 | 区域2和4均明确正常 |

同组两个区域分别维护；未知状态不能当作解除，另一组状态不影响本组。仅关注区域1～4，区域7/8等其他侧方检测不纳入。

## 3. 已确认的防撞依据

- `/algl/GantryColDetMsg` 实际类型是 `SafetyDetectMsg`，不是名为 GantryColDetMsg 的接口类型。
- `/rtg/plc_topic_35` 是当前工程 PLC 话题。
- 对齐当前控制侧距离规则：有效非负有限距离 `<16.0 m` 报警，`<5.5 m` 为停止等级；16米恰好正常，5.5米仍减速报警。没有用 `has_object` 或 zone 字段替代控制侧距离判定。
- 算法输出融合相机与雷达，故点云不能完整重现所有视觉触发原因。
- V2 算法自身部分边界是 `<=`，且有旧版/V1.1/V2 并存；现场运行版本尚未核验。当前录包阈值明确对齐 controller_api_node.py 的严格小于逻辑。
- 控制代码左防撞组合区域1/3，右防撞组合2/4，并分别限制大车两个运行方向。用户提供安装图显示动力房侧L20/L21、电气房侧L18/L19；不要按图片观察方向重新命名或改为L20/L21一组。
- 雷达区域映射：L21→设备37→区域1，L20→设备36→区域2，L18→设备10→区域3，L19→设备11→区域4。

## 4. 用户提供的设备配置（已写入 YAML）

Topic 按现有工程命名形式 `/livox/lidar_<序列号>` 生成。下表按 L18、L19、L20、L21 顺序列出：

| 设备 | L18 | L19 | L20 | L21 |
|---|---|---|---|---|
| 650 | 3GGDN4V00215991 | 3GGDN4M00218711 | 3GGDN4M00218871 | 3GGDN4V00215831 |
| 651 | 3GGDJA600101241 | 3GGDJ9Q00101241 | 3GGDJ9U00100441 | 3GGDJ9U00100191 |
| 652 | 3GGDJ9U00100891 | 3GGDJA500100121 | 3GGDJ9U00101121 | 3GGDJ9P00100821 |
| 653 | 3GGDJ9U00100411 | 3GGDJ9S00101101 | 3GGDJ9U00100181 | 3GGDJA500100521 |
| 654 | 3GGDJA500101151 | 3GGDJA600101201 | 3GGDJ9S00100741 | 3GGDJA600101431 |
| 655 | 3GGDN4M00218691 | 3GGDN5400200071 | 3GGDN4M00218751 | 3GGDN4V00216011 |

IP 规则（来自用户清单）：设备650～655分别为 `10.141.50`～`10.141.55` 网段；L18末段98，L19为99，L20为100，L21为101。IP记录在YAML注释，仅用于核对；录包程序不修改驱动IP。

- 已有 `config/recorder_650.yaml`～`recorder_655.yaml` 六份独立配置。
- `config/recorder.yaml` 默认654，与 recorder_654.yaml 解析后参数完全一致。
- **之前算法 README 中的四条历史 Topic 实际属于653，不是654。**已纠正新包默认配置，勿再次照抄到654。
- 之前留空的 Topic 已全部填好，缺少序列号映射不再是阻塞项。
- 尚需现场确认实际发布 Topic 没有重映射，消息类型与接口版本一致。

## 5. 实现文件及机制

- `src/gantry_collision_bag_recorder_node.cpp`：参数读取与校验、订阅、分组报警、事件目录、Writer写入/关闭、健康检查，按功能顺序组织。
- `include/rtg_gantry_collision_bag_recorder/alarm_state.hpp`：本组两个区域状态，拒绝负数/NaN/Inf及其他组区域，未知不视为正常。
- `launch/gantry_collision_bag_recorder.launch.py`：启动 `gantry_collision_bag_recorder_left` 和 `_right`；同一配置文件，通过节点名加载各组参数，公共参数使用 `/**`；异常退出后10秒重启。
- `CMakeLists.txt`、`package.xml`：Humble C++17独立包，依赖rosbag2_cpp/rosbag2_storage/rclcpp及消息包；直接用Writer，没有复用旧包的任务/集卡/闭锁/起升门控。
- `test/test_alarm_state.cpp`：无ROS依赖的状态测试。
- `README.md`：业务、部署、配置切换、输出和验收说明。

每个进程单线程处理回调。持续订阅本组点云，空闲不写盘、不创建空bag；报警时才打开Writer。bag时间优先采用DDS接收时间，RMW无时间时退回系统回调时间；不修改原消息时间字段。触发帧与最后解除帧写入后才关包。

输出：`./ros_bag/gantry_collision/YYYY_MM_DD/rtg_<设备>_gantry_collision_<left或right>_<日期时间纳秒>/`。重复名称加序号，不覆盖旧包。

目录包含DB3、metadata.yaml、event.txt。event.txt状态是 recording / alarm_cleared / shutdown_before_clear / error_before_clear，正常结束打印各话题成功提交Writer的计数。计数不是独立落盘验证。

默认sqlite3 resilient配置、同步写入（max_cache_size=0）、每卷1GiB、不按时长中断报警事件；少于2GiB可用空间故障退出，不自动删旧包。PLC/点云数据超时3秒只报警，不伪造解除；报警发布者全部消失使本组状态未知，已有事件继续写，恢复后等待明确解除。超过1小时仍未解除会定期告警。

## 6. 已完成验证与边界

此前本对话执行通过：

- g++ C++17，`-Wall -Wextra -Wpedantic -Werror` 独立状态测试。
- 同一状态测试 AddressSanitizer + UndefinedBehaviorSanitizer 通过。
- 左右互不干扰、同组重叠报警、阈值边界、无效距离保持报警、未知与断线恢复测试通过。
- launch Python AST、package.xml解析、YAML解析通过；这不代表执行过ROS launch。
- 六台设备24个序列号、左右Topic分组、IP及前缀核对通过；默认配置与654参数一致检查通过。
- 文件空白检查通过。

**未完成：完整ROS节点编译、真实launch启动、ROS参数合并实测、真实话题订阅、bag落盘/回放、双组并发吞吐及24小时现场运行。**本地检查时没有 `/opt/ros`、ros2、colcon、cmake；有g++。系统python3缺PyYAML，配置检查复用 `/home/wxh/code/dc531/.venv/bin/python`，不必新建环境。

已知限制/下一阶段重点：

- 报警结果按变化发布，没有消息不代表解除。启动错过某区域正常帧、单发布者失联但其他发布者尚在、丢失解除帧，都可能使事件保持开启。当前设计保守等待明确解除，不能宣称已完全解决这些场景。
- best effort网络和单线程同步磁盘写入不能保证零丢帧，必须实际测量两组同时写入吞吐。
- 关闭事件时尚未执行的其他话题回调不补录，不保证所有话题在同一纳秒切齐。
- respawn只在launch仍运行时有效，没有部署systemd等开机常驻服务。
- Ctrl+C/错误尝试关闭Writer；强杀、断电、磁盘满不能保证完整metadata，可能需恢复索引。

## 7. 下一步建议与命令

优先根据用户下一条指令推进；不要重复询问已确定的两路分组或序列号。若继续验收：

1. 在目标ROS2环境确认实际源码路径与接口版本（`colcon list`、现场Topic类型）；只构建本包及确有需要的依赖。
2. 实测默认654的左右节点参数，特别是公共 `/**` 与节点专属参数合并。
3. 隔离ROS_DOMAIN_ID模拟：正常不建包；左报警仅left两路；右报警独立right两路；两侧重叠各自解除；停车继续；断线不假解除；退出正确收尾。
4. 使用 `ros2 bag info`、DB3消息计数与回放确认每包仅本组两路点云、完整PLC、触发/解除帧。
5. 现场真实数据吞吐、磁盘空间、长期常驻验证后，才能称为可全天现场运行。

本地状态测试（包根目录执行）：

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror -fsanitize=address,undefined -Iinclude test/test_alarm_state.cpp -o /tmp/test_gantry_alarm_pair
/tmp/test_gantry_alarm_pair
```

目标ROS工作区启动（本包已放入src，接口包已安装）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select rtg_gantry_collision_bag_recorder
source install/setup.bash
ros2 launch rtg_gantry_collision_bag_recorder gantry_collision_bag_recorder.launch.py
```

切换651示例（其他设备替换配置文件编号）：

```bash
ros2 launch rtg_gantry_collision_bag_recorder gantry_collision_bag_recorder.launch.py config:="$(ros2 pkg prefix rtg_gantry_collision_bag_recorder)/share/rtg_gantry_collision_bag_recorder/config/recorder_651.yaml"
```

配置修改后必须构建更新install，单独source不会复制源码配置。默认输出相对启动目录，现场部署建议指定绝对输出目录。
