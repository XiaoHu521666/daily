#include "rtg_gantry_collision_bag_recorder/pre_record_buffer.hpp"
#include "rtg_gantry_collision_bag_recorder/recording_window.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rmw/rmw.h>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rtg_algorithm_interfaces/msg/safety_detect_msg.hpp>
#include <sensor_device_interfaces/msg/plc_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;
using namespace std::chrono_literals;
using Plc = sensor_device_interfaces::msg::PlcMsg;
using Alarm = rtg_algorithm_interfaces::msg::SafetyDetectMsg;
using Cloud = sensor_msgs::msg::PointCloud2;
using SteadyClock = std::chrono::steady_clock;

class GantryCollisionBagRecorder : public rclcpp::Node
{
public:
  GantryCollisionBagRecorder() : Node("gantry_collision_bag_recorder")
  {
    load_parameters();
    fs::create_directories(output_directory_);
    init_subscriptions();
    health_timer_ = create_wall_timer(1s, [this]() {check_health();});
    window_timer_ = create_wall_timer(50ms, [this]() {close_expired_window();});
    RCLCPP_INFO(get_logger(), "%s 组全天监听：预录 %.2f 秒，每条报警续录 %.2f 秒，最大录制 %.2f 秒",
      side_.c_str(), pre_record_seconds_, post_record_seconds_, max_record_seconds_);
    RCLCPP_INFO(get_logger(), "输出目录：%s", output_directory_.c_str());
    RCLCPP_INFO(get_logger(), "报警来源：header.frame_id=gantry_collision_detect_frame，本组触发，保存四区消息");
    RCLCPP_INFO(get_logger(), "触发后最大录制时长：%.2f 秒；DB3 分卷大小：%lld MiB（不是停止条件）",
      max_record_seconds_, static_cast<long long>(max_file_mb_));
    RCLCPP_INFO(get_logger(), "报警前缓存：%.2f 秒，每实例序列化容量上限 %lld MiB",
      pre_record_seconds_, static_cast<long long>(pre_record_max_mb_));
    for (const auto & topic : lidar_topics_) {
      RCLCPP_INFO(get_logger(), "点云：%s", topic.c_str());
    }
  }

  // executor 停止后调用；所有回调在单线程 executor 内顺序执行。
  void finish(const std::string & reason)
  {
    if (writer_) {
      close_event(reason);
    }
  }

private:
  struct BufferedMessage
  {
    std::shared_ptr<rclcpp::SerializedMessage> data;
    std::string topic;
    std::string type;
    rclcpp::Time stamp;
  };

  // 1. 每个实例负责一个防撞方向的触发，每包保存全部四路雷达。
  void load_parameters()
  {
    output_directory_ = fs::absolute(declare_parameter<std::string>(
      "output_base_directory", "./ros_bag/gantry_collision")).string();
    prefix_ = declare_parameter<std::string>("bag_name_prefix", "rtg_gantry_collision");
    side_ = declare_parameter<std::string>("collision_side", "left");
    if (side_ != "left" && side_ != "right") {
      throw std::invalid_argument("collision_side 必须为 left 或 right");
    }
    prefix_ += '_' + side_;
    plc_topic_ = declare_parameter<std::string>("plc_topic", "/rtg/plc_topic_35");
    alarm_topic_ = declare_parameter<std::string>("alarm_topic", "/algl/GantryColDetMsg");
    lidar_topics_ = declare_parameter<std::vector<std::string>>("lidar_topics", std::vector<std::string>{});
    slow_distance_ = declare_parameter<double>("slow_distance_m", 16.0);
    stop_distance_ = declare_parameter<double>("stop_distance_m", 5.5);
    pre_record_seconds_ = declare_parameter<double>("pre_record_seconds", 2.0);
    post_record_seconds_ = declare_parameter<double>("post_record_seconds", 5.0);
    max_record_seconds_ = declare_parameter<double>("max_record_seconds", 120.0);
    pre_record_max_mb_ = declare_parameter<int64_t>("pre_record_max_mb", 256);
    data_timeout_ = declare_parameter<double>("data_timeout_seconds", 3.0);
    min_free_gb_ = declare_parameter<double>("min_free_space_gb", 2.0);
    max_file_mb_ = declare_parameter<int64_t>("max_bagfile_size_mb", 1024);
    std::set<std::string> topics(lidar_topics_.begin(), lidar_topics_.end());
    topics.insert(plc_topic_);
    topics.insert(alarm_topic_);
    if (lidar_topics_.size() != 4 || topics.size() != 6 || topics.count("") != 0) {
      throw std::invalid_argument("请填写全部四路真实点云话题 L21/L18/L20/L19；不能重复或为空");
    }
    if (prefix_.empty() || prefix_.find_first_of("/\\") != std::string::npos ||
      !std::isfinite(slow_distance_) || !std::isfinite(stop_distance_) ||
      stop_distance_ <= 0.0 || slow_distance_ <= stop_distance_ ||
      !std::isfinite(pre_record_seconds_) || pre_record_seconds_ < 0.0 ||
      !std::isfinite(post_record_seconds_) || post_record_seconds_ <= 0.0 ||
      !std::isfinite(max_record_seconds_) || max_record_seconds_ <= 0.0 ||
      pre_record_max_mb_ <= 0 || pre_record_max_mb_ > INT64_MAX / (1024 * 1024) ||
      !std::isfinite(data_timeout_) || data_timeout_ <= 0.0 ||
      !std::isfinite(min_free_gb_) || min_free_gb_ < 0.0 || max_file_mb_ < 0)
    {
      throw std::invalid_argument("目录前缀、距离阈值、超时或存储参数无效");
    }
    pre_buffer_ = gantry_recorder::PreRecordBuffer<BufferedMessage>(
      pre_record_seconds_, static_cast<std::size_t>(pre_record_max_mb_) * 1024 * 1024);
  }

  // 2. 始终订阅并保留最近数据，录制中也更新缓存，供下一次事件使用。
  void init_subscriptions()
  {
    auto state_qos = rclcpp::SensorDataQoS();
    state_qos.keep_last(200);
    plc_sub_ = create_subscription<Plc>(plc_topic_, state_qos,
      [this](Plc::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
        close_expired_window();
        last_data_[0] = SteadyClock::now();
        plc_connected_ = msg->plc_connect;
        write_message(*msg, plc_topic_, message_time(info));
      });
    alarm_sub_ = create_subscription<Alarm>(alarm_topic_, state_qos,
      [this](Alarm::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
        on_alarm(*msg, info);
      });
    auto cloud_qos = rclcpp::SensorDataQoS();
    cloud_qos.keep_last(20);
    for (std::size_t i = 0; i < lidar_topics_.size(); ++i) {
      cloud_subs_.push_back(create_subscription<Cloud>(lidar_topics_[i], cloud_qos,
        [this, i](Cloud::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
          close_expired_window();
          last_data_[i + 1] = SteadyClock::now();
          write_message(*msg, lidar_topics_[i], message_time(info));
        }));
    }
  }

  // 保留 DDS 接收时间；原消息 header/source_timestamp 不修改。
  rclcpp::Time message_time(const rclcpp::MessageInfo & info)
  {
    const auto received = info.get_rmw_message_info().received_timestamp;
    return received > 0 ? rclcpp::Time(received, RCL_SYSTEM_TIME) : system_clock_.now();
  }

  // 3. 先核对防撞来源，再处理本组两区域，其他来源不入缓存或事件包。
  void on_alarm(const Alarm & msg, const rclcpp::MessageInfo & info)
  {
    close_expired_window();
    if (msg.header.frame_id != "gantry_collision_detect_frame" ||
      msg.area_code < 1 || msg.area_code > 4) {
      return;
    }
    const auto areas = side_ == "left" ? std::array<int, 2>{1, 3} :
      std::array<int, 2>{2, 4};
    const auto area = std::find(areas.begin(), areas.end(), msg.area_code);
    if (area == areas.end()) {
      write_message(msg, alarm_topic_, message_time(info));
      return;
    }
    PublisherId publisher;
    const auto & gid = info.get_rmw_message_info().publisher_gid;
    std::copy_n(gid.data, publisher.size(), publisher.begin());
    alarm_publishers_[area - areas.begin()] = publisher;
    const auto stamp = message_time(info);
    if (!std::isfinite(msg.object_distance) || msg.object_distance < 0.0) {
      write_message(msg, alarm_topic_, stamp);
      RCLCPP_WARN(get_logger(), "区域 %d 距离无效，不触发或延长录制", msg.area_code);
      return;
    }
    if (msg.object_distance < slow_distance_) {
      const auto now = SteadyClock::now();
      if (recording_window_.trigger(now, post_record_seconds_, max_record_seconds_)) {
        open_event(stamp, msg.area_code);
      }
      if (writer_) {
        record_until_ = std::min(system_clock_.now() + rclcpp::Duration::from_seconds(post_record_seconds_),
          event_start_ + rclcpp::Duration::from_seconds(max_record_seconds_));
      }
    }
    // 只有当前有效报警续期；正常或无效消息仅保存，不改变截止时间。
    write_message(msg, alarm_topic_, stamp);
  }

  // 4. 仅在报警触发时建立事件目录，空闲时不创建空 bag。
  fs::path event_path(const rclcpp::Time & stamp) const
  {
    const std::time_t seconds = stamp.nanoseconds() / 1000000000LL;
    std::tm local{};
    localtime_r(&seconds, &local);
    std::ostringstream date, name;
    date << std::put_time(&local, "%Y_%m_%d");
    name << std::put_time(&local, "%Y_%m_%d_%H_%M_%S")
         << '_' << std::setfill('0') << std::setw(9) << stamp.nanoseconds() % 1000000000LL
         << '_' << prefix_;
    const auto parent = fs::path(output_directory_) / date.str();
    fs::create_directories(parent);
    auto path = parent / name.str();
    for (int suffix = 1; fs::exists(path); ++suffix) {
      path = parent / (name.str() + '_' + std::to_string(suffix));
    }
    return path;
  }

  void open_event(const rclcpp::Time & stamp, int area)
  {
    // 开盘前固定历史窗口，避免开盘耗时把触发前数据淘汰。
    pre_buffer_.expire(SteadyClock::now());
    check_disk();
    active_path_ = event_path(stamp).string();
    rosbag2_storage::StorageOptions storage;
    storage.uri = active_path_;
    storage.storage_id = "sqlite3";
    storage.storage_preset_profile = "resilient";
    storage.max_bagfile_size = static_cast<uint64_t>(max_file_mb_) * 1024 * 1024;
    // 不使用有可能溢出丢消息的 rosbag 写缓存；磁盘吞吐需在现场验证。
    storage.max_cache_size = 0;
    auto writer = std::make_unique<rosbag2_cpp::Writer>();
    writer->open(storage);
    writer_ = std::move(writer);
    event_start_ = stamp;
    record_from_ = stamp - rclcpp::Duration::from_seconds(pre_record_seconds_);
    record_until_ = stamp + rclcpp::Duration::from_seconds(
      std::min(post_record_seconds_, max_record_seconds_));
    counts_.fill(0);
    create_topic(plc_topic_, "sensor_device_interfaces/msg/PlcMsg");
    create_topic(alarm_topic_, "rtg_algorithm_interfaces/msg/SafetyDetectMsg");
    for (const auto & topic : lidar_topics_) {
      create_topic(topic, "sensor_msgs/msg/PointCloud2");
    }
    write_status("recording");
    RCLCPP_INFO(get_logger(), "区域 %d 报警，开始录包：%s", area, active_path_.c_str());
    // 当前触发帧尚未入缓存，先补写历史，随后由 on_alarm 写触发帧一次。
    for (const auto & entry : pre_buffer_.entries()) {
      write_buffered(entry.message);
      if (!writer_) {
        break;
      }
    }
  }

  void create_topic(const std::string & name, const std::string & type)
  {
    rosbag2_storage::TopicMetadata metadata;
    metadata.name = name;
    metadata.type = type;
    metadata.serialization_format = rmw_get_serialization_format();
    writer_->create_topic(metadata);
  }

  template<typename Message>
  void write_message(const Message & msg, const std::string & topic, const rclcpp::Time & stamp)
  {
    // 截止检查在回调入口完成，确保开盘补写耗时较长时也保存当前触发报警。
    if (!writer_ && pre_record_seconds_ == 0.0) {
      return;
    }
    auto data = std::make_shared<rclcpp::SerializedMessage>();
    rclcpp::Serialization<Message> serialization;
    serialization.serialize_message(&msg, data.get());
    BufferedMessage buffered{data, topic, rosidl_generator_traits::name<Message>(), stamp};
    if (!pre_buffer_.push(buffered, SteadyClock::now(), data->capacity())) {
      RCLCPP_WARN_THROTTLE(get_logger(), system_clock_, 10000,
        "报警前缓存达到容量上限，部分历史消息未保留；请增大 pre_record_max_mb 或缩短预录时长");
    }
    write_buffered(buffered);
  }

  void write_buffered(const BufferedMessage & msg)
  {
    // 每条写入前检查最大时长，包含开盘补写过程；同步 I/O 返回后才能执行检查。
    if (writer_ && recording_window_.max_duration_reached(SteadyClock::now())) {
      close_event("max_record_duration");
    }
    if (!writer_ || msg.stamp < record_from_ || msg.stamp > record_until_) {
      return;
    }
    // Humble Writer 接管序列化内存，写副本以保留缓存供后续事件使用。
    writer_->write(std::make_shared<rclcpp::SerializedMessage>(*msg.data),
      msg.topic, msg.type, msg.stamp);
    const auto & topic = msg.topic;
    if (topic == plc_topic_) {
      ++counts_[0];
    } else if (topic == alarm_topic_) {
      ++counts_[1];
    } else {
      for (std::size_t i = 0; i < lidar_topics_.size(); ++i) {
        if (topic == lidar_topics_[i]) {
          ++counts_[i + 2];
          break;
        }
      }
    }
  }

  void close_event(const std::string & reason)
  {
    writer_->close();
    writer_.reset();
    write_status(reason);
    RCLCPP_INFO(get_logger(), "结束录包：%s，原因：%s", active_path_.c_str(), reason.c_str());
    for (std::size_t i = 0; i < counts_.size(); ++i) {
      const auto & topic = i == 0 ? plc_topic_ : (i == 1 ? alarm_topic_ : lidar_topics_[i - 2]);
      RCLCPP_INFO(get_logger(), "  %s：%llu 条", topic.c_str(),
        static_cast<unsigned long long>(counts_[i]));
      if (counts_[i] == 0) {
        RCLCPP_WARN(get_logger(), "本事件缺少话题数据：%s", topic.c_str());
      }
    }
  }

  void close_expired_window()
  {
    const auto now = SteadyClock::now();
    if (writer_ && recording_window_.max_duration_reached(now)) {
      close_event("max_record_duration");
    } else if (writer_ && recording_window_.expired(now)) {
      close_event("capture_window_complete");
    }
  }

  // event.txt 区分正常解除、退出和异常中断，计数为成功交给 Writer 的消息数。
  void write_status(const std::string & reason)
  {
    std::ofstream output(fs::path(active_path_) / "event.txt");
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output << "status=" << reason << '\n'
           << "start_receive_time_ns=" << event_start_.nanoseconds() << '\n'
           << "requested_record_from_ns=" << record_from_.nanoseconds() << '\n'
           << "requested_record_until_ns=" << record_until_.nanoseconds() << '\n'
           << "pre_record_seconds=" << pre_record_seconds_ << '\n'
           << "post_record_seconds=" << post_record_seconds_ << '\n'
           << "max_record_seconds=" << max_record_seconds_ << '\n'
           << "updated_time_ns=" << system_clock_.now().nanoseconds() << '\n';
    for (std::size_t i = 0; i < counts_.size(); ++i) {
      const auto & topic = i == 0 ? plc_topic_ : (i == 1 ? alarm_topic_ : lidar_topics_[i - 2]);
      output << topic << '=' << counts_[i] << '\n';
    }
    output.close();
  }

  // 5. 健康监测不把断线误当解除；固定窗口关闭不依赖报警来源在线。
  void check_disk() const
  {
    if (min_free_gb_ > 0.0 &&
      fs::space(output_directory_).available / (1024.0 * 1024 * 1024) < min_free_gb_)
    {
      throw std::runtime_error("可用磁盘空间不足，停止录包并保留已有数据");
    }
  }

  void check_health()
  {
    close_expired_window();
    const auto now = SteadyClock::now();
    pre_buffer_.expire(now);
    std::ostringstream missing;
    for (std::size_t i = 0; i < last_data_.size(); ++i) {
      if (last_data_[i] == SteadyClock::time_point{} ||
        std::chrono::duration<double>(now - last_data_[i]).count() > data_timeout_)
      {
        missing << ' ' << (i == 0 ? plc_topic_ : lidar_topics_[i - 1]);
      }
    }
    if (!plc_connected_) {
      missing << " PLC未连接";
    }
    // frame_id 来自消息，ROS 图只提供端点；用已收到消息的 GID 关联真实发布端。
    const auto publishers = get_publishers_info_by_topic(alarm_topic_);
    for (std::size_t i = 0; i < alarm_publishers_.size(); ++i) {
      auto & source = alarm_publishers_[i];
      if (source && std::none_of(publishers.begin(), publishers.end(),
          [&source](const auto & endpoint) {return endpoint.endpoint_gid() == *source;}))
      {
        source.reset();
      }
      if (!source) {
        const int area = (side_ == "left" ? 1 : 2) + static_cast<int>(i) * 2;
        missing << ' ' << alarm_topic_ << "(区域" << area
                << "防撞来源未确认或已离线，frame_id=gantry_collision_detect_frame)";
      }
    }
    const auto status = missing.str();
    if (status != last_health_warning_) {
      if (status.empty()) {
        RCLCPP_INFO(get_logger(), "PLC、四路点云及本组报警发布者检查恢复正常");
      } else {
        RCLCPP_WARN(get_logger(), "数据异常，缺失部分无法补录，不视为报警解除：%s", status.c_str());
      }
      last_health_warning_ = status;
    }
    if (writer_) {
      check_disk();
    }
  }

  std::string output_directory_, prefix_, side_, plc_topic_, alarm_topic_, active_path_;
  std::vector<std::string> lidar_topics_;
  double slow_distance_{16.0}, stop_distance_{5.5}, data_timeout_{3.0}, min_free_gb_{2.0};
  double pre_record_seconds_{2.0};
  double post_record_seconds_{5.0};
  double max_record_seconds_{120.0};
  int64_t pre_record_max_mb_{256};
  gantry_recorder::RecordingWindow recording_window_;
  gantry_recorder::PreRecordBuffer<BufferedMessage> pre_buffer_;
  int64_t max_file_mb_{1024};
  using PublisherId = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;
  std::array<std::optional<PublisherId>, 2> alarm_publishers_;
  rclcpp::Clock system_clock_{RCL_SYSTEM_TIME};
  rclcpp::Time event_start_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time record_from_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time record_until_{0, 0, RCL_SYSTEM_TIME};
  std::array<SteadyClock::time_point, 5> last_data_{};
  std::array<uint64_t, 6> counts_{};
  bool plc_connected_{false};
  std::string last_health_warning_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rclcpp::Subscription<Plc>::SharedPtr plc_sub_;
  rclcpp::Subscription<Alarm>::SharedPtr alarm_sub_;
  std::vector<rclcpp::Subscription<Cloud>::SharedPtr> cloud_subs_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::TimerBase::SharedPtr window_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger("gantry_collision_bag_recorder");
  std::shared_ptr<GantryCollisionBagRecorder> node;
  int result = 0;
  std::string reason = "shutdown_before_clear";
  try {
    node = std::make_shared<GantryCollisionBagRecorder>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(logger, "%s", error.what());
    reason = "error_before_clear";
    result = 1;
  }
  try {
    if (node) {
      node->finish(reason);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger, "关闭 bag 失败，请检查事件目录并恢复索引：%s", error.what());
    result = 1;
  }
  node.reset();
  rclcpp::shutdown();
  return result;
}
