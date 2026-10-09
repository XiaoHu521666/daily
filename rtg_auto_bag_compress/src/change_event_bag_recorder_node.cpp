#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_compression/compression_options.hpp>
#include <rosbag2_compression/sequential_compression_writer.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rtg_algorithm_interfaces/msg/exec_status_msg.hpp>
#include <sensor_device_interfaces/msg/plc_msg.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

class ChangeEventBagRecorder : public rclcpp::Node
{
public:
  ChangeEventBagRecorder()
  : Node("change_event_bag_recorder")
  {
    output_base_directory_ =
      declare_parameter<std::string>("output_base_directory", "./ros_bag/change_events");
    bag_name_prefix_ = declare_parameter<std::string>("bag_name_prefix", "rtg_change_event");
    plc_topic_ = declare_parameter<std::string>("plc_topic", "/rtg/plc_topic_35");
    exec_status_topic_ =
      declare_parameter<std::string>("exec_status_topic", "/rtg/task_exec_status");
    quiet_period_seconds_ = declare_parameter<double>("quiet_period_seconds", 5.0);
    max_segment_seconds_ = declare_parameter<double>("max_segment_seconds", 3600.0);
    plc_float_epsilon_ = declare_parameter<double>("plc_float_epsilon", 0.001);
    exec_float_epsilon_ = declare_parameter<double>("exec_float_epsilon", 0.001);
    compression_format_ = declare_parameter<std::string>("compression_format", "zstd");
    compression_mode_name_ = declare_parameter<std::string>("compression_mode", "file");
    compression_queue_size_ = declare_parameter<int>("compression_queue_size", 1);
    compression_threads_ = declare_parameter<int>("compression_threads", 0);

    validate_parameters();
    fs::create_directories(output_base_directory_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
    plc_sub_ = create_subscription<sensor_device_interfaces::msg::PlcMsg>(
      plc_topic_, qos,
      std::bind(&ChangeEventBagRecorder::on_plc, this, std::placeholders::_1));
    exec_status_sub_ = create_subscription<rtg_algorithm_interfaces::msg::ExecStatusMsg>(
      exec_status_topic_, qos,
      std::bind(&ChangeEventBagRecorder::on_exec_status, this, std::placeholders::_1));
    timer_ = create_wall_timer(100ms, std::bind(&ChangeEventBagRecorder::check_segment, this));

    RCLCPP_INFO(get_logger(), "变化事件录包节点已启动");
    RCLCPP_INFO(get_logger(), "PLC: %s", plc_topic_.c_str());
    RCLCPP_INFO(get_logger(), "任务状态: %s", exec_status_topic_.c_str());
    RCLCPP_INFO(
      get_logger(), "已接收 Topic 的业务字段均稳定 %.1f 秒后封包，单段最长 %.1f 秒",
      quiet_period_seconds_, max_segment_seconds_);
    RCLCPP_INFO(
      get_logger(), "压缩: mode=%s, format=%s；输出目录=%s",
      compression_mode_name_.c_str(), compression_format_.c_str(),
      fs::absolute(output_base_directory_).c_str());
  }

  ~ChangeEventBagRecorder() override
  {
    close_segment("节点退出");
  }

private:
  using PlcMsg = sensor_device_interfaces::msg::PlcMsg;
  using ExecStatusMsg = rtg_algorithm_interfaces::msg::ExecStatusMsg;
  using SteadyTime = std::chrono::steady_clock::time_point;

  void validate_parameters() const
  {
    if (quiet_period_seconds_ <= 0.0) {
      throw std::runtime_error("quiet_period_seconds 必须大于 0");
    }
    if (max_segment_seconds_ <= 0.0) {
      throw std::runtime_error("max_segment_seconds 必须大于 0");
    }
    if (plc_float_epsilon_ < 0.0 || exec_float_epsilon_ < 0.0) {
      throw std::runtime_error("浮点变化阈值不能小于 0");
    }
    if (compression_format_.empty()) {
      throw std::runtime_error("compression_format 不能为空");
    }
    if (compression_mode_name_ != "file" && compression_mode_name_ != "message") {
      throw std::runtime_error("compression_mode 只支持 file 或 message");
    }
    if (compression_queue_size_ <= 0 || compression_threads_ < 0) {
      throw std::runtime_error("压缩队列必须大于 0，压缩线程数不能小于 0");
    }
  }

  static double quantize(double value, double epsilon)
  {
    if (epsilon == 0.0) {
      return value;
    }
    const double result = std::round(value / epsilon) * epsilon;
    return result == 0.0 ? 0.0 : result;
  }

  static float quantize(float value, double epsilon)
  {
    return static_cast<float>(quantize(static_cast<double>(value), epsilon));
  }

  std::vector<uint8_t> plc_signature(const PlcMsg & message) const
  {
    PlcMsg normalized = message;
    normalized.header.stamp.sec = 0;
    normalized.header.stamp.nanosec = 0;
    normalized.source_timestamp = 0;

    normalized.spreader_weight = quantize(normalized.spreader_weight, plc_float_epsilon_);
    normalized.gantry_velocity_1 = quantize(normalized.gantry_velocity_1, plc_float_epsilon_);
    normalized.gantry_velocity_2 = quantize(normalized.gantry_velocity_2, plc_float_epsilon_);
    normalized.gantry_velocity_3 = quantize(normalized.gantry_velocity_3, plc_float_epsilon_);
    normalized.gantry_velocity_4 = quantize(normalized.gantry_velocity_4, plc_float_epsilon_);
    normalized.gantry_pos = quantize(normalized.gantry_pos, plc_float_epsilon_);
    normalized.trolley_pos = quantize(normalized.trolley_pos, plc_float_epsilon_);
    normalized.trolley_velocity = quantize(normalized.trolley_velocity, plc_float_epsilon_);
    normalized.hoist_height = quantize(normalized.hoist_height, plc_float_epsilon_);
    normalized.hoist_velocity = quantize(normalized.hoist_velocity, plc_float_epsilon_);
    normalized.left_crane_gantry_pos =
      quantize(normalized.left_crane_gantry_pos, plc_float_epsilon_);
    normalized.left_crane_gantry_velocity =
      quantize(normalized.left_crane_gantry_velocity, plc_float_epsilon_);
    normalized.right_crane_gantry_pos =
      quantize(normalized.right_crane_gantry_pos, plc_float_epsilon_);
    normalized.right_crane_gantry_velocity =
      quantize(normalized.right_crane_gantry_velocity, plc_float_epsilon_);
    return serialize(normalized);
  }

  std::vector<uint8_t> exec_status_signature(const ExecStatusMsg & message) const
  {
    ExecStatusMsg normalized = message;
    normalized.header.stamp.sec = 0;
    normalized.header.stamp.nanosec = 0;
    normalized.hoist_height = quantize(normalized.hoist_height, exec_float_epsilon_);
    normalized.trolley_pos = quantize(normalized.trolley_pos, exec_float_epsilon_);
    normalized.gantry_pos = quantize(normalized.gantry_pos, exec_float_epsilon_);
    normalized.object_height = quantize(normalized.object_height, exec_float_epsilon_);
    return serialize(normalized);
  }

  template<typename MessageT>
  static std::vector<uint8_t> serialize(const MessageT & message)
  {
    rclcpp::Serialization<MessageT> serialization;
    rclcpp::SerializedMessage serialized;
    serialization.serialize_message(&message, &serialized);
    const auto & raw = serialized.get_rcl_serialized_message();
    return std::vector<uint8_t>(raw.buffer, raw.buffer + raw.buffer_length);
  }

  void on_plc(const PlcMsg::SharedPtr message)
  {
    roll_segment_if_date_changed();
    const bool segment_was_open = static_cast<bool>(writer_);
    const auto received_time = now();
    const auto steady_now = std::chrono::steady_clock::now();
    auto signature = plc_signature(*message);
    const bool changed = have_plc_ && signature != plc_signature_;
    const auto previous_message = latest_plc_;
    const auto previous_time = latest_plc_time_;

    latest_plc_ = message;
    latest_plc_time_ = received_time;
    plc_signature_ = std::move(signature);
    have_plc_ = true;
    handle_message_change(
      changed, steady_now, "PlcMsg", previous_message, previous_time, plc_topic_);

    if (segment_was_open && writer_) {
      writer_->write(*message, plc_topic_, received_time);
    }
  }

  void on_exec_status(const ExecStatusMsg::SharedPtr message)
  {
    roll_segment_if_date_changed();
    const bool segment_was_open = static_cast<bool>(writer_);
    const auto received_time = now();
    const auto steady_now = std::chrono::steady_clock::now();
    auto signature = exec_status_signature(*message);
    const bool changed = have_exec_status_ && signature != exec_status_signature_;
    const auto previous_message = latest_exec_status_;
    const auto previous_time = latest_exec_status_time_;

    latest_exec_status_ = message;
    latest_exec_status_time_ = received_time;
    exec_status_signature_ = std::move(signature);
    have_exec_status_ = true;
    handle_message_change(
      changed, steady_now, "ExecStatusMsg", previous_message, previous_time,
      exec_status_topic_);

    if (segment_was_open && writer_) {
      writer_->write(*message, exec_status_topic_, received_time);
    }
  }

  template<typename MessageT>
  void handle_message_change(
    bool changed,
    const SteadyTime & steady_now,
    const char * source,
    const std::shared_ptr<MessageT> & previous_message,
    const rclcpp::Time & previous_time,
    const std::string & topic)
  {
    if (!changed) {
      return;
    }

    last_change_at_ = steady_now;
    if (!writer_) {
      open_segment(source);
      if (previous_message) {
        writer_->write(*previous_message, topic, previous_time);
        RCLCPP_INFO(get_logger(), "已写入 %s 变化前一条完整消息", source);
      }
      write_initial_states();
    }
  }

  void open_segment(const std::string & trigger_source)
  {
    const auto local_date = current_local_date();
    active_date_ = local_date.key;
    active_bag_path_ = next_bag_path(local_date).string();
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = active_bag_path_;
    storage_options.storage_id = "sqlite3";
    storage_options.max_cache_size = 10 * 1024 * 1024;

    rosbag2_compression::CompressionOptions compression_options;
    compression_options.compression_format = compression_format_;
    compression_options.compression_mode = compression_mode_name_ == "file" ?
      rosbag2_compression::CompressionMode::FILE :
      rosbag2_compression::CompressionMode::MESSAGE;
    compression_options.compression_queue_size =
      static_cast<uint64_t>(compression_queue_size_);
    compression_options.compression_threads = static_cast<uint64_t>(compression_threads_);

    auto compression_writer =
      std::make_unique<rosbag2_compression::SequentialCompressionWriter>(compression_options);
    writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(compression_writer));
    writer_->open(storage_options);

    create_topics();
    segment_started_at_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(), "检测到 %s 业务字段变化，开始事件段: %s",
      trigger_source.c_str(), active_bag_path_.c_str());
  }

  void create_topics()
  {
    rosbag2_storage::TopicMetadata plc_metadata;
    plc_metadata.name = plc_topic_;
    plc_metadata.type = "sensor_device_interfaces/msg/PlcMsg";
    plc_metadata.serialization_format = "cdr";
    writer_->create_topic(plc_metadata);

    rosbag2_storage::TopicMetadata exec_status_metadata;
    exec_status_metadata.name = exec_status_topic_;
    exec_status_metadata.type = "rtg_algorithm_interfaces/msg/ExecStatusMsg";
    exec_status_metadata.serialization_format = "cdr";
    writer_->create_topic(exec_status_metadata);
  }

  void write_initial_states()
  {
    if (latest_plc_ && latest_exec_status_ && latest_plc_time_ <= latest_exec_status_time_) {
      writer_->write(*latest_plc_, plc_topic_, latest_plc_time_);
      writer_->write(*latest_exec_status_, exec_status_topic_, latest_exec_status_time_);
    } else if (latest_plc_ && latest_exec_status_) {
      writer_->write(*latest_exec_status_, exec_status_topic_, latest_exec_status_time_);
      writer_->write(*latest_plc_, plc_topic_, latest_plc_time_);
    } else if (latest_plc_) {
      writer_->write(*latest_plc_, plc_topic_, latest_plc_time_);
    } else if (latest_exec_status_) {
      writer_->write(*latest_exec_status_, exec_status_topic_, latest_exec_status_time_);
    }
  }

  void check_segment()
  {
    if (!writer_) {
      return;
    }

    if (roll_segment_if_date_changed()) {
      return;
    }

    const auto steady_now = std::chrono::steady_clock::now();
    const double quiet_seconds =
      std::chrono::duration<double>(steady_now - last_change_at_).count();
    if (quiet_seconds >= quiet_period_seconds_) {
      close_segment("已接收 Topic 的业务字段均已稳定");
      return;
    }

    const double segment_seconds =
      std::chrono::duration<double>(steady_now - segment_started_at_).count();
    if (segment_seconds >= max_segment_seconds_) {
      close_segment("达到单段最长时间，连续事件切片");
      open_segment("连续事件切片");
      write_initial_states();
    }
  }

  bool roll_segment_if_date_changed()
  {
    if (!writer_ || current_local_date().key == active_date_) {
      return false;
    }

    close_segment("日期已变化，跨日切片");
    open_segment("跨日连续事件切片");
    write_initial_states();
    return true;
  }

  void close_segment(const std::string & reason)
  {
    if (!writer_) {
      return;
    }
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - segment_started_at_).count();
    writer_.reset();
    RCLCPP_INFO(
      get_logger(), "事件段已保存并完成压缩，原因=%s，时长=%.1f 秒，目录=%s",
      reason.c_str(), elapsed, active_bag_path_.c_str());
    active_bag_path_.clear();
    active_date_.clear();
  }

  struct LocalDate
  {
    std::string key;
    std::tm time{};
  };

  static LocalDate current_local_date()
  {
    const std::time_t now_time = std::time(nullptr);
    LocalDate result;
    localtime_r(&now_time, &result.time);

    std::ostringstream key;
    key << std::put_time(&result.time, "%Y_%m_%d");
    result.key = key.str();
    return result;
  }

  fs::path next_bag_path(const LocalDate & local_date)
  {
    const auto now_time_point = std::chrono::system_clock::now();
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
      now_time_point.time_since_epoch()).count() % 1000;
    const fs::path daily_directory = fs::path(output_base_directory_) / local_date.key;
    fs::create_directories(daily_directory);

    std::ostringstream name;
    name << bag_name_prefix_ << '_' << std::put_time(&local_date.time, "%Y_%m_%d_%H_%M_%S")
         << '_' << std::setfill('0') << std::setw(3) << milliseconds;
    fs::path candidate = daily_directory / name.str();
    for (int suffix = 1; fs::exists(candidate); ++suffix) {
      candidate = daily_directory / (name.str() + '_' + std::to_string(suffix));
    }
    return candidate;
  }

  std::string output_base_directory_;
  std::string bag_name_prefix_;
  std::string plc_topic_;
  std::string exec_status_topic_;
  double quiet_period_seconds_{5.0};
  double max_segment_seconds_{3600.0};
  double plc_float_epsilon_{0.001};
  double exec_float_epsilon_{0.001};
  std::string compression_format_{"zstd"};
  std::string compression_mode_name_{"file"};
  int compression_queue_size_{1};
  int compression_threads_{0};

  rclcpp::Subscription<PlcMsg>::SharedPtr plc_sub_;
  rclcpp::Subscription<ExecStatusMsg>::SharedPtr exec_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;

  PlcMsg::SharedPtr latest_plc_;
  ExecStatusMsg::SharedPtr latest_exec_status_;
  rclcpp::Time latest_plc_time_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time latest_exec_status_time_{0, 0, RCL_SYSTEM_TIME};
  std::vector<uint8_t> plc_signature_;
  std::vector<uint8_t> exec_status_signature_;
  bool have_plc_{false};
  bool have_exec_status_{false};
  std::string active_bag_path_;
  std::string active_date_;
  SteadyTime last_change_at_{};
  SteadyTime segment_started_at_{};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<ChangeEventBagRecorder>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("change_event_bag_recorder"), "%s", exception.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
