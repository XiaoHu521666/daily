#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <rtg_algorithm_interfaces/msg/exec_status_msg.hpp>
#include <rtg_algorithm_interfaces/msg/truck_detect_msg.hpp>
#include <sensor_device_interfaces/msg/plc_msg.hpp>

#include <chrono>
#include <cctype>
#include <ctime>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

class AutoBagRecorder : public rclcpp::Node
{
public:
  using Executor = rclcpp::executors::MultiThreadedExecutor;

  AutoBagRecorder()
  : Node("auto_bag_recorder")
  {
    output_base_directory_ = declare_parameter<std::string>("output_base_directory", "./ros_bag");
    bag_name_prefix_ = declare_parameter<std::string>("bag_name_prefix", "rtg_lidar_14_17");
    plc_topic_ = declare_parameter<std::string>("plc_topic", "/rtg/plc_topic_35");
    exec_status_topic_ =
      declare_parameter<std::string>("exec_status_topic", "/rtg/task_exec_status");
    truck_detect_topic_ =
      declare_parameter<std::string>("truck_detect_topic", "/algl/TruckDetectMsg");
    recorded_topics_ = declare_parameter<std::vector<std::string>>(
      "recorded_topics",
      {
        "/livox/lidar_3GGDJ9U00101051",
        "/livox/lidar_3GGDJ9T00100371",
        "/livox/lidar_3GGDJ9T00100931",
        "/livox/lidar_3GGDJ9S00100631",
        "/rtg/plc_topic_35"
      });

    require_pick_task_ = declare_parameter<bool>("require_pick_task", true);
    pick_action_type_ = declare_parameter<int>("pick_action_type", 2);
    require_truck_detect_ = declare_parameter<bool>("require_truck_detect", true);
    required_truck_type_ = declare_parameter<std::string>("required_truck_type", "OTK");
    max_start_trolley_position_ =
      declare_parameter<double>("max_start_trolley_position", 1.5);
    stop_on_spreader_unlock_ = declare_parameter<bool>("stop_on_spreader_unlock", true);
    stop_on_hoist_height_ = declare_parameter<bool>("stop_on_hoist_height", true);
    stop_hoist_height_ = declare_parameter<double>("stop_hoist_height", 10.0);
    max_record_seconds_ = declare_parameter<double>("max_record_seconds", 1800.0);
    start_on_initial_locked_ = declare_parameter<bool>("start_on_initial_locked", false);
    topic_check_timeout_seconds_ =
      declare_parameter<double>("topic_check_timeout_seconds", 10.0);

    validate_parameters();
    fs::create_directories(output_base_directory_);

    exec_status_sub_ = create_subscription<rtg_algorithm_interfaces::msg::ExecStatusMsg>(
      exec_status_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AutoBagRecorder::on_exec_status, this, std::placeholders::_1));
    truck_detect_sub_ = create_subscription<rtg_algorithm_interfaces::msg::TruckDetectMsg>(
      truck_detect_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AutoBagRecorder::on_truck_detect, this, std::placeholders::_1));
    plc_sub_ = create_subscription<sensor_device_interfaces::msg::PlcMsg>(
      plc_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AutoBagRecorder::on_plc, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "本次录制 Topic 配置（共 %zu 个）:", recorded_topics_.size());
    for (const auto & topic : recorded_topics_) {
      RCLCPP_INFO(
        get_logger(), "  %s [%s]", topic.c_str(), topic == plc_topic_ ? "PLC" : "雷达");
    }
    validate_lidar_publishers();
    process_timer_ = create_wall_timer(250ms, std::bind(&AutoBagRecorder::check_recorder, this));

    RCLCPP_INFO(get_logger(), "无人值守录包节点已启动，等待抓箱闭锁信号");
    RCLCPP_INFO(get_logger(), "输出根目录: %s", fs::absolute(output_base_directory_).c_str());
    RCLCPP_INFO(
      get_logger(), "开始条件: 小车位置 <= %.2f m", max_start_trolley_position_);
    RCLCPP_INFO(
      get_logger(), "停止条件: 开锁=%s, 起升高度=%s (阈值 %.2f m), 最长录制 %.1f s",
      stop_on_spreader_unlock_ ? "启用" : "禁用",
      stop_on_hoist_height_ ? "启用" : "禁用", stop_hoist_height_, max_record_seconds_);
  }

  ~AutoBagRecorder() override
  {
    shutdown_recorder();
  }

  void set_executor(const std::shared_ptr<Executor> & executor)
  {
    executor_ = executor;
  }

  void start_recorder_preparation()
  {
    prepare_recorder_if_ready();
  }

  void shutdown_recorder()
  {
    close_recorder(state_ == RecorderState::IDLE);
  }

private:
  enum class RecorderState {IDLE, RECORDING};

  void validate_parameters() const
  {
    if (recorded_topics_.empty()) {
      throw std::runtime_error("recorded_topics 不能为空");
    }
    if (!stop_on_spreader_unlock_ && !stop_on_hoist_height_ && max_record_seconds_ <= 0.0) {
      throw std::runtime_error("至少需要启用一种停止条件");
    }
    if (stop_on_hoist_height_ && stop_hoist_height_ <= 0.0) {
      throw std::runtime_error("stop_hoist_height 必须大于 0");
    }
    if (topic_check_timeout_seconds_ <= 0.0) {
      throw std::runtime_error("topic_check_timeout_seconds 必须大于 0");
    }
  }

  void validate_lidar_publishers() const
  {
    std::vector<std::string> lidar_topics;
    for (const auto & topic : recorded_topics_) {
      if (topic != plc_topic_) {
        lidar_topics.push_back(topic);
      }
    }
    if (lidar_topics.empty()) {
      throw std::runtime_error("recorded_topics 中没有需要检查的雷达 Topic");
    }

    RCLCPP_INFO(
      get_logger(), "正在检查 %zu 个雷达 Topic，最长等待 %.1f s",
      lidar_topics.size(), topic_check_timeout_seconds_);

    std::vector<std::string> missing_topics;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(topic_check_timeout_seconds_);
    do {
      missing_topics.clear();
      for (const auto & topic : lidar_topics) {
        if (count_publishers(topic) == 0) {
          missing_topics.push_back(topic);
        }
      }
      if (missing_topics.empty()) {
        RCLCPP_INFO(get_logger(), "雷达 Topic 检查通过，所有雷达均存在发布者");
        return;
      }
      std::this_thread::sleep_for(200ms);
    } while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline);

    std::ostringstream error;
    error << "雷达 Topic 检查失败，以下 Topic 没有发布者:";
    for (const auto & topic : missing_topics) {
      error << ' ' << topic;
    }
    throw std::runtime_error(error.str());
  }

  void on_exec_status(const rtg_algorithm_interfaces::msg::ExecStatusMsg::SharedPtr msg)
  {
    const bool task_changed = current_task_id_ != msg->task_id;
    current_action_type_ = msg->action_type;
    current_task_id_ = msg->task_id;
    have_exec_status_ = true;

    if (task_changed && state_ == RecorderState::IDLE && recorder_) {
      close_recorder(true);
    }
    prepare_recorder_if_ready();
    try_start_pending_recording();
  }

  void on_truck_detect(const rtg_algorithm_interfaces::msg::TruckDetectMsg::SharedPtr msg)
  {
    region_has_truck_ = msg->region_has_truck;
    detected_truck_type_ = msg->truck_type.result;
    have_truck_detect_ = true;
    prepare_recorder_if_ready();
    try_start_pending_recording();
  }

  void on_plc(const sensor_device_interfaces::msg::PlcMsg::SharedPtr msg)
  {
    const auto received_time = get_clock()->now();
    current_trolley_position_ = msg->trolley_pos;
    if (!have_plc_state_) {
      have_plc_state_ = true;
      previous_spreader_lock_ = msg->spreader_lock;
      previous_spreader_unlock_ = msg->spreader_unlock;
      if (start_on_initial_locked_ && msg->spreader_lock) {
        cache_lock_trigger(msg, received_time);
        try_start_recording("节点启动时吊具已闭锁");
      }
      return;
    }

    const bool was_recording = state_ == RecorderState::RECORDING;
    const bool lock_rising = msg->spreader_lock && !previous_spreader_lock_;
    const bool unlock_rising = msg->spreader_unlock && !previous_spreader_unlock_;
    previous_spreader_lock_ = msg->spreader_lock;
    previous_spreader_unlock_ = msg->spreader_unlock;
    if (!msg->spreader_lock) {
      clear_pending_lock_trigger();
    }

    if (state_ == RecorderState::IDLE && lock_rising) {
      cache_lock_trigger(msg, received_time);
      try_start_recording("检测到吊具闭锁上升沿，已抓到箱子");
    }

    if (state_ == RecorderState::IDLE && !lock_rising && pending_lock_trigger_ &&
      msg->spreader_lock)
    {
      pending_lock_message_ = std::make_shared<sensor_device_interfaces::msg::PlcMsg>(*msg);
      pending_lock_received_time_ = received_time;
      try_start_pending_recording();
    }

    if (was_recording) {
      writer_->write(*msg, plc_topic_, received_time);
    }

    if (state_ != RecorderState::RECORDING) {
      return;
    }

    if (stop_on_spreader_unlock_ && unlock_rising) {
      stop_recording("检测到吊具开锁上升沿，箱子已放下");
      return;
    }

    if (stop_on_hoist_height_ && msg->hoist_height >= stop_hoist_height_) {
      std::ostringstream reason;
      reason << "起升高度 " << std::fixed << std::setprecision(2) << msg->hoist_height
             << " m 已达到阈值 " << stop_hoist_height_ << " m";
      stop_recording(reason.str());
    }
  }

  bool pick_condition_ready() const
  {
    return !require_pick_task_ ||
           (have_exec_status_ && current_action_type_ == pick_action_type_);
  }

  bool truck_condition_ready() const
  {
    return !require_truck_detect_ ||
           (have_truck_detect_ && region_has_truck_ &&
           detected_truck_type_ == required_truck_type_);
  }

  bool trolley_condition_ready() const
  {
    return have_plc_state_ && current_trolley_position_ <= max_start_trolley_position_;
  }

  void prepare_recorder_if_ready()
  {
    if (recorder_ || state_ != RecorderState::IDLE ||
      !pick_condition_ready() || !truck_condition_ready())
    {
      return;
    }

    try {
      prepare_recorder();
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "录包器预热失败: %s", exception.what());
    }
  }

  void prepare_recorder()
  {
    auto executor = executor_.lock();
    if (!executor) {
      throw std::runtime_error("executor 尚未设置");
    }

    active_bag_path_ = next_bag_path().string();
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = active_bag_path_;
    storage_options.storage_id = "sqlite3";
    storage_options.max_cache_size = 100 * 1024 * 1024;

    rosbag2_transport::RecordOptions record_options;
    for (const auto & topic : recorded_topics_) {
      if (topic != plc_topic_) {
        record_options.topics.push_back(topic);
      }
    }
    record_options.rmw_serialization_format = rmw_get_serialization_format();
    record_options.topic_polling_interval = 100ms;
    record_options.start_paused = true;

    auto writer = std::make_shared<rosbag2_cpp::Writer>();
    rclcpp::NodeOptions recorder_node_options;
    recorder_node_options.use_global_arguments(false);
    auto recorder = std::make_shared<rosbag2_transport::Recorder>(
      writer, storage_options, record_options, next_recorder_node_name(), recorder_node_options);

    executor->add_node(recorder);
    try {
      recorder->record();
    } catch (...) {
      executor->remove_node(recorder);
      active_bag_path_.clear();
      throw;
    }

    writer_ = std::move(writer);
    recorder_ = std::move(recorder);
    RCLCPP_INFO(
      get_logger(), "下一次录包已预热，当前时间=%s，输出目录=%s",
      local_time_string().c_str(), active_bag_path_.c_str());
  }

  void try_start_recording(const std::string & reason)
  {
    if (!pick_condition_ready()) {
      RCLCPP_WARN(
        get_logger(), "暂存闭锁信号：当前动作类型=%d，要求抓箱动作类型=%d%s",
        current_action_type_, pick_action_type_, have_exec_status_ ? "" : "（尚未收到任务状态）");
      return;
    }
    if (!truck_condition_ready()) {
      RCLCPP_WARN(
        get_logger(),
        "暂存闭锁信号：集卡检测条件不满足，region_has_truck=%s, type=%s",
        region_has_truck_ ? "true" : "false", detected_truck_type_.c_str());
      return;
    }
    if (!trolley_condition_ready()) {
      RCLCPP_WARN(
        get_logger(), "暂存闭锁信号：当前小车位置=%.2f m，要求 <= %.2f m",
        current_trolley_position_, max_start_trolley_position_);
      return;
    }

    if (!recorder_) {
      prepare_recorder();
    }
    recorder_->resume();
    state_ = RecorderState::RECORDING;
    recording_started_at_ = std::chrono::steady_clock::now();
    write_pending_lock_message();

    RCLCPP_INFO(get_logger(), "开始录包，当前时间=%s: %s", local_time_string().c_str(), reason.c_str());
    RCLCPP_INFO(get_logger(), "输出目录=%s", active_bag_path_.c_str());
  }

  void try_start_pending_recording()
  {
    if (!pending_lock_trigger_ || !previous_spreader_lock_ || recorder_ == nullptr ||
      state_ != RecorderState::IDLE || !pick_condition_ready() || !truck_condition_ready() ||
      !trolley_condition_ready())
    {
      return;
    }

    try_start_recording("此前已检测到吊具闭锁，任务、集卡和小车位置条件现已满足");
  }

  void cache_lock_trigger(
    const sensor_device_interfaces::msg::PlcMsg::SharedPtr & msg,
    const rclcpp::Time & received_time)
  {
    pending_lock_trigger_ = true;
    pending_lock_message_ = std::make_shared<sensor_device_interfaces::msg::PlcMsg>(*msg);
    pending_lock_received_time_ = received_time;
  }

  void write_pending_lock_message()
  {
    if (!pending_lock_message_) {
      return;
    }

    writer_->write(*pending_lock_message_, plc_topic_, pending_lock_received_time_);
    RCLCPP_INFO(get_logger(), "已写入触发录包的闭锁 PLC 消息");
    clear_pending_lock_trigger();
  }

  void clear_pending_lock_trigger()
  {
    pending_lock_trigger_ = false;
    pending_lock_message_.reset();
  }

  void stop_recording(const std::string & reason)
  {
    if (state_ != RecorderState::RECORDING || !recorder_) {
      return;
    }

    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - recording_started_at_).count();
    const std::string finished_bag_path = active_bag_path_;
    close_recorder(false);

    RCLCPP_INFO(
      get_logger(), "录包已结束，当前时间=%s，原因=%s，时长=%.1f s，目录=%s",
      local_time_string().c_str(), reason.c_str(), elapsed, finished_bag_path.c_str());
    prepare_recorder_if_ready();
  }

  void close_recorder(bool discard_unused_bag)
  {
    if (!recorder_) {
      return;
    }

    recorder_->pause();
    if (auto executor = executor_.lock()) {
      executor->remove_node(recorder_);
    }
    recorder_->stop();
    recorder_.reset();
    writer_.reset();
    state_ = RecorderState::IDLE;

    if (discard_unused_bag && !active_bag_path_.empty()) {
      std::error_code error;
      fs::remove_all(active_bag_path_, error);
      if (error) {
        RCLCPP_WARN(
          get_logger(), "无法清理未使用的预热 bag %s: %s",
          active_bag_path_.c_str(), error.message().c_str());
      }
    }
    active_bag_path_.clear();
  }

  void check_recorder()
  {
    if (state_ != RecorderState::RECORDING || max_record_seconds_ <= 0.0) {
      return;
    }

    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - recording_started_at_).count();
    if (elapsed >= max_record_seconds_) {
      stop_recording("达到最长录制时间保护阈值");
    }
  }

  fs::path next_bag_path() const
  {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
    localtime_r(&now_time, &local_time);

    std::ostringstream name;
    name << bag_name_prefix_ << '_' << std::put_time(&local_time, "%Y_%m_%d_%H_%M_%S");
    const std::string task_id = sanitize_name(current_task_id_);
    if (!task_id.empty()) {
      name << "_task_" << task_id;
    }

    fs::path candidate = fs::path(output_base_directory_) / name.str();
    for (int suffix = 1; fs::exists(candidate); ++suffix) {
      candidate = fs::path(output_base_directory_) / (name.str() + '_' + std::to_string(suffix));
    }
    return candidate;
  }

  static std::string local_time_string()
  {
    const auto now = std::chrono::system_clock::now();
    const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
    localtime_r(&seconds, &local_time);
    const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
      now.time_since_epoch()).count() % 1000000000;

    std::ostringstream result;
    result << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S") << '.'
           << std::setw(9) << std::setfill('0') << nanoseconds;
    return result.str();
  }

  static std::string sanitize_name(const std::string & value)
  {
    std::string result;
    result.reserve(value.size());
    for (const unsigned char character : value) {
      if (std::isalnum(character) || character == '-' || character == '_') {
        result.push_back(static_cast<char>(character));
      }
    }
    return result.substr(0, 64);
  }

  std::string next_recorder_node_name()
  {
    return "auto_rosbag2_recorder_" + std::to_string(++recorder_sequence_);
  }

  std::string output_base_directory_;
  std::string bag_name_prefix_;
  std::string plc_topic_;
  std::string exec_status_topic_;
  std::string truck_detect_topic_;
  std::vector<std::string> recorded_topics_;
  bool require_pick_task_{true};
  int pick_action_type_{2};
  bool require_truck_detect_{true};
  std::string required_truck_type_;
  double max_start_trolley_position_{1.5};
  bool stop_on_spreader_unlock_{true};
  bool stop_on_hoist_height_{true};
  double stop_hoist_height_{10.0};
  double max_record_seconds_{1800.0};
  bool start_on_initial_locked_{false};
  double topic_check_timeout_seconds_{10.0};

  rclcpp::Subscription<rtg_algorithm_interfaces::msg::ExecStatusMsg>::SharedPtr exec_status_sub_;
  rclcpp::Subscription<rtg_algorithm_interfaces::msg::TruckDetectMsg>::SharedPtr truck_detect_sub_;
  rclcpp::Subscription<sensor_device_interfaces::msg::PlcMsg>::SharedPtr plc_sub_;
  rclcpp::TimerBase::SharedPtr process_timer_;

  std::weak_ptr<Executor> executor_;
  std::shared_ptr<rosbag2_cpp::Writer> writer_;
  std::shared_ptr<rosbag2_transport::Recorder> recorder_;
  RecorderState state_{RecorderState::IDLE};
  std::size_t recorder_sequence_{0};
  bool have_exec_status_{false};
  bool have_truck_detect_{false};
  bool region_has_truck_{false};
  bool have_plc_state_{false};
  double current_trolley_position_{0.0};
  bool previous_spreader_lock_{false};
  bool previous_spreader_unlock_{false};
  // 闭锁先于任务或集卡条件到达时，保留该触发直到开锁。
  bool pending_lock_trigger_{false};
  sensor_device_interfaces::msg::PlcMsg::SharedPtr pending_lock_message_;
  rclcpp::Time pending_lock_received_time_{0, 0, RCL_SYSTEM_TIME};
  int current_action_type_{0};
  std::string detected_truck_type_;
  std::string current_task_id_;
  std::string active_bag_path_;
  std::chrono::steady_clock::time_point recording_started_at_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<AutoBagRecorder>();
    auto executor = std::make_shared<AutoBagRecorder::Executor>();
    executor->add_node(node);
    node->set_executor(executor);
    node->start_recorder_preparation();
    executor->spin();
    node->shutdown_recorder();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("auto_bag_recorder"), "%s", exception.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
