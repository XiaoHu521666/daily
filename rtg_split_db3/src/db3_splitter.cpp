#include "rtg_split_db3/db3_splitter.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <regex>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rtg_algorithm_interfaces/msg/gantry_pos_msg.hpp>
#include <sensor_device_interfaces/msg/plc_msg.hpp>

namespace fs = std::filesystem;

namespace rtg_split_db3
{
namespace
{

constexpr char kPlcTopic[] = "/rtg/plc_topic_35";
constexpr char kGantryTopic[] = "/algl/GantryPosMsg";

rosbag2_storage::StorageOptions storage_options(const fs::path & bag)
{
  rosbag2_storage::StorageOptions options;
  options.uri = bag.string();
  options.storage_id = "sqlite3";
  return options;
}

rosbag2_cpp::ConverterOptions converter_options()
{
  return {"cdr", "cdr"};
}

fs::path create_temporary_directory()
{
  std::string pattern = (fs::temp_directory_path() / "rtg_split_db3_XXXXXX").string();
  std::vector<char> buffer(pattern.begin(), pattern.end());
  buffer.push_back('\0');
  const char * directory = mkdtemp(buffer.data());
  if (directory == nullptr) {
    throw std::runtime_error("无法创建本地临时目录");
  }
  return directory;
}

std::string normalize_bay(const std::string & value)
{
  const bool all_digits = std::all_of(
    value.begin(), value.end(), [](unsigned char character) {return std::isdigit(character);});
  if (!value.empty() && all_digits) {
    return std::to_string(std::stoi(value));
  }
  return value;
}

std::string infer_bridge(const fs::path & source_bag)
{
  std::smatch match;
  const std::regex pattern(R"((^|[^0-9])(6[0-9]{2})([^0-9]|$))");
  const std::string name = source_bag.filename().string();
  return std::regex_search(name, match, pattern) ? match[2].str() : "RTG";
}

std::string infer_date(const fs::path & source_bag)
{
  std::smatch match;
  const std::regex pattern(R"((20[0-9]{2})[_-]([0-9]{2})[_-]([0-9]{2}))");
  const std::string name = source_bag.filename().string();
  if (std::regex_search(name, match, pattern)) {
    return match[1].str() + "_" + match[2].str() + "_" + match[3].str();
  }
  return "unknown_date";
}

double seconds(std::int64_t nanoseconds)
{
  return static_cast<double>(nanoseconds) / 1e9;
}

}  // namespace

Db3Splitter::Db3Splitter(Options options)
: options_(std::move(options))
{
}

void Db3Splitter::run()
{
  if (!fs::is_directory(options_.source_bag) ||
    !fs::is_regular_file(options_.source_bag / "metadata.yaml"))
  {
    throw std::runtime_error("源路径必须是包含 metadata.yaml 的 rosbag2 目录");
  }
  if (options_.output_dir.empty()) {
    options_.output_dir = options_.source_bag / "split_by_gantry_stationary";
  }

  scan_control_topics();
  detect_segments();
  assign_output_names();

  if (segments_.empty()) {
    throw std::runtime_error("没有找到满足条件的大车静止区间");
  }

  std::cout << "找到 " << segments_.size() << " 个静止区间:" << std::endl;
  for (const auto & segment : segments_) {
    std::cout << "  " << segment.output_name
              << "  堆场=" << segment.block << "  贝位=" << segment.bay_no
              << "  时长=" << std::fixed << std::setprecision(2)
              << seconds(segment.end_ns - segment.start_ns) << "秒"
              << "  小车位置=" << segment.trolley_min << "~" << segment.trolley_max << "m"
              << std::endl;
  }

  if (options_.dry_run) {
    std::cout << "dry-run 完成，未写入文件。" << std::endl;
    return;
  }
  write_segments();
}

// 第一部分：只读取 PLC 和大车定位消息。
void Db3Splitter::scan_control_topics()
{
  rosbag2_cpp::readers::SequentialReader reader;
  reader.open(storage_options(options_.source_bag), converter_options());
  topics_ = reader.get_all_topics_and_types();

  std::unordered_map<std::string, std::string> topic_types;
  for (const auto & topic : topics_) {
    topic_types[topic.name] = topic.type;
  }
  if (topic_types[kPlcTopic] != "sensor_device_interfaces/msg/PlcMsg" ||
    topic_types[kGantryTopic] != "rtg_algorithm_interfaces/msg/GantryPosMsg")
  {
    throw std::runtime_error("DB3 缺少 PLC 或 GantryPosMsg，或消息类型不匹配");
  }

  rosbag2_storage::StorageFilter filter;
  filter.topics = {kPlcTopic, kGantryTopic};
  reader.set_filter(filter);

  rclcpp::Serialization<sensor_device_interfaces::msg::PlcMsg> plc_serializer;
  rclcpp::Serialization<rtg_algorithm_interfaces::msg::GantryPosMsg> gantry_serializer;

  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    rclcpp::SerializedMessage serialized(*bag_message->serialized_data);

    if (bag_message->topic_name == kPlcTopic) {
      sensor_device_interfaces::msg::PlcMsg message;
      plc_serializer.deserialize_message(&serialized, &message);
      const double max_velocity = std::max({
        std::abs(message.gantry_velocity_1), std::abs(message.gantry_velocity_2),
        std::abs(message.gantry_velocity_3), std::abs(message.gantry_velocity_4)});
      plc_samples_.push_back({bag_message->time_stamp, max_velocity, message.trolley_pos});
    } else {
      rtg_algorithm_interfaces::msg::GantryPosMsg message;
      gantry_serializer.deserialize_message(&serialized, &message);
      gantry_samples_.push_back(
        {bag_message->time_stamp, message.block, normalize_bay(message.bay_no)});
    }
  }
}

// 第二部分：按大车速度识别持续静止区间，再用占多数的 bay_no 标记贝位。
void Db3Splitter::detect_segments()
{
  if (plc_samples_.empty() || gantry_samples_.empty()) {
    throw std::runtime_error("PLC 或 GantryPosMsg 没有有效消息");
  }

  std::vector<std::pair<std::size_t, std::size_t>> runs;
  std::size_t start = 0;
  bool in_run = false;
  for (std::size_t i = 0; i < plc_samples_.size(); ++i) {
    const bool stationary = plc_samples_[i].max_gantry_velocity == 0.0;
    if (stationary && !in_run) {
      start = i;
      in_run = true;
    } else if (!stationary && in_run) {
      runs.emplace_back(start, i - 1);
      in_run = false;
    }
  }
  if (in_run) {
    runs.emplace_back(start, plc_samples_.size() - 1);
  }

  const auto trim_ns = static_cast<std::int64_t>(options_.edge_trim_seconds * 1e9);
  const auto min_ns = static_cast<std::int64_t>(options_.min_stationary_seconds * 1e9);
  for (const auto & run : runs) {
    const std::int64_t raw_start_ns = plc_samples_[run.first].timestamp_ns;
    const std::int64_t raw_end_ns = plc_samples_[run.second].timestamp_ns;
    if (raw_end_ns - raw_start_ns < min_ns) {
      continue;
    }
    const std::int64_t start_ns = raw_start_ns + trim_ns;
    const std::int64_t end_ns = raw_end_ns - trim_ns;
    if (end_ns <= start_ns) {
      continue;
    }

    std::map<std::pair<std::string, std::string>, std::size_t> labels;
    std::size_t label_total = 0;
    auto gantry = std::lower_bound(
      gantry_samples_.begin(), gantry_samples_.end(), start_ns,
      [](const GantrySample & sample, std::int64_t timestamp) {
        return sample.timestamp_ns < timestamp;
      });
    for (; gantry != gantry_samples_.end() && gantry->timestamp_ns <= end_ns; ++gantry) {
      if (!gantry->bay_no.empty()) {
        ++labels[{gantry->block, gantry->bay_no}];
        ++label_total;
      }
    }
    if (labels.empty()) {
      continue;
    }

    const auto label = std::max_element(
      labels.begin(), labels.end(),
      [](const auto & left, const auto & right) {return left.second < right.second;});
    if (label->second * 2 < label_total) {
      continue;
    }

    double trolley_min = std::numeric_limits<double>::max();
    double trolley_max = std::numeric_limits<double>::lowest();
    for (std::size_t i = run.first; i <= run.second; ++i) {
      trolley_min = std::min(trolley_min, plc_samples_[i].trolley_pos);
      trolley_max = std::max(trolley_max, plc_samples_[i].trolley_pos);
    }
    segments_.push_back(
      {start_ns, end_ns, label->first.first, label->first.second, "", trolley_min, trolley_max});
  }
}

void Db3Splitter::assign_output_names()
{
  const std::string prefix = infer_bridge(options_.source_bag);
  const std::string date = infer_date(options_.source_bag);
  std::map<std::string, std::size_t> totals;
  std::map<std::string, std::size_t> visits;
  for (const auto & segment : segments_) {
    ++totals[segment.bay_no];
  }
  for (auto & segment : segments_) {
    segment.output_name = prefix + "_" + segment.bay_no + "_" + date;
    if (totals[segment.bay_no] > 1) {
      segment.output_name += "_" + std::to_string(++visits[segment.bay_no]);
    }
  }
}

// 第三部分：小包先在本地临时生成，关闭 SQLite 后再写入最终目录。
void Db3Splitter::write_segments()
{
  fs::create_directories(options_.output_dir);
  const fs::path temporary_root = create_temporary_directory();
  const std::string transfer_suffix = ".incomplete_" + temporary_root.filename().string();
  std::vector<std::size_t> counts(segments_.size(), 0);
  std::vector<fs::path> local_bags;
  std::vector<fs::path> final_bags;
  std::vector<fs::path> transfer_bags;

  for (const auto & segment : segments_) {
    const fs::path final_bag = options_.output_dir / segment.output_name;
    if (fs::exists(final_bag)) {
      fs::remove_all(temporary_root);
      throw std::runtime_error("输出目录已存在: " + final_bag.string());
    }
    local_bags.push_back(temporary_root / segment.output_name);
    final_bags.push_back(final_bag);
    transfer_bags.push_back(options_.output_dir / (segment.output_name + transfer_suffix));
  }

  try {
    std::cout << "正在本地临时生成小包: " << temporary_root << std::endl;
    std::vector<std::unique_ptr<rosbag2_cpp::writers::SequentialWriter>> writers;
    for (const auto & local_bag : local_bags) {
      auto writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
      writer->open(storage_options(local_bag), converter_options());
      for (const auto & topic : topics_) {
        writer->create_topic(topic);
      }
      writers.push_back(std::move(writer));
    }

    rosbag2_cpp::readers::SequentialReader reader;
    reader.open(storage_options(options_.source_bag), converter_options());
    std::size_t segment_index = 0;
    while (reader.has_next() && segment_index < segments_.size()) {
      auto message = reader.read_next();
      while (segment_index < segments_.size() &&
        message->time_stamp > segments_[segment_index].end_ns)
      {
        ++segment_index;
      }
      if (segment_index < segments_.size() &&
        message->time_stamp >= segments_[segment_index].start_ns)
      {
        writers[segment_index]->write(message);
        ++counts[segment_index];
      }
    }
    writers.clear();

    for (std::size_t i = 0; i < segments_.size(); ++i) {
      std::cout << "正在写入网络盘: " << final_bags[i] << std::endl;
      fs::copy(local_bags[i], transfer_bags[i], fs::copy_options::recursive);
      fs::rename(transfer_bags[i], final_bags[i]);
    }
  } catch (...) {
    for (const auto & transfer_bag : transfer_bags) {
      std::error_code error;
      fs::remove_all(transfer_bag, error);
    }
    std::error_code error;
    fs::remove_all(temporary_root, error);
    throw;
  }

  fs::remove_all(temporary_root);

  for (std::size_t i = 0; i < segments_.size(); ++i) {
    std::cout << "完成: " << final_bags[i]
              << "  消息数=" << counts[i] << std::endl;
  }
}

void print_usage(const char * program)
{
  std::cout
    << "用法: " << program << " <bag目录> [选项]\n"
    << "  --output-dir <目录>            输出根目录\n"
    << "  --min-stationary-seconds <秒>  最短静止时间，默认 10\n"
    << "  --edge-trim-seconds <秒>       静止段首尾裁剪，默认 2\n"
    << "  --dry-run                      只识别，不写小包\n";
}

Options parse_options(int argc, char ** argv)
{
  if (argc >= 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
    print_usage(argv[0]);
    std::exit(0);
  }
  if (argc < 2) {
    print_usage(argv[0]);
    throw std::runtime_error("缺少 bag 目录");
  }
  Options options;
  options.source_bag = fs::absolute(argv[1]);

  for (int i = 2; i < argc; ++i) {
    const std::string argument = argv[i];
    auto next_value = [&]() -> std::string {
        if (++i >= argc) {
          throw std::runtime_error(argument + " 缺少参数值");
        }
        return argv[i];
      };

    if (argument == "--output-dir") {
      options.output_dir = fs::absolute(next_value());
    } else if (argument == "--min-stationary-seconds") {
      options.min_stationary_seconds = std::stod(next_value());
    } else if (argument == "--edge-trim-seconds") {
      options.edge_trim_seconds = std::stod(next_value());
    } else if (argument == "--dry-run") {
      options.dry_run = true;
    } else if (argument == "--help" || argument == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("未知参数: " + argument);
    }
  }

  if (options.min_stationary_seconds < 0 || options.edge_trim_seconds < 0)
  {
    throw std::runtime_error("时间参数不能为负数");
  }
  return options;
}

}  // namespace rtg_split_db3
