#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rtg_algorithm_interfaces/msg/data_sync_msg.hpp>
#include <rtg_algorithm_interfaces/msg/gantry_pos_msg.hpp>
#include <sensor_device_interfaces/msg/plc_msg.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;
using json = nlohmann::json;

enum class MessageKind { PointCloud, Plc, Imu, Gantry, DataSync };

struct Event
{
  fs::path path;
  std::string key;
  std::string topic;
  std::string type;
  MessageKind kind;
  int64_t timestamp_ns;
  std::vector<fs::path> related_paths;
};

constexpr const char * kPointCloudType = "sensor_msgs/msg/PointCloud2";
constexpr const char * kPlcType = "sensor_device_interfaces/msg/PlcMsg";
constexpr const char * kImuType = "sensor_msgs/msg/Imu";
constexpr const char * kGantryType = "rtg_algorithm_interfaces/msg/GantryPosMsg";
constexpr const char * kDataSyncType = "rtg_algorithm_interfaces/msg/DataSyncMsg";
constexpr const char * kGantryTopic = "/algl/GantryPosMsg";
constexpr const char * kDataSyncTopic = "/rtg/lidar_top_sync";
constexpr const char * kExecStatusTopic = "/rtg/task_exec_status";
constexpr const char * kExecStatusType = "rtg_algorithm_interfaces/msg/ExecStatusMsg";
constexpr const char * kResetTopic = "/algl/ResetScanRes";
constexpr const char * kResetType = "rtg_algorithm_interfaces/msg/ResetScanRes";

int64_t stamp_to_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
}

builtin_interfaces::msg::Time filename_stamp(const fs::path & path)
{
  static const std::regex pattern(R"(_(\d{10})_(\d{3})\.[^.]+$)", std::regex::icase);
  std::smatch match;
  const std::string name = path.filename().string();
  if (!std::regex_search(name, match, pattern)) {
    throw std::runtime_error("文件名不符合 <名称>_秒_毫秒.<扩展名>: " + path.string());
  }

  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(std::stoll(match[1].str()));
  stamp.nanosec = static_cast<uint32_t>(std::stoul(match[2].str())) * 1'000'000U;
  return stamp;
}

json read_json(const fs::path & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("无法读取 JSON: " + path.string());
  }
  return json::parse(input);
}

builtin_interfaces::msg::Time json_stamp(const json & data, const fs::path & path)
{
  try {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = data.at("header").at("stamp").at("sec").get<int32_t>();
    stamp.nanosec = data.at("header").at("stamp").at("nanosec").get<uint32_t>();
    return stamp;
  } catch (const json::exception & error) {
    throw std::runtime_error("JSON 缺少有效 header.stamp: " + path.string() + " (" + error.what() + ")");
  }
}

void fill_header(std_msgs::msg::Header & header, const json & data)
{
  const auto & source = data.at("header");
  header.stamp.sec = source.at("stamp").at("sec").get<int32_t>();
  header.stamp.nanosec = source.at("stamp").at("nanosec").get<uint32_t>();
  header.frame_id = source.value("frame_id", std::string{});
}

template<typename T>
void assign_if_present(const json & data, const char * name, T & target)
{
  if (const auto it = data.find(name); it != data.end()) {
    target = it->get<T>();
  }
}

sensor_device_interfaces::msg::PlcMsg load_plc(const fs::path & path)
{
  const json data = read_json(path);
  sensor_device_interfaces::msg::PlcMsg message;
  fill_header(message.header, data);

#define ASSIGN_PLC_FIELD(name) assign_if_present(data, #name, message.name)
  ASSIGN_PLC_FIELD(plc_id);
  ASSIGN_PLC_FIELD(source_timestamp);
  ASSIGN_PLC_FIELD(plc_connect);
  ASSIGN_PLC_FIELD(no_estop);
  ASSIGN_PLC_FIELD(spreader_lock);
  ASSIGN_PLC_FIELD(spreader_unlock);
  ASSIGN_PLC_FIELD(spreader_20f);
  ASSIGN_PLC_FIELD(spreader_40f);
  ASSIGN_PLC_FIELD(spreader_45f);
  ASSIGN_PLC_FIELD(spreader_landed);
  ASSIGN_PLC_FIELD(spreader_pump_on_state);
  ASSIGN_PLC_FIELD(spreader_home);
  ASSIGN_PLC_FIELD(spreader_weight);
  ASSIGN_PLC_FIELD(ctrl_on_state);
  ASSIGN_PLC_FIELD(ctrl_mode_local);
  ASSIGN_PLC_FIELD(ctrl_mode_remote);
  ASSIGN_PLC_FIELD(ctrl_mode_manual);
  ASSIGN_PLC_FIELD(ctrl_mode_auto);
  ASSIGN_PLC_FIELD(ros_connect_state);
  ASSIGN_PLC_FIELD(gantry_velocity_1);
  ASSIGN_PLC_FIELD(gantry_velocity_2);
  ASSIGN_PLC_FIELD(gantry_velocity_3);
  ASSIGN_PLC_FIELD(gantry_velocity_4);
  ASSIGN_PLC_FIELD(gantry_pos);
  ASSIGN_PLC_FIELD(joystick_gantry_fb);
  ASSIGN_PLC_FIELD(joystick_trolley_fb);
  ASSIGN_PLC_FIELD(joystick_hoist_fb);
  ASSIGN_PLC_FIELD(trolley_pos);
  ASSIGN_PLC_FIELD(trolley_velocity);
  ASSIGN_PLC_FIELD(hoist_height);
  ASSIGN_PLC_FIELD(hoist_velocity);
  ASSIGN_PLC_FIELD(left_crane_state);
  ASSIGN_PLC_FIELD(left_crane_gantry_pos);
  ASSIGN_PLC_FIELD(left_crane_gantry_velocity);
  ASSIGN_PLC_FIELD(right_crane_state);
  ASSIGN_PLC_FIELD(right_crane_gantry_pos);
  ASSIGN_PLC_FIELD(right_crane_gantry_velocity);
  ASSIGN_PLC_FIELD(trolley_anchor_state);
  ASSIGN_PLC_FIELD(spreader_skew_right);
  ASSIGN_PLC_FIELD(spreader_skew_left);
  ASSIGN_PLC_FIELD(spreader20ft_cmd);
  ASSIGN_PLC_FIELD(spreader40ft_cmd);
  ASSIGN_PLC_FIELD(spreader45ft_cmd);
  ASSIGN_PLC_FIELD(spreader_rope_loose);
  ASSIGN_PLC_FIELD(flipper_left_down);
  ASSIGN_PLC_FIELD(flipper_right_down);
  ASSIGN_PLC_FIELD(over_load_fb);
  ASSIGN_PLC_FIELD(wheel_at0);
  ASSIGN_PLC_FIELD(wheel_at90);
  ASSIGN_PLC_FIELD(wheel_at16);
  ASSIGN_PLC_FIELD(wheel_at_park);
  ASSIGN_PLC_FIELD(wheel_locked);
  ASSIGN_PLC_FIELD(wheel_unlocked);
  ASSIGN_PLC_FIELD(wheel_pump1_on);
  ASSIGN_PLC_FIELD(wheel_pump2_on);
  ASSIGN_PLC_FIELD(wheel_pump_off);
  ASSIGN_PLC_FIELD(wheel_turn_auto_fb);
  ASSIGN_PLC_FIELD(wheel_turn_manual_fb);
  ASSIGN_PLC_FIELD(laser1_fault);
  ASSIGN_PLC_FIELD(laser2_fault);
  ASSIGN_PLC_FIELD(laser3_fault);
  ASSIGN_PLC_FIELD(laser4_fault);
  ASSIGN_PLC_FIELD(laser5_fault);
  ASSIGN_PLC_FIELD(laser6_fault);
  ASSIGN_PLC_FIELD(laser7_fault);
  ASSIGN_PLC_FIELD(laser8_fault);
  ASSIGN_PLC_FIELD(laser1_height);
  ASSIGN_PLC_FIELD(laser2_height);
  ASSIGN_PLC_FIELD(laser3_height);
  ASSIGN_PLC_FIELD(laser4_height);
  ASSIGN_PLC_FIELD(laser5_height);
  ASSIGN_PLC_FIELD(laser6_height);
  ASSIGN_PLC_FIELD(laser7_height);
  ASSIGN_PLC_FIELD(laser8_height);
  ASSIGN_PLC_FIELD(auto_anti_sway);
  ASSIGN_PLC_FIELD(gantry_move_assist);
  ASSIGN_PLC_FIELD(agss1);
  ASSIGN_PLC_FIELD(agss2);
  ASSIGN_PLC_FIELD(aptt_gsd);
  ASSIGN_PLC_FIELD(general_bypass_fb);
  ASSIGN_PLC_FIELD(human_bypass_fb);
  ASSIGN_PLC_FIELD(agss_left_gsd);
  ASSIGN_PLC_FIELD(agss_right_gsd);
  ASSIGN_PLC_FIELD(speed_limit);
  ASSIGN_PLC_FIELD(spreader_not_top_limit);
  ASSIGN_PLC_FIELD(bypass_atl_fb);
  ASSIGN_PLC_FIELD(bypass_jack_set_fb);
  ASSIGN_PLC_FIELD(bypass_landing_fb);
  ASSIGN_PLC_FIELD(bypass_ls_fb);
  ASSIGN_PLC_FIELD(bypass_over_height_fb);
  ASSIGN_PLC_FIELD(bypass_over_load_fb);
  ASSIGN_PLC_FIELD(bypass_spreader_fb);
  ASSIGN_PLC_FIELD(gantry_run_permit_left);
  ASSIGN_PLC_FIELD(gantry_run_permit_right);
  ASSIGN_PLC_FIELD(trolley_run_permit_forward);
  ASSIGN_PLC_FIELD(trolley_run_permit_backward);
  ASSIGN_PLC_FIELD(hoist_run_permit_up);
  ASSIGN_PLC_FIELD(hoist_run_permit_down);
  ASSIGN_PLC_FIELD(skew_pos);
#undef ASSIGN_PLC_FIELD
  return message;
}

sensor_msgs::msg::Imu load_imu(const fs::path & path)
{
  const json data = read_json(path);
  sensor_msgs::msg::Imu message;
  fill_header(message.header, data);

  const auto orientation = data.at("orientation").get<std::vector<double>>();
  const auto angular_velocity = data.at("angular_velocity").get<std::vector<double>>();
  const auto linear_acceleration = data.at("linear_acceleration").get<std::vector<double>>();
  if (orientation.size() != 4 || angular_velocity.size() != 3 || linear_acceleration.size() != 3) {
    throw std::runtime_error("IMU JSON 数组长度错误: " + path.string());
  }
  message.orientation.x = orientation[0];
  message.orientation.y = orientation[1];
  message.orientation.z = orientation[2];
  message.orientation.w = orientation[3];
  message.angular_velocity.x = angular_velocity[0];
  message.angular_velocity.y = angular_velocity[1];
  message.angular_velocity.z = angular_velocity[2];
  message.linear_acceleration.x = linear_acceleration[0];
  message.linear_acceleration.y = linear_acceleration[1];
  message.linear_acceleration.z = linear_acceleration[2];
  return message;
}

rtg_algorithm_interfaces::msg::GantryPosMsg load_gantry(const fs::path & path)
{
  const json data = read_json(path);
  rtg_algorithm_interfaces::msg::GantryPosMsg message;
  fill_header(message.header, data);

#define ASSIGN_GANTRY_FIELD(name) assign_if_present(data, #name, message.name)
  ASSIGN_GANTRY_FIELD(block);
  ASSIGN_GANTRY_FIELD(bay_no);
  ASSIGN_GANTRY_FIELD(bay_size);
  ASSIGN_GANTRY_FIELD(gantry_pos);
  ASSIGN_GANTRY_FIELD(gps_position);
  ASSIGN_GANTRY_FIELD(gantry_deviation);
  ASSIGN_GANTRY_FIELD(gantry_angle);
  ASSIGN_GANTRY_FIELD(gps_state);
  ASSIGN_GANTRY_FIELD(bay_on_position);
  ASSIGN_GANTRY_FIELD(vision_pos_state);
  ASSIGN_GANTRY_FIELD(vision_offset);
  ASSIGN_GANTRY_FIELD(vision_deviation1);
  ASSIGN_GANTRY_FIELD(vision_deviation2);
  ASSIGN_GANTRY_FIELD(in_auto_alignment_range);
  ASSIGN_GANTRY_FIELD(on_position_distance_1);
  ASSIGN_GANTRY_FIELD(on_position_distance_2);
  ASSIGN_GANTRY_FIELD(gantry_left_deviation);
  ASSIGN_GANTRY_FIELD(gantry_right_deviation);
  ASSIGN_GANTRY_FIELD(trust_state);
  ASSIGN_GANTRY_FIELD(auto_steering_permit);
  ASSIGN_GANTRY_FIELD(offset_from_bay_center);
#undef ASSIGN_GANTRY_FIELD
  return message;
}

sensor_msgs::msg::PointCloud2 load_point_cloud(
  const fs::path & path, const std::string & frame_id)
{
  constexpr std::uint32_t kPointStep = 26;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  if (pcl::io::loadPCDFile(path.string(), cloud) < 0) {
    throw std::runtime_error("无法读取 PCD: " + path.string());
  }

  sensor_msgs::msg::PointCloud2 message;
  message.header.frame_id = frame_id;
  message.header.stamp = filename_stamp(path);
  message.height = cloud.height;
  message.width = cloud.width;
  message.is_bigendian = false;
  message.point_step = kPointStep;
  message.row_step = message.width * message.point_step;
  message.is_dense = cloud.is_dense;

  const std::vector<std::pair<std::string, std::uint32_t>> fields = {
    {"x", 0}, {"y", 4}, {"z", 8}, {"intensity", 12}};
  for (const auto & [name, offset] : fields) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    message.fields.push_back(std::move(field));
  }

  message.data.assign(cloud.size() * kPointStep, 0);
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    const auto & point = cloud.points[index];
    auto * target = message.data.data() + index * kPointStep;
    std::memcpy(target, &point.x, sizeof(float));
    std::memcpy(target + 4, &point.y, sizeof(float));
    std::memcpy(target + 8, &point.z, sizeof(float));
    std::memcpy(target + 12, &point.intensity, sizeof(float));
  }
  return message;
}

rtg_algorithm_interfaces::msg::DataSyncMsg load_data_sync(const Event & event)
{
  if (event.related_paths.size() != 5) {
    throw std::runtime_error("DataSyncMsg 重建路径数量错误");
  }

  rtg_algorithm_interfaces::msg::DataSyncMsg message;
  message.anchor_stamp_ns = static_cast<std::uint64_t>(event.timestamp_ns);
  message.group = "group_3";
  for (std::size_t index = 0; index < 4; ++index) {
    message.pointcloud_list.push_back(
      load_point_cloud(event.related_paths[index], std::to_string(index + 1)));
  }
  message.plc_msg_list.push_back(load_plc(event.related_paths[4]));
  return message;
}

template<typename MessageT>
void write_message(
  rosbag2_cpp::Writer & writer, const MessageT & message, const Event & event)
{
  writer.write(message, event.topic, rclcpp::Time(event.timestamp_ns));
}

fs::path resolve_config(const std::string & argument)
{
  const fs::path supplied(argument);
  if (supplied.is_absolute() || supplied.has_parent_path()) {
    return fs::absolute(supplied);
  }
  return fs::path(ament_index_cpp::get_package_share_directory("rtg_recover_db3")) /
         "config" / supplied;
}

std::vector<fs::path> sorted_files(const fs::path & directory, const std::string & extension)
{
  std::vector<fs::path> files;
  if (!fs::is_directory(directory)) {
    return files;
  }
  for (const auto & entry : fs::directory_iterator(directory)) {
    if (entry.is_regular_file() && entry.path().extension() == extension) {
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());
  return files;
}

std::vector<Event> collect_events(const fs::path & root, const fs::path & config_path)
{
  const YAML::Node mappings = YAML::LoadFile(config_path.string())["lidar_to_topic"];
  if (!mappings || !mappings.IsMap()) {
    throw std::runtime_error("配置缺少 lidar_to_topic 映射: " + config_path.string());
  }

  std::vector<Event> events;
  std::unordered_set<std::string> configured_keys;
  for (const auto & item : mappings) {
    const std::string key = item.first.as<std::string>();
    configured_keys.insert(key);
    const std::string topic = item.second.as<std::string>();
    const fs::path directory = root / key;
    if (!fs::is_directory(directory)) {
      std::cout << "[跳过] 不存在目录: " << directory << '\n';
      continue;
    }

    MessageKind kind;
    std::string extension;
    std::string type;
    if (key == "plc") {
      kind = MessageKind::Plc;
      extension = ".json";
      type = kPlcType;
    } else if (key.rfind("imu", 0) == 0) {
      kind = MessageKind::Imu;
      extension = ".json";
      type = kImuType;
    } else {
      kind = MessageKind::PointCloud;
      extension = ".pcd";
      type = kPointCloudType;
    }

    std::size_t count = 0;
    for (const auto & entry : fs::directory_iterator(directory)) {
      if (!entry.is_regular_file() || entry.path().extension() != extension) {
        continue;
      }
      const auto stamp = kind == MessageKind::PointCloud ?
        filename_stamp(entry.path()) : json_stamp(read_json(entry.path()), entry.path());
      events.push_back({entry.path(), key, topic, type, kind, stamp_to_ns(stamp), {}});
      ++count;
    }
    std::cout << topic << ": " << count << " 条\n";
  }

  configured_keys.insert("GantryPosMsg");
  const auto gantry_files = sorted_files(root / "GantryPosMsg", ".json");
  for (const auto & path : gantry_files) {
    const auto stamp = json_stamp(read_json(path), path);
    events.push_back(
      {path, "GantryPosMsg", kGantryTopic, kGantryType, MessageKind::Gantry,
        stamp_to_ns(stamp), {}});
  }
  if (!gantry_files.empty()) {
    std::cout << kGantryTopic << ": " << gantry_files.size() << " 条\n";
  }

  const std::vector<std::string> sync_keys = {"1", "2", "3", "4", "plc_frame_35"};
  std::vector<std::vector<fs::path>> sync_files;
  sync_files.reserve(sync_keys.size());
  for (const auto & key : sync_keys) {
    configured_keys.insert(key);
    sync_files.push_back(sorted_files(root / key, key == "plc_frame_35" ? ".json" : ".pcd"));
  }
  const std::size_t sync_count = std::max_element(
    sync_files.begin(), sync_files.end(),
    [](const auto & left, const auto & right) {return left.size() < right.size();})->size();
  if (sync_count > 0) {
    for (const auto & files : sync_files) {
      if (files.size() != sync_count) {
        throw std::runtime_error("DataSyncMsg 各子目录文件数量不一致，无法可靠分组");
      }
    }
    for (std::size_t index = 0; index < sync_count; ++index) {
      std::vector<fs::path> related_paths;
      related_paths.reserve(sync_files.size());
      for (const auto & files : sync_files) {
        related_paths.push_back(files[index]);
      }
      const auto stamp = json_stamp(read_json(related_paths.back()), related_paths.back());
      events.push_back(
        {related_paths.back(), "DataSyncMsg", kDataSyncTopic, kDataSyncType,
          MessageKind::DataSync, stamp_to_ns(stamp), std::move(related_paths)});
    }
    std::cout << kDataSyncTopic << ": " << sync_count << " 条\n";
  }

  for (const auto & entry : fs::directory_iterator(root)) {
    if (entry.is_directory() && configured_keys.count(entry.path().filename().string()) == 0) {
      std::cout << "[未重建] 配置中没有 Topic 映射: " << entry.path() << '\n';
    }
  }

  std::sort(events.begin(), events.end(), [](const Event & left, const Event & right) {
    if (left.timestamp_ns != right.timestamp_ns) {
      return left.timestamp_ns < right.timestamp_ns;
    }
    return left.path.string() < right.path.string();
  });
  return events;
}

void print_usage(const char * program)
{
  std::cerr << "用法: " << program
            << " <提取目录> [输出bag目录] [配置文件]\n";
}

fs::path default_output_path(const fs::path & source)
{
  constexpr const char * kExtractedSuffix = "_extracted";
  std::string source_name = source.filename().string();
  const std::string extracted_suffix = kExtractedSuffix;
  if (source_name.size() >= extracted_suffix.size() &&
    source_name.compare(
      source_name.size() - extracted_suffix.size(), extracted_suffix.size(), extracted_suffix) == 0)
  {
    source_name.resize(source_name.size() - extracted_suffix.size());
  }

  const std::string output_name = source_name + "_recovered_bag";
  fs::path output = source.parent_path() / output_name;
  for (std::uint32_t index = 2; fs::exists(output); ++index) {
    output = source.parent_path() / (output_name + "_" + std::to_string(index));
  }
  return output;
}

std::string infer_config_name(const fs::path & source)
{
  static const std::regex machine_pattern(R"((651|652|653|654|655))");
  std::smatch match;
  const std::string source_name = source.filename().string();
  if (!std::regex_search(source_name, match, machine_pattern)) {
    throw std::runtime_error("无法从提取目录名识别 651–655 机号");
  }
  return match[1].str() + ".yaml";
}

void create_topic(
  rosbag2_cpp::Writer & writer, const std::string & name, const std::string & type)
{
  int reliability = 2;
  int durability = 2;
  if (name == kExecStatusTopic) {
    reliability = 1;
  } else if (name == kResetTopic) {
    reliability = 1;
    durability = 1;
  }

  rosbag2_storage::TopicMetadata metadata;
  metadata.name = name;
  metadata.type = type;
  metadata.serialization_format = "cdr";
  metadata.offered_qos_profiles =
    "- history: 3\n"
    "  depth: 0\n"
    "  reliability: " + std::to_string(reliability) + "\n"
    "  durability: " + std::to_string(durability) + "\n"
    "  deadline:\n"
    "    sec: 9223372036\n"
    "    nsec: 854775807\n"
    "  lifespan:\n"
    "    sec: 9223372036\n"
    "    nsec: 854775807\n"
    "  liveliness: 1\n"
    "  liveliness_lease_duration:\n"
    "    sec: 9223372036\n"
    "    nsec: 854775807\n"
    "  avoid_ros_namespace_conventions: false";
  writer.create_topic(metadata);
}

int main(int argc, char ** argv)
{
  if (argc < 2 || argc > 4) {
    print_usage(argv[0]);
    return 1;
  }

  fs::path temporary_root;
  bool bag_complete = false;
  try {
    const fs::path extracted_dir = fs::absolute(argv[1]).lexically_normal();
    if (!fs::is_directory(extracted_dir)) {
      throw std::runtime_error("提取目录不存在: " + extracted_dir.string());
    }
    const fs::path output_bag = argc >= 3 ?
      fs::absolute(argv[2]).lexically_normal() : default_output_path(extracted_dir);
    const fs::path config_path = resolve_config(
      argc == 4 ? argv[3] : infer_config_name(extracted_dir));
    if (!fs::is_regular_file(config_path)) {
      throw std::runtime_error("配置文件不存在: " + config_path.string());
    }
    if (fs::exists(output_bag)) {
      throw std::runtime_error("输出路径已存在，请换一个新目录: " + output_bag.string());
    }
    fs::create_directories(output_bag.parent_path());

    temporary_root = fs::temp_directory_path() /
      ("rtg_recover_db3_" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directory(temporary_root);
    const fs::path temporary_bag = temporary_root / output_bag.filename();

    std::cout << "提取目录: " << extracted_dir << '\n'
              << "输出目录: " << output_bag << '\n'
              << "配置文件: " << config_path << '\n'
              << "本地临时目录: " << temporary_bag << '\n';
    auto events = collect_events(extracted_dir, config_path);
    if (events.empty()) {
      throw std::runtime_error("没有找到与配置匹配的 PCD/JSON 文件");
    }
    std::cout << "合计: " << events.size() << " 条，开始按时间写入 rosbag2\n";

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = temporary_bag.string();
    storage_options.storage_id = "sqlite3";
    auto writer = std::make_unique<rosbag2_cpp::Writer>();
    writer->open(storage_options);

    std::unordered_map<std::string, std::string> topics;
    for (const auto & event : events) {
      if (topics.emplace(event.topic, event.type).second) {
        create_topic(*writer, event.topic, event.type);
      }
    }
    create_topic(*writer, kExecStatusTopic, kExecStatusType);
    create_topic(*writer, kResetTopic, kResetType);

    std::size_t written = 0;
    for (const auto & event : events) {
      switch (event.kind) {
        case MessageKind::PointCloud:
          write_message(*writer, load_point_cloud(event.path, event.key), event);
          break;
        case MessageKind::Plc:
          write_message(*writer, load_plc(event.path), event);
          break;
        case MessageKind::Imu:
          write_message(*writer, load_imu(event.path), event);
          break;
        case MessageKind::Gantry:
          write_message(*writer, load_gantry(event.path), event);
          break;
        case MessageKind::DataSync:
          write_message(*writer, load_data_sync(event), event);
          break;
      }
      ++written;
      if (written % 1000 == 0 || written == events.size()) {
        std::cout << "[写入] " << written << '/' << events.size() << '\n';
      }
    }

    writer.reset();
    bag_complete = true;
    std::cout << "本地业务 bag 生成完成，正在复制到目标目录\n";
    try {
      fs::copy(temporary_bag, output_bag, fs::copy_options::recursive);
    } catch (const std::exception & error) {
      std::error_code cleanup_error;
      fs::remove_all(output_bag, cleanup_error);
      throw std::runtime_error(
              "复制到输出目录失败: " + std::string(error.what()) +
              "\n本地完整临时 bag 已保留: " + temporary_bag.string());
    }

    std::error_code cleanup_error;
    fs::remove_all(temporary_root, cleanup_error);
    if (cleanup_error) {
      std::cerr << "警告: 未能删除本地临时目录: " << temporary_root << '\n';
    }
    std::cout << "重建完成: " << output_bag << '\n'
              << "实际写入: " << written << " 条\n";
    return 0;
  } catch (const std::exception & error) {
    if (!bag_complete && !temporary_root.empty()) {
      std::error_code cleanup_error;
      fs::remove_all(temporary_root, cleanup_error);
    }
    std::cerr << "错误: " << error.what() << '\n';
    return 1;
  }
}
