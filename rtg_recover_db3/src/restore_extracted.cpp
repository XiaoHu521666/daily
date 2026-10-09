#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace fs = std::filesystem;

constexpr const char * kArchiveTopic = "/rtg/recover_db3/extracted_file";

bool is_safe_relative_path(const fs::path & path)
{
  if (path.empty() || path.is_absolute()) {
    return false;
  }
  for (const auto & component : path) {
    if (component == "..") {
      return false;
    }
  }
  return true;
}

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "用法: " << argv[0] << " <保真bag目录> <输出提取目录>\n";
    return 1;
  }

  try {
    const fs::path bag = fs::absolute(argv[1]);
    const fs::path output = fs::absolute(argv[2]);
    if (!fs::is_directory(bag) || !fs::is_regular_file(bag / "metadata.yaml")) {
      throw std::runtime_error("输入不是完整 rosbag2 目录: " + bag.string());
    }
    if (fs::exists(output)) {
      throw std::runtime_error("输出路径已存在: " + output.string());
    }
    fs::create_directories(output);

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag.string();
    storage_options.storage_id = "sqlite3";
    rosbag2_cpp::ConverterOptions converter_options{
      rmw_get_serialization_format(), rmw_get_serialization_format()};
    rosbag2_cpp::readers::SequentialReader reader;
    reader.open(storage_options, converter_options);

    rclcpp::Serialization<std_msgs::msg::UInt8MultiArray> serializer;
    std::string active_path;
    std::uint32_t expected_chunk = 0;
    std::uint32_t expected_count = 0;
    std::ofstream output_file;
    std::size_t file_count = 0;
    std::uint64_t total_bytes = 0;

    while (reader.has_next()) {
      const auto serialized_bag_message = reader.read_next();
      if (serialized_bag_message->topic_name != kArchiveTopic) {
        continue;
      }

      rclcpp::SerializedMessage serialized(*serialized_bag_message->serialized_data);
      std_msgs::msg::UInt8MultiArray message;
      serializer.deserialize_message(&serialized, &message);
      if (message.layout.dim.size() != 1) {
        throw std::runtime_error("归档消息布局无效");
      }

      const auto & dimension = message.layout.dim[0];
      const fs::path relative_path(dimension.label);
      if (!is_safe_relative_path(relative_path) || dimension.stride == 0 ||
        dimension.size >= dimension.stride)
      {
        throw std::runtime_error("归档文件路径或分块信息无效: " + dimension.label);
      }

      if (dimension.label != active_path) {
        if (!active_path.empty() && expected_chunk != expected_count) {
          throw std::runtime_error("文件分块不完整: " + active_path);
        }
        output_file.close();
        const fs::path destination = output / relative_path;
        fs::create_directories(destination.parent_path());
        output_file.open(destination, std::ios::binary | std::ios::trunc);
        if (!output_file) {
          throw std::runtime_error("无法创建文件: " + destination.string());
        }
        active_path = dimension.label;
        expected_chunk = 0;
        expected_count = dimension.stride;
        ++file_count;
      }

      if (dimension.size != expected_chunk || dimension.stride != expected_count) {
        throw std::runtime_error("文件分块顺序错误: " + dimension.label);
      }
      if (!message.data.empty()) {
        output_file.write(
          reinterpret_cast<const char *>(message.data.data()), message.data.size());
        if (!output_file) {
          throw std::runtime_error("写入文件失败: " + dimension.label);
        }
      }
      total_bytes += message.data.size();
      ++expected_chunk;
    }

    if (active_path.empty()) {
      throw std::runtime_error("bag 中没有保真归档 Topic");
    }
    if (expected_chunk != expected_count) {
      throw std::runtime_error("文件分块不完整: " + active_path);
    }
    output_file.close();
    std::cout << "原文件恢复完成: " << output << '\n'
              << "文件数: " << file_count << "，恢复字节数: " << total_bytes << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "错误: " << error.what() << '\n';
    return 1;
  }
}
