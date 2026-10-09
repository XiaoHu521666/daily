#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace fs = std::filesystem;

constexpr const char * kArchiveTopic = "/rtg/recover_db3/extracted_file";
constexpr const char * kArchiveType = "std_msgs/msg/UInt8MultiArray";
constexpr std::uint64_t kChunkBytes = 64ULL * 1024ULL * 1024ULL;

std::vector<fs::path> collect_files(const fs::path & root)
{
  std::vector<fs::path> files;
  for (const auto & entry : fs::recursive_directory_iterator(root)) {
    if (entry.is_regular_file()) {
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end(), [&root](const fs::path & left, const fs::path & right) {
    return fs::relative(left, root).generic_string() < fs::relative(right, root).generic_string();
  });
  return files;
}

void archive_file(
  rosbag2_cpp::Writer & writer, const fs::path & root, const fs::path & path,
  std::uint64_t & sequence, std::uint64_t & total_bytes)
{
  const std::uint64_t file_size = fs::file_size(path);
  const std::uint64_t chunk_count_64 = std::max<std::uint64_t>(1, (file_size + kChunkBytes - 1) / kChunkBytes);
  if (chunk_count_64 > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error("文件分块数量超出格式限制: " + path.string());
  }
  const auto chunk_count = static_cast<std::uint32_t>(chunk_count_64);

  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("无法读取文件: " + path.string());
  }

  for (std::uint32_t chunk_index = 0; chunk_index < chunk_count; ++chunk_index) {
    const std::uint64_t offset = static_cast<std::uint64_t>(chunk_index) * kChunkBytes;
    const std::size_t chunk_size = static_cast<std::size_t>(
      std::min<std::uint64_t>(kChunkBytes, file_size - offset));

    std_msgs::msg::UInt8MultiArray message;
    message.layout.dim.resize(1);
    message.layout.dim[0].label = fs::relative(path, root).generic_string();
    message.layout.dim[0].size = chunk_index;
    message.layout.dim[0].stride = chunk_count;
    message.data.resize(chunk_size);
    if (chunk_size > 0) {
      input.read(reinterpret_cast<char *>(message.data.data()), chunk_size);
      if (input.gcount() != static_cast<std::streamsize>(chunk_size)) {
        throw std::runtime_error("读取文件不完整: " + path.string());
      }
    }
    writer.write(message, kArchiveTopic, rclcpp::Time(static_cast<int64_t>(sequence++)));
    total_bytes += chunk_size;
  }
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

int main(int argc, char ** argv)
{
  if (argc != 2 && argc != 3) {
    std::cerr << "用法: " << argv[0] << " <源提取目录> [输出bag目录]\n";
    return 1;
  }

  fs::path temporary_root;
  bool archive_complete = false;
  try {
    const fs::path source = fs::absolute(argv[1]).lexically_normal();
    if (!fs::is_directory(source)) {
      throw std::runtime_error("源提取目录不存在: " + source.string());
    }
    const fs::path output = argc == 3 ?
      fs::absolute(argv[2]).lexically_normal() : default_output_path(source);
    if (fs::exists(output)) {
      throw std::runtime_error("输出路径已存在: " + output.string());
    }
    std::cout << "输出目录: " << output << '\n';
    fs::create_directories(output.parent_path());

    temporary_root = fs::temp_directory_path() /
      ("rtg_recover_db3_" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directory(temporary_root);
    const fs::path temporary_bag = temporary_root / output.filename();
    std::cout << "本地临时目录: " << temporary_bag << '\n';

    const auto files = collect_files(source);
    if (files.empty()) {
      throw std::runtime_error("源提取目录中没有文件");
    }

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = temporary_bag.string();
    storage_options.storage_id = "sqlite3";
    auto writer = std::make_unique<rosbag2_cpp::Writer>();
    writer->open(storage_options);

    rosbag2_storage::TopicMetadata metadata;
    metadata.name = kArchiveTopic;
    metadata.type = kArchiveType;
    metadata.serialization_format = "cdr";
    writer->create_topic(metadata);

    std::uint64_t sequence = 0;
    std::uint64_t total_bytes = 0;
    for (std::size_t index = 0; index < files.size(); ++index) {
      archive_file(*writer, source, files[index], sequence, total_bytes);
      if ((index + 1) % 100 == 0 || index + 1 == files.size()) {
        std::cout << "[归档] " << index + 1 << '/' << files.size() << " 个文件\n";
      }
    }
    writer.reset();
    archive_complete = true;

    std::cout << "本地归档完成，正在复制到: " << output << '\n';
    try {
      fs::copy(temporary_bag, output, fs::copy_options::recursive);
    } catch (const std::exception & error) {
      std::error_code cleanup_error;
      fs::remove_all(output, cleanup_error);
      throw std::runtime_error(
              "复制到输出目录失败: " + std::string(error.what()) +
              "\n本地完整临时 bag 已保留: " + temporary_bag.string());
    }

    std::error_code cleanup_error;
    fs::remove_all(temporary_root, cleanup_error);
    if (cleanup_error) {
      std::cerr << "警告: 未能删除本地临时目录: " << temporary_root << '\n';
    }

    std::cout << "保真归档完成: " << output << '\n'
              << "文件数: " << files.size() << "，原始字节数: " << total_bytes << '\n';
    return 0;
  } catch (const std::exception & error) {
    if (!archive_complete && !temporary_root.empty()) {
      std::error_code cleanup_error;
      fs::remove_all(temporary_root, cleanup_error);
    }
    std::cerr << "错误: " << error.what() << '\n';
    return 1;
  }
}
