#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <rosbag2_storage/topic_metadata.hpp>

namespace rtg_split_db3
{

struct Options
{
  std::filesystem::path source_bag;
  std::filesystem::path output_dir;
  double min_stationary_seconds{10.0};
  double edge_trim_seconds{2.0};
  bool dry_run{false};
};

struct PlcSample
{
  std::int64_t timestamp_ns;
  double max_gantry_velocity;
  double trolley_pos;
};

struct GantrySample
{
  std::int64_t timestamp_ns;
  std::string block;
  std::string bay_no;
};

struct Segment
{
  std::int64_t start_ns;
  std::int64_t end_ns;
  std::string block;
  std::string bay_no;
  std::string output_name;
  double trolley_min;
  double trolley_max;
};

class Db3Splitter
{
public:
  explicit Db3Splitter(Options options);
  void run();

private:
  void scan_control_topics();
  void detect_segments();
  void write_segments();
  void assign_output_names();

  Options options_;
  std::vector<PlcSample> plc_samples_;
  std::vector<GantrySample> gantry_samples_;
  std::vector<Segment> segments_;
  std::vector<rosbag2_storage::TopicMetadata> topics_;
};

Options parse_options(int argc, char ** argv);
void print_usage(const char * program);

}  // namespace rtg_split_db3
