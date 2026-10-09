cmake_minimum_required(VERSION 3.8)
project(rtg_auto_bag_compress)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rtg_algorithm_interfaces REQUIRED)
find_package(sensor_device_interfaces REQUIRED)

add_executable(change_event_bag_recorder_node src/change_event_bag_recorder_node.cpp)
ament_target_dependencies(change_event_bag_recorder_node
  rclcpp
  rosbag2_compression
  rosbag2_cpp
  rosbag2_storage
  rtg_algorithm_interfaces
  sensor_device_interfaces
)

install(TARGETS change_event_bag_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/extract_time_range.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
