cmake_minimum_required(VERSION 3.8)
project(rtg_auto_bag_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rmw REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rtg_algorithm_interfaces REQUIRED)
find_package(sensor_device_interfaces REQUIRED)

add_executable(auto_bag_recorder_node src/auto_bag_recorder_node.cpp)
add_executable(format_ros_log_time src/format_ros_log_time.cpp)
ament_target_dependencies(auto_bag_recorder_node
  rclcpp
  rmw
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  rtg_algorithm_interfaces
  sensor_device_interfaces
)

install(TARGETS auto_bag_recorder_node format_ros_log_time
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
