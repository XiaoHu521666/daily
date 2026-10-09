cmake_minimum_required(VERSION 3.8)
project(rtg_split_db3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rtg_algorithm_interfaces REQUIRED)
find_package(sensor_device_interfaces REQUIRED)

add_executable(rtg_split_db3
  src/main.cpp
  src/db3_splitter.cpp
)

target_compile_features(rtg_split_db3 PUBLIC cxx_std_17)
target_include_directories(rtg_split_db3 PUBLIC include)

ament_target_dependencies(rtg_split_db3
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  rtg_algorithm_interfaces
  sensor_device_interfaces
)

install(TARGETS rtg_split_db3 DESTINATION lib/${PROJECT_NAME})

ament_package()
