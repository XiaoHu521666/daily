cmake_minimum_required(VERSION 3.8)
project(rtg_recover_db3)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rtg_algorithm_interfaces REQUIRED)
find_package(sensor_device_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(rtg_recover_db3 src/recover_db3.cpp)
target_include_directories(rtg_recover_db3 PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(rtg_recover_db3
  ${PCL_LIBRARIES} yaml-cpp nlohmann_json::nlohmann_json)
ament_target_dependencies(rtg_recover_db3
  ament_index_cpp
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  rtg_algorithm_interfaces
  sensor_device_interfaces
  sensor_msgs
)

add_executable(rtg_archive_extracted src/archive_extracted.cpp)
ament_target_dependencies(rtg_archive_extracted rclcpp rosbag2_cpp rosbag2_storage std_msgs)

add_executable(rtg_restore_extracted src/restore_extracted.cpp)
ament_target_dependencies(rtg_restore_extracted rclcpp rosbag2_cpp rosbag2_storage std_msgs)

install(TARGETS
  rtg_recover_db3
  rtg_archive_extracted
  rtg_restore_extracted
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
