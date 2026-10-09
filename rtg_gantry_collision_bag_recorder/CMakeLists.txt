cmake_minimum_required(VERSION 3.8)
project(rtg_gantry_collision_bag_recorder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sensor_device_interfaces REQUIRED)
find_package(rtg_algorithm_interfaces REQUIRED)

add_executable(gantry_collision_bag_recorder_node src/gantry_collision_bag_recorder_node.cpp)
target_include_directories(gantry_collision_bag_recorder_node PRIVATE include)
ament_target_dependencies(gantry_collision_bag_recorder_node
  rclcpp rmw rosbag2_cpp rosbag2_storage sensor_msgs
  sensor_device_interfaces rtg_algorithm_interfaces)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_alarm_state test/test_alarm_state.cpp)
  target_include_directories(test_alarm_state PRIVATE include)
  add_test(NAME alarm_state COMMAND test_alarm_state)
  add_executable(test_pre_record_buffer test/test_pre_record_buffer.cpp)
  target_include_directories(test_pre_record_buffer PRIVATE include)
  add_test(NAME pre_record_buffer COMMAND test_pre_record_buffer)
  add_executable(test_recording_window test/test_recording_window.cpp)
  target_include_directories(test_recording_window PRIVATE include)
  add_test(NAME recording_window COMMAND test_recording_window)
endif()

install(TARGETS gantry_collision_bag_recorder_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/source_integration_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME source_integration_test)
install(PROGRAMS scripts/recording_window_integration_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME recording_window_integration_test)
install(PROGRAMS scripts/verify_recording_window_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME verify_recording_window_test)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
