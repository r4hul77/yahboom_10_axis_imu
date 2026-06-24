cmake_minimum_required(VERSION 3.8)
project(yahboom_10_axis_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(yahboom_10_axis_imu_core
  src/protocol.cpp
  src/imu_packet.cpp
  src/termios_serial_port.cpp
  src/write_protocol.cpp
)

target_include_directories(yahboom_10_axis_imu_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(yahboom_10_axis_imu_node
  src/yahboom_10_axis_imu_node.cpp
)

target_link_libraries(yahboom_10_axis_imu_node
  yahboom_10_axis_imu_core
)

ament_target_dependencies(yahboom_10_axis_imu_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS yahboom_10_axis_imu_core yahboom_10_axis_imu_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_protocol_parser
    test/test_protocol_parser.cpp
  )
  target_link_libraries(test_protocol_parser yahboom_10_axis_imu_core)

  ament_add_gtest(test_imu_packet
    test/test_imu_packet.cpp
  )
  target_link_libraries(test_imu_packet yahboom_10_axis_imu_core)

  ament_add_gtest(test_write_protocol
    test/test_write_protocol.cpp
  )
  target_link_libraries(test_write_protocol yahboom_10_axis_imu_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(yahboom_10_axis_imu_core)

ament_package()
