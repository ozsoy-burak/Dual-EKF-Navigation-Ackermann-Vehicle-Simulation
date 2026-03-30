cmake_minimum_required(VERSION 3.8)
project(sensor_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vehicle_interfaces REQUIRED)

add_executable(sensor_publisher_node
  src/main.cpp
  src/sensor_publisher.cpp
)

target_include_directories(sensor_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(sensor_publisher_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  vehicle_interfaces
)

install(TARGETS sensor_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
