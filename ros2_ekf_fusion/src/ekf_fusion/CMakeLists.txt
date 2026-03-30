cmake_minimum_required(VERSION 3.8)
project(ekf_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vehicle_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(ekf_fusion_node
  src/main.cpp
  src/ekf_fusion.cpp
  src/global_drift_ekf.cpp
)

target_include_directories(ekf_fusion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(ekf_fusion_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  vehicle_interfaces
)

target_link_libraries(ekf_fusion_node
  Eigen3::Eigen
)

install(TARGETS ekf_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
