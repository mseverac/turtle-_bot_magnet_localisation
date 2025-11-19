cmake_minimum_required(VERSION 3.5)
project(ekf_localization)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Add executable
add_executable(ekf_localization_node src/ekf_localization_node.cpp)

# Link against libraries
ament_target_dependencies(ekf_localization_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS ekf_localization_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()


