cmake_minimum_required(VERSION 3.5)
project(measurement_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(measurement_node src/measurement_node.cpp)
ament_target_dependencies(measurement_node rclcpp std_msgs geometry_msgs)

install(TARGETS
  measurement_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
