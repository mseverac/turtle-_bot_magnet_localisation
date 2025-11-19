cmake_minimum_required(VERSION 3.5)
project(data_replay)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executables
add_executable(data_replay src/data_replay.cpp)

ament_target_dependencies(data_replay
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  data_replay
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

