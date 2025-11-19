cmake_minimum_required(VERSION 3.8)

project(sensor_simulation)



# Default to C++14

if(NOT CMAKE_CXX_STANDARD)

  set(CMAKE_CXX_STANDARD 14)

endif()



# find dependencies

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)



add_executable(sensor_simulation_node src/sensor_simulation_node.cpp)

ament_target_dependencies(sensor_simulation_node rclcpp std_msgs)



install(TARGETS

  sensor_simulation_node

  DESTINATION lib/${PROJECT_NAME})



ament_package()
