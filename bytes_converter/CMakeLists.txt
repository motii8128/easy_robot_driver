cmake_minimum_required(VERSION 3.5)
project(bytes_converter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(wheel_converter_node src/wheel_converter.cpp)
add_executable(one_motor_converter_node src/one_motor_converter.cpp)
ament_target_dependencies(wheel_converter_node rclcpp std_msgs)
ament_target_dependencies(one_motor_converter_node rclcpp std_msgs)

install(TARGETS
  wheel_converter_node
  DESTINATION lib/${PROJECT_NAME})

  install(TARGETS
  one_motor_converter_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()