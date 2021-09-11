cmake_minimum_required(VERSION 3.8)
project(cpp_custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cpp_tutorial_interfaces REQUIRED)                         # CHANGE

add_executable(talker src/minimal_publisher.cpp)
ament_target_dependencies(talker rclcpp cpp_tutorial_interfaces)         # CHANGE

add_executable(listener src/minimal_subscriber.cpp)
ament_target_dependencies(listener rclcpp cpp_tutorial_interfaces)     # CHANGE

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()