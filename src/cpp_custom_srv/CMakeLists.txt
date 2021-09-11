cmake_minimum_required(VERSION 3.8)
project(cpp_custom_srv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cpp_tutorial_interfaces REQUIRED)        # CHANGE

add_executable(server src/add_three_ints_server.cpp)
ament_target_dependencies(server
  rclcpp cpp_tutorial_interfaces)                      #CHANGE

add_executable(client src/add_three_ints_client.cpp)
ament_target_dependencies(client
  rclcpp cpp_tutorial_interfaces)                      #CHANGE

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
