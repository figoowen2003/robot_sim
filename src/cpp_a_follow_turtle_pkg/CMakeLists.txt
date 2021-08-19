cmake_minimum_required(VERSION 3.5)
project(cpp_a_follow_turtle_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(turtle_spawner src/new_turtle_spawner.cpp)
add_executable(turtle_pub src/turtles_pose_pub.cpp)
add_executable(turtle2_controller src/turtle2_controller.cpp)
ament_target_dependencies(turtle_spawner rclcpp turtlesim)
ament_target_dependencies(turtle_pub rclcpp turtlesim tf2_ros geometry_msgs)
ament_target_dependencies(turtle2_controller rclcpp tf2_ros geometry_msgs)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


install(TARGETS turtle_spawner
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS turtle_pub
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS turtle2_controller
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
