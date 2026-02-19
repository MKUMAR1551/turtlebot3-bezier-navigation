cmake_minimum_required(VERSION 3.8)
project(assignment_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 1. Path Planner Node
add_executable(path_planner_node src/path_planner.cpp)
ament_target_dependencies(path_planner_node rclcpp geometry_msgs nav_msgs)

# 2. Trajectory Generator Node
add_executable(trajectory_generator_node src/trajectory_generator.cpp)
ament_target_dependencies(trajectory_generator_node rclcpp geometry_msgs nav_msgs)

# 3. Controller Node
add_executable(controller_node src/controller.cpp)
ament_target_dependencies(controller_node rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

# Install Executables
install(TARGETS
  path_planner_node
  trajectory_generator_node
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Launch Files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
