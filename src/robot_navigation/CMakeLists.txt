cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Add waypoint publisher node
add_executable(waypoint_publisher_node src/waypoint_publisher_node.cpp)
ament_target_dependencies(waypoint_publisher_node
  rclcpp
  nav_msgs
  geometry_msgs
  ament_index_cpp
  visualization_msgs
  yaml-cpp)
# Explicitly link the yaml-cpp library to the executable
target_link_libraries(waypoint_publisher_node 
  yaml-cpp::yaml-cpp)


# Add path smoother node
add_executable(path_smoother_node src/path_smoother_node.cpp)
ament_target_dependencies(path_smoother_node
  rclcpp
  nav_msgs
  geometry_msgs)


# Add trajectory tracker node
add_executable(trajectory_tracker_node src/trajectory_tracker_node.cpp)
ament_target_dependencies(trajectory_tracker_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2)

install(TARGETS
  waypoint_publisher_node
  path_smoother_node
  trajectory_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY rviz 
  DESTINATION share/${PROJECT_NAME}
)


# Install python scripts
install(PROGRAMS
  scripts/plotter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

