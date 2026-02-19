cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# ---- Dependencies ----
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
find_package(Eigen3 REQUIRED)           # For MPC matrix math (header-only, ships with ROS2)
find_package(osqp_vendor REQUIRED)      # OSQP QP solver
find_package(OsqpEigen REQUIRED)        # C++ Eigen wrapper for OSQP

# ---- Include directories ----
# Makes `#include "robot_navigation/spline_generator.hpp"` work
include_directories(include)

# ---- Nodes ----

add_executable(waypoint_publisher_node src/waypoint_publisher_node.cpp)
ament_target_dependencies(waypoint_publisher_node
  rclcpp nav_msgs geometry_msgs ament_index_cpp visualization_msgs yaml-cpp)
target_link_libraries(waypoint_publisher_node yaml-cpp::yaml-cpp)

add_executable(path_smoother_node src/path_smoother_node.cpp)
ament_target_dependencies(path_smoother_node rclcpp nav_msgs geometry_msgs)
# SplineGenerator is header-only — no extra link needed

add_executable(trajectory_tracker_node src/trajectory_tracker_node.cpp)
ament_target_dependencies(trajectory_tracker_node
  rclcpp nav_msgs geometry_msgs sensor_msgs tf2 visualization_msgs)
target_link_libraries(trajectory_tracker_node
  Eigen3::Eigen OsqpEigen::OsqpEigen osqp::osqp)

# ---- Tests (standalone, no ROS runtime required) ----
add_executable(test_spline test/test_spline.cpp)
target_include_directories(test_spline PRIVATE include)
# No link deps: spline is header-only, uses only std + cmath

add_executable(test_mpc test/test_mpc.cpp)
target_include_directories(test_mpc PRIVATE include)
target_link_libraries(test_mpc Eigen3::Eigen OsqpEigen::OsqpEigen osqp::osqp)

# ---- Install ----
install(TARGETS
  waypoint_publisher_node
  path_smoother_node
  trajectory_tracker_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/)   # Install headers for downstream use
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS scripts/plotter.py scripts/mission_report.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()