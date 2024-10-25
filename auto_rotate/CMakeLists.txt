cmake_minimum_required(VERSION 3.5)
project(auto_rotate)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(rotate_after_distance src/rotate_after_distance.cpp)
ament_target_dependencies(rotate_after_distance rclcpp rclcpp_action geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs nav2_msgs)

install(TARGETS
  rotate_after_distance
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
