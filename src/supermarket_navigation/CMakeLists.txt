cmake_minimum_required(VERSION 3.5)
project(supermarket_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(send_goals src/send_goals.cpp)
ament_target_dependencies(send_goals rclcpp rclcpp_action nav2_msgs tf2 tf2_geometry_msgs)

install(TARGETS send_goals DESTINATION lib/${PROJECT_NAME})

ament_package()
