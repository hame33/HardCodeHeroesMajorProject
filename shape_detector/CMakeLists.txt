cmake_minimum_required(VERSION 3.5)
project(shape_detector)

find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(shape_detector src/shape_detector.cpp src/shape_detector_utils.cpp src/main.cpp)
ament_target_dependencies(shape_detector rclcpp cv_bridge image_transport OpenCV)
install(TARGETS shape_detector DESTINATION lib/${PROJECT_NAME})
