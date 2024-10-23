cmake_minimum_required(VERSION 3.5)
project(shape_detector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(shape_detector src/shape_detector.cpp)

ament_target_dependencies(shape_detector
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

install(TARGETS
  shape_detector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
