################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)  # Use at least version 3.8 for ROS 2

project(turtlebot3_gazebo)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

# Enable position independent code
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)

################################################################################
# Build
################################################################################
# Enable automatic MOC, UIC, and RCC for Qt5
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Include directories
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Link directories (if needed)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Define dependencies
set(dependencies
  geometry_msgs
  nav_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  std_msgs
  gazebo_ros_pkgs
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Set the executable name
set(EXEC_NAME "turtlebot3_shop_keeper")

# Add executable
add_executable(${EXEC_NAME}
  src/main.cpp
  src/MainWindow.cpp
  include/MainWindow.hpp
  src/InventoryWaypointsNode.cpp
  src/SensorProcessor.cpp
  src/WaypointGenerator.cpp
  src/WaypointManager.cpp
  src/MapManager.cpp
  src/MotionController.cpp
)

# Set compile options (after the target is created)
target_compile_options(${EXEC_NAME} PRIVATE -fPIC)

# Link libraries
target_link_libraries(${EXEC_NAME}
  Qt5::Widgets
  ${GAZEBO_LIBRARIES}
)

# Add ament dependencies
ament_target_dependencies(${EXEC_NAME} ${dependencies})

# Add libraries for obstacle plugins
add_library(obstacle1 SHARED models/turtlebot3_dqn_world/obstacle_plugin/obstacle1.cc)
target_link_libraries(obstacle1 ${GAZEBO_LIBRARIES})

add_library(obstacle2 SHARED models/turtlebot3_dqn_world/obstacle_plugin/obstacle2.cc)
target_link_libraries(obstacle2 ${GAZEBO_LIBRARIES})

add_library(obstacles SHARED models/turtlebot3_dqn_world/obstacle_plugin/obstacles.cc)
target_link_libraries(obstacles ${GAZEBO_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
