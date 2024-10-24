// SensorProcessor.hpp

#ifndef SENSOR_PROCESSOR_HPP_
#define SENSOR_PROCESSOR_HPP_

// --- Includes ---
#include <array>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "Constants.hpp"
#include <utility>

// --- SensorProcessor Class Interface ---
// The SensorProcessor class handles the processing of sensor data, including LIDAR scans 
// and odometry information. It provides methods for updating and retrieving sensor data, 
// and computing errors and pose changes based on this data.
class SensorProcessor {
public:
  // Constructor - Initializes the SensorProcessor
  SensorProcessor();
  
  // --- Processing Methods ---
  void process_odom(const nav_msgs::msg::Odometry::SharedPtr scan_msg);      // Processes odometry (robot position) data

  // --- Getter Methods ---
  double get_robot_x_pos() const; 
  double get_robot_y_pos() const; 
  double get_robot_yaw() const;  

private:
  // --- Data ---
  double robot_x_pos_;  // Robot's x position
  double robot_y_pos_;  // Robot's y position
  double robot_yaw_;    // Robot's yaw rotation
};


#endif  // SENSOR_PROCESSOR_HPP_
