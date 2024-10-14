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
  void process_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);  // Processes LIDAR scan data
  void process_odom(const nav_msgs::msg::Odometry::SharedPtr msg);      // Processes odometry (robot position) data

  // --- Getter Methods ---
  std::array<double, Constants::NUM_SCAN_POSITIONS> get_scan_distance_data() const;  // Returns LIDAR scan distance data
  std::array<std::pair<double, double>, Constants::NUM_SCAN_POSITIONS> get_scan_location_data() const; // Returns LIDAR scan location data

private:
  // --- Scan Data ---
  std::array<double, Constants::NUM_SCAN_POSITIONS> scan_distance_data_;  // Array storing raw LIDAR scan data (distance of scan from bot)
  std::array<std::pair<double, double>, Constants::NUM_SCAN_POSITIONS> scan_location_data_; // Array storing processed scan data (x and y global coordinates per scan)

  // --- Data ---
  double robot_x_pos_;  // Robot's x position
  double robot_y_pos_;  // Robot's y position
  double robot_yaw_;    // Robot's yaw rotation

  // --- Helper Methods ---
  void calc_global_scan_coord(int num);
};


#endif  // SENSOR_PROCESSOR_HPP_
