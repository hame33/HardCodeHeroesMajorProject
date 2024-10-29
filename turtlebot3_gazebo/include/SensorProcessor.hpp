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
// The SensorProcessor class is responsible for handling and processing sensor 
// data, particularly LIDAR scan data and odometry information. It includes 
// methods to parse and update this data, providing accurate and up-to-date 
// information on the robot's surroundings and pose. This class computes the 
// global coordinates of scan points and tracks the robot’s position and 
// orientation for use in navigation and decision-making.
class SensorProcessor {
public:
    // --- Constructor ---
    SensorProcessor();

    // --- SensorProcessor Functions ---

    // process_scan - Processes LIDAR scan data
    void process_scan(const sensor_msgs::msg::LaserScan::SharedPtr odom_msg);

    // process_odom - Processes odometry (robot position) data
    void process_odom(const nav_msgs::msg::Odometry::SharedPtr scan_msg);

    // --- Getter Methods ---
    std::array<double, Constants::NUM_SCAN_POSITIONS> get_scan_distance_data() const;
    std::array<std::pair<double, double>, Constants::NUM_SCAN_POSITIONS> get_scan_location_data() const;
    double get_robot_x_pos() const;
    double get_robot_y_pos() const;
    double get_robot_yaw() const;

private:
    // --- Scan Data ---
    std::array<double, Constants::NUM_SCAN_POSITIONS> scan_distance_data_;                     // Array storing raw LIDAR scan data (distance of scan from bot)
    std::array<std::pair<double, double>, Constants::NUM_SCAN_POSITIONS> scan_location_data_;  // Array storing processed scan data (x and y global coordinates per scan)

    // --- Data ---
    double robot_x_pos_;  // Robot's x position
    double robot_y_pos_;  // Robot's y position
    double robot_yaw_;    // Robot's yaw rotation

    // --- Helper Methods ---
    void calc_global_scan_coord(int num);  // Calculates global scan coordinates
};

#endif  // SENSOR_PROCESSOR_HPP_
