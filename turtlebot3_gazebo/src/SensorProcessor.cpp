// --- Includes ---
#include "SensorProcessor.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <utility>

// --- SensorProcessor Implementation ---

// --- Constructor ---
SensorProcessor::SensorProcessor()
{
  scan_distance_data_.fill(0.0);
}

// --- process_odom ---
void SensorProcessor::process_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_pos_ = msg->pose.pose.position.x;
  robot_y_pos_ = msg->pose.pose.position.y; // On gazebo the x is vertical and the y is horizontal

  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_yaw_ = yaw;
}

// --- process_scan ---
void SensorProcessor::process_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  for (int num = 0; num < Constants::NUM_SCAN_POSITIONS; num++) 
  {
    if (std::isinf(msg->ranges.at(num * Constants::NUM_SCANNERS/Constants::NUM_SCAN_POSITIONS))) 
    {
      scan_distance_data_[num] = msg->range_max;
    } 
    else 
    {
      scan_distance_data_[num] = msg->ranges.at(num * Constants::NUM_SCANNERS/Constants::NUM_SCAN_POSITIONS);
    }
    calc_global_scan_coord(num);
  }
}

// --- get_scan_distance_data ---
std::array<double, Constants::NUM_SCAN_POSITIONS> SensorProcessor::get_scan_distance_data() const
{
  return scan_distance_data_;
}

// --- get_scan_location_data ---
std::array<std::pair<double, double>, Constants::NUM_SCAN_POSITIONS> SensorProcessor::get_scan_location_data() const
{
  return scan_location_data_;
}

// --- calc_global_scan_coord ---
void SensorProcessor::calc_global_scan_coord(int num)
{
    scan_location_data_[num].first = robot_x_pos_ + scan_distance_data_[num] * cos(robot_yaw_);
    scan_location_data_[num].second = robot_y_pos_ + scan_distance_data_[num] * sin(robot_yaw_);
}

