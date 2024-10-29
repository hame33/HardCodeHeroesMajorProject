// --- Includes ---
#include "SensorProcessor.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <utility>
#include <iostream>

// --- SensorProcessor Implementation ---

// --- Constructor ---
SensorProcessor::SensorProcessor()
{
  scan_distance_data_.fill(0.0);
  scan_location_data_.fill(std::make_pair(0.0, 0.0));
  robot_x_pos_ = 0.0;
  robot_y_pos_ = 0.0;
  robot_yaw_ = 0.0;
}

// --- process_odom ---
void SensorProcessor::process_odom(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  robot_x_pos_ = odom_msg->pose.pose.position.x;
  robot_y_pos_ = odom_msg->pose.pose.position.y;

  tf2::Quaternion q(
    odom_msg->pose.pose.orientation.x,
    odom_msg->pose.pose.orientation.y,
    odom_msg->pose.pose.orientation.z,
    odom_msg->pose.pose.orientation.w
  );
  
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_yaw_ = yaw;
}

// --- process_scan ---
void SensorProcessor::process_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  for (int num = 0; num < Constants::NUM_SCAN_POSITIONS; num++) 
  {
    if (std::isinf(scan_msg->ranges.at(num * Constants::ANGLE_BETWEEN_SCANS))) 
    {
      scan_distance_data_[num] = scan_msg->range_max;
    } 
    else 
    {
      scan_distance_data_[num] = scan_msg->ranges.at(num * Constants::ANGLE_BETWEEN_SCANS);
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

// --- get_robot_x_pos ---
double SensorProcessor::get_robot_x_pos() const 
{
  return robot_x_pos_;
}

// --- get_robot_y_pos ---
double SensorProcessor::get_robot_y_pos() const 
{
  return robot_y_pos_;
}

// --- get_robot_yaw ---
double SensorProcessor::get_robot_yaw() const 
{
  return robot_yaw_;
}

// --- calc_global_scan_coord ---
void SensorProcessor::calc_global_scan_coord(int num)
{
  scan_location_data_[num].first = robot_x_pos_ + scan_distance_data_[num] * 
    (cos(robot_yaw_ + (num * Constants::ANGLE_BETWEEN_SCANS) * Constants::DEG2RAD));
  
  scan_location_data_[num].second = robot_y_pos_ + scan_distance_data_[num] *
    (sin(robot_yaw_ + (num * Constants::ANGLE_BETWEEN_SCANS) * Constants::DEG2RAD));
}
