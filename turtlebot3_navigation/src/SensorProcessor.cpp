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
  robot_x_pos_ = 0.0;
  robot_y_pos_ = 0.0;
  robot_yaw_ = 0.0;
}

// --- process_odom ---
void SensorProcessor::process_odom(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  robot_x_pos_ = odom_msg->pose.pose.position.x;
  robot_y_pos_ = odom_msg->pose.pose.position.y; // On gazebo the x is vertical and the y is horizontal

  tf2::Quaternion q(
    odom_msg->pose.pose.orientation.x,
    odom_msg->pose.pose.orientation.y,
    odom_msg->pose.pose.orientation.z,
    odom_msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_yaw_ = yaw;
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

