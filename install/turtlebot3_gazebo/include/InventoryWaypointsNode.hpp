#ifndef INVENTORY_WAYPOINTS_HPP_
#define INVENTORY_WAYPOINTS_HPP_

// --- Includes ---

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "SensorProcessor.hpp"
#include "WaypointGenerator.hpp"
#include "WaypointManager.hpp"
#include "Constants.hpp"

// --- InventoryWaypointsNode Class Interface ---
class InventoryWaypointsNode : public rclcpp::Node 
{
public:
  // Constructor - Initialises InventoryWaypointsNode
  InventoryWaypointsNode();

  // Destructor
  ~InventoryWaypointsNode();

private:
  // --- ROS Publishers ---
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub_;  // Publisher for waypoints

  // --- ROS Subscribers ---
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;     // Subscriber for LIDAR scan data
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;         // Subscriber for odometry data

  // --- Components ---
  std::shared_ptr<SensorProcessor> sensor_processor_;     // Sensor data processing component
  std::shared_ptr<WaypointGenerator> waypoint_generator_; // Waypoint generating component
  std::shared_ptr<WaypointManager> waypoint_manager_;     // Waypoint managing component

  // --- ROS Timer ---
  rclcpp::TimerBase::SharedPtr update_timer_;  // Timer for periodic updates

  // --- Callback Functions ---
  void waypoint_callback();  // callback to handle publishing, creating, and managing waypoints
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg); // callback to handle scan data 
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg); // callback to handle odom data
};



#endif // INVENTORY_WAYPOINTS_HPP_