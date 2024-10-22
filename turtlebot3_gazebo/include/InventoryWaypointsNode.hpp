#ifndef INVENTORY_WAYPOINTS_HPP_
#define INVENTORY_WAYPOINTS_HPP_

// --- Includes ---

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "SensorProcessor.hpp"
#include "WaypointGenerator.hpp"
#include "WaypointManager.hpp"
#include "Constants.hpp"

// --- InventoryWaypointsNode Class Interface ---
class InventoryWaypointsNode
{
public:
  // Constructor that accepts a shared_ptr to rclcpp::Node
  InventoryWaypointsNode(rclcpp::Node::SharedPtr node);

  void start_navigation();
  void stop_navigation();

  // Destructor
  ~InventoryWaypointsNode();

private:
  // --- ROS Publishers ---
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub_;  // Publisher for waypoints
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_; // Publisher for waypoints as markers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_; // Publisher for Nav2 goals

  // --- ROS Subscribers ---
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;     // Subscriber for LIDAR scan data
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;         // Subscriber for odometry data

  // --- Components ---
  rclcpp::Node::SharedPtr node_;
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