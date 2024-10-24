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
#include "MapManager.hpp"
#include "Constants.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

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
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_; // Publisher for waypoints as markers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_; // Publisher for Nav2 goals

  // --- ROS Subscribers ---
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;     // Subscriber for LIDAR scan data
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;         // Subscriber for odometry data
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr ocp_grid_sub_;  // Subscriber to the occupancy grid topic 
  rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Result>::SharedPtr goal_result_sub_;  // Subscriber to the nav2 goal result client  //rclcpp::Service<>

  // --- Components ---
  std::shared_ptr<SensorProcessor> sensor_processor_;     // Sensor data processing component
  std::shared_ptr<WaypointGenerator> waypoint_generator_; // Waypoint generating component
  std::shared_ptr<WaypointManager> waypoint_manager_;     // Waypoint managing component
  std::shared_ptr<MapManager> map_manager_;  // Map Managing component

  // --- TF2 Components ---
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- ROS Timer ---
  rclcpp::TimerBase::SharedPtr update_timer_;  // Timer for periodic updates

  // --- Callback Functions ---
  void waypoint_callback();  // callback to handle publishing, creating, and managing waypoints
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg); // callback to handle scan data 
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg); // callback to handle odom data
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg);  // Handles map and occupancy grid data
  void goal_result_callback(const nav2_msgs::action::NavigateToPose::Result goal_result);  // Handles nav2 goal result data
};



#endif // INVENTORY_WAYPOINTS_HPP_