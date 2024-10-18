#ifndef WAYPOINT_FOLLOWER_NODE_HPP_
#define WAYPOINT_FOLLOWER_NODE_HPP_

// --- Includes ---
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "MotionController.hpp"
#include "MapManager.hpp"
#include "SensorProcessor.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


// --- GoalNavigatorNode Class Interface ---
// The GoalNavigatorNode class is a ROS2 node responsible for managing the TurtleBot3's
// driving behavior. It subscribes to sensor data (LIDAR and odometry), processes it, 
// and publishes velocity commands to control the robot. It also manages internal components 
// such as the sensor processor, motion controller, and state machine.
class GoalNavigatorNode : public rclcpp::Node {
public:
  // Constructor - Initializes the GoalNavigatorNode
  GoalNavigatorNode();

  // Destructor - Cleans up resources used by the GoalNavigatorNode
  ~GoalNavigatorNode();

  // get_goal_to_follow
  geometry_msgs::msg::PoseStamped get_goal_to_follow();

private:
  // --- ROS Subscribers ---
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;  // Subscriber to the inventory goals topic (PoseStamped messages)
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr ocp_grid_sub_;  // Subscriber to the occupancy grid topic 
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;         // Subscriber for odometry data

  // --- ROS Action Clients ---
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigator_client_;  // Action client for following nav2 goals

  // --- ROS Timer ---
  rclcpp::TimerBase::SharedPtr update_timer_;  // Timer for periodic updates

  // --- Components ---
  std::shared_ptr<MotionController> motion_controller_;   // Motion control component
  std::shared_ptr<MapManager> map_manager_;  // Map, occupancy grid, coordinates managing component
  std::shared_ptr<SensorProcessor> sensor_processor_;     // Sensor data processing component

  // --- TF2 Components ---
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- Data ---
  geometry_msgs::msg::PoseStamped goal_to_follow_;  // Stores location of the to be navigated to

  // --- Callback Functions ---
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg);  // Handles inventory goals messages (PoseStamped messages)
  void navigation_callback(); // Handles navigation to goal action
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg); // callback to handle odom data
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg);  // Handles map and occupancy grid data
};

#endif  // WAYPOINT_FOLLOWER_NODE_HPP
