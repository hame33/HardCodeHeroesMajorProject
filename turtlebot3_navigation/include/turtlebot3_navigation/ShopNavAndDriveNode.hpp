#ifndef TURTLEBOT3_DRIVE_NODE_HPP_
#define TURTLEBOT3_DRIVE_NODE_HPP_

// --- Includes ---
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav2_msgs/action/follow_waypoint.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "SensorProcessor.hpp"
#include "MotionController.hpp"
#include "StateMachine.hpp"

// --- WaypointFollowerNode Class Interface ---
// The WaypointFollowerNode class is a ROS2 node responsible for managing the TurtleBot3's
// driving behavior. It subscribes to sensor data (LIDAR and odometry), processes it, 
// and publishes velocity commands to control the robot. It also manages internal components 
// such as the sensor processor, motion controller, and state machine.
class WaypointFollowerNode : public rclcpp::Node {
public:
  // Constructor - Initializes the WaypointFollowerNode
  WaypointFollowerNode();

  // Destructor - Cleans up resources used by the WaypointFollowerNode
  ~WaypointFollowerNode();

private:
  // --- ROS Subscribers ---
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>SharedPtr goal_sub_;  // Subscriber to the inventory goals topic (PoseStamped messages)

  // --- ROS Action Clients ---
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose> goal_client_;  // Action client for following nav2 goals

  // --- ROS Timer ---
  rclcpp::TimerBase::SharedPtr update_timer_;  // Timer for periodic updates

  // --- Components ---
  std::shared_ptr<MotionController> motion_controller_;   // Motion control component

  // --- Callback Functions ---
  void goal_callback();  // Handles waypoint following action
};

#endif  // TURTLEBOT3_DRIVE_NODE_HPP_
