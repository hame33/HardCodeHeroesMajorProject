// MotionController.hpp

#ifndef MOTION_CONTROLLER_HPP_
#define MOTION_CONTROLLER_HPP_

// --- Includes ---

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "SensorProcessor.hpp"

// --- MotionController Class Interface ---
// The MotionController class manages the robot's movement, using sensor data 
// to calculate turns and publish velocity commands. It interfaces with ROS to send 
// movement commands and handles the logic for proportional and derivative turns.
class MotionController {
public:
  // Constructor - Initializes the controller with a velocity publisher and sensor processor
  MotionController(rclcpp_action::Client<FollowWaypoints>::SharedPtr goal_follower_client);

  // goal_follower
  void goal_follower(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg); // Actions waypoint follower
private:
  // --- ROS Action Clients ---
  rclcpp_action::Client<FollowWaypoints>::SharedPtr goal_client_; // Pointer to goal follower action client
};

#endif  // MOTION_CONTROLLER_HPP_
