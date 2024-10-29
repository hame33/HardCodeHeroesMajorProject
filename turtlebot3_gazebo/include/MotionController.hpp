#ifndef MOTION_CONTROLLER_HPP_
#define MOTION_CONTROLLER_HPP_

// --- Includes ---
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

// --- MotionController Class Interface ---
// The MotionController class manages the robot's movement by calculating turn angles 
// and sending velocity commands based on goal navigation. It leverages ROS action 
// clients to handle navigation goals and publishes movement commands to drive the 
// robot autonomously. Callback functions allow it to handle the results of navigation 
// goals, making it essential for waypoint following and directional control in response 
// to dynamic navigation data.
class MotionController : public rclcpp::Node 
{
public:
  // --- Constructor ---
  MotionController(rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigator_client,
                   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_result_pub);

  // --- MotionController Functions ---

  // goal_follower - actions goal navigator
  void goal_follower(geometry_msgs::msg::PoseStamped goal_to_follow);  

private:
  // --- ROS Action Clients ---
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigator_client_;  // Pointer to goal follower action client

  // --- ROS Publishers ---
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_result_pub_;  // Publisher for goal results
  
  // --- Callback Functions ---
  void goal_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result);  // Processes the result of navigation goals
};

#endif  // MOTION_CONTROLLER_HPP_
