// --- Includes ---
#include "MotionController.hpp"
#include "Constants.hpp"

#include <iostream>

// --- MotionController Implementation ---

// --- Constructor ---
MotionController::MotionController(rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigator_client)
: navigator_client_(navigator_client)
{

}



// --- goal_follower ---
void MotionController::goal_follower(geometry_msgs::msg::PoseStamped goal_to_follow)
{  
  if (!navigator_client_) {
    std::cout << "Navigator client is not initialized" << std::endl;
    return;
  }

  if (!navigator_client_->wait_for_action_server()) {
    std::cout << "NavigateToPose action server is not available." << std::endl;
    return;
  }

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose = goal_to_follow;

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  auto goal_handle_future = navigator_client_->async_send_goal(goal, send_goal_options);
  //std::cout << "Navigating to goal @ x: " << goal_to_follow.pose.position.x << " y: " << goal_to_follow.pose.position.y << std::endl;

  // auto goal_handle = goal_handle_future.get();

  // if (!goal_handle) {
  //     std::cout << "Goal was rejected by the server" << std::endl;
  //     return;
  // }

  // std::cout << "Goal accepted" << std::endl;

  // auto result_future = navigator_client_->async_get_result(goal_handle);
  // auto result_value = result_future.get();

  // if (result_value.result) {
  //     std::cout << "Result: " << result_value.result << std::endl;
  // } else {
  //     std::cout << "Failed to get result" << std::endl;
  // }

}
