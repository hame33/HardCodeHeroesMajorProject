// --- Includes ---
#include "MotionController.hpp"
#include "Constants.hpp"

#include <iostream>

// --- MotionController Implementation ---

// --- Constructor ---
MotionController::MotionController(rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigator_client, 
                                   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_result_pub)
  : Node("goal_navigator_motion_controller"), navigator_client_(navigator_client), goal_result_pub_(goal_result_pub)
{
}

// --- goal_follower ---
void MotionController::goal_follower(geometry_msgs::msg::PoseStamped goal_to_follow)
{
  if (!navigator_client_) 
  {
    std::cout << "Navigator client is not initialized" << std::endl;
    return;
  }

  if (!navigator_client_->wait_for_action_server()) 
  {
    std::cout << "NavigateToPose action server is not available." << std::endl;
    return;
  }

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose = goal_to_follow;

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&MotionController::goal_result_callback, this, std::placeholders::_1);

  navigator_client_->async_send_goal(goal, send_goal_options);
}

// --- goal_result_callback ---
void MotionController::goal_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result)
{
  std_msgs::msg::String msg;

  switch (result.code) 
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      std::cout << "Goal reached successfully!" << std::endl;
      msg.data = "Success";
      goal_result_pub_->publish(msg);
      break;

    case rclcpp_action::ResultCode::ABORTED:
      std::cout << "Goal was aborted." << std::endl;
      msg.data = "Aborted";
      goal_result_pub_->publish(msg);
      break;

    case rclcpp_action::ResultCode::CANCELED:
      msg.data = "Canceled";
      goal_result_pub_->publish(msg);
      break;

    default:
      break;
  }
}
