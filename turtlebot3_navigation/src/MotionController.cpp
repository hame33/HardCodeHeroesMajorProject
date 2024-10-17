// --- Includes ---
#include "MotionController.hpp"
#include "Constants.hpp"

#include <iostream>

// --- MotionController Implementation ---

// --- Constructor ---
MotionController::MotionController(rclcpp_action::Client<FollowWaypoints>::SharedPtr goal_client)
: goal_client_(goal_client)
{

}



// --- goal_follower ---
MotionController::goal_follower(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
{
  if (!goal_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server is not available.");
    return;
  }

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose = *goal_msg;

  std::cout << "Navigating to goal @ x: " << goal_msg->x << " y: " << std::endl;
  goal_client_->async_send_goal(goal);
}
