#include <chrono>
#include <cmath>
#include <memory>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class UserNavigation : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  UserNavigation() : Node("user_navigation")
  {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    while (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for navigation server...");
    }

    RCLCPP_INFO(this->get_logger(), "Navigation server is ready.");
    start_navigation();
  }

private:
  void start_navigation()
  {
    double goal_x, goal_y;
    std::cout << "Enter goal position (X, Y): ";
    std::cin >> goal_x >> goal_y;

    auto goal_msg = create_goal(goal_x, goal_y);
    send_goal(goal_msg);
  }

  geometry_msgs::msg::PoseStamped create_goal(double x, double y)
  {
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = this->now();
    goal_msg.pose.position.x = x;
    goal_msg.pose.position.y = y;
    goal_msg.pose.orientation.w = 1.0;
    return goal_msg;
  }

  void send_goal(const geometry_msgs::msg::PoseStamped &goal_msg)
  {
    auto goal = NavigateToPose::Goal();
    goal.pose = goal_msg;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback =
      std::bind(&UserNavigation::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&UserNavigation::result_callback, this, std::placeholders::_1);

    RCLCPP_INFO(this->get_logger(), "Sending goal: X=%f, Y=%f", goal_msg.pose.position.x, goal_msg.pose.position.y);
    action_client_->async_send_goal(goal, send_goal_options);
  }

  void feedback_callback(
    rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Distance to goal: %.2f meters", feedback->distance_remaining);
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal was canceled.");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal failed.");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown error occurred.");
        break;
    }
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UserNavigation>());
  rclcpp::shutdown();
  return 0;
}
