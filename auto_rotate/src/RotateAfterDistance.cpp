#include "RotateAfterDistance.hpp"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

// --- Constructor ---
RotateAfterDistance::RotateAfterDistance()
: Node("rotate_after_distance")
{
  // Initialize subscribers and publishers
  goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "inventory_goals", 10, std::bind(&RotateAfterDistance::pose_callback, this, std::placeholders::_1));

  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&RotateAfterDistance::odom_callback, this, std::placeholders::_1));

  vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Initialize service client for managing nav2 lifecycle
  pause_nav2_client_ = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
    "/lifecycle_manager_navigation/manage_nodes");

  // Create timer for velocity control
  timer_ = this->create_wall_timer(
    100ms, std::bind(&RotateAfterDistance::control_loop, this));

  // Wait for nav2 lifecycle service to be available
  while (!pause_nav2_client_->wait_for_service(1s)) {
    RCLCPP_INFO(this->get_logger(), "Waiting for the nav2 lifecycle service...");
  }

  // Initialize action client for sending goals
  action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
}

// --- odom_callback ---
void RotateAfterDistance::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  if (!initial_position_set_) {
    initial_x_ = x;
    initial_y_ = y;
    initial_position_set_ = true;
    return;
  }

  double distance = std::hypot(x - initial_x_, y - initial_y_);
  current_distance_ += distance;

  if (!rotate_ && current_distance_ >= target_distance_) {
    rotate_ = true;
    RCLCPP_INFO(this->get_logger(), "Target distance reached. Pausing nav2 and starting rotation.");
    pause_nav2();
  }

  if (!rotate_) {
    initial_x_ = x;
    initial_y_ = y;
  }
}

// --- control_loop ---
void RotateAfterDistance::control_loop()
{
  auto vel_msg = geometry_msgs::msg::Twist();

  if (rotate_) {
    vel_msg.angular.z = rotation_speed_;

    if (rotation_time_ >= total_rotation_duration_) {
      RCLCPP_INFO(this->get_logger(), "Rotation complete. Resuming nav2.");
      rotate_ = false;
      current_distance_ = 0.0;
      rotation_time_ = 0.0;
      resume_nav2();
      send_goal(stored_pose_);
      return;
    }
    rotation_time_ += 0.1;
  }

  vel_publisher_->publish(vel_msg);
}

// --- pose_callback ---
void RotateAfterDistance::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  current_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(*pose);
  RCLCPP_INFO(this->get_logger(), "Received goal at position: x=%f, y=%f",
              pose->pose.position.x, pose->pose.position.y);
}

// --- pause_nav2 ---
void RotateAfterDistance::pause_nav2()
{
  if (!pause_nav2_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(this->get_logger(), "Service not available to pause nav2.");
    return;
  }

  stored_pose_ = current_pose_;
  auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
  request->command = nav2_msgs::srv::ManageLifecycleNodes::Request::PAUSE;
  pause_nav2_client_->async_send_request(request);

  RCLCPP_INFO(this->get_logger(), "Requested nav2 pause.");
}

// --- resume_nav2 ---
void RotateAfterDistance::resume_nav2()
{
  if (!pause_nav2_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(this->get_logger(), "Service not available to resume nav2.");
    return;
  }

  auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
  request->command = nav2_msgs::srv::ManageLifecycleNodes::Request::RESUME;
  pause_nav2_client_->async_send_request(request);

  RCLCPP_INFO(this->get_logger(), "Requested nav2 resume.");
}

// --- send_goal ---
void RotateAfterDistance::send_goal(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  if (!pose) {
    RCLCPP_WARN(this->get_logger(), "No goal available to resend.");
    return;
  }

  if (!action_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available.");
    return;
  }

  auto goal = NavigateToPose::Goal();
  goal.pose = *pose;
  action_client_->async_send_goal(goal);

  RCLCPP_INFO(this->get_logger(), "Goal resent to position: x=%f, y=%f",
              pose->pose.position.x, pose->pose.position.y);
}
