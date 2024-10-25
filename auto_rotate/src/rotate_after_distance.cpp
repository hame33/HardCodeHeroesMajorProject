#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"

class RotateAfterDistance : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  RotateAfterDistance() : Node("rotate_after_distance"), rotate_(false), initial_position_set_(false)
  {
    // Goal subscriber to save goals
    goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "inventory_goals", 10, std::bind(&RotateAfterDistance::pose_callback, this, std::placeholders::_1));

    // Create the action client for sending navigation goals
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Service clients for pausing and resuming nav2
    pause_nav2_client_ = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_navigation/manage_nodes");

    // Subscribe to odometry to get distance information
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&RotateAfterDistance::odom_callback, this, std::placeholders::_1));

    // Publisher to send velocity commands
    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Set timer for publishing velocity commands
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&RotateAfterDistance::control_loop, this));

    // Wait for the service clients to become available
    while (!pause_nav2_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for the nav2 manage lifecycle service to become available...");
    }
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Get current position
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    // Initialize the initial position if it hasn't been set yet
    if (!initial_position_set_)
    {
      initial_x_ = x;
      initial_y_ = y;
      initial_position_set_ = true;
      return;  // Return early to prevent distance calculation in the first callback
    }

    // Calculate distance traveled since start (hypotenuse of (x, y))
    double distance = std::hypot(x - initial_x_, y - initial_y_);
    current_distance_ += distance;

    // If we're not rotating yet and distance exceeds target, start rotating
    if (!rotate_ && current_distance_ >= target_distance_)
    {
      rotate_ = true;
      RCLCPP_INFO(this->get_logger(), "Target distance reached. Pausing nav2 and starting rotation.");
      pause_nav2();  // Pause nav2 before starting the rotation
    }

    // Save current position for next callback
    if (!rotate_) {
      initial_x_ = x;
      initial_y_ = y;
    }
  }

  void control_loop()
  {
    auto vel_msg = geometry_msgs::msg::Twist();

    if (rotate_)
    {
      // Perform the rotation
      vel_msg.angular.z = rotation_speed_;

      // Logic to determine if rotation is complete
      if (rotation_time_ >= total_rotation_duration_) {
        RCLCPP_INFO(this->get_logger(), "Rotation complete. Resuming nav2.");
        rotate_ = false;
        current_distance_ = 0.0;
        rotation_time_ = 0.0;
        resume_nav2();  // Resume nav2 after rotation completes
        send_goal(stored_pose_);
        return;
      }
      rotation_time_ += 0.1;  // Simulate time tracking
    }

    // Publish velocity command
    vel_publisher_->publish(vel_msg);
  }

  // Callback to receive new goals
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
  {
    current_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(*pose);

    RCLCPP_INFO(this->get_logger(), "Callback on goal with position: x=%f, y=%f", 
            pose->pose.position.x, pose->pose.  position.y);
  }

  void pause_nav2()
  {
    if (!pause_nav2_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "Service /lifecycle_manager_navigation/manage_nodes not available");
      return;
    }

    stored_pose_ = current_pose_;

    auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    request->command = nav2_msgs::srv::ManageLifecycleNodes::Request::PAUSE;

    auto future_result = pause_nav2_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Sent request to pause nav2...");
  }

  void resume_nav2()
  {
    if (!pause_nav2_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "Service /lifecycle_manager_navigation/manage_nodes not available");
      return;
    }

    // Create a request to activate the nav2 nodes
    auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    request->command = nav2_msgs::srv::ManageLifecycleNodes::Request::RESUME;

    auto future_result = pause_nav2_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Sent request to resume nav2...");
  }

  // Function to send a goal via the action client
  void send_goal(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
  {
    if (!pose) {
      RCLCPP_WARN(this->get_logger(), "No goal available to resend.");
      return;
    }
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Sending goal with position: x=%f, y=%f", 
            pose->pose.position.x, pose->pose.position.y);

    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose = *pose;

    auto goal_handle_future = action_client_->async_send_goal(goal);
    RCLCPP_INFO(this->get_logger(), "Goal prior to pause resent.");
  }

  // Subscribers, publishers, and service clients
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr pause_nav2_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  // Store a goal to resend if paused
  geometry_msgs::msg::PoseStamped::SharedPtr stored_pose_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;

  // Distance and rotation control
  double initial_x_ = 0.0, initial_y_ = 0.0;
  bool rotate_;

  bool initial_position_set_;

  // Parameters
  double current_distance_ = 0.0;
  double target_distance_ = 1.0;
  double rotation_speed_ = 1.0;
  double rotation_time_ = 0.0;
  double total_rotation_duration_ = 2 * M_PI / rotation_speed_;  // Time for a 360-degree rotation
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RotateAfterDistance>());
  rclcpp::shutdown();
  return 0;
}
