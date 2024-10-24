#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"

class RotateAfterDistance : public rclcpp::Node
{
public:
  RotateAfterDistance()
  : Node("rotate_after_distance"), rotate_(false), initial_position_set_(false)
  {
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

    // Parameter for distance before rotating (can also be set dynamically)
    this->declare_parameter<double>("target_distance", 2.0);  // Default distance: 2 meters
    this->declare_parameter<double>("rotation_speed", 0.5);   // Rotation speed: 0.5 rad/s

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

    // If we're not rotating yet and distance exceeds target, start rotating
    if (!rotate_ && distance >= target_distance_)
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
      RCLCPP_INFO(this->get_logger(), "Sending rotaton message");

      // Logic to determine if rotation is complete
      if (rotation_time_ >= total_rotation_duration_) {
        RCLCPP_INFO(this->get_logger(), "Rotation complete. Resuming nav2.");
        rotate_ = false;
        resume_nav2();  // Resume nav2 after rotation completes
        return;
      }
      rotation_time_ += 0.1;  // Simulate time tracking
    }

    // Publish velocity command
    vel_publisher_->publish(vel_msg);
  }

  void pause_nav2()
  {
    if (!pause_nav2_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "Service /lifecycle_manager_navigation/manage_nodes not available");
      return;
    }

    auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    request->command = 2;  // 2 is the command to deactivate nodes

    auto future_result = pause_nav2_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Sent request to pause nav2...");

    try {
      auto response = future_result.get();  // This will block until the result is ready
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Successfully paused nav2.");
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to pause nav2.");
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Service call to pause nav2 failed: %s", e.what());
    }
  }

  void resume_nav2()
  {
    // Create a request to activate the nav2 nodes
    auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    request->command = 1;  // 1 is the command to activate nodes

    auto future_result = pause_nav2_client_->async_send_request(request);
    try {
      auto response = future_result.get();
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Successfully resumed nav2.");
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to resume nav2.");
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Service call to resume nav2 failed: %s", e.what());
    }
  }

  // Subscribers, publishers, and service clients
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr pause_nav2_client_;

  // Distance and rotation control
  double initial_x_ = 0.0, initial_y_ = 0.0;
  bool rotate_;
  bool initial_position_set_;
  double target_distance_;
  double rotation_speed_;
  double rotation_time_ = 0.0;
  double total_rotation_duration_ = 6.28 / 0.5;  // Time for a 360-degree rotation at 0.5 rad/s
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RotateAfterDistance>());
  rclcpp::shutdown();
  return 0;
}
