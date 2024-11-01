#ifndef ROTATE_AFTER_DISTANCE_HPP
#define ROTATE_AFTER_DISTANCE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>

class RotateAfterDistance : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  RotateAfterDistance();
  ~RotateAfterDistance() = default;

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void control_loop();
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
  void pause_nav2();
  void resume_nav2();
  void send_goal(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

  // Subscribers, publishers, and service clients
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr pause_nav2_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  // Distance and rotation control
  double initial_x_ = 0.0, initial_y_ = 0.0;
  double current_distance_ = 0.0;
  double target_distance_ = 1.0;
  double rotation_speed_ = 1.0;
  double rotation_time_ = 0.0;
  double total_rotation_duration_ = 2 * M_PI / rotation_speed_;
  bool rotate_ = false;
  bool initial_position_set_ = false;

  // Store goal for resending
  geometry_msgs::msg::PoseStamped::SharedPtr stored_pose_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
};

#endif // ROTATE_AFTER_DISTANCE_HPP
