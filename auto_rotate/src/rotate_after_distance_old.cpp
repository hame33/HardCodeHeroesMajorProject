#include <memory>
#include <cmath>
#include <chrono>
#include <functional>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>

class RotateAfterDistance : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using ClientGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  RotateAfterDistance();

private:
  void check_distance();
  void rotate_360();
  void cancel_navigation_goal();
  void resend_navigation_goal();
  void handle_goal_response(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);
  void on_new_goal(const nav2_msgs::action::NavigateToPose::Goal::SharedPtr goal, const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle);
  void nav2_goal_callback(const nav2_msgs::msg::NavigateToPoseActionGoal::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Variables
  double traveled_distance_;
  bool rotate_;
  double target_distance_;
  double last_x_;
  double last_y_;

  // ROS 2 Components
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_action_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<nav2_msgs::msg::NavigateToPoseActionGoal>::SharedPtr nav2_goal_subscription_;

  // Store current navigation goal and its handle
  nav2_msgs::action::NavigateToPose::Goal::SharedPtr current_goal_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr current_goal_handle_;
};

RotateAfterDistance::RotateAfterDistance()
  : Node("rotate_after_distance"), traveled_distance_(0.0), rotate_(false), target_distance_(5.0), last_x_(0.0), last_y_(0.0)
{
  // Initialize the action client for navigation
  nav2_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

  nav2_goal_subscription_ = this->create_subscription<nav2_msgs::msg::NavigateToPoseActionGoal>(
    "/navigate_to_pose/_action/send_goal", 10, std::bind(&RotateAfterDistance::nav2_goal_callback, this, std::placeholders::_1));

  // Create a publisher for velocity commands
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Create a subscriber for odometry
  odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&RotateAfterDistance::odometry_callback, this, std::placeholders::_1));

  // Create a timer to check distance periodically
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&RotateAfterDistance::check_distance, this));
}

void RotateAfterDistance::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Get the current position from the odometry message
  double current_x = msg->pose.pose.position.x;
  double current_y = msg->pose.pose.position.y;

  // Calculate the distance traveled since the last odometry update
  double distance = std::hypot(current_x - last_x_, current_y - last_y_);
  traveled_distance_ += distance;

  // Update the last known position
  last_x_ = current_x;
  last_y_ = current_y;
}

void RotateAfterDistance::check_distance()
{
  if (traveled_distance_ >= target_distance_ && !rotate_) {
    RCLCPP_INFO(this->get_logger(), "Target distance reached. Canceling navigation task and starting rotation.");

    // Cancel the current navigation goal
    cancel_navigation_goal();

    // Set rotation flag to true to start the rotation
    rotate_ = true;

    // Start rotating
    rotate_360();
  }
}

void RotateAfterDistance::rotate_360()
{
  // Logic to rotate 360 degrees
  RCLCPP_INFO(this->get_logger(), "Performing 360-degree rotation...");

  // Simulate a 360-degree rotation with cmd_vel
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.angular.z = 0.5;  // Set angular velocity for rotation

  auto start_time = this->now();
  while ((this->now() - start_time).seconds() < (2 * M_PI / twist_msg.angular.z)) {
    cmd_vel_publisher_->publish(twist_msg);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  // Stop rotation
  twist_msg.angular.z = 0.0;
  cmd_vel_publisher_->publish(twist_msg);

  RCLCPP_INFO(this->get_logger(), "Rotation complete. Resending the original navigation goal.");
  
  // Resend the original navigation goal
  resend_navigation_goal();

  // Reset the rotation flag
  rotate_ = false;
}

void RotateAfterDistance::cancel_navigation_goal()
{
  if (!current_goal_handle_) {
    RCLCPP_WARN(this->get_logger(), "No active navigation goal to cancel.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Canceling the current navigation goal...");

  // Cancel the goal
  auto cancel_future = nav2_action_client_->async_cancel_goal(current_goal_handle_);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), cancel_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to cancel the navigation goal.");
    return;
  }

  // Reset the goal handle
  current_goal_handle_ = nullptr;
}

void RotateAfterDistance::resend_navigation_goal()
{
  if (!current_goal_) {
    RCLCPP_ERROR(this->get_logger(), "No previous goal stored to resend");
    return;
  }

  if (!nav2_action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Navigation action server is not available.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Resending the navigation goal...");

  // Set options for handling the goal response
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions options;
  options.result_callback = std::bind(&RotateAfterDistance::handle_goal_response, this, std::placeholders::_1);

  // Send the stored goal to the nav2 action server
  auto goal_handle_future = nav2_action_client_->async_send_goal(*current_goal_, options);

  // Wait for the goal handle future to complete and extract the goal handle
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send goal.");
    return;
  }

  // Now we can get the goal handle from the future
  auto goal_handle = goal_handle_future.get();
  if (goal_handle) {
    current_goal_handle_ = goal_handle;
    RCLCPP_INFO(this->get_logger(), "Goal sent successfully.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to send goal.");
  }
}

void RotateAfterDistance::handle_goal_response(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
{
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "Navigation goal completed successfully.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Navigation goal failed.");
  }
}

void RotateAfterDistance::on_new_goal(const nav2_msgs::action::NavigateToPose::Goal::SharedPtr goal, const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received new navigation goal.");
  
  // Store the current goal for later
  current_goal_ = std::make_shared<nav2_msgs::action::NavigateToPose::Goal>(*goal);
  current_goal_handle_ = goal_handle;  // Save the new goal handle
}

void RotateAfterDistance::nav2_goal_callback(const nav2_msgs::msg::NavigateToPoseActionGoal::SharedPtr msg)

{

  RCLCPP_INFO(this->get_logger(), "New Nav2 goal received");

  // Access the goal pose
  const auto& pose = msg->goal.pose;

  RCLCPP_INFO(this->get_logger(), "Goal position: x=%f, y=%f, z=%f",
              pose.position.x, pose.position.y, pose.position.z);

  // Here you can implement your custom logic when a new goal is set
  // For example, you might want to reset your distance tracking:
  traveled_distance_ = 0.0;
  // You can also store the goal for later use if needed
  current_goal_ = msg->goal;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RotateAfterDistance>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
