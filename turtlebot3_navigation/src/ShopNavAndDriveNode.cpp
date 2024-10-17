// WaypointFollowerNode.cpp

// --- Includes ---
#include "WaypointFollowerNode.hpp"
#include "Constants.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>

using namespace std::chrono_literals;

// --- WaypointFollowerNode Implementation ---

// --- Constructor ---
WaypointFollowerNode::WaypointFollowerNode()
: Node("shop_nav_and_drive_node")
{
  // Generalised Qos to be used
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialize components
  motion_controller_ = std::make_shared<MotionController>(goal_client_);

  // Initialize subscribers
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "inventory_goals"
    qos,
    std::bind(&WaypointFollowerNode::goal_callback, this, std::placeholders::_1));

  // Initialise clients
  goal_client_ = rclcpp_action::create_client<nav2_msgs::action::Navigate2Pose>(this, "navigate_to_goal")

  // Initialize timer
  update_timer_ = this->create_wall_timer(
    10ms,
    std::bind(&WaypointFollowerNode::goal_callback, this));

  RCLCPP_INFO(this->get_logger(), "WaypointFollowerNode has been initialized");
}

// --- Destructor ---
WaypointFollowerNode::~WaypointFollowerNode()
{
  RCLCPP_INFO(this->get_logger(), "WaypointFollowerNode has been terminated");
}

// --- goal_callback ---
WaypointFollowerNode::goal_callback()
{
  motion_controller_->goal_follower();
}
