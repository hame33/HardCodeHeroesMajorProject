// GoalNavigatorNode.cpp

// --- Includes ---
#include "GoalNavigatorNode.hpp"
#include "Constants.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>

using namespace std::chrono_literals;

// --- GoalNavigatorNode Implementation ---

// --- Constructor ---
GoalNavigatorNode::GoalNavigatorNode()
: Node("goal_navigator")
{
  // Generalised Qos to be used
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise clients
  navigator_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

  // Initialise TF2 components
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize components
  sensor_processor_ = std::make_shared<SensorProcessor>();
  motion_controller_ = std::make_shared<MotionController>(navigator_client_);
  // map_manager_ = std::make_shared<MapManager>(tf_buffer_, tf_listener_, sensor_processor_);
  map_manager_ = std::make_shared<MapManager>(sensor_processor_);

  // Initialize subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom",
    qos,
    std::bind(&GoalNavigatorNode::odom_callback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "inventory_goals",
    qos,
    std::bind(&GoalNavigatorNode::goal_callback, this, std::placeholders::_1));

  ocp_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", 
    qos, 
    std::bind(&GoalNavigatorNode::map_callback, this, std::placeholders::_1));
    

  // Initialize timer
  update_timer_ = this->create_wall_timer(
    10ms,
    std::bind(&GoalNavigatorNode::navigation_callback, this));

  RCLCPP_INFO(this->get_logger(), "GoalNavigatorNode has been initialized");

  // Initialise data
  geometry_msgs::msg::PoseStamped goal_to_follow_;
  goal_to_follow_.header.frame_id = "map";
  goal_to_follow_.header.stamp = rclcpp::Clock().now();
  goal_to_follow_.pose.position.x = 0.0;
  goal_to_follow_.pose.position.y = 0.0;
  goal_to_follow_.pose.position.z = 0.0;
  goal_to_follow_.pose.orientation = geometry_msgs::msg::Quaternion();
  goal_to_follow_.pose.orientation.x = 0.0;
  goal_to_follow_.pose.orientation.y = 0.0;
  goal_to_follow_.pose.orientation.z = 0.0;
  goal_to_follow_.pose.orientation.w = 1.0;
};

// --- Destructor ---
GoalNavigatorNode::~GoalNavigatorNode()
{
  RCLCPP_INFO(this->get_logger(), "GoalNavigatorNode has been terminated");
}

// --- goal_callback ---
void GoalNavigatorNode::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
{
  goal_to_follow_ = *goal_msg;
}

// --- navigation_callback ---
void GoalNavigatorNode::navigation_callback()
{
  geometry_msgs::msg::PoseStamped goal_to_follow = get_goal_to_follow();

  motion_controller_->goal_follower(goal_to_follow);
}

// --- odom_callback ---
void GoalNavigatorNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  sensor_processor_->process_odom(odom_msg);
}

// --- map_callback ---
void GoalNavigatorNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg)
{
  map_manager_->process_map_data(ocp_grid_msg);
}

// --- get_goal_to_follow ---
geometry_msgs::msg::PoseStamped GoalNavigatorNode::get_goal_to_follow()
{
  return goal_to_follow_;
}
