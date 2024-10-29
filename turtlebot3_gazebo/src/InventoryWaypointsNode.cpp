#include "InventoryWaypointsNode.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>
#include <iostream>
#include <utility>

using namespace std::chrono_literals;

// --- InventoryWaypointsNode Implementation ---

// --- Constructor ---
InventoryWaypointsNode::InventoryWaypointsNode(rclcpp::Node::SharedPtr node)
: node_(node)
{
  // Initialize QoS settings
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  std::cout << "QoS defined" << std::endl;

  // Initialize TF2 components for coordinate transforms
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize action client for navigation
  navigator_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");

  // Initialize components
  motion_controller_ = std::make_shared<MotionController>(
    navigator_client_,
    node_->create_publisher<std_msgs::msg::String>("goal_result", qos)
  );
  sensor_processor_ = std::make_shared<SensorProcessor>();
  waypoint_manager_ = std::make_shared<WaypointManager>(
    nullptr, // Initially pass nullptr for map_manager_
    motion_controller_,
    node_->create_publisher<geometry_msgs::msg::Point>("inventory_waypoints", qos),
    node_->create_publisher<visualization_msgs::msg::Marker>("waypoint_markers", qos),
    node_->create_publisher<geometry_msgs::msg::PoseStamped>("inventory_goals", qos)
  );

  // Set up map manager with waypoint manager
  map_manager_ = std::make_shared<MapManager>(sensor_processor_, waypoint_manager_);
  waypoint_manager_->set_map_manager(map_manager_);

  waypoint_generator_ = std::make_shared<WaypointGenerator>(sensor_processor_, waypoint_manager_);

  std::cout << "Node class components initialized" << std::endl;

  // Initialize Subscribers
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan",
    rclcpp::SensorDataQoS(),
    std::bind(&InventoryWaypointsNode::scan_callback, this, std::placeholders::_1)
  );

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom",
    qos,
    std::bind(&InventoryWaypointsNode::odom_callback, this, std::placeholders::_1)
  );

  ocp_grid_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map",
    qos,
    std::bind(&InventoryWaypointsNode::map_callback, this, std::placeholders::_1)
  );

  goal_result_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "goal_result",
    qos,
    std::bind(&InventoryWaypointsNode::goal_result_callback, this, std::placeholders::_1)
  );

  // Initialize timer for periodic updates
  update_timer_ = node_->create_wall_timer(
    10ms,
    std::bind(&InventoryWaypointsNode::waypoint_callback, this)
  );

  RCLCPP_INFO(node_->get_logger(), "InventoryWaypointsNode has been initialized");
}

// --- Destructor ---
InventoryWaypointsNode::~InventoryWaypointsNode()
{
  RCLCPP_INFO(node_->get_logger(), "InventoryWaypointsNode has been terminated");
}

// --- waypoint_callback ---
void InventoryWaypointsNode::waypoint_callback()
{
  // Callback to manage waypoint updates
}

// --- scan_callback ---
void InventoryWaypointsNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  sensor_processor_->process_scan(scan_msg); // Process incoming LIDAR scan data
}

// --- odom_callback ---
void InventoryWaypointsNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  sensor_processor_->process_odom(odom_msg); // Process incoming odometry data
}

// --- map_callback ---
void InventoryWaypointsNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg)
{
  map_manager_->process_map_data(ocp_grid_msg); // Process occupancy grid map data
  waypoint_manager_->publish_goal();            // Publish a new navigation goal
}

// --- goal_result_callback ---
void InventoryWaypointsNode::goal_result_callback(const std_msgs::msg::String::SharedPtr goal_result)
{
  waypoint_manager_->process_goal_result(goal_result); // Handle navigation goal result
}

// --- start_navigation ---
void InventoryWaypointsNode::start_navigation()
{
  update_timer_->reset(); // Start periodic waypoint updates
}

// --- stop_navigation ---
void InventoryWaypointsNode::stop_navigation()
{
  update_timer_->cancel(); // Stop periodic waypoint updates
}
