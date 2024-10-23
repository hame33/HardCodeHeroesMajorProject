// InventoryWaypointsNode.cpp

// --- Includes ---
#include "InventoryWaypointsNode.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>
#include <iostream>
#include <utility>

using namespace std::chrono_literals;

// --- InventroyWaypointsNode class implementation ---
InventoryWaypointsNode::InventoryWaypointsNode()
: Node("inventory_waypoints_node")
{
  // Generalised Qos to be used
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  std::cout << "qos defined" << std::endl;

  // Initialise TF2 components
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialise Components
  sensor_processor_ = std::make_shared<SensorProcessor>();
  map_manager_ = std::make_shared<MapManager>(sensor_processor_, waypoint_manager_);
  waypoint_manager_ = std::make_shared<WaypointManager>(map_manager_,
                                                        this->create_publisher<geometry_msgs::msg::Point>("inventory_waypoints", qos),
                                                        this->create_publisher<visualization_msgs::msg::Marker>("waypoint_markers", qos),
                                                        this->create_publisher<geometry_msgs::msg::PoseStamped>("inventory_goals", qos));
  waypoint_generator_ = std::make_shared<WaypointGenerator>(sensor_processor_, waypoint_manager_);
  // map_manager_ = std::make_shared<MapManager>(sensor_processor_);

  std::cout << "node class components initialised" << std::endl;

  // Initialise Subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
  "scan",
  rclcpp::SensorDataQoS(),
  std::bind(&InventoryWaypointsNode::scan_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
  "odom",
  qos,
  std::bind(&InventoryWaypointsNode::odom_callback, this, std::placeholders::_1));

  ocp_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
  "map", 
  qos, 
  std::bind(&InventoryWaypointsNode::map_callback, this, std::placeholders::_1));

  goal_result_sub_ = this->create_subscription<std_msgs::msg::String>(
    "goal_result",
    qos,
    std::bind(&InventoryWaypointsNode::goal_result_callback, this, std::placeholders::_1));

  // Initialise Timer
  update_timer_ = this->create_wall_timer(
    10ms,
    std::bind(&InventoryWaypointsNode::waypoint_callback, this));

  RCLCPP_INFO(this->get_logger(), "InventoryWaypointsNode has been initialized");
}

// --- Destructor ---
InventoryWaypointsNode::~InventoryWaypointsNode()
{
  RCLCPP_INFO(this->get_logger(), "InventoryWaypointsNode has been terminated");
}


// // --- waypoint_callback ---
// void InventoryWaypointsNode::waypoint_callback()
// {
//   std::array<double, Constants::NUM_SCAN_POSITIONS> scan_distance_data = sensor_processor_->get_scan_distance_data();
//   std::array<std::pair<double, double>, Constants::NUM_SCAN_POSITIONS> scan_location_data = sensor_processor_->get_scan_location_data();


//   for (int num = 0; num < Constants::NUM_SCAN_POSITIONS; num++ )
//   {
//       waypoint_generator_->create_waypoint(num, scan_distance_data, scan_location_data);
//   }
//   waypoint_manager_->publish_waypoints();
//   waypoint_manager_->publish_markers();
//   waypoint_manager_->publish_goals();
//   //waypoint_manager_->print_waypoints();
//   waypoint_manager_->clear_waypoints();
// }

// --- waypoint_callback ---
void InventoryWaypointsNode::waypoint_callback()
{

}

// --- scan_callback ---
void InventoryWaypointsNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  sensor_processor_->process_scan(scan_msg);
}

// --- odom_callback ---
void InventoryWaypointsNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  sensor_processor_->process_odom(odom_msg);
}

// --- map_callback ---
void InventoryWaypointsNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg)
{
  map_manager_->process_map_data(ocp_grid_msg);
  waypoint_manager_->publish_goal();
}

// --- goal_result_callback ---
void InventoryWaypointsNode::goal_result_callback(const std_msgs::msg::String goal_result)
{
  waypoint_manager_->process_goal_result(goal_result);
}