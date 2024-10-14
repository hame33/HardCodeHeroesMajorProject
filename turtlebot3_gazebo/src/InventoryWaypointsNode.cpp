// InventoryWaypointsNode.cpp

// --- Includes ---
#include "InventoryWaypointsNode.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

// --- InventroyWaypointsNode class implementation ---
InventoryWaypointsNode::InventoryWaypointsNode()
: Node("inventory_waypoints_node")
{
  // Generalised Qos to be used
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  std::cout << "qos defined" << std::endl;

  // Initialise Components
  sensor_processor_ = std::make_shared<SensorProcessor>();
  waypoint_manager_ = std::make_shared<WaypointManager>();
  waypoint_generator_ = std::make_shared<WaypointGenerator>(sensor_processor_, waypoint_manager_);

  std::cout << "node class components initialised" << std::endl;

  // Initialise Publisher
  waypoint_pub_ = this->create_publisher<geometry_msgs::msg::Point>("inventory_waypoints", qos);

  // Initialise Subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
  "scan",
  rclcpp::SensorDataQoS(),
  std::bind(&InventoryWaypointsNode::scan_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
  "odom",
  qos,
  std::bind(&InventoryWaypointsNode::odom_callback, this, std::placeholders::_1));

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


// --- waypoint_callback ---
void InventoryWaypointsNode::waypoint_callback()
{
  waypoint_generator_->create_waypoint();
  waypoint_manager_->print_waypoints();
  waypoint_manager_->clear_waypoints();
}

// --- scan_callback ---
void InventoryWaypointsNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  sensor_processor_->process_scan(msg);
}

// --- odom_callback ---
void InventoryWaypointsNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  sensor_processor_->process_odom(msg);
}