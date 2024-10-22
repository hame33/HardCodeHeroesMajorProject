// InventoryWaypointsNode.cpp

// --- Includes ---
#include "InventoryWaypointsNode.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

// --- InventroyWaypointsNode class implementation ---
InventoryWaypointsNode::InventoryWaypointsNode(rclcpp::Node::SharedPtr node)
: node_(node)
{
  // Generalised Qos to be used
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  std::cout << "qos defined" << std::endl;

  // Initialise Components
  sensor_processor_ = std::make_shared<SensorProcessor>();
  waypoint_manager_ = std::make_shared<WaypointManager>(node_);
  waypoint_generator_ = std::make_shared<WaypointGenerator>(sensor_processor_, waypoint_manager_);

  std::cout << "node class components initialised" << std::endl;

  // Initialise Subscribers
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

  // Initialize Timer but do not start it yet
  update_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&InventoryWaypointsNode::waypoint_callback, this)
  );
  update_timer_->cancel(); // Cancel timer until navigation is started

  RCLCPP_INFO(node_->get_logger(), "InventoryWaypointsNode has been initialized");
}

void InventoryWaypointsNode::start_navigation()
{
  update_timer_->reset(); // Start the timer
}

void InventoryWaypointsNode::stop_navigation()
{
  update_timer_->cancel(); // Stop the timer
}

// --- Destructor ---
InventoryWaypointsNode::~InventoryWaypointsNode()
{
  RCLCPP_INFO(node_->get_logger(), "InventoryWaypointsNode has been terminated");
}


// --- waypoint_callback ---
void InventoryWaypointsNode::waypoint_callback()
{
  std::array<double, Constants::NUM_SCAN_POSITIONS> scan_distance_data = sensor_processor_->get_scan_distance_data();
  std::array<std::pair<double, double>, Constants::NUM_SCAN_POSITIONS> scan_location_data = sensor_processor_->get_scan_location_data();


  for (int num = 0; num < Constants::NUM_SCAN_POSITIONS; num++ )
  {
      waypoint_generator_->create_waypoint(num, scan_distance_data, scan_location_data);
  }
  waypoint_manager_->publish_waypoints();
  waypoint_manager_->publish_markers();
  waypoint_manager_->publish_goals();
  //waypoint_manager_->print_waypoints();
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