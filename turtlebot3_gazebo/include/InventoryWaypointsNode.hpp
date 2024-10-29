#ifndef INVENTORY_WAYPOINTS_HPP_
#define INVENTORY_WAYPOINTS_HPP_

// --- Includes ---
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "SensorProcessor.hpp"
#include "WaypointGenerator.hpp"
#include "MotionController.hpp"
#include "WaypointManager.hpp"
#include "MapManager.hpp"
#include "Constants.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

// --- InventoryWaypointsNode Class Interface ---
// InventoryWaypointsNode manages the navigation waypoints for a robot in an 
// inventory setting. It subscribes to sensor and odometry data to process 
// waypoints, map updates, and navigation goals, and handles navigation 
// commands and feedback for effective waypoint tracking and obstacle avoidance.
class InventoryWaypointsNode
{
public:
    // --- Constructor ---
    InventoryWaypointsNode(rclcpp::Node::SharedPtr node);

    // --- Destructor ---
    ~InventoryWaypointsNode();

    // --- InventoryWaypointsNode Functions ---

    // stop_navigation - Stops navigation for the robot
    void stop_navigation();

    // start_navigation - Initiates navigation for the robot
    void start_navigation();

private:
    // --- ROS Core Node ---
    rclcpp::Node::SharedPtr node_;                          // Core ROS node for communication

    // --- ROS Publishers ---
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub_;         // Publisher for waypoints
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;     // Publisher for waypoints as markers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;       // Publisher for Nav2 goals

    // --- ROS Subscribers ---
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;        // Subscriber for LIDAR scan data
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;            // Subscriber for odometry data
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr ocp_grid_sub_;   // Subscriber to the occupancy grid topic
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_result_sub_;       // Subscriber to the nav2 goal result

    // --- Components ---
    std::shared_ptr<SensorProcessor> sensor_processor_;     // Sensor data processing component
    std::shared_ptr<WaypointGenerator> waypoint_generator_; // Waypoint generating component
    std::shared_ptr<WaypointManager> waypoint_manager_;     // Waypoint managing component
    std::shared_ptr<MapManager> map_manager_;               // Map managing component
    std::shared_ptr<MotionController> motion_controller_;   // Motion control component

    // --- ROS Action Clients ---
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigator_client_;  // Action client for following nav2 goals

    // --- TF2 Components ---
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;            // TF2 buffer for transformations
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // TF2 listener for transformations

    // --- ROS Timer ---
    rclcpp::TimerBase::SharedPtr update_timer_;             // Timer for periodic updates

    // --- Callback Functions ---
    void waypoint_callback();                               // Handles waypoint publishing, creation, and management
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);  // Handles LIDAR scan data
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);      // Handles odometry data
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg); // Handles map and occupancy grid data
    void goal_result_callback(const std_msgs::msg::String::SharedPtr goal_result);  // Handles nav2 goal result data
};

#endif // INVENTORY_WAYPOINTS_HPP_
