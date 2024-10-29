#ifndef MAP_MANAGER_HPP_
#define MAP_MANAGER_HPP_

// --- Includes ---
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "SensorProcessor.hpp"
#include "Constants.hpp"
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>
#include <queue>
#include <limits>
#include <memory>

// --- Forward Declarations ---
class WaypointManager;  // To get around circular header includes

// --- MapManager Class Interface ---
// The MapManager class processes and manages map data, primarily working 
// with occupancy grid information to identify and handle frontier pixels 
// for navigation. It retrieves transformation data through TF2, enabling 
// it to track the robot’s position in the map. The class interacts with 
// SensorProcessor and WaypointManager to aid in creating waypoints and 
// determining the closest frontier. Its methods include functions for 
// processing occupancy grids, searching for frontiers, and publishing 
// updates for navigation purposes.
class MapManager {
public:
    // --- Constructor ---
    MapManager(std::shared_ptr<tf2_ros::Buffer> tf_buffer,
               std::shared_ptr<tf2_ros::TransformListener> tf_listener,
               std::shared_ptr<SensorProcessor> sensor_processor);

    MapManager(std::shared_ptr<SensorProcessor> sensor_processor,
               std::shared_ptr<WaypointManager> waypoint_manager);

    // --- MapManager Functions ---
    
    // Process map data - processes occupancy grid map data and stores the processed data
    void process_map_data(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg);

    // Print frontier pixels - outputs detected frontier pixels for debugging
    void print_frontier_pixels();

    // --- Getter Methods ---
    std::pair<double, double> get_closest_frontier() const;

private:
    // --- TF2 Components ---
    std::shared_ptr<tf2_ros::Buffer>           tf_buffer_;    // Buffer for managing transform data
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // Listener for transform data

    // --- Components ---
    std::shared_ptr<SensorProcessor> sensor_processor_;  // Sensor data processing component
    std::weak_ptr<WaypointManager> waypoint_manager_;    // Waypoint managing component

    // --- Data ---
    std::vector<std::pair<int, int>> frontier_pixels_;   // Vector storing frontier pixels (free space bordering unknown)
    int robot_grid_x_pos_;                               // Robot's x position in occupancy grid map
    int robot_grid_y_pos_;                               // Robot's y position in occupancy grid map
    std::pair<double, double> closest_frontier_;         // Closest frontier pixel to robot in world coordinates
    std::pair<int, int> closest_frontier_pixels_;        // Closest frontier pixel to robot in pixel coordinates

    // --- Helper Functions ---
    void search_for_frontiers(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg, int grid_cell_x, int grid_cell_y);       // Searches for fr
