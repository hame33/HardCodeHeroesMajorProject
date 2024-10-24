// MapManager.hpp

#ifndef MAP_MANAGER_HPP_
#define MAP_MANAGER_HPP_

// --- includes ---
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "SensorProcessor.hpp"
#include "Constants.hpp"
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>

// --- Forward Declarations ---
class WaypointManager;  // To get around circular header includes

// --- MapManager Class Implementation ---
class MapManager {
public:
  // Constructor
  MapManager(std::shared_ptr<tf2_ros::Buffer> tf_buffer, std::shared_ptr<tf2_ros::TransformListener> tf_listener, std::shared_ptr<SensorProcessor> sensor_processor);
  MapManager(std::shared_ptr<SensorProcessor> sensor_processor, std::shared_ptr<WaypointManager> waypoint_manager);


  // process_map_data
  void process_map_data(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg);

  // print_frontier_pixels
  void print_frontier_pixels();

  // --- Getter methods ---
  std::pair<double,double> get_closest_frontier() const;

private:
  // --- TF2 Components ---
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_; 

  // --- Components ---
  std::shared_ptr<SensorProcessor> sensor_processor_;     // Sensor data processing component
  std::weak_ptr<WaypointManager> waypoint_manager_;  // Waypoint managing component

  // --- Data ---
  std::vector<std::pair<int, int>> frontier_pixels_;  // Vector storing frontier pixels (free space bordering unknown) 
  int robot_grid_x_pos_;  // Robot's x position in terms of occupancy grid map
  int robot_grid_y_pos_;  // Robot's y position in terms of occupancy grid map
  std::pair<double, double> closest_frontier_;  // Closest frontier pixel to robot in world coordinates
  std::pair<int,int> closest_frontier_pixels_; // Closest frontier pixel to robot in pixel coordinates

  // --- Helper functions ---
  void search_for_frontiers(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg, int grid_cell_x, int grid_cell_y);  
  int find_min_cell_value(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg);
  void find_closest_frontier(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg);
  void find_frontiers_with_bfs(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg);
  void check_walls_at_frontier(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg, int& frontier_pixel_x, int& frontier_pixel_y);
};
#endif // MAP_MANAGER_HPP