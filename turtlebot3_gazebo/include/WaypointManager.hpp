#ifndef WAYPOINT_MANAGER_HPP_
#define WAYPOINT_MANAGER_HPP_

// --- Includes ---

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <map>
#include <vector>
#include "Constants.hpp"
#include "MapManager.hpp"
#include "MotionController.hpp"

// --- WaypointManager class interface ---
class WaypointManager : public rclcpp::Node
{
public:
  // Constructor - Initalise WaypointManager class
  WaypointManager(std::shared_ptr<MapManager> map_manager,
                  std::shared_ptr<MotionController> motion_controller,
                  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub,
                  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub,
                  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub);

  void add_waypoint(geometry_msgs::msg::Point::SharedPtr waypoint, double distance_from_bot); // Adds waypoint to back of vector
  void remove_waypoint(double distance_from_bot);  // Remove a Waypoint
  void clear_waypoints(); // Clears waypoints from map.
  void print_waypoints(); // Prints out waypoints
  void publish_waypoints(); // Publishes waypoints 
  void publish_markers(); // Publishes waypoints as markers 
  void publish_goal();  // Publishes Nav2 goals (closest frontier becomes goal)
  void process_goal_result(const std_msgs::msg::String goal_result);  // Processes nav2 goal result 
  void print_completed_goals();  // Prints out completed_goals_

  // --- Getter Methods ---
  std::vector<std::pair<double, double>> get_completed_goals() const;

  // --- Setter Methods ---
  void set_map_manager(std::shared_ptr<MapManager> map_manager); 
private:
  // --- Components ---
  std::shared_ptr<MapManager> map_manager_;
  std::shared_ptr<MotionController> motion_controller_;
  
  // --- ROS Publishers ---
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub_;  // Publisher for waypoints
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;  // Publisher for waypoints as markers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;  // Publisher for Nav2 goals

  // --- Data ---
  std::map<double ,geometry_msgs::msg::Point::SharedPtr> waypoints_;  // Map storing waypoints and their associated distance from bot
  std::pair<double, double> closest_frontier_goal_;  // Closest frontier pixel to robot in world coordinates
  std::vector<std::pair<double, double>> completed_goals_; // List of completed goals, so the map manager knows not to pick one of these as the closest_frontier_
  int publish_goal_check_;
};



#endif // WAYPOINT_MANAGER_HPP_