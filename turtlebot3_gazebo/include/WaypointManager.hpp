#ifndef WAYPOINT_MANAGER_HPP_
#define WAYPOINT_MANAGER_HPP_

// --- Includes ---

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <map>
#include "Constants.hpp"

// --- WaypointManager class interface ---
class WaypointManager : public rclcpp::Node
{
public:
  // Constructor - Initalise WaypointManager class
  WaypointManager(rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub,
                  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub,
                  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub);

  void add_waypoint(geometry_msgs::msg::Point::SharedPtr waypoint, double distance_from_bot); // Adds waypoint to back of vector
  void remove_waypoint(double distance_from_bot);  // Remove a Waypoint
  void clear_waypoints(); // Clears waypoints from map.
  void print_waypoints(); // Prints out waypoints
  void publish_waypoints(); // Publishes waypoints 
  void publish_markers(); // Publishes waypoints as markers 
  void publish_goals(); // Publishes Nav2 goals
private:
  // --- ROS Publishers ---
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub_;  // Publisher for waypoints
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;  // Publisher for waypoints as markers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;  // Publisher for Nav2 goals

  // --- Data ---
  std::map<double ,geometry_msgs::msg::Point::SharedPtr> waypoints_;  // Map storing waypoints and their associated distance from bot
};



#endif // WAYPOINT_MANAGER_HPP_