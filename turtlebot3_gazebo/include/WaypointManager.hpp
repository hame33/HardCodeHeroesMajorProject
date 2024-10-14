#ifndef WAYPOINT_MANAGER_HPP_
#define WAYPOINT_MANAGER_HPP_

// --- Includes ---

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <map>
#include "Constants.hpp"

// --- WaypointManager class interface ---
class WaypointManager
{
public:
  // Constructor - Initalise WaypointManager class
  WaypointManager();

  void add_waypoint(geometry_msgs::msg::Point::SharedPtr waypoint, double distance_from_bot); // Adds waypoint to back of vector
  void remove_waypoint(double distance_from_bot);  // Remove a Waypoint
  void clear_waypoints(); // Clears waypoints from map.
  void print_waypoints(); // Prints out waypoints

private:
  // --- Data ---
  std::map<double ,geometry_msgs::msg::Point::SharedPtr> waypoints_;  // Map storing waypoints and their associated distance from bot
};



#endif // WAYPOINT_MANAGER_HPP_