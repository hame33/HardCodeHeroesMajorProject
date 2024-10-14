// WaypointManager.cpp

// --- Includes ---
#include "WaypointManager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <algorithm> 
#include <utility> 
#include <iostream>

// --- Constructor ---
WaypointManager::WaypointManager() 
{
  waypoints_[0] = std::make_shared<geometry_msgs::msg::Point>();
  std::cout << "Waypoint manager constructor" << std::endl;
}

// --- add_waypoint - Adds waypoint to the map ---
void WaypointManager::add_waypoint(geometry_msgs::msg::Point::SharedPtr waypoint, double distance_from_bot) 
{
    waypoints_[distance_from_bot] = waypoint;
}

// --- remove_waypoint - Removes a waypoint ---
void WaypointManager::remove_waypoint(double distance_from_bot) {
    waypoints_.erase(distance_from_bot);
}

// --- clear_waypoints ---
void WaypointManager::clear_waypoints()
{
    waypoints_.clear();
}

// --- print_waypoints ---
void WaypointManager::print_waypoints()
{
    for (const auto& [key,value] : waypoints_)
    {
        std::cout << '[' << key << "] = {x: " << value->x << ", y: " << value->y << "}" << std::endl;
    }
}

// // --- print_waypoint ---
// void WaypointManager::print_waypoint(double distance_from_bot_key, geometry_msgs::msg::Point::SharedPtr waypoint_value)
// {
    
// }