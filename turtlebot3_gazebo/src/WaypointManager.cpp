// WaypointManager.cpp

// --- Includes ---
#include "WaypointManager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <algorithm> 
#include <utility> 
#include <iostream>
#include <map>

// --- Constructor ---
WaypointManager::WaypointManager(rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub,
                                 rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub)
: Node("waypoint_manager"), waypoint_pub_(waypoint_pub), marker_pub_(marker_pub)
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
    std::cout << "Waypoints count" << waypoints_.size() << std::endl;
}

// --- publish_waypoints ---
void WaypointManager::publish_waypoints()
{
  for (const auto& [distance, waypoint] : waypoints_)
  {
    waypoint_pub_->publish(*waypoint);
  }
  
}

// --- publish_waypoints ---
void WaypointManager::publish_markers()
{
  int id = 0;
    for (const auto& [distance, waypoint] : waypoints_) 
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";  // Set your reference frame
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "waypoints";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position of the marker
        marker.pose.position = *waypoint;
        marker.pose.orientation.w = 1.0;

        // Set the scale (size) of the marker
        marker.scale.x = 0.08;
        marker.scale.y = 0.08;
        marker.scale.z = 0.08;

        // Set the color of the marker
        marker.color.r = 10.0;  // Red
        marker.color.g = 1.0;  // Green
        marker.color.b = 0.0;  // Blue
        marker.color.a = 1.0;  // Opaque

        // Publish the marker
        marker_pub_->publish(marker);
    }
}
// // --- print_waypoint ---
// void WaypointManager::print_waypoint(double distance_from_bot_key)
// {

// }