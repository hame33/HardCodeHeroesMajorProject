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
                                 rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub,
                                 rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub)
: Node("waypoint_manager"), waypoint_pub_(waypoint_pub), marker_pub_(marker_pub), goal_pub_(goal_pub)
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
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "waypoints";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position = *waypoint;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;

    marker.color.r = 10.0; 
    marker.color.g = 1.0;  
    marker.color.b = 0.0;  
    marker.color.a = 1.0;  

    marker_pub_->publish(marker);
  }
}

// --- publish_goals ---
void WaypointManager::publish_goals()
{
  auto goal = geometry_msgs::msg::PoseStamped();
  goal.header.frame_id = "map";
  
  goal.header.stamp = this->get_clock()->now();

  if(waypoints_.find(Constants::MAX_SCAN_DISTANCE) != waypoints_.end())
  {
    geometry_msgs::msg::Point::SharedPtr waypoint_as_goal = std::make_shared<geometry_msgs::msg::Point>();
    waypoint_as_goal = waypoints_.at(Constants::MAX_SCAN_DISTANCE);

    goal.pose.position.x = waypoint_as_goal->x;
    goal.pose.position.y = waypoint_as_goal->y;
    goal.pose.position.z = 0.0;

    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;

    goal_pub_->publish(goal);
    RCLCPP_INFO(this->get_logger(), "Published goal @: [%f, %f]",waypoint_as_goal->x, waypoint_as_goal->y);
  }
  else
  {
    std::cout << "Waypoint with key " << Constants::MAX_SCAN_DISTANCE << " not found" << std::endl;
  }
}