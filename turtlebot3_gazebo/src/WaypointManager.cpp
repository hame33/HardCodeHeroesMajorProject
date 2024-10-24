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
WaypointManager::WaypointManager(std::shared_ptr<MapManager> map_manager,
                                 std::shared_ptr<MotionController> motion_controller,
                                 rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub,
                                 rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub,
                                 rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub)
: Node("waypoint_manager"), map_manager_(map_manager), motion_controller_(motion_controller), waypoint_pub_(waypoint_pub), marker_pub_(marker_pub), goal_pub_(goal_pub)
{
  waypoints_[0] = std::make_shared<geometry_msgs::msg::Point>();
  std::cout << "Waypoint manager constructor" << std::endl;
  closest_frontier_goal_ = std::make_pair(0.0,0.0);
  completed_goals_.push_back({0.0,0.0});
  publish_goal_check_ = 1;
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
void WaypointManager::publish_goal()
{
  closest_frontier_goal_ = map_manager_->get_closest_frontier();

  auto goal = geometry_msgs::msg::PoseStamped();
  goal.header.frame_id = "map";
  
  goal.header.stamp = this->get_clock()->now();

    goal.pose.position.x = closest_frontier_goal_.first;
    goal.pose.position.y = closest_frontier_goal_.second;
    goal.pose.position.z = 0.0;

    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;

    if(publish_goal_check_)
    {
      goal_pub_->publish(goal);
      std::cout << "About to send goal follow action" << std::endl;
      motion_controller_->goal_follower(goal);
      RCLCPP_INFO(this->get_logger(), "Published goal @: [%f, %f]",goal.pose.position.x, goal.pose.position.y);
      publish_goal_check_ = 0;
    }
}

// --- process_goal_result ---
void WaypointManager::process_goal_result(const std_msgs::msg::String::SharedPtr goal_result)
{
  if (goal_result->data == "Success") 
  {
    std::cout << "Adding " << closest_frontier_goal_.first << " " << closest_frontier_goal_.second << " to the list of completed goals" << std::endl;
    completed_goals_.push_back(closest_frontier_goal_);
    publish_goal_check_ = 1;
  } 
  else if (goal_result->data == "Aborted")
  {
    return; 
  }
  else 
  {
    return;
  }
}

// --- print_completed_goals ---
void WaypointManager::print_completed_goals() {
    std::cout << "Completed Goals:" << std::endl;
    for (const auto& goal : completed_goals_) {
        std::cout << "(" << goal.first << ", " << goal.second << ")" << std::endl;
    }
}
// --- get_completed_goals_ ---
std::vector<std::pair<double,double>> WaypointManager::get_completed_goals() const
{
  return completed_goals_;
}

// --- set_map_manager ---
void WaypointManager::set_map_manager(std::shared_ptr<MapManager> map_manager)
{
  map_manager_ = map_manager;
}