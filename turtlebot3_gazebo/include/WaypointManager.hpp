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

// --- WaypointManager Class Interface ---
// Manages waypoints for autonomous robot navigation, including functions to 
// add, remove, and clear waypoints. It works with MapManager for coordinate 
// transformations and MotionController to execute navigation. The class 
// publishes waypoints and markers to ROS, allowing for navigation and visual 
// feedback in RViz. Supports setting goals, processing their results, and a 
// return-to-start function for safe navigation.
class WaypointManager : public rclcpp::Node
{
public:
    // --- Constructor ---
    WaypointManager(std::shared_ptr<MapManager> map_manager,
                    std::shared_ptr<MotionController> motion_controller,
                    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub,
                    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub,
                    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub);

    // --- WaypointManager Functions ---

    // add_waypoint - Adds a waypoint to the map
    void add_waypoint(geometry_msgs::msg::Point::SharedPtr waypoint, double distance_from_bot);

    // remove_waypoint - Removes a waypoint at a specified distance
    void remove_waypoint(double distance_from_bot);

    // clear_waypoints - Clears all stored waypoints
    void clear_waypoints();

    // print_waypoints - Outputs all waypoints for debugging
    void print_waypoints();

    // publish_waypoints - Publishes all waypoints to the ROS topic
    void publish_waypoints();

    // publish_markers - Publishes waypoints as markers for RViz
    void publish_markers();

    // publish_goal - Publishes the closest frontier as a navigation goal
    void publish_goal();

    // process_goal_result - Processes the result of the latest goal
    void process_goal_result(const std_msgs::msg::String::SharedPtr goal_result);

    // print_completed_goals - Outputs completed goals for debugging
    void print_completed_goals();

    // publish_return_to_start - Publishes a return-to-start goal
    void publish_return_to_start();

    // --- Getter Method ---
    std::vector<std::pair<double, double>> get_completed_goals() const;  // Returns completed goals

    // --- Setter Method ---
    void set_map_manager(std::shared_ptr<MapManager> map_manager);  // Sets the MapManager

private:
    // --- Components ---
    std::shared_ptr<MapManager> map_manager_;              // Pointer to map managing component
    std::shared_ptr<MotionController> motion_controller_;  // Pointer to motion controller component

    // --- ROS Publishers ---
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub_;      // Publisher for waypoints
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;  // Publisher for waypoints as markers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;    // Publisher for Nav2 goals

    // --- Data ---
    std::map<double, geometry_msgs::msg::Point::SharedPtr> waypoints_;   // Map storing waypoints with distance from bot
    std::pair<double, double> closest_frontier_goal_;                    // Closest frontier goal in world coordinates
    std::vector<std::pair<double, double>> completed_goals_;             // List of completed goals
    int publish_goal_check_;                                             // Flag to control goal publishing
};

#endif // WAYPOINT_MANAGER_HPP_
