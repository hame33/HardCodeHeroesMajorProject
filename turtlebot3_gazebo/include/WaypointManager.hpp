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

// --- WaypointManager Class Definition ---
class WaypointManager : public rclcpp::Node
{
public:
    // Constructor
    WaypointManager(std::shared_ptr<MapManager> map_manager,
                   std::shared_ptr<MotionController> motion_controller,
                   rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub,
                   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub,
                   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub);

    // Waypoint management functions
    void add_waypoint(geometry_msgs::msg::Point::SharedPtr waypoint, double distance_from_bot); // Adds waypoint to map
    void remove_waypoint(double distance_from_bot);                                            // Removes a waypoint
    void clear_waypoints();                                                                   // Clears all waypoints
    void print_waypoints();                                                                   // Prints all waypoints
    void publish_waypoints();                                                                 // Publishes waypoints
    void publish_markers();                                                                   // Publishes waypoints as markers
    void publish_goal();                                                                       // Publishes Nav2 goals (closest frontier becomes goal)
    void process_goal_result(const std_msgs::msg::String::SharedPtr goal_result);            // Processes Nav2 goal result
    void print_completed_goals();                                                             // Prints completed goals
    void publish_return_to_start();                                                           // Publishes return to start goal

    // Getter method
    std::vector<std::pair<double, double>> get_completed_goals() const;                      // Returns completed goals

    // Setter method
    void set_map_manager(std::shared_ptr<MapManager> map_manager);                           // Sets the MapManager

private:
    // Components
    std::shared_ptr<MapManager> map_manager_;
    std::shared_ptr<MotionController> motion_controller_;

    // ROS Publishers
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub_;             // Publisher for waypoints
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;         // Publisher for waypoints as markers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;           // Publisher for Nav2 goals

    // Data
    std::map<double, geometry_msgs::msg::Point::SharedPtr> waypoints_;                  // Map storing waypoints with distance from bot
    std::pair<double, double> closest_frontier_goal_;                                   // Closest frontier goal in world coordinates
    std::vector<std::pair<double, double>> completed_goals_;                            // List of completed goals
    int publish_goal_check_;                                                            // Flag to control goal publishing
};

#endif // WAYPOINT_MANAGER_HPP_
