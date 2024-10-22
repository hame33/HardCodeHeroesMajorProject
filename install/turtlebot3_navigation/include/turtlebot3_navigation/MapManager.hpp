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
#include "MotionController.hpp"
#include "SensorProcessor.hpp"

// --- MapManager Class Implementation ---
class MapManager {
public:
  // Constructor
//   MapManager(std::shared_ptr<tf2_ros::Buffer> tf_buffer, std::shared_ptr<tf2_ros::TransformListener> tf_listener, std::shared_ptr<SensorProcessor> sensor_processor);
  MapManager(std::shared_ptr<SensorProcessor> sensor_processor);


  // process_map_data
  void process_map_data(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg);

private:
  // --- TF2 Components ---
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;   

  // --- Components ---
  std::shared_ptr<SensorProcessor> sensor_processor_;     // Sensor data processing component
};


#endif // MAP_MANAGER_HPP