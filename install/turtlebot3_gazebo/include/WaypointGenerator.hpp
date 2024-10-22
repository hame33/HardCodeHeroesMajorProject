#ifndef WAYPOINT_GENERATOR_HPP_
#define WAYPOINT_GENERATOR_HPP_

// --- Includes ---

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <geometry_msgs/msg/point.hpp>
#include "Constants.hpp"
#include "SensorProcessor.hpp"
#include "WaypointManager.hpp"
#include <utility>
#include <array>

// --- WaypointGenerator Class Interface ---
class WaypointGenerator
{
public:
  // Constructor - Initialises WaypointGenerator class
  WaypointGenerator(std::shared_ptr<SensorProcessor> sensor_processor, std::shared_ptr<WaypointManager> waypoint_manager);

  // create_waypoint - creates a waypoint on the map 
  void create_waypoint(int num, std::array<double, Constants::NUM_SCAN_POSITIONS> scan_distance_data, std::array<std::pair<double, double>, Constants::NUM_SCAN_POSITIONS> scan_location_data);

private:
  // --- Components --- 
  std::shared_ptr<SensorProcessor> sensor_processor_;   // Ptr to the SensorProcessor instance
  std::shared_ptr<WaypointManager> waypoint_manager_;   // Ptr to the WaypointGenerator instance
};


#endif // WAYPOINT_GENERATOR_HPP_