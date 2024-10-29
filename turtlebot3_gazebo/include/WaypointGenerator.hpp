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
// The WaypointGenerator class is responsible for creating waypoints on the map 
// based on sensor data processed by SensorProcessor. This class uses scan data 
// on distances and positions to generate relevant waypoints for navigation. 
// It sends these waypoints to WaypointManager, facilitating seamless integration 
// into the robot’s autonomous navigation framework.
class WaypointGenerator
{
public:
  // --- Constructor ---
  WaypointGenerator(std::shared_ptr<SensorProcessor> sensor_processor, 
                    std::shared_ptr<WaypointManager> waypoint_manager);

  // --- Waypoint Generator Functions ---

  // create_waypoint - Creates a waypoint for the map
  void create_waypoint(int num, 
                       std::array<double, 
                       Constants::NUM_SCAN_POSITIONS> scan_distance_data, 
                       std::array<std::pair<double, double>, Constants::NUM_SCAN_POSITIONS> scan_location_data);  

private:
  // --- Components --- 
  std::shared_ptr<SensorProcessor> sensor_processor_;   // Ptr to the SensorProcessor instance
  std::shared_ptr<WaypointManager> waypoint_manager_;   // Ptr to the WaypointGenerator instance
};


#endif // WAYPOINT_GENERATOR_HPP_
