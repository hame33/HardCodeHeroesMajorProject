// --- Includes ---
#include "WaypointGenerator.hpp"

// --- WaypointGenerator Implementation ---

// --- Constructor ---
WaypointGenerator::WaypointGenerator(std::shared_ptr<SensorProcessor> sensor_processor, std::shared_ptr<WaypointManager> waypoint_manager)
: sensor_processor_(sensor_processor), waypoint_manager_(waypoint_manager)
{
}

// --- create_waypoint ---
void WaypointGenerator::create_waypoint(int num, std::array<double, Constants::NUM_SCAN_POSITIONS> scan_distance_data, std::array<std::pair<double, double>, Constants::NUM_SCAN_POSITIONS> scan_location_data)
{
  geometry_msgs::msg::Point::SharedPtr new_waypoint = std::make_shared<geometry_msgs::msg::Point>();  
  new_waypoint->x = scan_location_data[num].first;
  new_waypoint->y = scan_location_data[num].second;
  new_waypoint->z = 0.0;

  waypoint_manager_->add_waypoint(new_waypoint, scan_distance_data[num]);
}
