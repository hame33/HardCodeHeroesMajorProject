// WaypointGenerator.cpp

// --- Includes ---
#include "WaypointGenerator.hpp"

// --- WaypointGenerator class implementation ---

// --- Constructor ---
WaypointGenerator::WaypointGenerator(std::shared_ptr<SensorProcessor> sensor_processor, std::shared_ptr<WaypointManager> waypoint_manager)
: sensor_processor_(sensor_processor), waypoint_manager_(waypoint_manager)
{
}


// --- create_waypoint ---
void WaypointGenerator::create_waypoint(int num, std::array<double, Constants::NUM_SCAN_POSITIONS> scan_distance_data, std::array<std::pair<double, double>, Constants::NUM_SCAN_POSITIONS> scan_location_data)
{
  geometry_msgs::msg::Point::SharedPtr new_waypoint_ = std::make_shared<geometry_msgs::msg::Point>();  
  new_waypoint_->x = scan_location_data[num].first;
  new_waypoint_->y = scan_location_data[num].second;
  new_waypoint_->z = 0.0;

  waypoint_manager_->add_waypoint(new_waypoint_, scan_distance_data[num]);
}