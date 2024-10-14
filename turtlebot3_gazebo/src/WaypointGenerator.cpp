// WaypointGenerator.cpp

// --- Includes ---
#include "WaypointGenerator.hpp"

// --- WaypointGenerator class implementation ---

// --- Constructor ---
WaypointGenerator::WaypointGenerator(std::shared_ptr<SensorProcessor> sensor_processor, std::shared_ptr<WaypointManager> waypoint_manager)
: sensor_processor_(sensor_processor), waypoint_manager_(waypoint_manager)
{
  new_waypoint_ = std::make_shared<geometry_msgs::msg::Point>();
  new_waypoint_->x = 0.0;
  new_waypoint_->y = 0.0;
  new_waypoint_->z = 0.0;
}


// --- create_waypoint ---
void WaypointGenerator::create_waypoint()
{
  std::array<double, Constants::NUM_SCAN_POSITIONS> scan_distance_data = sensor_processor_->get_scan_distance_data();
  std::array<std::pair<double, double>, Constants::NUM_SCAN_POSITIONS> scan_location_data = sensor_processor_->get_scan_location_data();
  
  for (int num = 0; num < Constants::NUM_SCAN_POSITIONS; num++)
  {
    if (scan_distance_data[num] == Constants::MAX_SCAN_DISTANCE)
    {
      new_waypoint_->x = scan_location_data[num].first;
      new_waypoint_->y = scan_location_data[num].second;
      new_waypoint_->z = 0.0;  
      
      waypoint_manager_->add_waypoint(new_waypoint_, scan_distance_data[num]);
    }
  }
}