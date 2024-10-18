#include "MapManager.hpp"
#include <iostream>

// --- MapManager Class Implementation ---

// // Constructor
// MapManager::MapManager(std::shared_ptr<tf2_ros::Buffer> tf_buffer, std::shared_ptr<tf2_ros::TransformListener> tf_listener, std::shared_ptr<SensorProcessor> sensor_processor)
// : tf_buffer_(tf_buffer), tf_listener_(tf_listener), sensor_processor_(sensor_processor)
// {

// }

// Constructor
MapManager::MapManager(std::shared_ptr<SensorProcessor> sensor_processor)
: sensor_processor_(sensor_processor)
{

}

// process_map_data
void MapManager::process_map_data(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg) 
{    
  std::cout << "map_callback" << std::endl;
  std::cout << "Received map with size: " << ocp_grid_msg->info.width << " by " << ocp_grid_msg->info.height << std::endl;

  for (unsigned int i = 0; i < ocp_grid_msg->info.width * ocp_grid_msg->info.height; i++) 
  {
    int8_t cell_value = ocp_grid_msg->data[i]; 
  }

  double robot_x_pos = sensor_processor_->get_robot_x_pos();
  double robot_y_pos = sensor_processor_->get_robot_y_pos();

  double cell_x = ocp_grid_msg->info.origin.position.x + (robot_x_pos * ocp_grid_msg->info.resolution);
  double cell_y = ocp_grid_msg->info.origin.position.y + (robot_y_pos * ocp_grid_msg->info.resolution);
  std::cout << "Cell world coordinates: (" << cell_x << "," << cell_y << ")" << std::endl;
}
