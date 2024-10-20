#include "MapManager.hpp"
#include <iostream>
#include <utility>
#include <vector>

// --- MapManager Class Implementation ---

// --- Constructor ---
MapManager::MapManager(std::shared_ptr<tf2_ros::Buffer> tf_buffer, std::shared_ptr<tf2_ros::TransformListener> tf_listener, std::shared_ptr<SensorProcessor> sensor_processor)
: tf_buffer_(tf_buffer), tf_listener_(tf_listener), sensor_processor_(sensor_processor)
{
  robot_grid_x_pos_ = 0;
  robot_grid_y_pos_ = 0;
  closest_frontier_ = std::make_pair(0.0,0.0);
}

MapManager::MapManager(std::shared_ptr<SensorProcessor> sensor_processor)
: sensor_processor_(sensor_processor)
{
  robot_grid_x_pos_ = 0;
  robot_grid_y_pos_ = 0;
  closest_frontier_ = std::make_pair(0,0);
}

// --- process_map_data ---
void MapManager::process_map_data(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg) 
{    
  std::cout << "map_callback" << std::endl;
  std::cout << "Received map with size: " << ocp_grid_msg->info.width << " by " << ocp_grid_msg->info.height << std::endl;

  int8_t min_cell_value = find_min_cell_value(ocp_grid_msg);

  for (unsigned int row = (ocp_grid_msg->info.height - 1); row > 0; row--) 
  {
    for (unsigned int col = 0; col < ocp_grid_msg->info.width; col++)
    {
      unsigned int index = (ocp_grid_msg->info.width*row) + col;
      int8_t cell_value = ocp_grid_msg->data[index]; 
      if (cell_value <= -1)
      {
        std::cout << "? ";
      }
      else if (cell_value > min_cell_value)
      {
        std::cout << "| ";
      }
      else
      {
        std::cout << "  ";
        search_for_frontiers(ocp_grid_msg, col, row);
      }
    }
    std::cout << std::endl;
  }

  print_frontier_pixels();

  double robot_x_pos = sensor_processor_->get_robot_x_pos();
  double robot_y_pos = sensor_processor_->get_robot_y_pos();

  robot_grid_x_pos_ = std::abs(ocp_grid_msg->info.origin.position.x) / ocp_grid_msg->info.resolution;
  robot_grid_y_pos_ = std::abs(ocp_grid_msg->info.origin.position.y) / ocp_grid_msg->info.resolution;

  find_closest_frontier(ocp_grid_msg);

  std::cout << "Grid map origin" << ocp_grid_msg->info.origin.position.x << " " << ocp_grid_msg->info.origin.position.y << std::endl;
  std::cout << "Robot world coordinates: (" << robot_x_pos << "," << robot_y_pos << ")" << std::endl;
  std::cout << "Robot grid coordinates: (" << robot_grid_x_pos_ << "," << robot_grid_y_pos_ << ")" << std::endl;
  std::cout << "Closest Frontier: (" << closest_frontier_.first << "," << closest_frontier_.second << ")" << std::endl;

  frontier_pixels_.clear();
}

// --- search_for_frontiers ---
void MapManager::search_for_frontiers(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg, unsigned int grid_cell_x, unsigned int grid_cell_y)
{
  unsigned int index = (ocp_grid_msg->info.width * grid_cell_y) + grid_cell_x;
  if (ocp_grid_msg->data[index - ocp_grid_msg->info.width] <= -1)
  {
    std::pair<unsigned int, unsigned int> frontier_pixel = std::make_pair(grid_cell_x, grid_cell_y);
    frontier_pixels_.push_back(frontier_pixel);
  }
  else if (ocp_grid_msg->data[index + ocp_grid_msg->info.width] <= -1)
  {
    std::pair<unsigned int, unsigned int> frontier_pixel = std::make_pair(grid_cell_x, grid_cell_y);
    frontier_pixels_.push_back(frontier_pixel);
  }
  else if (ocp_grid_msg->data[index - 1] <= -1)
  {
    std::pair<unsigned int, unsigned int> frontier_pixel = std::make_pair(grid_cell_x, grid_cell_y);
    frontier_pixels_.push_back(frontier_pixel);
  }
  else if (ocp_grid_msg->data[index + 1] <= -1)
  {
    std::pair<unsigned int, unsigned int> frontier_pixel = std::make_pair(grid_cell_x, grid_cell_y);
    frontier_pixels_.push_back(frontier_pixel);
  }
}

// --- print_frontier_pixels ---
void MapManager::print_frontier_pixels() 
{
  std::cout << "Frontier Pixels:" << std::endl;
  for (const auto& pixel : frontier_pixels_) {
      std::cout << "(" << pixel.first << ", " << pixel.second << ")" << std::endl;
  }
}

// --- find_min_cell_value ---
int8_t MapManager::find_min_cell_value(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg)
{
  int8_t min_cell_value = 100;
  for (unsigned int i = 0; i < ocp_grid_msg->info.width * ocp_grid_msg->info.height; i++)
  {
    if ((ocp_grid_msg->data[i] < min_cell_value) && (ocp_grid_msg->data[i] > -1))
    {
      min_cell_value = ocp_grid_msg->data[i];
    }
  }

  return min_cell_value;
}

// --- find_closest_frontier ---
void MapManager::find_closest_frontier(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg)
{
  double shortest_distance = 0;
  for (const auto& pixel : frontier_pixels_)
  {
    double current_x_distance = robot_grid_x_pos_ - pixel.first;
    double current_y_distance = robot_grid_y_pos_ - pixel.second;
    double current_distance = std::sqrt((current_x_distance * current_x_distance) + (current_y_distance * current_y_distance));
    if (shortest_distance == 0)
    {
      shortest_distance = current_distance;
    }
    else if (current_distance < shortest_distance)
    {
      shortest_distance = current_distance;
      closest_frontier_.first = (pixel.first - std::abs(ocp_grid_msg->info.origin.position.x)) * ocp_grid_msg->info.resolution;
      closest_frontier_.second = (pixel.second - std::abs(ocp_grid_msg->info.origin.position.y)) * ocp_grid_msg->info.resolution;
    }
  }  
 }

 // --- get_closest_frontier_ ---
 std::pair<double,double> MapManager::get_closest_frontier()
 {
  return closest_frontier_;
 }
